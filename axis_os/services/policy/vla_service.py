"""
AXIS-OS — Service VLA : fait tourner une politique apprise (GR00T) sur le robot.

Relie, à la cadence de commande du robot :

    caméra de tête + état des bras ──> GR00TConnector.predict_action()
                                          │  (une inférence toutes les `intervalle_s`)
                                          v
              ┌───────── arms : cibles écrites dans `bras` (pose_bras de l'adaptateur)
    VLAAction ┤                 à pas LIMITÉ par tick, comme SetpointFollower
              └───────── base : loco.set_velocity(vx, vy, wz)

AUCUN CHEMIN RAPIDE. Ce service ne touche pas un actionneur :
  - les bras : il écrit des CIBLES dans le dictionnaire de l'adaptateur, qui les
    ramène dans les butées puis les envoie par send_targets -> HAL ->
    JointSafetyGuard, exactement comme le ManipulationService ;
  - la base : LocomotionService.set_velocity borne la vitesse (max_twist), la
    périme au bout de command_timeout_s, et le gardien de proximité humaine la
    plafonne ou l'annule. Ce service n'a aucun moyen de les désactiver.

TROIS GARANTIES, chacune testée (test/test_vla_service.py) :

1. ÉCHÉANCE DE FRAÎCHEUR. Sans action fraîche depuis max_age_s, les bras gardent
   leur dernière cible (on ne progresse PAS vers une décision périmée) et la base
   reçoit halt() — arrêt en équilibre, jamais une coupure.
2. UN SEUL ÉCRIVAIN. Le ManipulationService est l'écrivain de `bras`. Ce service
   refuse de démarrer, et se met en DÉFAUT, si `autre_ecrivain()` est vrai : deux
   écrivains sur le même dictionnaire, c'est une cible qui saute d'un geste à
   l'autre sans que personne ne l'ait décidé.
3. ARRÊT DE SÉCURITÉ. Si `arret_securite()` est vrai, les bras sont FIGÉS (pas
   relâchés), comme le fait ManipulationService : un humain assez proche pour
   arrêter la marche l'est assez pour qu'un bras en mouvement le heurte.

UNE ERREUR D'INFÉRENCE ne plante pas la boucle de commande : la dernière action
reste valable jusqu'à son échéance ; `defauts_max` erreurs d'affilée -> DÉFAUT
(base à l'arrêt, bras figés). Une image absente saute l'inférence (jamais de
décision sans image).

TEMPS. `horloge` est injectable. En simulation synchrone (l'inférence bloque la
boucle), le temps simulé n'avance pas pendant l'appel : la LATENCE n'est alors PAS
exercée — elle est mesurée à part (stats["latence_max_s"]) et doit être comparée à
max_age_s sur la vraie machine avant de conclure quoi que ce soit.

LIMITE : un pas refusé par le JointSafetyGuard sur un bras n'est pas encore
remonté ici (loco.stats["refused"] le compte, ce service ne le lit pas).
"""
from __future__ import annotations

import math
import time
from enum import Enum, auto
from typing import Any, Callable, Dict, Optional

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability


class EtatVLA(Enum):
    INACTIF = auto()   # pas de mission
    ACTIF = auto()     # action fraîche, on suit
    PERIME = auto()    # plus d'action fraîche : bras tenus, base à l'arrêt
    GEL = auto()       # arrêt de sécurité : bras figés
    DEFAUT = auto()    # erreurs d'inférence répétées ou conflit d'écrivain


class VLAService(IService):
    def __init__(
        self,
        connecteur: Any,
        bras: Dict[str, float],
        loco: Any,
        lire_etat_bras: Callable[[], Dict[str, float]],
        lire_image: Callable[[], Optional[Any]],
        delta_max_par_joint: Dict[str, float],
        horloge: Callable[[], float] = time.monotonic,
        intervalle_s: float = 0.1,
        max_age_s: float = 0.5,
        facteur_securite: float = 0.5,
        defauts_max: int = 3,
        arret_securite: Optional[Callable[[], bool]] = None,
        autre_ecrivain: Optional[Callable[[], bool]] = None,
        journal: Callable[[str], None] = print,
    ):
        """
        connecteur          : GR00TConnector (ou équivalent : arm_joints, reset(),
                              predict_action(image, instruction, state), is_connected()).
        bras                : le dictionnaire de cibles de bras de l'adaptateur
                              (adapt.pose_bras). Écrit SEULEMENT pendant une mission.
        loco                : LocomotionService (set_velocity, halt).
        lire_etat_bras      : () -> {joint_id: position MESURÉE} des 14 bras.
        lire_image          : () -> image (H, W, 3) uint8 de la tête, ou None.
        delta_max_par_joint : {joint_id: rad max par tick} — mêmes valeurs que
                              pour SetpointFollower (position_delta_max).
        """
        self._c, self._bras, self._loco = connecteur, bras, loco
        self._etat_bras, self._image = lire_etat_bras, lire_image
        self._horloge = horloge
        self._intervalle, self._max_age = intervalle_s, max_age_s
        self._facteur, self._defauts_max = facteur_securite, defauts_max
        self._arret, self._autre = arret_securite, autre_ecrivain
        self._log = journal
        self._arm_joints = list(connecteur.arm_joints)
        inconnus = [j for j in self._arm_joints if j not in bras]
        if inconnus:
            raise KeyError(f"articulations absentes du dictionnaire de bras : {inconnus}")
        sans_delta = [j for j in self._arm_joints if j not in delta_max_par_joint]
        if sans_delta:
            raise KeyError(f"delta_max_par_joint manque pour : {sans_delta} — pas de "
                           "limite par défaut devinée")
        self._delta = dict(delta_max_par_joint)

        self.etat = EtatVLA.INACTIF
        self.raison = ""
        self._instruction = ""
        self._action = None
        self._t_action = 0.0
        self._t_infer: Optional[float] = None
        self._defauts = 0
        self.stats = {"inferences": 0, "erreurs": 0, "sans_image": 0, "perimees": 0,
                      "gels": 0, "latence_max_s": 0.0, "ticks": 0}

    # ---- IService ----------------------------------------------------------
    @property
    def service_id(self) -> str:
        return "vla"

    @property
    def capability(self) -> Capability:
        return Capability.AI_POLICY

    def start(self) -> HALStatus:
        self.etat = EtatVLA.INACTIF
        return HALStatus.OK

    def stop(self) -> None:
        self.terminer_mission()

    def health(self) -> ServiceHealth:
        return ServiceHealth(running=self.etat is not EtatVLA.INACTIF,
                             healthy=self.etat is not EtatVLA.DEFAUT,
                             detail=self.raison or self.etat.name)

    # ---- API des apps ------------------------------------------------------
    def demarrer_mission(self, instruction: str) -> ServiceResult:
        if self._autre is not None and self._autre():
            return ServiceResult(ok=False, reason="un autre service écrit les bras "
                                                  "(manipulation en cours) — un seul écrivain")
        if not self._c.is_connected():
            return ServiceResult(ok=False, reason="connecteur GR00T non connecté")
        try:
            self._c.reset()
        except Exception as exc:                          # noqa: BLE001
            return ServiceResult(ok=False, reason=f"reset du serveur GR00T impossible : {exc}")
        self._instruction = instruction
        self._action, self._t_infer, self._defauts, self.raison = None, None, 0, ""
        self.etat = EtatVLA.ACTIF
        return ServiceResult(ok=True, value=instruction)

    def terminer_mission(self) -> None:
        """Fin de mission : base à l'arrêt en équilibre, bras tenus où ils sont."""
        if self.etat is not EtatVLA.INACTIF:
            self._loco.halt()
        self.etat, self._action = EtatVLA.INACTIF, None

    def veut_image(self, t: float) -> bool:
        """Vrai si le tick à l'instant t fera une inférence : en simulation, la
        boucle ne rend la caméra de tête que dans ce cas (10 fois par seconde,
        pas 50 — comme à la collecte)."""
        return self.etat in (EtatVLA.ACTIF, EtatVLA.PERIME, EtatVLA.GEL) and (
            self._t_infer is None or t - self._t_infer >= self._intervalle - 1e-9)

    # ---- boucle de commande (à appeler à chaque tick du robot) -------------
    def tick(self) -> ServiceResult:
        if self.etat in (EtatVLA.INACTIF, EtatVLA.DEFAUT):
            return ServiceResult(ok=False, reason=self.etat.name)
        self.stats["ticks"] += 1
        if self._autre is not None and self._autre():
            self._defaut("un autre service écrit les bras — conflit d'écrivain")
            return ServiceResult(ok=False, reason=self.raison)
        t = self._horloge()
        if self._arret is not None and self._arret():
            self.etat = EtatVLA.GEL
            self.stats["gels"] += 1
            return ServiceResult(ok=True, value=self.etat,
                                 reason="bras figés : arrêt de sécurité en cours")
        if self._t_infer is None or t - self._t_infer >= self._intervalle:
            self._inferer(t)
            if self.etat is EtatVLA.DEFAUT:
                return ServiceResult(ok=False, reason=self.raison)
        if self._action is None or t - self._t_action > self._max_age:
            if self.etat is not EtatVLA.PERIME:
                self.stats["perimees"] += 1
                self._log(f"[VLA] aucune action fraîche (> {self._max_age}s) — bras tenus, "
                          "base à l'arrêt")
            self.etat = EtatVLA.PERIME
            self._loco.halt()
            return ServiceResult(ok=True, value=self.etat, reason="action périmée")
        self.etat = EtatVLA.ACTIF
        self._suivre(self._action)
        v = self._action.base_velocity
        self._loco.set_velocity(v[0], v[1], v[2])
        return ServiceResult(ok=True, value=self.etat)

    # ---- internes ----------------------------------------------------------
    def _defaut(self, raison: str) -> None:
        self.etat, self.raison = EtatVLA.DEFAUT, raison
        self._loco.halt()
        self._log(f"[VLA] DÉFAUT : {raison}")

    def _inferer(self, t: float) -> None:
        self._t_infer = t
        image = self._image()
        if image is None:
            self.stats["sans_image"] += 1
            return
        try:
            action = self._c.predict_action(image, self._instruction, self._etat_bras())
        except Exception as exc:                          # noqa: BLE001
            self.stats["erreurs"] += 1
            self._defauts += 1
            self._log(f"[VLA] inférence en échec ({self._defauts}/{self._defauts_max}) : {exc}")
            if self._defauts >= self._defauts_max:
                self._defaut(f"{self._defauts} erreurs d'inférence d'affilée")
            return
        self._defauts = 0
        etrangers = [j for j in action.target_angles if j not in self._delta]
        if etrangers:
            self._defaut(f"l'action vise des articulations hors des bras : {etrangers}")
            return
        self.stats["inferences"] += 1
        self.stats["latence_max_s"] = max(self.stats["latence_max_s"],
                                          float(getattr(action, "latency_s", 0.0)))
        # Âge compté depuis la CAPTURE (t, avant l'appel), pas depuis la réponse.
        self._action, self._t_action = action, t

    def _suivre(self, action: Any) -> None:
        """Un pas vers la cible, d'au plus delta_max × facteur par articulation."""
        for j, cible in action.target_angles.items():
            if not math.isfinite(cible):
                continue
            budget = self._delta[j] * self._facteur
            actuel = self._bras[j]
            erreur = cible - actuel
            self._bras[j] = cible if abs(erreur) <= budget else actuel + math.copysign(budget, erreur)
