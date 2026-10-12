"""
AXIS-OS — Service VLA : fait tourner une politique apprise (GR00T) sur le
robot.

Un VLA (Vision-Language-Action) est un modèle appris qui reçoit une image
de caméra, une instruction en langage naturel (« prends le carton ») et
l'état du robot, et qui propose des mouvements. GR00T est le modèle de ce
type publié par NVIDIA pour les robots humanoïdes ; on l'interroge par un
connecteur (GR00TConnector), et chaque interrogation du modèle s'appelle
une inférence. Sa réponse, une VLAAction, donne des angles cibles pour les
bras, une vitesse pour la base et, souvent, un « chunk » : une courte
suite de pas futurs, prévus pour être exécutés l'un après l'autre.

Ce service fait le lien, à la cadence de la boucle de commande du robot (un
« tick » est un passage dans cette boucle) :

    caméra de tête + état des bras ──> GR00TConnector.predict_action()
                                         │  (une inférence toutes les
                                         │   `intervalle_s`)
                                         v
              ┌──── bras : cibles écrites dans `bras` (pose_bras de
              │     l'adaptateur), à pas limité par tick, comme
    VLAAction ┤     SetpointFollower
              └──── base : loco.set_velocity(vx, vy, wz)

Aucun chemin rapide. Un raccourci vers les moteurs contournerait les
protections déjà en place (voir docs/groot_architecture.md) : ce service ne
commande donc directement aucun actionneur.
  - Les bras : il écrit des cibles dans le dictionnaire de l'adaptateur de
    locomotion (GR00TWBCAdapter, autour de la politique de marche GR00T
    WBC, pour « Whole-Body Control » : elle commande les jambes et la
    taille, et l'adaptateur y joint les cibles des bras). L'adaptateur
    ramène ces cibles dans les butées (les limites mécaniques des
    articulations), puis les envoie par send_targets -> HAL (couche
    d'abstraction matérielle, commune au vrai robot et au simulateur) ->
    JointSafetyGuard (le garde qui vérifie chaque commande d'articulation),
    exactement comme le ManipulationService.
  - La base : LocomotionService.set_velocity borne la vitesse (max_twist),
    la périme au bout de command_timeout_s (une vitesse non renouvelée cesse
    de s'appliquer), et le gardien de proximité humaine la plafonne ou
    l'annule. Ce service n'a aucun moyen de désactiver ces protections.

Trois garanties, chacune vérifiée par test/test_vla_service.py :

1. Échéance de fraîcheur. Une consigne périmée exécutée indéfiniment est un
   mode de défaillance déjà rencontré sur ce projet (voir
   setpoint_follower.py). Sans action fraîche depuis max_age_s, les bras
   gardent donc leur dernière cible (on ne progresse pas vers une décision
   périmée) et la base reçoit halt() : un arrêt en équilibre, jamais une
   coupure des moteurs, qui ferait tomber un robot debout.
2. Un seul écrivain. Deux services qui écrivent le même dictionnaire de
   cibles, c'est une cible qui saute d'un geste à l'autre sans que personne
   ne l'ait décidé. Le ManipulationService est l'écrivain de `bras` : si
   `autre_ecrivain()` est vrai, ce service refuse de démarrer, ou se met en
   DÉFAUT si la mission est déjà en cours.
3. Arrêt de sécurité. Un humain assez proche pour arrêter la marche l'est
   assez pour qu'un bras en mouvement le heurte. Si `arret_securite()` est
   vrai, les bras sont donc figés (pas relâchés), comme le fait le
   ManipulationService. La base reçoit aussi halt() à chaque tick du gel,
   depuis un audit de sécurité du 2026-10-10 : jusque-là, seul le gardien
   de proximité la tenait à zéro ; désormais le VLA retire aussi sa propre
   consigne, et aucune vitesse décidée avant le gel ne lui survit.

Erreurs d'inférence. Une inférence en échec ne fait pas planter la boucle
de commande : la dernière action reste valable jusqu'à son échéance, et ce
n'est qu'après `defauts_max` erreurs d'affilée que le service passe en
DÉFAUT (base à l'arrêt, bras figés). Une image absente fait sauter
l'inférence : jamais de décision sans image.

Temps. `horloge` est injectable, pour la simulation. En simulation
synchrone (l'inférence bloque la boucle), le temps simulé n'avance pas
pendant l'appel : la latence de l'inférence n'y est donc pas exercée. Elle
est mesurée à part (stats["latence_max_s"]) et doit être comparée à
max_age_s sur la vraie machine avant de conclure quoi que ce soit.

Limite connue : un pas refusé par le JointSafetyGuard sur un bras n'est pas
encore remonté jusqu'ici (loco.stats["refused"] le compte, mais ce service
ne le lit pas).
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
    """États du service VLA (voir l'en-tête du module)."""
    INACTIF = auto()   # pas de mission
    ACTIF = auto()     # action fraîche, on la suit
    PERIME = auto()    # plus d'action fraîche : bras tenus, base à l'arrêt
    GEL = auto()       # arrêt de sécurité en cours : bras figés
    DEFAUT = auto()    # erreurs d'inférence répétées ou conflit d'écrivain


class VLAService(IService):
    """Exécute une politique VLA en mission, sans jamais toucher un
    actionneur directement : les bras passent par l'adaptateur et le
    JointSafetyGuard, la base par le LocomotionService."""

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
        pas_executes: int = 1,
        periode_pas_s: float = 0.1,
    ):
        """
        connecteur          : GR00TConnector, ou tout objet équivalent qui
                              offre arm_joints, reset(),
                              predict_action(image, instruction, state) et
                              is_connected().
        bras                : le dictionnaire des cibles de bras de
                              l'adaptateur (adapt.pose_bras). Il n'est écrit
                              que pendant une mission.
        loco                : LocomotionService (set_velocity, halt).
        lire_etat_bras      : () -> {joint_id: position mesurée} des 14
                              articulations des bras.
        lire_image          : () -> image de la caméra de tête, ou None.
                              Format (H, W, 3) uint8 : hauteur, largeur,
                              trois couleurs, une valeur de 0 à 255 chacune.
        delta_max_par_joint : {joint_id: déplacement maximal par tick, en
                              radians}, mêmes valeurs que pour
                              SetpointFollower (position_delta_max). Une
                              valeur est exigée pour chaque articulation du
                              connecteur : aucune limite n'est devinée par
                              défaut (KeyError sinon).
        horloge             : () -> temps en secondes. Injectable pour la
                              simulation (voir « Temps » dans l'en-tête).
        intervalle_s        : écart minimal entre deux inférences (par
                              défaut 0,1 s, soit 10 inférences par
                              seconde).
        max_age_s           : âge maximal d'une action, compté depuis la
                              capture de l'image ; au-delà, elle est
                              périmée. Doit couvrir les pas exécutés
                              (pas_executes × periode_pas_s), sinon
                              ValueError.
        facteur_securite    : part de delta_max réellement utilisée à
                              chaque tick, par marge (par défaut 0,5, soit
                              la moitié).
        defauts_max         : nombre d'erreurs d'inférence d'affilée qui
                              met le service en DÉFAUT.
        arret_securite      : () -> un arrêt de sécurité est-il en cours ?
                              Optionnel.
        autre_ecrivain      : () -> un autre service écrit-il déjà les
                              bras ? Optionnel.
        journal             : fonction qui reçoit les messages du service.
        pas_executes        : nombre de pas du chunk exécutés à la suite (un
                              pas toutes les periode_pas_s, la cadence de
                              l'entraînement) avant la décision suivante.
                              1 = seulement le premier pas, à chaque
                              inférence. Pourquoi plus d'un pas : lors d'un
                              essai sur une machine à GPU (carte graphique
                              de calcul, où tourne le modèle), en mode « prise
                              seule » (le VLA ne commande que les bras,
                              voir demarrer_mission), avec 1 les bras
                              restaient figés 25 s en pré-saisie, juste
                              avant de saisir. Le premier pas d'un chunk ne
                              dit que « rester là », le geste vient dans
                              les pas suivants, et il n'arrivait jamais.
                              NVIDIA exécute lui aussi le chunk.
        periode_pas_s       : durée d'un pas du chunk, en secondes.
        """
        # Une configuration incohérente est refusée dès la création : il
        # faut au moins un pas par décision, et une action qui périmerait
        # avant la fin des pas prévus serait coupée en plein geste.
        if pas_executes < 1:
            raise ValueError("pas_executes >= 1")
        if max_age_s < pas_executes * periode_pas_s:
            raise ValueError(f"max_age_s ({max_age_s}) doit couvrir les {pas_executes} pas "
                             f"exécutés ({pas_executes * periode_pas_s:.2f} s)")
        self._pas_executes, self._periode_pas = pas_executes, periode_pas_s
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
        self._piloter_base = True
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
    def demarrer_mission(self, instruction: str, piloter_base: bool = True) -> ServiceResult:
        """Démarre une mission décrite par `instruction` (langage naturel).

        Refusée si un autre service écrit déjà les bras, si le connecteur
        GR00T n'est pas connecté ou si son serveur ne peut pas être remis à
        zéro.

        piloter_base=False : le VLA ne commande que les bras ; la base est
        tenue à l'arrêt et les vitesses qu'il propose sont ignorées. C'est
        le mode « prise seule » : une navigation programmée de façon
        classique a déjà placé le robot devant l'objet.
        """
        if self._autre is not None and self._autre():
            return ServiceResult(ok=False, reason="un autre service écrit les bras "
                                                  "(manipulation en cours) — un seul écrivain")
        if not self._c.is_connected():
            return ServiceResult(ok=False, reason="connecteur GR00T non connecté")
        try:
            self._c.reset()
        except Exception as exc:                          # noqa: BLE001
            return ServiceResult(ok=False, reason=f"reset du serveur GR00T impossible : {exc}")
        self._instruction, self._piloter_base = instruction, piloter_base
        self._action, self._t_infer, self._defauts, self.raison = None, None, 0, ""
        self.etat = EtatVLA.ACTIF
        return ServiceResult(ok=True, value=instruction)

    def terminer_mission(self) -> None:
        """Fin de mission : base à l'arrêt en équilibre, bras tenus où ils
        sont."""
        if self.etat is not EtatVLA.INACTIF:
            self._loco.halt()
        self.etat, self._action = EtatVLA.INACTIF, None

    def veut_image(self, t: float) -> bool:
        """Vrai si le tick à l'instant t fera une inférence.

        Sert à la simulation : la boucle n'y calcule l'image de la caméra de
        tête que dans ce cas, soit 10 fois par seconde au lieu de 50, comme
        lors de la collecte des données d'entraînement.
        """
        return self.etat in (EtatVLA.ACTIF, EtatVLA.PERIME, EtatVLA.GEL) and (
            self._t_infer is None or t - self._t_infer >= self._intervalle - 1e-9)

    # ---- boucle de commande (à appeler à chaque tick du robot) -------------
    def tick(self) -> ServiceResult:
        """Un tick de mission. Ordre des vérifications : conflit
        d'écrivain, arrêt de sécurité, inférence si `intervalle_s` est
        écoulé, fraîcheur de l'action ; puis un pas limité vers le pas
        courant du chunk, et la vitesse de la base (ou halt() si
        piloter_base est faux)."""
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
            # À la reprise, on part d'une nouvelle décision, pas de la suite
            # d'un chunk calculé avant le gel.
            self._t_infer = None
            # Base à l'arrêt en équilibre, commandée par le VLA lui-même :
            # c'est une défense en profondeur (le gardien met déjà la
            # vitesse à zéro), et la dernière vitesse proposée ne survit
            # pas au gel.
            self._loco.halt()
            return ServiceResult(ok=True, value=self.etat,
                                 reason="bras figés : arrêt de sécurité en cours")
        # Le « - 1e-9 » absorbe les erreurs d'arrondi des flottants : sans
        # lui, une inférence prévue pile à l'échéance pourrait glisser d'un
        # tick.
        if self._t_infer is None or t - self._t_infer >= self._intervalle - 1e-9:
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
        angles, v = self._pas_courant(t)
        self._suivre_angles(angles)
        if self._piloter_base:
            self._loco.set_velocity(v[0], v[1], v[2])
        else:
            self._loco.halt()
        return ServiceResult(ok=True, value=self.etat)

    # ---- internes ----------------------------------------------------------
    def _defaut(self, raison: str) -> None:
        """Passe en DÉFAUT : base à l'arrêt, bras figés sur leur cible."""
        self.etat, self.raison = EtatVLA.DEFAUT, raison
        self._loco.halt()
        self._log(f"[VLA] DÉFAUT : {raison}")

    def _inferer(self, t: float) -> None:
        """Demande une action au modèle pour l'image capturée à
        l'instant t."""
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
        # Une action qui vise une articulation sans delta_max, donc hors des
        # bras que ce service a le droit d'écrire, est refusée en bloc, et
        # le service passe en DÉFAUT.
        etrangers = [j for j in action.target_angles if j not in self._delta]
        if etrangers:
            self._defaut(f"l'action vise des articulations hors des bras : {etrangers}")
            return
        self.stats["inferences"] += 1
        self.stats["latence_max_s"] = max(self.stats["latence_max_s"],
                                          float(getattr(action, "latency_s", 0.0)))
        # Âge compté depuis la capture de l'image (t, avant l'appel), pas
        # depuis la réponse : sinon la durée de l'inférence serait invisible
        # et une décision prise sur une image ancienne paraîtrait fraîche.
        self._action, self._t_action = action, t

    def _pas_courant(self, t: float):
        """Renvoie (angles, vitesse de base) du pas du chunk qui correspond à
        l'instant t.

        Le pas k est le nombre de periode_pas_s écoulées depuis la capture,
        plafonné à pas_executes - 1 et à la longueur du chunk. Sans chunk,
        ou pour k = 0, on prend les target_angles et la base_velocity de
        l'action. Le chunk donne les pas de chaque bras (« bras_gauche » et
        « bras_droit », mis bout à bout dans cet ordre puis associés à
        arm_joints) et, s'il y en a, ceux de la base (« base »).
        """
        a = self._action
        k = 0
        if self._pas_executes > 1:
            k = min(int((t - self._t_action) / self._periode_pas + 1e-6), self._pas_executes - 1)
        ch = getattr(a, "chunk", None) or {}
        if k == 0 or not ch:
            return a.target_angles, a.base_velocity
        g, d = ch.get("bras_gauche"), ch.get("bras_droit")
        k = min(k, len(g) - 1, len(d) - 1)
        valeurs = list(g[k]) + list(d[k])
        base = ch.get("base")
        v = tuple(float(x) for x in base[min(k, len(base) - 1)]) if base is not None \
            else a.base_velocity
        return {j: float(x) for j, x in zip(self._arm_joints, valeurs)}, v

    def _suivre(self, action: Any) -> None:
        """Un pas vers les target_angles de l'action, c'est-à-dire vers son
        premier pas, sans tenir compte du chunk. tick() passe, lui, par
        _pas_courant()."""
        self._suivre_angles(action.target_angles)

    def _suivre_angles(self, angles: Dict[str, float]) -> None:
        """Un pas vers la cible, d'au plus delta_max × facteur par
        articulation.

        Une cible non finie (NaN, « pas un nombre », ou infinie) est
        ignorée et l'articulation garde sa cible actuelle : c'est la même
        protection que dans SetpointFollower, où une cible NaN faisait
        dériver l'articulation jusqu'à sa butée (voir setpoint_follower.py).
        """
        for j, cible in angles.items():
            if not math.isfinite(cible):
                continue
            budget = self._delta[j] * self._facteur
            actuel = self._bras[j]
            erreur = cible - actuel
            self._bras[j] = cible if abs(erreur) <= budget else actuel + math.copysign(budget, erreur)
