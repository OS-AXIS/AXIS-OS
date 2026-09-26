"""
AXIS-OS — Service de manipulation : prendre et poser un objet.

Sixième service système. Une app dit « prends ce carton », puis « pose-le »
— comme elle dit « va au poste B » au service de navigation. Elle ne
connaît ni les angles des bras, ni les pièges de la prise.

CE SERVICE EST LE SEUL À COMMANDER LES BRAS. C'est le premier pas de
l'arbitrage des actionneurs : la locomotion possède les jambes et la
taille, la manipulation possède les bras. Une app n'écrit jamais d'angle
d'épaule.

LES COMPÉTENCES reprennent ce que les essais en simulation ont validé
(docs/validation_port_carton.md, W3 et W4) :

  prendre : ouvrir les bras autour de l'objet — SERRER, la consigne
            visant au-delà du contact pour que la raideur des articulations
            produise la force — SOULEVER — VÉRIFIER que l'objet a suivi.
  poser   : redescendre l'objet sur la surface — OUVRIR — le relâcher.

TROIS RÈGLES, chacune tirée d'un échec constaté :

  1. On ne serre que si l'objet est ENTRE LES MAINS (règle de Kenzo, W4) :
     mains connues par les articulations, objet connu par la perception.
     Sinon, refus motivé — c'est à l'app de placer le robot.

  2. On VÉRIFIE la prise avant de la déclarer tenue. Un robot qui part les
     mains vides en croyant porter un carton est pire qu'un robot qui
     s'arrête. En simulation, la vérification lit la hauteur de l'objet ;
     sur le vrai robot, ce sera l'effort mesuré aux épaules.

  3. L'objet à prendre, posé sur une table, arrêterait le robot : le
     service demande au gardien de proximité une EXEMPTION bornée autour
     de lui, et la lève dès la fin de l'opération — réussie ou non.

Le temps est celui du ROBOT (horloge injectée), comme partout ailleurs.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Dict, Optional, Tuple

import numpy as np

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability


@dataclass(frozen=True)
class Postures:
    """Angles BRUTS, calculés par cinématique directe et validés en
    simulation (W3, W4) — pas traduits par les étiquettes anatomiques, dont
    deux se sont révélées inversées."""
    tangage_port: float = -0.30        # mains devant le bassin, à ~0.90 m
    tangage_leve: float = -0.50        # soulevé : les mains montent de ~6 cm
    coude: float = 0.0                 # angle droit, zéro du G1
    roulis_ouvert: float = 0.20        # 33 cm entre les mains
    roulis_serre: float = -0.40        # vise AU-DELÀ du contact
    duree_serrage_s: float = 0.5
    duree_levage_s: float = 0.8
    duree_pose_s: float = 0.8
    duree_ouverture_s: float = 0.5


def posture_bras(roulis: float, tangage: float, coude: float) -> Dict[str, float]:
    return {"left_shoulder_pitch_joint": tangage,
            "right_shoulder_pitch_joint": tangage,
            "left_shoulder_roll_joint": roulis,
            "right_shoulder_roll_joint": -roulis,
            "left_elbow_joint": coude, "right_elbow_joint": coude}


@dataclass(frozen=True)
class Objet:
    """Ce que la perception sait de l'objet : centre et dimensions, en
    repère MONDE (largeur = entre les mains, profondeur = vers l'avant)."""
    centre: Tuple[float, float, float]
    largeur: float
    profondeur: float
    hauteur: float


class Etat(Enum):
    LIBRE = auto()        # bras en posture de port, rien en main
    SERRAGE = auto()
    LEVAGE = auto()
    TENU = auto()
    POSE = auto()
    OUVERTURE = auto()
    ECHEC = auto()


class ManipulationService(IService):

    def __init__(self, bras: Dict[str, float],
                 mains: Callable[[], Tuple[np.ndarray, np.ndarray]],
                 vers_robot: Callable[[np.ndarray], np.ndarray],
                 verifier_prise: Callable[[], bool],
                 gardien=None, horloge: Callable[[], float] = time.monotonic,
                 postures: Postures = Postures(),
                 arret_securite: Optional[Callable[[], bool]] = None):
        """
        bras            : le dictionnaire des cibles de bras de l'adaptateur
                          de locomotion. CE SERVICE EST SON SEUL ÉCRIVAIN.
        mains           : () -> positions MONDE des mains gauche et droite
                          (proprioception : cinématique des articulations).
        vers_robot      : point monde -> repère du robot (x devant, y gauche).
        verifier_prise  : () -> l'objet a-t-il suivi les mains ?
        gardien         : GardienProximite, pour l'exemption autour de
                          l'objet. Optionnel.
        arret_securite  : () -> un arrêt de sécurité est-il en cours ? Par
                          défaut, lu sur le gardien. Pendant l'arrêt, les
                          bras sont FIGÉS en place : un humain assez proche
                          pour arrêter la marche l'est assez pour qu'un bras
                          en mouvement le heurte. Le geste reprend là où il
                          s'était figé. Les bras gardent leur prise : figer
                          n'est pas lâcher.
        """
        self._bras = bras
        self._mains = mains
        self._vers_robot = vers_robot
        self._verifier = verifier_prise
        self._gardien = gardien
        self._horloge = horloge
        self.p = postures
        self.etat = Etat.LIBRE
        self.objet: Optional[Objet] = None
        self.raison = ""
        self._t0 = 0.0
        self._demarre = False
        if arret_securite is None and gardien is not None \
                and hasattr(type(gardien), "arrete"):
            arret_securite = lambda: gardien.arrete
        self._arret_securite = arret_securite
        self._dernier_tick: Optional[float] = None
        self.stats = {"prises": 0, "prises_ratees": 0, "refus": 0, "poses": 0,
                      "gels_securite": 0}

    # ---- IService --------------------------------------------------------

    @property
    def service_id(self) -> str:
        return "manipulation"

    @property
    def capability(self) -> Capability:
        return Capability.GRASP_DUAL

    def start(self) -> HALStatus:
        self._demarre = True
        self._ecrire(self.p.roulis_ouvert, self.p.tangage_port)
        return HALStatus.OK

    def stop(self) -> None:
        self._lever_exemption()
        self._demarre = False

    def health(self) -> ServiceHealth:
        return ServiceHealth(running=self._demarre,
                             healthy=self.etat is not Etat.ECHEC,
                             detail=self.raison or self.etat.name)

    # ---- outils ----------------------------------------------------------

    def _ecrire(self, roulis: float, tangage: float) -> None:
        self._bras.update(posture_bras(roulis, tangage, self.p.coude))

    def _interp(self, a: float, b: float, duree: float) -> Tuple[float, bool]:
        f = min(1.0, (self._horloge() - self._t0) / duree)
        return a + f * (b - a), f >= 1.0

    def _aller(self, etat: Etat) -> None:
        self.etat, self._t0 = etat, self._horloge()

    def _lever_exemption(self) -> None:
        if self._gardien is not None:
            self._gardien.lever_exemption()

    def entre_les_mains(self, objet: Objet) -> Tuple[bool, str]:
        """Règle de Kenzo : l'objet est-il entre les mains ?

        Sur le côté : son centre entre les deux mains, et la place de s'y
        loger avec 1 cm de marge. En profondeur : les mains entre sa face
        avant et sa face arrière, à 2 cm près.
        """
        g, d = self._mains()
        gr, dr = self._vers_robot(g), self._vers_robot(d)
        cr = self._vers_robot(np.asarray(objet.centre, dtype=float))
        milieu = (gr[1] + dr[1]) / 2.0
        jeu = abs(gr[1] - dr[1]) / 2.0 - objet.largeur / 2.0
        mains_x = (gr[0] + dr[0]) / 2.0
        if jeu <= 0.01:
            return False, (f"mains trop proches ({abs(gr[1] - dr[1]):.2f} m) "
                           f"pour un objet de {objet.largeur:.2f} m")
        if abs(cr[1] - milieu) >= jeu - 0.01:
            return False, (f"objet décalé de {cr[1] - milieu:+.2f} m sur le "
                           f"côté (marge {jeu - 0.01:.2f} m)")
        avant = cr[0] - objet.profondeur / 2.0 + 0.02
        arriere = cr[0] + objet.profondeur / 2.0 - 0.02
        if not (avant < mains_x < arriere):
            return False, (f"mains à {mains_x:.2f} m, objet entre "
                           f"{avant:.2f} et {arriere:.2f} m devant le robot")
        return True, ""

    # ---- API des apps ----------------------------------------------------

    def prendre(self, objet: Objet) -> ServiceResult:
        """Non bloquant. Suivre la progression par etat, ou tick()."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if self.etat is not Etat.LIBRE:
            return ServiceResult(ok=False, reason=f"occupé : {self.etat.name}")
        ok, raison = self.entre_les_mains(objet)
        if not ok:
            self.stats["refus"] += 1
            return ServiceResult(ok=False, reason=f"prise refusée — {raison}")
        if self._gardien is not None:
            cx, cy, cz = objet.centre
            m = 0.03
            dx, dy, dz = (objet.profondeur / 2 + m, objet.largeur / 2 + m,
                          objet.hauteur / 2 + m)
            ok_e, r_e = self._gardien.exempter(
                (cx - dx, cx + dx, cy - dy, cy + dy, cz - dz, cz + dz + 0.10),
                duree_s=15.0, maintenant=self._horloge(),
                motif="manipulation : prise")
            if not ok_e:
                self.stats["refus"] += 1
                return ServiceResult(ok=False, reason=f"exemption refusée — {r_e}")
        self.objet, self.raison = objet, ""
        self._aller(Etat.SERRAGE)
        return ServiceResult(ok=True)

    def poser(self) -> ServiceResult:
        if self.etat is not Etat.TENU:
            return ServiceResult(ok=False, reason="rien en main")
        self._aller(Etat.POSE)
        return ServiceResult(ok=True)

    def reinitialiser(self) -> None:
        """Après un échec : bras ouverts en posture de port, exemption levée."""
        self._lever_exemption()
        self._ecrire(self.p.roulis_ouvert, self.p.tangage_port)
        self.objet = None
        self.etat = Etat.LIBRE

    # ---- boucle ----------------------------------------------------------

    _EN_MOUVEMENT = (Etat.SERRAGE, Etat.LEVAGE, Etat.POSE, Etat.OUVERTURE)

    def tick(self) -> ServiceResult:
        p = self.p
        maintenant = self._horloge()
        dt = 0.0 if self._dernier_tick is None else maintenant - self._dernier_tick
        self._dernier_tick = maintenant
        if self.etat in self._EN_MOUVEMENT and self._arret_securite is not None \
                and self._arret_securite():
            # Gel : on n'écrit rien (les bras restent où ils sont) et on
            # décale l'origine du geste pour qu'il reprenne sans à-coup.
            self._t0 += dt
            self.stats["gels_securite"] += 1
            return ServiceResult(ok=True, value=self.etat,
                                 reason="bras figés : arrêt de sécurité en cours")
        if self.etat is Etat.SERRAGE:
            r, fini = self._interp(p.roulis_ouvert, p.roulis_serre, p.duree_serrage_s)
            self._ecrire(r, p.tangage_port)
            if fini:
                self._aller(Etat.LEVAGE)
        elif self.etat is Etat.LEVAGE:
            t, fini = self._interp(p.tangage_port, p.tangage_leve, p.duree_levage_s)
            self._ecrire(p.roulis_serre, t)
            if fini:
                if self._verifier():
                    self.stats["prises"] += 1
                    # L'objet a quitté la table : l'exemption n'a plus lieu
                    # d'être. Le carton porté est couvert par le volume de
                    # port du gardien.
                    self._lever_exemption()
                    self.etat = Etat.TENU
                else:
                    self.stats["prises_ratees"] += 1
                    self.raison = "prise NON confirmée — l'objet n'a pas suivi"
                    self._lever_exemption()
                    self._ecrire(p.roulis_ouvert, p.tangage_port)
                    self.etat = Etat.ECHEC
        elif self.etat is Etat.POSE:
            t, fini = self._interp(p.tangage_leve, p.tangage_port, p.duree_pose_s)
            self._ecrire(p.roulis_serre, t)
            if fini:
                self._aller(Etat.OUVERTURE)
        elif self.etat is Etat.OUVERTURE:
            r, fini = self._interp(p.roulis_serre, p.roulis_ouvert, p.duree_ouverture_s)
            self._ecrire(r, p.tangage_port)
            if fini:
                self.stats["poses"] += 1
                self.objet = None
                self.etat = Etat.LIBRE
        return ServiceResult(ok=self.etat is not Etat.ECHEC, value=self.etat,
                             reason=self.raison or None)
