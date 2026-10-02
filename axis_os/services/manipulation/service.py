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
    # Le bras CHARGÉ rejoint la consigne avec retard (voir la montée) : la
    # prise est confirmée dès que l'objet a suivi, en laissant jusqu'à ce
    # délai après la fin de la consigne de levage. Constaté sur RTX 5090 :
    # carton bien serré, déclaré « non confirmé » à l'instant exact où la
    # consigne finissait, prise refaite en boucle.
    attente_confirmation_s: float = 1.0
    duree_pose_s: float = 0.8
    duree_ouverture_s: float = 0.5
    # Pré-saisie : bras OUVERTS PLUS LARGE avant de glisser les mains autour
    # de l'objet, comme le ferait un humain. Sans elle, le robot avançait
    # mains à 33 cm l'une de l'autre vers un carton de 24 cm, et le heurtait
    # (constaté sur GPU, T1).
    roulis_presaisie: float = 0.32
    duree_presaisie_s: float = 0.6
    # Franchissement : avant d'avancer au-dessus d'un plateau, le fond de
    # l'objet porté doit le dépasser de `marge_franchissement`. Constaté sur
    # GPU : le carton, qui glisse dans les mains pendant le trajet (jusqu'à
    # 9 cm mesurés dans W3), butait contre le bord de la table. Marge large
    # pour cette raison : la hauteur du fond est ESTIMÉE par les mains.
    marge_franchissement: float = 0.10
    tangage_haut: float = -0.85        # jamais plus haut
    vitesse_montee: float = 0.30       # rad/s
    # Le bras CHARGÉ suit la consigne avec retard (gains doux, poids du
    # carton). Constaté sur GPU : l'échec « plateau trop haut » tombait à
    # l'instant où la CONSIGNE touchait la limite, les mains encore en
    # train de monter. On laisse le bras rejoindre avant de conclure.
    stabilisation_montee_s: float = 1.2
    # Après stabilisation, une marge réduite suffit : la détection de
    # blocage de la façade reste le filet si le carton touche malgré tout.
    marge_franchissement_min: float = 0.05
    # Dépose sur un plateau : descente en BOUCLE FERMÉE sur la hauteur
    # mesurée des mains (proprioception), jusqu'à ce que le fond de l'objet
    # soit à `jeu_depose` du plateau. Jamais au-delà de `tangage_bas`.
    tangage_bas: float = 0.10
    vitesse_descente: float = 0.25     # rad/s
    jeu_depose: float = 0.01           # m : on ouvre juste au-dessus


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
    PRESAISIE = auto()    # ouverture large, avant l'approche finale
    PRET = auto()         # bras grands ouverts, prêts à entourer l'objet
    SERRAGE = auto()
    LEVAGE = auto()
    TENU = auto()
    POSE = auto()
    DESCENTE = auto()     # vers un plateau, en boucle fermée
    MONTEE = auto()       # au-dessus d'un plateau avant d'avancer
    OUVERTURE = auto()
    LACHER = auto()       # après un échec : ouverture large PROGRESSIVE
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
        self._surface: Optional[float] = None
        self._franchi: Optional[float] = None     # plateau dégagé par la dernière montée
        self._t_limite: Optional[float] = None
        self._roulis_depart_serrage = postures.roulis_ouvert
        self._tangage_courant = postures.tangage_port
        self._roulis_courant = postures.roulis_ouvert
        self._lacher_de = self._lacher_vers = (postures.roulis_ouvert, postures.tangage_port)
        self._tangage_depart = postures.tangage_leve
        self._tangage_ouverture = postures.tangage_port
        self._dernier_tick: Optional[float] = None
        self.stats = {"prises": 0, "prises_ratees": 0, "refus": 0, "poses": 0,
                      "gels_securite": 0, "deposes_plateau": 0,
                      "deposes_impossibles": 0}

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
        self._tangage_courant, self._roulis_courant = tangage, roulis
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

    def presaisir(self) -> ServiceResult:
        """Ouvre les bras en grand, À L'ARRÊT, avant l'approche finale.
        Non bloquant : suivre l'état jusqu'à PRET."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if self.etat is Etat.PRET:
            return ServiceResult(ok=True)
        if self.etat is not Etat.LIBRE:
            return ServiceResult(ok=False, reason=f"occupé : {self.etat.name}")
        self._aller(Etat.PRESAISIE)
        return ServiceResult(ok=True)

    def prendre(self, objet: Objet) -> ServiceResult:
        """Non bloquant. Suivre la progression par etat, ou tick()."""
        if not self._demarre:
            return ServiceResult(ok=False, reason="service non démarré")
        if self.etat not in (Etat.LIBRE, Etat.PRET):
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
        self.objet, self.raison, self._franchi = objet, "", None
        self._roulis_depart_serrage = (self.p.roulis_presaisie if self.etat is Etat.PRET
                                       else self.p.roulis_ouvert)
        self._aller(Etat.SERRAGE)
        return ServiceResult(ok=True)

    def _fond_objet(self) -> float:
        g, d = self._mains()
        h = self.objet.hauteur if self.objet is not None else 0.0
        return (float(g[2]) + float(d[2])) / 2.0 - h / 2.0

    def degage_au_dessus(self, hauteur_surface: float) -> bool:
        """L'objet porté passe-t-il au-dessus d'un plateau à cette hauteur ?"""
        return self._fond_objet() >= hauteur_surface + self.p.marge_franchissement

    def peut_franchir(self, hauteur_surface: float) -> bool:
        """Dégagé avec la marge pleine, ou montée terminée avec la marge
        réduite pour CE plateau."""
        return (self.degage_au_dessus(hauteur_surface)
                or (self.etat is Etat.TENU and self._franchi is not None
                    and abs(self._franchi - hauteur_surface) < 1e-6))

    def franchir(self, hauteur_surface: float) -> ServiceResult:
        """Soulève l'objet tenu au-dessus d'un plateau AVANT d'avancer vers
        lui — ce que fait quelqu'un qui pose un carton sur une table. Non
        bloquant : état MONTEE puis TENU. Déjà assez haut : rien à faire."""
        if self.etat is not Etat.TENU:
            return ServiceResult(ok=False, reason="rien en main")
        if self.degage_au_dessus(hauteur_surface):
            return ServiceResult(ok=True, value="déjà au-dessus")
        self._surface, self._tangage_depart = hauteur_surface, self._tangage_courant
        self.raison, self._franchi, self._t_limite = "", None, None
        self._aller(Etat.MONTEE)
        return ServiceResult(ok=True)

    def poser(self, hauteur_surface: Optional[float] = None) -> ServiceResult:
        """Sans plateau : redescend à hauteur de port et ouvre. Avec
        `hauteur_surface` (m, monde) : descend jusqu'à ce que le fond de
        l'objet effleure le plateau, PUIS ouvre — sans lâcher de haut."""
        if self.etat is not Etat.TENU:
            return ServiceResult(ok=False, reason="rien en main")
        self.raison, self._franchi = "", None
        if hauteur_surface is None:
            self._surface, self._tangage_ouverture = None, self.p.tangage_port
            self._aller(Etat.POSE)
            return ServiceResult(ok=True)
        fond = self._fond_objet()
        if fond < hauteur_surface + self.p.jeu_depose:
            return ServiceResult(ok=False, reason=(
                f"plateau trop haut ({hauteur_surface:.2f} m) : le fond de "
                f"l'objet est déjà à {fond:.2f} m"))
        self._surface = hauteur_surface
        self._tangage_depart = self._tangage_courant    # d'où qu'on parte
        self._aller(Etat.DESCENTE)
        return ServiceResult(ok=True)

    def reinitialiser(self, large: bool = False) -> None:
        """Après un échec : bras ouverts en posture de port, exemption levée.

        large=True : ouverture GRANDE (celle de la pré-saisie), pour LÂCHER
        un objet coincé entre les mains. Constaté sur GPU (T1, 30 min) :
        après une prise ratée sur les coins d'un carton, les mains rouvertes
        à 33 cm le tenaient encore ; le robot a reculé en le tirant de 11 cm,
        jusqu'à le faire dépasser du bord de la table — et s'est bloqué
        devant lui.

        Le geste est PROGRESSIF (état LACHER, durée d'ouverture), jamais un
        saut : constaté sur GPU, le saut de « serré » (-0.40) à « grand
        ouvert » (0.32) dépassait l'écart maximal consigne/position du
        JointSafetyGuard (0.625 rad) — 570 consignes refusées, bras restés
        SERRÉS, carton tiré hors de la table et tombé."""
        self._lever_exemption()
        self.objet, self._franchi = None, None
        self._lacher_de = (self._roulis_courant, self._tangage_courant)
        self._lacher_vers = (self.p.roulis_presaisie if large else self.p.roulis_ouvert,
                             self.p.tangage_port)
        self._aller(Etat.LACHER)

    # ---- boucle ----------------------------------------------------------

    _EN_MOUVEMENT = (Etat.PRESAISIE, Etat.MONTEE, Etat.SERRAGE, Etat.LEVAGE, Etat.POSE, Etat.OUVERTURE,
                     Etat.DESCENTE, Etat.LACHER)

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
        if self.etat is Etat.LACHER:
            r, _ = self._interp(self._lacher_de[0], self._lacher_vers[0], p.duree_ouverture_s)
            t, fini = self._interp(self._lacher_de[1], self._lacher_vers[1],
                                   p.duree_ouverture_s)
            self._ecrire(r, t)
            if fini:
                self.etat = Etat.LIBRE
        elif self.etat is Etat.PRESAISIE:
            r, fini = self._interp(p.roulis_ouvert, p.roulis_presaisie, p.duree_presaisie_s)
            self._ecrire(r, p.tangage_port)
            if fini:
                self.etat = Etat.PRET
        elif self.etat is Etat.SERRAGE:
            r, fini = self._interp(self._roulis_depart_serrage, p.roulis_serre,
                                   p.duree_serrage_s)
            self._ecrire(r, p.tangage_port)
            if fini:
                self._aller(Etat.LEVAGE)
        elif self.etat is Etat.LEVAGE:
            t, fini = self._interp(p.tangage_port, p.tangage_leve, p.duree_levage_s)
            self._ecrire(p.roulis_serre, t)
            if fini:
                confirme = self._verifier()
                if not confirme and (self._horloge() - self._t0 - p.duree_levage_s
                                     < p.attente_confirmation_s):
                    pass                     # on laisse le bras rejoindre
                elif confirme:
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
                    # Pas de saut vers la posture de port (0.6 rad d'un coup,
                    # à la limite du JointSafetyGuard) : reinitialiser() rouvre
                    # PROGRESSIVEMENT.
                    self.etat = Etat.ECHEC
        elif self.etat is Etat.POSE:
            t, fini = self._interp(p.tangage_leve, p.tangage_port, p.duree_pose_s)
            self._ecrire(p.roulis_serre, t)
            if fini:
                self._aller(Etat.OUVERTURE)
        elif self.etat is Etat.MONTEE:
            maintenant = self._horloge()
            t = max(p.tangage_haut,
                    self._tangage_depart - p.vitesse_montee * (maintenant - self._t0))
            self._ecrire(p.roulis_serre, t)
            if self.degage_au_dessus(self._surface):
                self.stats["franchissements"] = self.stats.get("franchissements", 0) + 1
                self._franchi = self._surface
                self.etat = Etat.TENU
            elif t <= p.tangage_haut:
                # Consigne à la limite : on attend que le bras y arrive.
                if self._t_limite is None:
                    self._t_limite = maintenant
                elif maintenant - self._t_limite >= p.stabilisation_montee_s:
                    fond = self._fond_objet()
                    if fond >= self._surface + p.marge_franchissement_min:
                        self.stats["franchissements"] = \
                            self.stats.get("franchissements", 0) + 1
                        self.stats["franchissements_justes"] = \
                            self.stats.get("franchissements_justes", 0) + 1
                        self._franchi = self._surface
                    else:
                        self.stats["franchissements_impossibles"] = \
                            self.stats.get("franchissements_impossibles", 0) + 1
                        self.raison = (
                            f"plateau à {self._surface:.2f} m trop haut pour y passer "
                            f"l'objet : bras au plus haut, fond du carton à "
                            f"{fond:.2f} m (il faut {self._surface + p.marge_franchissement_min:.2f})")
                    self.etat = Etat.TENU
        elif self.etat is Etat.DESCENTE:
            t = self._tangage_depart + p.vitesse_descente * (self._horloge() - self._t0)
            if t >= p.tangage_bas:
                # Le plateau est hors d'atteinte : on NE lâche PAS de haut.
                # On remonte en tenant, et on le dit.
                self.stats["deposes_impossibles"] += 1
                self.raison = (f"dépose impossible : plateau à "
                               f"{self._surface:.2f} m hors d'atteinte des bras")
                self._ecrire(p.roulis_serre, p.tangage_leve)
                self.etat = Etat.TENU
            else:
                self._ecrire(p.roulis_serre, t)
                if self._fond_objet() <= self._surface + p.jeu_depose:
                    self._tangage_ouverture = t
                    self.stats["deposes_plateau"] += 1
                    self._aller(Etat.OUVERTURE)
        elif self.etat is Etat.OUVERTURE:
            r, fini = self._interp(p.roulis_serre, p.roulis_ouvert, p.duree_ouverture_s)
            self._ecrire(r, self._tangage_ouverture)
            if fini:
                self.stats["poses"] += 1
                self.objet = None
                # Mains ouvertes, l'objet est posé : retour en posture de port.
                self._ecrire(p.roulis_ouvert, p.tangage_port)
                self._tangage_ouverture = p.tangage_port
                self.etat = Etat.LIBRE
        return ServiceResult(ok=self.etat is not Etat.ECHEC, value=self.etat,
                             reason=self.raison or None)
