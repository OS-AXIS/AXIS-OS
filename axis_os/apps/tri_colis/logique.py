"""
Tri de colis — la logique métier, séparée du robot.

Cette machine à états décide QUOI faire : quel colis prendre, où le porter,
que faire quand ça rate. Elle ne commande jamais un moteur : elle parle à une
façade `Robot` (navigation, manipulation, perception, sécurité), branchée en
simulation, sur le vrai robot, ou sur un monde factice pour les tests.

Principes :
  - la règle de tri (couleur -> poste) est de la connaissance métier, dans la
    config, jamais dans l'IA ;
  - un arrêt de sécurité (humain trop près) n'est PAS une panne : la logique
    se fige, son temps est compté à part, aucun délai ne court ;
  - tout échec a une issue : nouvel essai, abandon signalé, couleur suspendue,
    arrêt propre — jamais d'état sans sortie ;
  - la mission ne se termine jamais les bras chargés.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Protocol, Tuple


@dataclass(frozen=True)
class Colis:
    couleur: str
    position: Tuple[float, float, float]
    confiance: float


class Robot(Protocol):
    def maintenant(self) -> float: ...
    def bloque_par_securite(self) -> bool: ...
    def chercher(self, couleur: str) -> List[Colis]: ...
    def aller_au_poste(self, poste: str) -> Tuple[bool, str]: ...
    def accoster(self, colis: Colis) -> Tuple[bool, str]: ...
    def navigation(self) -> str: ...            # EN_ROUTE / ARRIVE / ECHEC
    def prendre(self, colis: Colis) -> Tuple[bool, str]: ...
    def poser(self) -> Tuple[bool, str]: ...
    def manipulation(self) -> str: ...          # LIBRE / TENU / ECHEC / en cours
    def reinitialiser_manipulation(self) -> None: ...
    def journal(self, message: str) -> None: ...


@dataclass
class ConfigTri:
    regles: Dict[str, str]                  # couleur -> poste de dépose
    poste_prise: str = "prise"
    essais_prise_max: int = 3
    essais_nav_max: int = 3
    essais_pose_max: int = 2
    delai_nav_s: float = 90.0
    delai_manip_s: float = 20.0
    attente_recherche_s: float = 5.0
    duree_mission_s: float = 1800.0
    rayon_ignore_m: float = 0.15
    # Un emplacement abandonné est ignoré un temps, pas pour toujours :
    # l'opérateur peut l'avoir dégagé et un colis neuf y être posé.
    oubli_ignore_s: float = 300.0
    pause_prise_inaccessible_s: float = 30.0
    alerte_inactivite_s: float = 120.0     # rien à trier depuis ce temps
    alerte_arret_prolonge_s: float = 60.0  # bloqué par la sécurité depuis ce temps


@dataclass
class Bilan:
    colis_tries: int = 0
    par_couleur: Dict[str, int] = field(default_factory=dict)
    livraisons: List[Tuple[str, str]] = field(default_factory=list)
    prises_ratees: int = 0
    reussies_apres_nouvel_essai: int = 0
    colis_abandonnes: int = 0
    couleurs_suspendues: List[str] = field(default_factory=list)
    temps_arret_securite_s: float = 0.0
    echecs_navigation: int = 0
    alertes: List[str] = field(default_factory=list)

    def taux_reussite(self) -> float:
        tentes = self.colis_tries + self.colis_abandonnes
        return self.colis_tries / tentes if tentes else 1.0


# États où la mission peut se terminer : bras vides, robot au poste de prise.
_ETATS_TERMINABLES = ("CHERCHER", "ATTENDRE", "PAUSE")


class TriColis:
    def __init__(self, robot: Robot, cfg: ConfigTri):
        self.robot, self.cfg = robot, cfg
        self.bilan = Bilan()
        self.etat = "REVENIR"           # d'abord se placer au poste de prise
        self.termine = False
        self.colis: Optional[Colis] = None
        self.destination: Optional[str] = None
        self.ignores: List[Tuple[Tuple[float, float, float], float]] = []
        self._debut: Optional[float] = None
        self._prec: Optional[float] = None
        self._temps_etat = 0.0          # temps ACTIF passé dans l'état
        self._entree = True             # premier pas dans l'état courant
        self._essais_prise = 0
        self._derniere_prise_ratee = None   # (position, couleur, essais) de la dernière prise ratée
        self._essais_nav = 0
        self._essais_pose = 0
        self._echecs_accostage: List[Tuple[float, float, float]] = []
        self._inactif_depuis: Optional[float] = None
        self._bloque_s = 0.0
        self._blocage_signale = False
        self._inactivite_signalee = False

    # ------------------------------------------------------------------
    def duree_ecoulee(self) -> float:
        if self._debut is None:
            return 0.0
        return self.robot.maintenant() - self._debut

    def pas(self) -> None:
        if self.termine:
            return
        t = self.robot.maintenant()
        if self._debut is None:
            self._debut = self._prec = t
        dt, self._prec = max(0.0, t - self._prec), t
        if self.robot.bloque_par_securite():
            # L'OS a arrêté le robot : on attend, sans rien commander, et
            # sans que ce temps ne compte dans les délais de l'état. Si ça
            # dure, quelqu'un est planté devant le robot : on le signale.
            self.bilan.temps_arret_securite_s += dt
            self._bloque_s += dt
            if self._bloque_s >= self.cfg.alerte_arret_prolonge_s \
                    and not self._blocage_signale:
                self._blocage_signale = True
                self._alerte(f"arrêt de sécurité depuis {self._bloque_s:.0f} s "
                             f"({self.etat}) — une personne ou un objet bloque "
                             "le robot ; merci de dégager le passage")
            return
        self._bloque_s, self._blocage_signale = 0.0, False
        self._temps_etat += dt
        getattr(self, "_etat_" + self.etat.lower())()

    def _aller(self, etat: str) -> None:
        self.etat, self._temps_etat, self._entree = etat, 0.0, True

    def _log(self, msg: str) -> None:
        self.robot.journal(f"[{self.duree_ecoulee():7.1f}s] {msg}")

    def _alerte(self, msg: str) -> None:
        self.bilan.alertes.append(msg)
        self._log("ALERTE " + msg)

    def _mission_finie(self) -> bool:
        return self.duree_ecoulee() >= self.cfg.duree_mission_s

    def _finir(self) -> None:
        self.termine = True
        self._aller("FIN")
        b = self.bilan
        self._log(f"FIN DE MISSION — {b.colis_tries} triés, "
                  f"{b.colis_abandonnes} abandonnés, "
                  f"taux {100 * b.taux_reussite():.0f} %")

    # ------------------------------------------------------------------
    # Navigation générique : aller quelque part, avec nouvel essai.
    def _naviguer_vers(self, poste: str, suivant: str,
                       en_echec: Callable[[], None]) -> None:
        if self._entree:
            self._entree = False
            ok, raison = self.robot.aller_au_poste(poste)
            if not ok:
                self._log(f"navigation vers {poste} refusée : {raison}")
                self._nav_en_echec(en_echec)
            return
        etat = self.robot.navigation()
        if etat == "ARRIVE":
            self._essais_nav = 0
            self._aller(suivant)
        elif etat == "ECHEC" or self._temps_etat > self.cfg.delai_nav_s:
            self._log(f"navigation vers {poste} en échec ({etat})")
            self._nav_en_echec(en_echec)

    def _nav_en_echec(self, en_echec: Callable[[], None]) -> None:
        self.bilan.echecs_navigation += 1
        self._essais_nav += 1
        if self._essais_nav < self.cfg.essais_nav_max:
            self._temps_etat, self._entree = 0.0, True     # on réessaie
        else:
            self._essais_nav = 0
            en_echec()

    # ------------------------------------------------------------------
    def _etat_revenir(self) -> None:
        self._naviguer_vers(self.cfg.poste_prise, "CHERCHER",
                            en_echec=self._prise_inaccessible)

    def _prise_inaccessible(self) -> None:
        # Bras vides à cet instant : aucun risque à patienter. On signale,
        # on marque une pause, puis on réessaie — la mission ne s'arrête pas
        # sur une obstruction passagère.
        self._alerte("poste de prise inaccessible — pause "
                     f"{self.cfg.pause_prise_inaccessible_s:.0f} s puis nouvel essai")
        self._aller("PAUSE")

    def _etat_pause(self) -> None:
        if self._mission_finie():
            self._finir()
        elif self._temps_etat >= self.cfg.pause_prise_inaccessible_s:
            self._aller("REVENIR")

    def _ignore(self, c: Colis) -> bool:
        t = self.duree_ecoulee()
        self.ignores = [(p, d) for p, d in self.ignores
                        if t - d < self.cfg.oubli_ignore_s]
        return any(math.dist(c.position, p) < self.cfg.rayon_ignore_m
                   for p, _ in self.ignores)

    def _etat_chercher(self) -> None:
        if self._mission_finie():
            self._finir()
            return
        candidats: List[Colis] = []
        for couleur in self.cfg.regles:
            if couleur in self.bilan.couleurs_suspendues:
                continue
            candidats += [c for c in self.robot.chercher(couleur)
                          if c.couleur == couleur and not self._ignore(c)]
        if not candidats:
            t = self.duree_ecoulee()
            if self._inactif_depuis is None:
                self._inactif_depuis = t
            elif (t - self._inactif_depuis >= self.cfg.alerte_inactivite_s
                  and not self._inactivite_signalee):
                self._inactivite_signalee = True
                self._alerte(f"aucun colis à trier depuis "
                             f"{t - self._inactif_depuis:.0f} s — table vide, "
                             "ou encombrée de colis sans règle ?")
            self._aller("ATTENDRE")
            return
        self._inactif_depuis, self._inactivite_signalee = None, False
        self.colis = max(candidats, key=lambda c: c.confiance)
        # Même colis qu'une prise ratée juste avant (même couleur, même
        # place) : les essais continuent de compter. Sinon, on repart à zéro.
        rate = self._derniere_prise_ratee
        self._essais_prise = rate[2] if (
            rate is not None and rate[1] == self.colis.couleur
            and math.dist(rate[0], self.colis.position) < self.cfg.rayon_ignore_m) else 0
        self._derniere_prise_ratee = None
        self._log(f"colis {self.colis.couleur} repéré "
                  f"(confiance {self.colis.confiance:.2f})")
        self._aller("ACCOSTER")

    def _etat_attendre(self) -> None:
        if self._mission_finie():
            self._finir()
        elif self._temps_etat >= self.cfg.attente_recherche_s:
            self._aller("CHERCHER")

    def _etat_accoster(self) -> None:
        if self._entree:
            self._entree = False
            ok, raison = self.robot.accoster(self.colis)
            if not ok:
                self._accostage_rate(f"refusé : {raison}")
            return
        etat = self.robot.navigation()
        if etat == "ARRIVE":
            self._aller("PRENDRE")
        elif etat == "ECHEC" or self._temps_etat > self.cfg.delai_nav_s:
            self._accostage_rate(f"en échec ({etat})")

    def _accostage_rate(self, raison: str) -> None:
        # Le robot n'est peut-être plus au poste de prise : il y retourne
        # avant de chercher à nouveau. Le colis n'est abandonné qu'après
        # plusieurs accostages ratés AU MÊME endroit.
        self.bilan.echecs_navigation += 1
        pos = self.colis.position
        self._echecs_accostage.append(pos)
        n = sum(1 for p in self._echecs_accostage
                if math.dist(p, pos) < self.cfg.rayon_ignore_m)
        if n >= self.cfg.essais_nav_max:
            self._echecs_accostage = [p for p in self._echecs_accostage
                                      if math.dist(p, pos) >= self.cfg.rayon_ignore_m]
            self._abandonner_colis(f"{n} accostages ratés, dernier {raison}")
        else:
            self._log(f"accostage {raison} — retour au poste de prise")
            self.colis = None
            self._aller("REVENIR")

    def _etat_prendre(self) -> None:
        if self._entree:
            self._entree = False
            ok, raison = self.robot.prendre(self.colis)
            if not ok:
                self._prise_ratee(raison)
            return
        etat = self.robot.manipulation()
        if etat == "TENU":
            if self._essais_prise > 0:
                self.bilan.reussies_apres_nouvel_essai += 1
            self.destination = self.cfg.regles[self.colis.couleur]
            self._log(f"colis {self.colis.couleur} en main -> {self.destination}")
            self._aller("LIVRER")
        elif etat == "ECHEC" or self._temps_etat > self.cfg.delai_manip_s:
            self._prise_ratee("prise non confirmée" if etat == "ECHEC"
                              else "prise trop longue")

    def _prise_ratee(self, raison: str) -> None:
        self.bilan.prises_ratees += 1
        self.robot.reinitialiser_manipulation()
        self._essais_prise += 1
        self._log(f"prise ratée ({raison}), essai {self._essais_prise}"
                  f"/{self.cfg.essais_prise_max}")
        if self._essais_prise < self.cfg.essais_prise_max:
            # On RE-REGARDE la table avant de réessayer. Constaté sur GPU :
            # le carton, bousculé par la prise ratée, était tombé au sol ; le
            # robot a refermé deux fois les mains sur le vide, à l'ancienne
            # position. Une prise ratée peut avoir déplacé le colis.
            self._derniere_prise_ratee = (self.colis.position, self.colis.couleur,
                                          self._essais_prise)
            self.colis = None
            self._aller("REVENIR")          # recul, puis nouvelle recherche
        else:
            self._abandonner_colis(f"{self._essais_prise} prises ratées")

    def _abandonner_colis(self, raison: str) -> None:
        self.ignores.append((self.colis.position, self.duree_ecoulee()))
        self.bilan.colis_abandonnes += 1
        self._log(f"colis {self.colis.couleur} SIGNALÉ et laissé en place : {raison}")
        self.colis, self._essais_prise = None, 0
        self._aller("REVENIR")          # se replacer avant de chercher

    def _etat_livrer(self) -> None:
        self._naviguer_vers(self.destination, "POSER",
                            en_echec=self._destination_inaccessible)

    def _destination_inaccessible(self) -> None:
        couleur = self.colis.couleur
        if couleur not in self.bilan.couleurs_suspendues:
            self.bilan.couleurs_suspendues.append(couleur)
        self._alerte(f"poste {self.destination} inaccessible — tri des colis "
                     f"{couleur} SUSPENDU, colis ramené")
        self.destination = self.cfg.poste_prise
        self._aller("RAMENER")

    def _etat_ramener(self) -> None:
        self._naviguer_vers(self.cfg.poste_prise, "POSER",
                            en_echec=self._poser_sur_place)

    def _poser_sur_place(self) -> None:
        self._alerte("retour impossible — colis déposé sur place")
        self.destination = None
        self._aller("POSER")

    def _etat_poser(self) -> None:
        if self._entree:
            self._entree = False
            ok, raison = self.robot.poser()
            if not ok:
                self._pose_ratee(raison)
            return
        etat = self.robot.manipulation()
        if etat == "LIBRE":
            c, dest = self.colis, self.destination
            if dest is not None and dest != self.cfg.poste_prise:
                b = self.bilan
                b.colis_tries += 1
                b.par_couleur[c.couleur] = b.par_couleur.get(c.couleur, 0) + 1
                b.livraisons.append((c.couleur, dest))
                self._log(f"colis {c.couleur} LIVRÉ au poste {dest} "
                          f"(total {b.colis_tries})")
            else:
                # ramené ou posé en route : il ne doit pas être repris.
                self.ignores.append((c.position, self.duree_ecoulee()))
                self.bilan.colis_abandonnes += 1
            self.colis, self.destination, self._essais_pose = None, None, 0
            self._aller("REVENIR")
        elif etat == "ECHEC" or self._temps_etat > self.cfg.delai_manip_s:
            self._pose_ratee("dépose non confirmée")

    def _pose_ratee(self, raison: str) -> None:
        self._essais_pose += 1
        if self._essais_pose < self.cfg.essais_pose_max:
            self._log(f"dépose ratée ({raison}), nouvel essai")
            self._temps_etat, self._entree = 0.0, True
        else:
            # Situation inconnue avec un objet en main : on ne devine pas.
            self._alerte(f"dépose impossible ({raison}) — arrêt propre, "
                         "intervention requise")
            self.termine = True
            self._aller("ARRET")

    # États finaux : plus rien à faire.
    def _etat_fin(self) -> None:
        pass

    def _etat_arret(self) -> None:
        pass
