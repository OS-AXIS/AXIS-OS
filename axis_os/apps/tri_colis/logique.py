"""
Tri de colis : la logique métier, séparée du robot.

Cette machine à états (un programme qui passe d'un état nommé à un autre,
CHERCHER, ACCOSTER, PRENDRE, LIVRER…, selon ce qui arrive) décide quoi
faire : quel colis prendre, où le porter, que faire quand ça rate. Elle ne
commande jamais un moteur : elle parle à une façade `Robot` (navigation,
manipulation, perception, sécurité), branchée en simulation, sur le vrai
robot (facade.py), ou sur un monde factice pour les tests
(test/monde_tri_factice.py, test/test_tri_colis.py).

Principes :
  - la règle de tri (couleur -> poste) est de la connaissance métier : elle
    vit dans la configuration, jamais dans l'IA ;
  - un arrêt de sécurité (humain trop près) n'est pas une panne : la
    logique se fige, ce temps est compté à part, et aucun délai ne court ;
  - tout échec a une issue (nouvel essai, abandon signalé, couleur
    suspendue, arrêt propre) : aucun état n'est sans sortie ;
  - la mission ne se termine jamais les bras chargés.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Protocol, Tuple


@dataclass(frozen=True)
class Colis:
    """Un colis repéré : sa couleur, la position (x, y, z) de son centre
    dans le repère monde (le repère fixe de la carte), en mètres, et la
    confiance de sa détection."""
    couleur: str
    position: Tuple[float, float, float]
    confiance: float


class Robot(Protocol):
    """Ce que la logique attend du robot ; facade.FacadeServices le réalise
    avec les services d'AXIS-OS.

    Les méthodes qui lancent une action (aller_au_poste, accoster, prendre,
    poser) rendent (acceptée, raison d'un refus) sans attendre la fin du
    geste : la logique en suit ensuite l'avancement par navigation() et
    manipulation()."""
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
    """Réglages de la mission. Durées en secondes, distances en mètres.
    Les règles de tri peuvent être changées en cours de mission par le
    superviseur (superviseur.py)."""
    regles: Dict[str, str]                  # couleur -> poste de dépose
    poste_prise: str = "prise"
    # Nombre d'essais avant d'abandonner : prise d'un même colis ; trajet
    # vers un poste, ou accostage d'un colis au même endroit ; dépose.
    essais_prise_max: int = 3
    essais_nav_max: int = 3
    essais_pose_max: int = 2
    # Durées maximales d'un trajet ou d'un accostage, puis d'une prise ou
    # d'une dépose ; pause entre deux recherches qui n'ont rien trouvé.
    delai_nav_s: float = 90.0
    delai_manip_s: float = 20.0
    attente_recherche_s: float = 5.0
    duree_mission_s: float = 1800.0
    # Deux positions plus proches que ce rayon désignent le même colis, ou
    # le même emplacement.
    rayon_ignore_m: float = 0.15
    # Un emplacement abandonné est ignoré un temps, pas pour toujours :
    # l'opérateur peut l'avoir dégagé et un colis neuf y être posé.
    oubli_ignore_s: float = 300.0
    pause_prise_inaccessible_s: float = 30.0
    alerte_inactivite_s: float = 120.0     # rien à trier depuis ce temps
    alerte_arret_prolonge_s: float = 60.0  # bloqué par la sécurité depuis ce temps


@dataclass
class Bilan:
    """Compteurs de la mission. Le rapport du superviseur est écrit à
    partir d'eux, par du code."""
    colis_tries: int = 0
    par_couleur: Dict[str, int] = field(default_factory=dict)
    # Chaque livraison réussie : (couleur, poste).
    livraisons: List[Tuple[str, str]] = field(default_factory=list)
    prises_ratees: int = 0
    reussies_apres_nouvel_essai: int = 0
    colis_abandonnes: int = 0
    couleurs_suspendues: List[str] = field(default_factory=list)
    temps_arret_securite_s: float = 0.0
    echecs_navigation: int = 0
    alertes: List[str] = field(default_factory=list)

    def taux_reussite(self) -> float:
        """Colis triés sur colis tentés (triés ou abandonnés) ; 1.0 tant
        que rien n'a été tenté."""
        tentes = self.colis_tries + self.colis_abandonnes
        return self.colis_tries / tentes if tentes else 1.0


# États où la mission peut se terminer : bras vides, robot au poste de prise.
_ETATS_TERMINABLES = ("CHERCHER", "ATTENDRE", "PAUSE")


class TriColis:
    """La mission de tri. pas() est appelé à chaque cycle de la boucle de
    commande ; chaque état a sa méthode _etat_<nom>().

    Cycle normal : REVENIR (au poste de prise) -> CHERCHER -> ACCOSTER ->
    PRENDRE -> LIVRER -> POSER -> REVENIR. Autres états : ATTENDRE (rien à
    trier), PAUSE (poste de prise inaccessible), RAMENER (poste de dépose
    inaccessible, le colis revient au poste de prise), puis les états
    finaux FIN (mission terminée) et ARRET (intervention requise)."""

    def __init__(self, robot: Robot, cfg: ConfigTri):
        self.robot, self.cfg = robot, cfg
        self.bilan = Bilan()
        self.etat = "REVENIR"           # d'abord se placer au poste de prise
        self.termine = False
        self.colis: Optional[Colis] = None
        self.destination: Optional[str] = None
        # Emplacements abandonnés (position, instant de l'abandon) : on n'y
        # prend plus rien pendant oubli_ignore_s.
        self.ignores: List[Tuple[Tuple[float, float, float], float]] = []
        self._debut: Optional[float] = None   # instant du premier pas
        self._prec: Optional[float] = None    # instant du pas précédent
        # Temps actif passé dans l'état courant : les arrêts de sécurité
        # n'y comptent pas.
        self._temps_etat = 0.0
        self._entree = True             # premier pas dans l'état courant
        self._essais_prise = 0
        # Prises ratées par colis (position, couleur, essais) : un colis
        # garde son compte même si le robot en a pris un autre entre-temps.
        self._prises_ratees: List[Tuple[Tuple[float, float, float], str, int]] = []
        self._essais_nav = 0
        self._essais_pose = 0
        # Positions des accostages ratés, pour compter les échecs au même
        # endroit.
        self._echecs_accostage: List[Tuple[float, float, float]] = []
        self._inactif_depuis: Optional[float] = None   # rien à trier depuis
        self._bloque_s = 0.0            # durée de l'arrêt de sécurité en cours
        self._blocage_signale = False
        self._inactivite_signalee = False

    # ------------------------------------------------------------------
    def duree_ecoulee(self) -> float:
        """Temps écoulé depuis le premier pas, arrêts de sécurité compris."""
        if self._debut is None:
            return 0.0
        return self.robot.maintenant() - self._debut

    def pas(self) -> None:
        """Un pas de la mission, à appeler à chaque cycle de commande."""
        if self.termine:
            return
        t = self.robot.maintenant()
        if self._debut is None:
            self._debut = self._prec = t
        dt, self._prec = max(0.0, t - self._prec), t
        if self.robot.bloque_par_securite():
            # L'OS a arrêté le robot : on attend, sans rien commander, et
            # sans que ce temps compte dans les délais de l'état. Si l'arrêt
            # dure, quelqu'un se tient devant le robot : on le signale.
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
        if self.etat in ("LIVRER", "RAMENER") and self.robot.manipulation() == "ECHEC":
            # Le colis en main n'est plus tenu comme il faut (penché malgré
            # la correction) : on ne marche pas avec, on appelle un humain.
            self._alerte("colis mal tenu pendant le transport — arrêt propre, "
                         "intervention requise")
            self.termine = True
            self._aller("ARRET")
            return
        self._temps_etat += dt
        # Chaque état a sa méthode : CHERCHER -> _etat_chercher(), etc.
        getattr(self, "_etat_" + self.etat.lower())()

    def _aller(self, etat: str) -> None:
        """Passe dans `etat`, avec un compteur de temps remis à zéro."""
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
        """Au premier pas de l'état, demande le départ vers `poste` ;
        ensuite, attend l'arrivée (on passe alors à `suivant`) ou l'échec.
        Après essais_nav_max échecs, appelle `en_echec`."""
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
        """Vrai si le colis se trouve à un emplacement abandonné il y a
        moins de oubli_ignore_s. Oublie au passage les abandons plus
        anciens."""
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
        # Le candidat le plus sûr, toutes couleurs confondues.
        self.colis = max(candidats, key=lambda c: c.confiance)
        # Colis déjà raté (même couleur, même place) : ses essais continuent
        # de compter, même si un autre a été pris entre-temps.
        self._essais_prise = 0
        for i, (pos, coul, n) in enumerate(self._prises_ratees):
            if coul == self.colis.couleur and \
                    math.dist(pos, self.colis.position) < self.cfg.rayon_ignore_m:
                self._essais_prise = n
                del self._prises_ratees[i]
                break
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
        # essais_nav_max accostages ratés au même endroit.
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
            # On regarde de nouveau la table avant de réessayer, car une
            # prise ratée peut avoir déplacé le colis. Constaté en
            # simulation physique sur carte graphique (GPU), sous Isaac
            # Sim : le carton, bousculé par la prise ratée, était tombé au
            # sol, et le robot a refermé deux fois les mains sur le vide, à
            # l'ancienne position.
            self._prises_ratees.append((self.colis.position, self.colis.couleur,
                                        self._essais_prise))
            self.colis = None
            self._aller("REVENIR")          # recul, puis nouvelle recherche
        else:
            self._abandonner_colis(f"{self._essais_prise} prises ratées")

    def _abandonner_colis(self, raison: str) -> None:
        """Laisse le colis en place, le signale dans le journal, et ignore
        son emplacement pendant oubli_ignore_s."""
        self.ignores.append((self.colis.position, self.duree_ecoulee()))
        self.bilan.colis_abandonnes += 1
        self._log(f"colis {self.colis.couleur} SIGNALÉ et laissé en place : {raison}")
        self.colis, self._essais_prise = None, 0
        self._aller("REVENIR")          # se replacer avant de chercher

    def _etat_livrer(self) -> None:
        self._naviguer_vers(self.destination, "POSER",
                            en_echec=self._destination_inaccessible)

    def _destination_inaccessible(self) -> None:
        """Le poste de dépose est inaccessible : le tri de cette couleur est
        suspendu (le superviseur pourra le reprendre ou changer la règle),
        et le colis en main est ramené au poste de prise."""
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
        """Même le retour au poste de prise échoue : plutôt que de garder
        le colis en main, le robot le pose là où il est."""
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
                # Colis ramené au poste de prise ou posé en route : compté
                # comme abandonné, et son emplacement ignoré pour qu'il ne
                # soit pas repris.
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
            # Situation inconnue avec un objet en main : on ne devine pas,
            # on s'arrête et on appelle un humain.
            self._alerte(f"dépose impossible ({raison}) — arrêt propre, "
                         "intervention requise")
            self.termine = True
            self._aller("ARRET")

    # États finaux : plus rien à faire. FIN : mission terminée bras vides ;
    # ARRET : arrêt propre, intervention requise.
    def _etat_fin(self) -> None:
        pass

    def _etat_arret(self) -> None:
        pass
