"""
Collecte de démonstrations de prise, pour entraîner le VLA par imitation.

Un VLA (Vision-Language-Action) est un modèle appris qui reçoit une image,
une consigne en langage naturel et l'état du robot, et qui propose des
mouvements. On l'entraîne ici par imitation : il apprend à refaire ce que
lui montre le robot piloté par ses réflexes codés (le comportement
programmé à la main). Chaque tentative enregistrée s'appelle un épisode.

Pourquoi un banc dédié plutôt que le scénario de tri : le scénario complet
(livrer à 4-6 m, humains qui passent) produit environ un colis toutes les
deux minutes. Il faudrait des heures de GPU (la carte de calcul qui fait
tourner la simulation) pour les quelque 200 épisodes que recommande NVIDIA.
Ici, on ne répète que le geste à apprendre, en boucle :

    PLACER    l'opérateur simulé pose UN carton, au hasard : position le
              long de la table, recul, rotation (±12°). C'est la variété
              que les réflexes codés gèrent mal et qu'on veut apprendre ;
    CHERCHER  le robot le repère, comme en tri, avec la caméra et OWL-ViT
              (un modèle qui détecte un objet décrit par un texte) ;
    ACCOSTER  approche, visée de face, pré-saisie, insertion ;
    PRENDRE   serrage, levage, confirmation     -> fin de l'ÉPISODE ;
    POSER     le robot repose le carton sur la table, en posture de port
              (le fond du carton revient à hauteur du plateau) ;
    RETOUR    l'opérateur REPREND le carton, PUIS le robot recule vers le
              poste de prise.

Pourquoi l'opérateur reprend le carton avant le recul. Problème constaté
en simulation, sur la machine GPU, et relevé par un audit de sécurité du
2026-10-10 : reposé sans poste de dépose, le carton dépassait parfois du
bord de la table (obstacle vu à y = 1.95, la table commençant à y = 2.00,
en mètres dans le repère de la scène). Tant que le robot restait collé à
la table, ce bout de carton se trouvait dans son volume de port (la zone
devant le torse où se trouve le carton porté, que le gardien de proximité
ignore). Au recul, il sortait de ce volume, à moins de d_arret (la
distance sous laquelle le gardien arrête le robot ; voir
axis_os/services/locomotion/proximite.py). Le gardien arrêtait alors le
robot, à raison : il ne peut pas savoir que ce n'est pas une main. Et le
robot, immobile, ne pouvait plus s'en éloigner : il restait bloqué jusqu'au
délai du retour. Même blocage avec un carton tombé au sol après une prise
ratée. Le gardien, sa marge et ses délais ne changent pas : c'est le banc
de collecte qui ne laisse plus d'obstacle derrière lui. Les épisodes
enregistrés (approche et prise) sont inchangés : ils se terminent avant.

Un épisode enregistré va de l'ordre d'approche jusqu'à la prise confirmée
(ou ratée). Les ratés sont gardés dans le journal des épisodes, et écartés
à la conversion (vers_lerobot.py).

La collecte n'a aucun passe-droit : tout passe par la façade du tri
(FacadeServices, l'interface par laquelle l'application de tri commande le
robot, dans axis_os/apps/tri_colis/facade.py), donc par le gardien de
proximité, par le JointSafetyGuard (le garde-fou qui vérifie chaque
commande d'articulation) et par la navigation.
"""
from __future__ import annotations

import random
from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np

# Noms des couleurs en anglais, pour la consigne donnée au VLA.
COULEURS_EN = {"rouge": "red", "bleu": "blue", "vert": "green", "jaune": "yellow"}


def consigne(couleur: str) -> str:
    """Consigne en langage naturel donnée au VLA. Elle est en anglais : les
    modèles de NVIDIA sont pré-entraînés sur des consignes en anglais."""
    return f"pick up the {COULEURS_EN.get(couleur, couleur)} box"


class CollecteurPrises:
    """Machine à états de la collecte (voir l'en-tête du module), avancée
    d'un pas à chaque appel de pas() par la boucle de simulation.

    facade        : la façade du tri (chercher, accoster, prendre, poser,
                    aller_au_poste…).
    enregistreur  : un EnregistreurEpisodes (enregistreur.py).
    placer(rng)   : l'opérateur simulé pose un carton tiré au hasard avec
                    rng ; rend (couleur, méta-données du placement).
    retirer()     : l'opérateur simulé reprend le carton.
    horloge()     : le temps simulé, en secondes.
    etat_action() : (état des bras, consignes envoyées), à enregistrer.
    image()       : l'image de la caméra de tête, ou None si elle n'est pas
                    encore rendue.
    n_episodes    : nombre d'épisodes à enregistrer avant de s'arrêter.
    graine        : graine du tirage des placements, c'est-à-dire la valeur
                    qui fixe le tirage au hasard (même graine, même suite
                    de cartons).
    journal       : fonction qui reçoit les messages de suivi (print par
                    défaut).
    poste_prise   : poste de navigation où le robot attend chaque carton.
    delai_s       : délai maximal de l'accostage, et du retour au poste.
    """

    def __init__(self, facade, enregistreur, placer: Callable[[random.Random], Tuple[str, Dict]],
                 retirer: Callable[[], None], horloge: Callable[[], float],
                 etat_action: Callable[[], Tuple[Sequence[float], Sequence[float]]],
                 image: Callable[[], Optional[np.ndarray]], n_episodes: int = 200,
                 graine: int = 0, journal: Callable[[str], None] = print,
                 poste_prise: str = "prise", delai_s: float = 60.0):
        self.f, self.enr = facade, enregistreur
        self._placer, self._retirer = placer, retirer
        self._horloge, self._etat_action, self._image = horloge, etat_action, image
        self.n_episodes, self.poste = n_episodes, poste_prise
        self.rng = random.Random(graine)
        self._journal = journal
        self.delai_s = delai_s
        self.etat, self._t_etat, self._entree = "RETOUR", 0.0, True
        self.couleur: Optional[str] = None
        self.meta: Dict = {}
        self._colis = None
        self._essais_recherche = 0
        self.stats = {"episodes": 0, "reussis": 0, "rates": 0, "sans_detection": 0}

    # ---- interface avec la boucle de simulation ------------------------
    @property
    def termine(self) -> bool:
        return self.etat == "FIN"

    def veut_rendu(self, t_prochain: float) -> bool:
        """Vrai si la boucle doit rendre la caméra de tête au pas suivant :
        pendant la recherche, ou quand l'enregistreur gardera ce pas."""
        return self.etat == "CHERCHER" or self.enr.veut_image(t_prochain)

    def _aller(self, etat: str) -> None:
        self.etat, self._t_etat, self._entree = etat, self._horloge(), True

    def _duree(self) -> float:
        return self._horloge() - self._t_etat

    def _log(self, msg: str) -> None:
        self._journal(f"[{self._horloge():7.1f}s] VLA : {msg}")

    def _fin_episode(self, succes: bool, raison: str = "") -> None:
        """Clôt l'épisode en cours, s'il y en a un, et met à jour les
        compteurs."""
        if self.enr.actif:
            self.enr.fin(succes, self._horloge(), raison=raison)
            self.stats["episodes"] += 1
            self.stats["reussis" if succes else "rates"] += 1
            self._log(f"épisode {self.stats['episodes']}/{self.n_episodes} "
                      f"{'RÉUSSI' if succes else 'raté'}{' — ' + raison if raison else ''} "
                      f"({self.stats['reussis']} réussis)")

    # ---- un pas de commande ----------------------------------------------
    def pas(self) -> None:
        t = self._horloge()
        if self.enr.actif and self.enr.veut_image(t):
            # Image et état lus seulement quand l'enregistreur garde ce pas
            # (10 fois par seconde) : copier l'image à chaque pas de la
            # boucle, à 50 Hz, ralentirait la simulation pour rien.
            etat, action = self._etat_action()
            self.enr.pas(t, self._image(), etat, action)
        # Traitement de l'état courant : _etat_placer, _etat_chercher…
        getattr(self, f"_etat_{self.etat.lower()}")()

    def _etat_fin(self) -> None:
        pass

    def _etat_placer(self) -> None:
        if self._entree:
            self._entree = False
            if self.stats["episodes"] >= self.n_episodes:
                self._retirer()
                self._log(f"collecte terminée : {self.stats}")
                self._aller("FIN")
                return
            self.couleur, self.meta = self._placer(self.rng)
            self._essais_recherche = 0
            return
        if self._duree() >= 1.0:              # 1 s : le carton se pose
            self._aller("CHERCHER")

    def _etat_chercher(self) -> None:
        if self._duree() < 0.3:               # attendre une image fraîche
            return
        vus = self.f.chercher(self.couleur)
        if not vus:
            # Trois recherches vaines : le carton est retiré et un nouveau
            # est tiré au hasard, sans ouvrir d'épisode (compté dans
            # « sans_detection »). Entre deux recherches, on attend de
            # nouveau une image fraîche.
            self._essais_recherche += 1
            if self._essais_recherche >= 3:
                self.stats["sans_detection"] += 1
                self._log(f"carton {self.couleur} non repéré ({self.meta}) — replacé")
                self._retirer()
                self._aller("PLACER")
            else:
                self._t_etat = self._horloge()
            return
        # Carton repéré : l'épisode commence ici, avec l'ordre d'approche.
        self._colis = vus[0]
        self.enr.debut(consigne(self.couleur), self._horloge(),
                       meta={**self.meta, "couleur": self.couleur})
        ok, raison = self.f.accoster(self._colis)
        if not ok:
            self._fin_episode(False, f"accostage refusé : {raison}")
            self._aller("RETOUR")
            return
        self._aller("ACCOSTER")

    def _etat_accoster(self) -> None:
        e = self.f.navigation()
        if e == "ARRIVE":
            ok, raison = self.f.prendre(self._colis)
            if not ok:
                self._fin_episode(False, f"prise refusée : {raison}")
                self.f.reinitialiser_manipulation()
                self._aller("RETOUR")
                return
            self._aller("PRENDRE")
        elif e == "ECHEC" or self._duree() > self.delai_s:
            self._fin_episode(False, f"accostage en échec ({e})")
            self._aller("RETOUR")

    def _etat_prendre(self) -> None:
        # Prise confirmée (TENU) : l'épisode est réussi et se termine ici.
        # Échec, ou plus de 20 s : épisode raté.
        m = self.f.manipulation()
        if m == "TENU":
            self._fin_episode(True)
            ok, _ = self.f.poser()            # reposé sur la table, en posture de port
            self._aller("POSER" if ok else "RETOUR")
        elif m == "ECHEC" or self._duree() > 20.0:
            self._fin_episode(False, "prise non confirmée" if m == "ECHEC" else "prise trop longue")
            self.f.reinitialiser_manipulation()
            self._aller("RETOUR")

    def _etat_poser(self) -> None:
        # 10 s au plus pour que les bras soient libres ; au-delà, la
        # manipulation est réinitialisée.
        m = self.f.manipulation()
        if m == "LIBRE" or self._duree() > 10.0:
            if m != "LIBRE":
                self.f.reinitialiser_manipulation()
            self._aller("RETOUR")

    def _etat_retour(self) -> None:
        if self._entree:
            self._entree = False
            # Carton repris avant le recul (voir l'en-tête du module) : un
            # carton qui dépasse de la table ou gît au sol bloquait le robot
            # jusqu'au délai du retour.
            self._retirer()
            self.f.aller_au_poste(self.poste)
            return
        e = self.f.navigation()
        if e in ("ARRIVE", "ECHEC") or self._duree() > self.delai_s:
            if e != "ARRIVE":
                self._log(f"retour au poste de prise : {e} — on continue")
            self._aller("PLACER")
