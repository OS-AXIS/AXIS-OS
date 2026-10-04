"""
Collecte de démonstrations de PRISE, pour entraîner le VLA par imitation.

Le scénario de tri complet (livrer à 4-6 m, humains qui passent) produit
environ un colis toutes les deux minutes : il faudrait des heures de GPU
pour les ~200 épisodes que NVIDIA recommande. Ici, on ne répète que le
geste à apprendre, en boucle :

    PLACER    l'opérateur simulé pose UN carton, au hasard : position le
              long de la table, recul, rotation (±12°) — la variété que
              les réflexes codés gèrent mal et qu'on veut apprendre ;
    CHERCHER  le robot le repère (caméra + OWL-ViT, comme en tri) ;
    ACCOSTER  approche, visée de face, pré-saisie, insertion ;
    PRENDRE   serrage, levage, confirmation           -> fin de l'ÉPISODE
    POSER     il repose le carton sur la table (posture de port : le fond
              du carton revient à hauteur du plateau) ;
    RETOUR    recul vers le poste de prise ; l'opérateur retire le carton.

Un épisode enregistré = de l'ordre d'approche jusqu'à la prise confirmée
(ou ratée). Les ratés sont gardés dans le journal, et écartés à la
conversion.

Tout passe par la FAÇADE du tri — donc par le gardien, le JointSafetyGuard,
la navigation : la collecte n'a aucun passe-droit.
"""
from __future__ import annotations

import random
from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np

COULEURS_EN = {"rouge": "red", "bleu": "blue", "vert": "green", "jaune": "yellow"}


def consigne(couleur: str) -> str:
    """Consigne en langage naturel donnée au VLA (en anglais : les modèles
    de NVIDIA sont pré-entraînés sur des consignes en anglais)."""
    return f"pick up the {COULEURS_EN.get(couleur, couleur)} box"


class CollecteurPrises:
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
        """La boucle rend la caméra de tête au pas suivant si vrai."""
        return self.etat == "CHERCHER" or self.enr.veut_image(t_prochain)

    def _aller(self, etat: str) -> None:
        self.etat, self._t_etat, self._entree = etat, self._horloge(), True

    def _duree(self) -> float:
        return self._horloge() - self._t_etat

    def _log(self, msg: str) -> None:
        self._journal(f"[{self._horloge():7.1f}s] VLA : {msg}")

    def _fin_episode(self, succes: bool, raison: str = "") -> None:
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
            # Image et état lus SEULEMENT quand l'enregistreur garde ce pas
            # (10 par seconde) : copier l'image à 50 Hz ralentirait la
            # simulation pour rien.
            etat, action = self._etat_action()
            self.enr.pas(t, self._image(), etat, action)
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
        if self._duree() >= 1.0:              # le carton se pose
            self._aller("CHERCHER")

    def _etat_chercher(self) -> None:
        if self._duree() < 0.3:               # une image fraîche rendue
            return
        vus = self.f.chercher(self.couleur)
        if not vus:
            self._essais_recherche += 1
            if self._essais_recherche >= 3:
                self.stats["sans_detection"] += 1
                self._log(f"carton {self.couleur} non repéré ({self.meta}) — replacé")
                self._retirer()
                self._aller("PLACER")
            else:
                self._t_etat = self._horloge()
            return
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
        m = self.f.manipulation()
        if m == "TENU":
            self._fin_episode(True)
            ok, _ = self.f.poser()            # reposé sur la table, à hauteur de port
            self._aller("POSER" if ok else "RETOUR")
        elif m == "ECHEC" or self._duree() > 20.0:
            self._fin_episode(False, "prise non confirmée" if m == "ECHEC" else "prise trop longue")
            self.f.reinitialiser_manipulation()
            self._aller("RETOUR")

    def _etat_poser(self) -> None:
        m = self.f.manipulation()
        if m == "LIBRE" or self._duree() > 10.0:
            if m != "LIBRE":
                self.f.reinitialiser_manipulation()
            self._aller("RETOUR")

    def _etat_retour(self) -> None:
        if self._entree:
            self._entree = False
            self.f.aller_au_poste(self.poste)
            return
        e = self.f.navigation()
        if e in ("ARRIVE", "ECHEC") or self._duree() > self.delai_s:
            if e != "ARRIVE":
                self._log(f"retour au poste de prise : {e} — on continue")
            self._retirer()
            self._aller("PLACER")
