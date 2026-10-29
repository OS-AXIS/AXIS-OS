"""
Évaluation du VLA contre les réflexes codés : mêmes cartons, même robot.

Un VLA (Vision-Language-Action) est un modèle appris qui reçoit une image,
une consigne en langage naturel et l'état du robot, et qui propose des
mouvements. Ici, c'est la politique GR00T de NVIDIA (la « politique » est
le modèle qui choisit les mouvements), entraînée sur les démonstrations de
collecte.py. Les réflexes codés sont le comportement programmé à la main
qui a servi de professeur. Déroulement :

    pour chaque carton i (position, recul, rotation tirés d'avance) :
        pour chaque mode demandé (« reflexes », « vla_prise », « vla ») :
            PLACER   l'opérateur simulé pose le carton i, à l'identique ;
            ATTENTE  1 s, le temps que le carton se pose ;
            ESSAI    reflexes  : chercher (caméra + OWL-ViT, un modèle qui
                                 détecte un objet décrit par un texte),
                                 accoster, prendre : exactement le chemin
                                 de la collecte ;
                     vla_prise : les réflexes cherchent et accostent (bras
                                 ouverts autour du carton), puis le VLA
                                 fait seul la prise, base immobile ;
                     vla       : la politique GR00T reçoit l'image de la
                                 tête, l'état des bras et la consigne
                                 (« pick up the red box ») et pilote bras
                                 et base. Rien d'autre : ni détection, ni
                                 navigation, ni geste codé ;
            LIBERER  reflexes : reposer le carton ; vla et vla_prise : bras
                     ramenés pas à pas à la posture de départ (d'abord sans
                     ouvrir les bras, puis en les ouvrant) ;
            RETOUR   l'opérateur REPREND le carton, PUIS le robot retourne
                     au poste de prise. Un carton tombé au sol pendant
                     l'essai ou la libération est ramassé aussitôt.

Pourquoi reprendre le carton avant le recul. Problème constaté en
simulation, sur la machine GPU (la machine équipée d'une carte de calcul,
qui fait tourner la simulation), et relevé par un audit de sécurité du
2026-10-10 : reposé sans poste de dépose, le carton dépassait parfois du
bord de la table ; un carton tombé au sol restait près du robot. Dans les
deux cas, le gardien de proximité (qui arrête le robot quand un obstacle
inexpliqué, peut-être humain, est trop proche) voyait un obstacle à moins
de d_arret, sa distance d'arrêt, et arrêtait le robot, qui ne pouvait plus
s'en éloigner. Le robot restait bloqué jusqu'au délai du retour (60 s), et
même indéfiniment en mode vla : les bras sont figés pendant l'arrêt de
sécurité, et le délai de LIBERER est alors suspendu. Le gardien ne change
pas : c'est le banc qui ne laisse plus d'obstacle derrière lui. Les
résultats des essais sont inchangés : ils sont notés avant.

Réussite, avec le même critère physique pour tous les modes : le carton a
quitté la table, entre les mains (`carton_souleve()`), pendant `tenue_s`
secondes d'affilée. Pour les réflexes, c'est déjà ce que vérifie la
manipulation avant de dire TENU.

Échec : délai dépassé, carton au sol, VLA en défaut (serveur muet, erreurs
répétées), refus de la façade (FacadeServices, l'interface par laquelle
l'application de tri commande le robot). La raison est notée.

Aucun passe-droit. En mode vla, les bras passent par VLAService (qui limite
l'amplitude de chaque pas, fige les bras pendant un arrêt de sécurité et
arrête le mouvement si le modèle ne fournit plus de consigne fraîche), puis
par l'adaptateur (qui ramène les cibles dans les butées, les limites
mécaniques des articulations) et par le JointSafetyGuard (le garde-fou qui
vérifie chaque commande d'articulation). La base passe par
LocomotionService (vitesses bornées) et par le gardien de proximité. Les
refus du garde-fou et les arrêts de sécurité sont comptés par essai : un
VLA qui « réussit » en se faisant refuser des consignes n'est pas un VLA
qui réussit.

Mesures de la prise, par essai (si la boucle fournit mesures_prise()) :
l'inclinaison maximale du carton tant qu'il est soulevé, la garde minimale
des avant-bras au-dessus d'un plateau, et le nombre de pas où ils le
touchent (garde de moins de 5 mm). Les compteurs du VLA (inférences,
erreurs…) sont ceux de l'essai ; leur total est à part (vla_total()).

Les cartons sont tirés avec une graine (la valeur qui fixe le tirage au
hasard) différente de celle de la collecte : le modèle est jugé sur des
positions qu'il n'a jamais vues.
"""
from __future__ import annotations

import json
import math
import random
import statistics
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from axis_os.vla.collecte import consigne

# Modes d'essai (voir l'en-tête) ; MODES_VLA : ceux où le VLA pilote.
MODES = ("reflexes", "vla", "vla_prise")
MODES_VLA = ("vla", "vla_prise")
# Compteurs du juge des contacts du banc (contact estimé par l'OS contre
# contact réel), cumulés : rendus par compteurs() s'il y a un juge.
CONTACTS_JUGE = ("contacts_vrais", "contacts_manques", "faux_contacts")


def tirer_placements(n: int, graine: int, couleurs: Sequence[str] = ("rouge", "bleu"),
                     x: Tuple[float, float] = (3.55, 4.45), dy: Tuple[float, float] = (0.0, 0.05),
                     lacet_deg: float = 12.0) -> List[Tuple[str, Dict]]:
    """Les n cartons de l'évaluation, tirés d'avance avec les mêmes bornes
    que la collecte : chaque mode voit exactement la même suite. Chaque
    carton : (couleur, {"x": position le long de la table, "dy": recul,
    "lacet": rotation en degrés, entre -lacet_deg et +lacet_deg})."""
    rng = random.Random(graine)
    out = []
    for _ in range(n):
        out.append((rng.choice(list(couleurs)),
                    {"x": round(rng.uniform(*x), 3), "dy": round(rng.uniform(*dy), 3),
                     "lacet": round(rng.uniform(-lacet_deg, lacet_deg), 1)}))
    return out


class EvaluateurPrises:
    """Machine à états de l'évaluation (voir l'en-tête du module), avancée
    d'un pas à chaque appel de pas() par la boucle de simulation. Les
    résultats s'accumulent dans self.resultats, un par essai."""

    def __init__(self, placements: Sequence[Tuple[str, Dict]], modes: Sequence[str],
                 placer: Callable[[str, Dict], None], retirer: Callable[[], None],
                 horloge: Callable[[], float], facade, vla, bras: Dict[str, float],
                 carton_souleve: Callable[[], bool], carton_au_sol: Callable[[], bool],
                 arret_securite: Callable[[], bool], halt: Callable[[], None],
                 compteurs: Callable[[], Dict[str, float]],
                 journal: Callable[[str], None] = print, poste_prise: str = "prise",
                 mesures_prise: Optional[Callable[[], Dict[str, Optional[float]]]] = None,
                 duree_max_s: float = 45.0, tenue_s: float = 1.0,
                 delta_liberation: float = 0.02, roulis: Sequence[str] = (
                     "left_shoulder_roll_joint", "right_shoulder_roll_joint")):
        """
        placements  : les cartons, tirés par tirer_placements().
        modes       : les modes essayés sur chaque carton (voir MODES).
        placer(couleur, meta) / retirer()  : l'opérateur simulé pose ou
                      reprend le carton.
        horloge()   : le temps simulé, en secondes.
        facade      : FacadeServices (mode reflexes, et retour au poste).
        vla         : VLAService (modes vla et vla_prise) ; None si seul le
                      mode reflexes tourne.
        bras        : adapt.pose_bras, les cibles des bras dans
                      l'adaptateur ; sa valeur au premier essai est la
                      posture de départ, celle où la manipulation croit
                      les bras.
        carton_souleve() / carton_au_sol() : état physique du carton.
        arret_securite() : vrai pendant un arrêt de sécurité.
        halt()      : met la base à l'arrêt, en équilibre.
        compteurs() : {"refus": …, "arrets": …} cumulés (garde-fou,
                      gardien), et ceux du juge des contacts s'il y en a un
                      (CONTACTS_JUGE) ; l'évaluateur en fait la différence
                      par essai.
        duree_max_s : délai d'un essai, au-delà duquel il est raté.
        tenue_s     : durée pendant laquelle le carton doit rester soulevé
                      pour que l'essai réussisse.
        delta_liberation : pas maximal de chaque articulation, à chaque pas
                      de commande, quand les bras sont ramenés à la posture
                      de départ.
        roulis      : articulations qui ouvrent les bras (roulis des
                      épaules), ramenées en dernier.
        journal     : fonction qui reçoit les messages de suivi.
        poste_prise : poste de navigation où le robot attend chaque carton.
        mesures_prise() : {"inclinaison_deg": inclinaison du carton (ou
                      None), "garde_avant_bras_m": distance du dessous des
                      avant-bras au plateau qu'ils survolent (None : aucun)},
                      relevées à chaque pas de l'essai.
        """
        inconnus = [m for m in modes if m not in MODES]
        if inconnus or not modes:
            raise ValueError(f"modes inconnus : {inconnus} (possibles : {MODES})")
        if any(m in MODES_VLA for m in modes) and vla is None:
            raise ValueError("mode vla demandé sans VLAService")
        self.essais = [(i, m) for i in range(len(placements)) for m in modes]
        self.placements = list(placements)
        self._placer, self._retirer, self._horloge = placer, retirer, horloge
        self.f, self.vla, self._bras = facade, vla, bras
        self._souleve, self._au_sol = carton_souleve, carton_au_sol
        self._arret, self._halt, self._compteurs = arret_securite, halt, compteurs
        self._journal, self.poste = journal, poste_prise
        self.duree_max_s, self.tenue_s = duree_max_s, tenue_s
        self._delta, self._roulis = delta_liberation, tuple(roulis)
        self._depart: Optional[Dict[str, float]] = None
        self.k = 0                                    # indice de l'essai courant
        self.etat, self._t_etat, self._entree = "PLACER", 0.0, True
        self.sous_etat = ""
        self.resultats: List[Dict] = []
        self._t_essai = 0.0
        self._t_tenu: Optional[float] = None
        self._c0: Dict[str, float] = {}
        self._colis = None
        self._recherches = 0
        self._succes: Optional[bool] = None
        self._t_pas: Optional[float] = None
        self._dt = 0.0
        self._mesures = mesures_prise
        self._m: Dict[str, float] = {}
        self._v0: Dict[str, float] = {}
        self._latence_totale = 0.0

    # ---- interface avec la boucle de simulation ------------------------
    @property
    def termine(self) -> bool:
        return self.etat == "FIN"

    @property
    def mode(self) -> str:
        return self.essais[min(self.k, len(self.essais) - 1)][1]

    def veut_rendu(self, t_prochain: float) -> bool:
        """Vrai si la boucle doit rendre la caméra de tête au pas suivant :
        quand le VLA attend une image, ou pendant la recherche par les
        réflexes."""
        if self.etat != "ESSAI":
            return False
        if self.sous_etat == "POLITIQUE":
            return self.vla.veut_image(t_prochain)
        return self.sous_etat == "CHERCHER"

    def _aller(self, etat: str) -> None:
        self.etat, self._t_etat, self._entree = etat, self._horloge(), True

    def _duree(self) -> float:
        return self._horloge() - self._t_etat

    def _log(self, msg: str) -> None:
        self._journal(f"[{self._horloge():7.1f}s] ÉVAL : {msg}")

    # ---- un pas de commande ----------------------------------------------
    def pas(self) -> None:
        # _dt : temps écoulé depuis le pas précédent, pour suspendre un
        # délai pendant un arrêt de sécurité (voir _liberer_vla).
        t = self._horloge()
        self._dt = 0.0 if self._t_pas is None else t - self._t_pas
        self._t_pas = t
        # Traitement de l'état courant : _etat_placer, _etat_essai…
        getattr(self, f"_etat_{self.etat.lower()}")()

    def _etat_fin(self) -> None:
        pass

    def _etat_placer(self) -> None:
        if self.k >= len(self.essais):
            self._retirer()
            self._log("évaluation terminée")
            self._aller("FIN")
            return
        if self._depart is None:
            self._depart = dict(self._bras)           # posture de port, bras vides
        i, mode = self.essais[self.k]
        couleur, meta = self.placements[i]
        self._placer(couleur, meta)
        self._aller("ATTENTE")

    def _etat_attente(self) -> None:
        if self._duree() >= 1.0:
            self._commencer()

    def _commencer(self) -> None:
        """Démarre l'essai courant : relève les compteurs, puis lance le
        VLA (mode vla) ou la recherche par les réflexes (autres modes)."""
        i, mode = self.essais[self.k]
        couleur, _ = self.placements[i]
        self._t_essai, self._t_tenu, self._succes = self._horloge(), None, None
        self._c0 = dict(self._compteurs())
        self._m = {"inclinaison_max_deg": 0.0, "garde_min_avant_bras_m": math.inf,
                   "contacts_avant_bras": 0}
        if self.vla is not None:
            # Compteurs de l'essai : différence avec ce relevé ; la latence
            # maximale (un maximum, pas un cumul) repart de zéro.
            self._latence_totale = max(self._latence_totale,
                                       float(self.vla.stats.get("latence_max_s", 0.0)))
            self.vla.stats["latence_max_s"] = 0.0
            self._v0 = dict(self.vla.stats)
        self._aller("ESSAI")
        if mode == "vla":
            self.f.suspendre()                        # un seul pilote sur la base
            r = self.vla.demarrer_mission(consigne(couleur), piloter_base=True)
            if not r.ok:
                self._finir(False, f"VLA non démarré : {r.reason}")
            self.sous_etat = "POLITIQUE"
        else:
            self._recherches = 0
            self.sous_etat = "CHERCHER"

    def _finir(self, succes: bool, raison: str = "") -> None:
        """Note le résultat de l'essai, avec les refus du garde-fou et les
        arrêts de sécurité survenus pendant l'essai, puis passe à la
        libération. Un seul résultat par essai : les appels suivants sont
        ignorés."""
        if self._succes is not None:
            return
        i, mode = self.essais[self.k]
        couleur, meta = self.placements[i]
        c1 = self._compteurs()
        r = {"carton": i, "mode": mode, "couleur": couleur, "meta": meta,
             "succes": bool(succes), "raison": raison,
             "duree_s": round(self._horloge() - self._t_essai, 2),
             "refus_garde_fou": int(c1.get("refus", 0) - self._c0.get("refus", 0)),
             "arrets_securite": int(c1.get("arrets", 0) - self._c0.get("arrets", 0))}
        # Juge des contacts (vérité simulation, si le banc en fournit un).
        for cle in CONTACTS_JUGE:
            if cle in c1:
                r[cle] = int(c1[cle] - self._c0.get(cle, 0))
        if mode in MODES_VLA:
            r["vla"] = {k: (v if k == "latence_max_s" else v - self._v0.get(k, 0))
                        for k, v in self.vla.stats.items()}
        if self._mesures is not None:
            g = self._m["garde_min_avant_bras_m"]
            r["inclinaison_max_deg"] = round(self._m["inclinaison_max_deg"], 1)
            r["garde_min_avant_bras_cm"] = None if g == math.inf else round(100 * g, 1)
            r["contacts_avant_bras"] = self._m["contacts_avant_bras"]
        self.resultats.append(r)
        self._succes = succes
        n_ok = sum(x["succes"] for x in self.resultats if x["mode"] == mode)
        n = sum(1 for x in self.resultats if x["mode"] == mode)
        self._log(f"carton {i + 1}/{len(self.placements)} {couleur} {meta} — {mode} : "
                  f"{'RÉUSSI' if succes else 'raté'} en {r['duree_s']:.1f} s"
                  f"{' — ' + raison if raison else ''} ({mode} : {n_ok}/{n})"
                  f"{'  [garde-fou : %d refus]' % r['refus_garde_fou'] if r['refus_garde_fou'] else ''}")
        self._aller("LIBERER")

    def vla_total(self) -> Optional[Dict[str, float]]:
        """Compteurs du VLA cumulés sur toute l'évaluation (None sans VLA)."""
        if self.vla is None:
            return None
        t = dict(self.vla.stats)
        t["latence_max_s"] = max(self._latence_totale, float(t.get("latence_max_s", 0.0)))
        return t

    def _relever(self) -> None:
        """Mesures de la prise à ce pas (voir l'en-tête du module)."""
        try:
            v = self._mesures()
        except Exception:                 # noqa: BLE001 — une mesure manquée, pas l'essai
            return
        inc, garde = v.get("inclinaison_deg"), v.get("garde_avant_bras_m")
        if inc is not None and math.isfinite(inc) and self._souleve():
            self._m["inclinaison_max_deg"] = max(self._m["inclinaison_max_deg"], float(inc))
        if garde is not None and math.isfinite(garde):
            self._m["garde_min_avant_bras_m"] = min(self._m["garde_min_avant_bras_m"],
                                                    float(garde))
            if garde < 0.005:
                self._m["contacts_avant_bras"] += 1

    # ---- ESSAI -------------------------------------------------------------
    def _etat_essai(self) -> None:
        if self._mesures is not None:
            self._relever()
        if self._au_sol():
            # Ramassé aussitôt (voir l'en-tête du module) : laissé au sol, il
            # tenait le robot à l'arrêt de sécurité, bras figés, sans fin.
            self._retirer()
            self._log("carton tombé au sol — ramassé par l'opérateur")
            self._finir(False, "carton tombé au sol")
            return
        if self._horloge() - self._t_essai > self.duree_max_s:
            self._finir(False, f"délai de {self.duree_max_s:.0f} s dépassé ({self.sous_etat})")
            return
        if self.sous_etat == "POLITIQUE":
            self._essai_vla()
        else:
            self._essai_reflexes()

    def _tenu_assez(self) -> bool:
        """Vrai quand le carton est soulevé depuis au moins tenue_s secondes
        d'affilée ; le chronomètre repart de zéro dès qu'il ne l'est plus."""
        if self._souleve():
            if self._t_tenu is None:
                self._t_tenu = self._horloge()
            return self._horloge() - self._t_tenu >= self.tenue_s
        self._t_tenu = None
        return False

    def _essai_vla(self) -> None:
        """Le VLA pilote (mode vla, ou vla_prise après l'approche) : l'essai
        réussit quand le carton est tenu assez longtemps."""
        self.vla.tick()
        if self.vla.etat.name == "DEFAUT":
            self._finir(False, f"VLA en défaut : {self.vla.raison}")
            return
        if self._tenu_assez():
            self._finir(True)

    def _essai_reflexes(self) -> None:
        """Chemin des réflexes codés : CHERCHER, ACCOSTER, puis PRENDRE ; en
        mode vla_prise, la main passe au VLA une fois l'accostage fini."""
        se = self.sous_etat
        if se == "CHERCHER":
            if self._duree() < 0.3:                   # attendre une image fraîche
                return
            i, _ = self.essais[self.k]
            vus = self.f.chercher(self.placements[i][0])
            if not vus:
                self._recherches += 1
                if self._recherches >= 3:
                    self._finir(False, "carton non repéré par la caméra")
                else:
                    self._t_etat = self._horloge()
                return
            self._colis = vus[0]
            ok, raison = self.f.accoster(self._colis)
            if not ok:
                self._finir(False, f"accostage refusé : {raison}")
                return
            self.sous_etat = "ACCOSTER"
        elif se == "ACCOSTER":
            e = self.f.navigation()
            if e == "ARRIVE" and self.mode == "vla_prise":
                # Approche codée terminée, bras ouverts autour du carton :
                # le VLA prend la main sur les bras seuls, base à l'arrêt.
                self.f.suspendre(au_contact=True)
                i, _ = self.essais[self.k]
                r = self.vla.demarrer_mission(consigne(self.placements[i][0]),
                                              piloter_base=False)
                if not r.ok:
                    self._finir(False, f"VLA non démarré : {r.reason}")
                    return
                self.sous_etat = "POLITIQUE"
            elif e == "ARRIVE":
                ok, raison = self.f.prendre(self._colis)
                if not ok:
                    # La raison du service garde son détail (« prise
                    # refusée — hors de portée des bras… »).
                    self._finir(False, raison if raison.startswith(("prise refusée",
                                                                    "serrage refusé"))
                                else f"prise refusée : {raison}")
                    return
                self.sous_etat = "PRENDRE"
            elif e == "ECHEC":
                self._finir(False, "accostage en échec")
        elif se == "PRENDRE":
            # Même critère physique que pour le VLA : TENU ne suffit pas, le
            # carton doit aussi être soulevé depuis tenue_s.
            m = self.f.manipulation()
            if m == "TENU" and self._tenu_assez():
                self._finir(True)
            elif m == "ECHEC":
                manip = getattr(self.f, "manip", None)
                self._finir(False, getattr(manip, "raison", "") or "prise non confirmée")

    # ---- LIBERER -------------------------------------------------------------
    def _etat_liberer(self) -> None:
        if self._au_sol():
            # Lâché au sol pendant la libération (quand le robot desserre
            # sa prise, après un essai raté) : ramassé aussitôt. Une revue
            # des correctifs de l'audit de sécurité du 2026-10-10 a trouvé
            # ce cas : sinon le gardien arrêtait le robot, les bras restaient
            # figés et le délai de libération suspendu, d'où une évaluation
            # sans fin. Le gel des bras lui-même ne change pas.
            self._retirer()
            self._log("carton tombé au sol pendant la libération — ramassé par l'opérateur")
        if self.mode in MODES_VLA:
            self._liberer_vla()
        else:
            self._liberer_reflexes()

    def _liberer_reflexes(self) -> None:
        """Mode reflexes : reposer le carton si l'essai a réussi, sinon
        réinitialiser la manipulation ; 10 s au plus pour que les bras
        soient libres, puis retour au poste."""
        if self._entree:
            self._entree = False
            if self._succes:
                ok, _ = self.f.poser()
                if not ok:
                    self.f.reinitialiser_manipulation()
            else:
                self.f.reinitialiser_manipulation()
            return
        m = self.f.manipulation()
        if m == "LIBRE" or self._duree() > 10.0:
            if m != "LIBRE":
                self.f.reinitialiser_manipulation()
            self._aller("RETOUR")

    def _liberer_vla(self) -> None:
        """Bras ramenés à la posture de départ, à pas limité
        (delta_liberation par pas) : d'abord tout sauf l'ouverture (roulis
        des épaules), pour que le carton redescende vers la table, puis
        l'ouverture. Figés pendant un arrêt de sécurité, comme partout
        ailleurs. 8 s au plus par phase."""
        if self._entree:
            self._entree = False
            self.vla.terminer_mission()
            self.f.suspendre(au_contact=True)         # la sortie commencera par un recul
            self._halt()
            self.sous_etat = "DESCENDRE"
            return
        if self._arret():
            self._t_etat += self._dt                  # rien ne bouge, délai suspendu
            return
        ouvrir = self.sous_etat == "OUVRIR"
        reste = 0.0
        for j, cible in self._depart.items():
            if not ouvrir and j in self._roulis:
                continue
            e = cible - self._bras[j]
            self._bras[j] = cible if abs(e) <= self._delta else \
                self._bras[j] + math.copysign(self._delta, e)
            reste = max(reste, abs(e))
        if reste <= 1e-6 or self._duree() > 8.0:
            if ouvrir:
                self.f.reprendre_bras()               # la manipulation repart d'ici
                self._aller("RETOUR")
            else:
                self.sous_etat, self._t_etat = "OUVRIR", self._horloge()

    # ---- RETOUR ------------------------------------------------------------
    def _etat_retour(self) -> None:
        if self._entree:
            self._entree = False
            # Carton repris avant le recul (voir l'en-tête du module).
            self._retirer()
            self.f.aller_au_poste(self.poste)
            return
        e = self.f.navigation()
        if e in ("ARRIVE", "ECHEC") or self._duree() > 60.0:
            if e != "ARRIVE":
                self._log(f"retour au poste de prise : {e} — on continue")
            self.k += 1
            self._aller("PLACER")


# ---- bilan -----------------------------------------------------------------
def bilan(resultats: Sequence[Dict]) -> Dict:
    """Par mode : taux de réussite, durées des essais réussis, refus du
    garde-fou, arrêts de sécurité et raisons d'échec. Puis la comparaison
    appariée, carton par carton, entre les réflexes et chaque mode VLA :
    les deux modes ont vu les mêmes cartons."""
    out: Dict = {"par_mode": {}, "apparie": {}}
    for m in MODES:
        rs = [r for r in resultats if r["mode"] == m]
        if not rs:
            continue
        ok = [r for r in rs if r["succes"]]
        durees = [r["duree_s"] for r in ok]
        raisons: Dict[str, int] = {}
        for r in rs:
            if not r["succes"]:
                # Raison réduite à sa partie fixe (avant « ( » ou « : »),
                # pour regrouper les échecs de même nature.
                cle = r["raison"].split(" (")[0].split(" :")[0]
                raisons[cle] = raisons.get(cle, 0) + 1
        out["par_mode"][m] = {
            "essais": len(rs), "reussis": len(ok),
            "taux": round(len(ok) / len(rs), 3),
            "duree_moyenne_s": round(statistics.mean(durees), 1) if durees else None,
            "duree_mediane_s": round(statistics.median(durees), 1) if durees else None,
            "refus_garde_fou": sum(r["refus_garde_fou"] for r in rs),
            "arrets_securite": sum(r["arrets_securite"] for r in rs),
            "echecs": raisons,
        }
        if any(CONTACTS_JUGE[0] in r for r in rs):
            out["par_mode"][m]["contacts"] = {c: sum(r.get(c, 0) for r in rs)
                                              for c in CONTACTS_JUGE}
        if any("inclinaison_max_deg" in r for r in rs):
            gardes = [r["garde_min_avant_bras_cm"] for r in rs
                      if r.get("garde_min_avant_bras_cm") is not None]
            out["par_mode"][m].update({
                "inclinaison_max_deg": max(r.get("inclinaison_max_deg", 0.0) for r in rs),
                "garde_min_avant_bras_cm": min(gardes) if gardes else None,
                "contacts_avant_bras": sum(r.get("contacts_avant_bras", 0) for r in rs),
                "essais_avec_contact_avant_bras": sum(1 for r in rs
                                                      if r.get("contacts_avant_bras", 0)),
            })
    par_carton: Dict[int, Dict[str, bool]] = {}
    for r in resultats:
        par_carton.setdefault(r["carton"], {})[r["mode"]] = r["succes"]
    # Comparaison appariée : réflexes contre chaque mode VLA présent.
    for vm in MODES_VLA:
        paires = [v for v in par_carton.values() if "reflexes" in v and vm in v]
        if paires:
            out["apparie"][vm] = {
                "cartons": len(paires),
                "les_deux": sum(v[vm] and v["reflexes"] for v in paires),
                "vla_seul": sum(v[vm] and not v["reflexes"] for v in paires),
                "reflexes_seuls": sum(v["reflexes"] and not v[vm] for v in paires),
                "aucun": sum(not v[vm] and not v["reflexes"] for v in paires),
            }
    return out


def texte_bilan(b: Dict) -> str:
    """Le bilan en quelques lignes lisibles, pour le journal."""
    lignes = []
    for m, s in b["par_mode"].items():
        lignes.append(f"  {m:9s} : {s['reussis']}/{s['essais']} réussis ({100 * s['taux']:.0f} %)"
                      f", durée médiane {s['duree_mediane_s']} s, garde-fou {s['refus_garde_fou']}"
                      f" refus, {s['arrets_securite']} arrêts de sécurité")
        if "inclinaison_max_deg" in s:
            lignes.append(f"              carton penché au plus de {s['inclinaison_max_deg']}°"
                          f", avant-bras à {s['garde_min_avant_bras_cm']} cm du plateau au plus"
                          f" près, contact avec le plateau dans "
                          f"{s['essais_avec_contact_avant_bras']} essai(s)")
        if "contacts" in s:
            c = s["contacts"]
            lignes.append(f"              juge (vérité simulation) : {c['contacts_vrais']} vrais "
                          f"contacts, {c['contacts_manques']} manqués, {c['faux_contacts']} faux")
        if s["echecs"]:
            lignes.append("              échecs : " + ", ".join(
                f"{k} ×{v}" for k, v in sorted(s["echecs"].items(), key=lambda kv: -kv[1])))
    for vm, a in b.get("apparie", {}).items():
        lignes.append(f"  même carton, réflexes contre {vm} : les deux {a['les_deux']}, "
                      f"{vm} seul {a['vla_seul']}, réflexes seuls {a['reflexes_seuls']}, "
                      f"aucun {a['aucun']} (sur {a['cartons']})")
    return "\n".join(lignes)


def ecrire(resultats: Sequence[Dict], chemin: str,
           vla_total: Optional[Dict[str, float]] = None) -> Dict:
    """Écrit le bilan et le détail des essais en JSON dans `chemin` (avec
    les compteurs du VLA cumulés sur l'évaluation, s'ils sont donnés) ;
    rend le bilan."""
    b = bilan(resultats)
    sortie = {"bilan": b, "essais": list(resultats)}
    if vla_total is not None:
        sortie["vla_total"] = vla_total
    with open(chemin, "w", encoding="utf-8") as f:
        json.dump(sortie, f, ensure_ascii=False, indent=1)
    return b
