"""
Évaluation du VLA contre les réflexes codés : MÊMES cartons, MÊME robot.

    pour chaque carton i (position, recul, rotation tirés d'avance) :
        pour chaque mode (« reflexes », « vla ») :
            PLACER   l'opérateur pose le carton i, à l'identique ;
            ATTENTE  1 s, le carton se pose ;
            ESSAI    reflexes : chercher (caméra + OWL-ViT), accoster, prendre —
                               exactement le chemin de la collecte ;
                     vla_prise: les réflexes cherchent et accostent (bras
                               ouverts autour du carton), puis le VLA fait
                               SEUL la prise, base immobile ;
                     vla      : la politique GR00T reçoit l'image de la tête,
                               l'état des bras et la consigne (« pick up the
                               red box ») et pilote BRAS et BASE. Rien d'autre :
                               ni détection, ni navigation, ni geste codé ;
            LIBERER  reflexes : reposer le carton ; vla : bras ramenés pas à pas
                     à la posture de départ (d'abord sans ouvrir, puis en
                     ouvrant) ;
            RETOUR   retour au poste de prise, l'opérateur retire le carton.

RÉUSSITE, le même critère physique pour les deux modes : le carton a QUITTÉ la
table, entre les mains (`carton_souleve()`), pendant `tenue_s` d'affilée. Pour
les réflexes, c'est déjà ce que vérifie la manipulation avant de dire TENU.

ÉCHEC : délai dépassé, carton au sol, VLA en défaut (serveur muet, erreurs
répétées), refus de la façade. La raison est notée.

AUCUN PASSE-DROIT. En mode vla, les bras passent par VLAService (pas limité,
gel sur arrêt de sécurité, échéance de fraîcheur) puis par l'adaptateur (butées)
et le JointSafetyGuard ; la base par LocomotionService (vitesses bornées) et le
gardien de proximité. Les refus du garde-fou et les arrêts de sécurité sont
comptés PAR ESSAI : un VLA qui « réussit » en se faisant refuser des consignes
n'est pas un VLA qui réussit.

Les cartons sont tirés avec une graine DIFFÉRENTE de celle de la collecte : le
modèle est jugé sur des positions qu'il n'a jamais vues.
"""
from __future__ import annotations

import json
import math
import random
import statistics
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from axis_os.vla.collecte import consigne

MODES = ("reflexes", "vla", "vla_prise")
MODES_VLA = ("vla", "vla_prise")


def tirer_placements(n: int, graine: int, couleurs: Sequence[str] = ("rouge", "bleu"),
                     x: Tuple[float, float] = (3.55, 4.45), dy: Tuple[float, float] = (0.0, 0.05),
                     lacet_deg: float = 12.0) -> List[Tuple[str, Dict]]:
    """Les n cartons de l'évaluation, tirés D'AVANCE (mêmes bornes que la
    collecte) : chaque mode voit exactement la même suite."""
    rng = random.Random(graine)
    out = []
    for _ in range(n):
        out.append((rng.choice(list(couleurs)),
                    {"x": round(rng.uniform(*x), 3), "dy": round(rng.uniform(*dy), 3),
                     "lacet": round(rng.uniform(-lacet_deg, lacet_deg), 1)}))
    return out


class EvaluateurPrises:
    def __init__(self, placements: Sequence[Tuple[str, Dict]], modes: Sequence[str],
                 placer: Callable[[str, Dict], None], retirer: Callable[[], None],
                 horloge: Callable[[], float], facade, vla, bras: Dict[str, float],
                 carton_souleve: Callable[[], bool], carton_au_sol: Callable[[], bool],
                 arret_securite: Callable[[], bool], halt: Callable[[], None],
                 compteurs: Callable[[], Dict[str, float]],
                 journal: Callable[[str], None] = print, poste_prise: str = "prise",
                 duree_max_s: float = 45.0, tenue_s: float = 1.0,
                 delta_liberation: float = 0.02, roulis: Sequence[str] = (
                     "left_shoulder_roll_joint", "right_shoulder_roll_joint")):
        """
        placer(couleur, meta) / retirer()  : l'opérateur simulé.
        facade      : FacadeServices (mode reflexes, et retour au poste).
        vla         : VLAService (mode vla) — None si seul le mode reflexes tourne.
        bras        : adapt.pose_bras ; sa valeur au premier essai est la
                      posture de départ, celle où la manipulation croit les bras.
        compteurs   : () -> {"refus": …, "arrets": …} cumulés (garde-fou,
                      gardien) ; l'évaluateur en fait la différence par essai.
        halt        : () -> base à l'arrêt en équilibre.
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
        self.k = 0                                    # essai courant
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

    # ---- interface avec la boucle de simulation ------------------------
    @property
    def termine(self) -> bool:
        return self.etat == "FIN"

    @property
    def mode(self) -> str:
        return self.essais[min(self.k, len(self.essais) - 1)][1]

    def veut_rendu(self, t_prochain: float) -> bool:
        """La boucle rend la caméra de tête au pas suivant si vrai."""
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
        t = self._horloge()
        self._dt = 0.0 if self._t_pas is None else t - self._t_pas
        self._t_pas = t
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
        i, mode = self.essais[self.k]
        couleur, _ = self.placements[i]
        self._t_essai, self._t_tenu, self._succes = self._horloge(), None, None
        self._c0 = dict(self._compteurs())
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
        if mode in MODES_VLA:
            r["vla"] = dict(self.vla.stats)
        self.resultats.append(r)
        self._succes = succes
        n_ok = sum(x["succes"] for x in self.resultats if x["mode"] == mode)
        n = sum(1 for x in self.resultats if x["mode"] == mode)
        self._log(f"carton {i + 1}/{len(self.placements)} {couleur} {meta} — {mode} : "
                  f"{'RÉUSSI' if succes else 'raté'} en {r['duree_s']:.1f} s"
                  f"{' — ' + raison if raison else ''} ({mode} : {n_ok}/{n})"
                  f"{'  [garde-fou : %d refus]' % r['refus_garde_fou'] if r['refus_garde_fou'] else ''}")
        self._aller("LIBERER")

    # ---- ESSAI -------------------------------------------------------------
    def _etat_essai(self) -> None:
        if self._au_sol():
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
        if self._souleve():
            if self._t_tenu is None:
                self._t_tenu = self._horloge()
            return self._horloge() - self._t_tenu >= self.tenue_s
        self._t_tenu = None
        return False

    def _essai_vla(self) -> None:
        self.vla.tick()
        if self.vla.etat.name == "DEFAUT":
            self._finir(False, f"VLA en défaut : {self.vla.raison}")
            return
        if self._tenu_assez():
            self._finir(True)

    def _essai_reflexes(self) -> None:
        se = self.sous_etat
        if se == "CHERCHER":
            if self._duree() < 0.3:                   # une image fraîche rendue
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
                # le VLA prend la main sur les BRAS seuls, base à l'arrêt.
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
                    self._finir(False, f"prise refusée : {raison}")
                    return
                self.sous_etat = "PRENDRE"
            elif e == "ECHEC":
                self._finir(False, "accostage en échec")
        elif se == "PRENDRE":
            m = self.f.manipulation()
            if m == "TENU" and self._tenu_assez():
                self._finir(True)
            elif m == "ECHEC":
                self._finir(False, "prise non confirmée")

    # ---- LIBERER -------------------------------------------------------------
    def _etat_liberer(self) -> None:
        if self.mode in MODES_VLA:
            self._liberer_vla()
        else:
            self._liberer_reflexes()

    def _liberer_reflexes(self) -> None:
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
        """Bras ramenés à la posture de départ, À PAS LIMITÉ : d'abord tout
        sauf l'ouverture (le carton redescend vers la table), puis l'ouverture.
        Figés pendant un arrêt de sécurité, comme partout ailleurs."""
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
            self.f.aller_au_poste(self.poste)
            return
        e = self.f.navigation()
        if e in ("ARRIVE", "ECHEC") or self._duree() > 60.0:
            if e != "ARRIVE":
                self._log(f"retour au poste de prise : {e} — on continue")
            self._retirer()
            self.k += 1
            self._aller("PLACER")


# ---- bilan -----------------------------------------------------------------
def bilan(resultats: Sequence[Dict]) -> Dict:
    """Par mode : taux de réussite, durées, refus, arrêts ; et la comparaison
    APPARIÉE, carton par carton (les deux modes ont vu les mêmes)."""
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
    par_carton: Dict[int, Dict[str, bool]] = {}
    for r in resultats:
        par_carton.setdefault(r["carton"], {})[r["mode"]] = r["succes"]
    # Comparaison appariée : réflexes contre CHAQUE mode VLA présent.
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
    lignes = []
    for m, s in b["par_mode"].items():
        lignes.append(f"  {m:9s} : {s['reussis']}/{s['essais']} réussis ({100 * s['taux']:.0f} %)"
                      f", durée médiane {s['duree_mediane_s']} s, garde-fou {s['refus_garde_fou']}"
                      f" refus, {s['arrets_securite']} arrêts de sécurité")
        if s["echecs"]:
            lignes.append("              échecs : " + ", ".join(
                f"{k} ×{v}" for k, v in sorted(s["echecs"].items(), key=lambda kv: -kv[1])))
    for vm, a in b.get("apparie", {}).items():
        lignes.append(f"  même carton, réflexes contre {vm} : les deux {a['les_deux']}, "
                      f"{vm} seul {a['vla_seul']}, réflexes seuls {a['reflexes_seuls']}, "
                      f"aucun {a['aucun']} (sur {a['cartons']})")
    return "\n".join(lignes)


def ecrire(resultats: Sequence[Dict], chemin: str) -> Dict:
    b = bilan(resultats)
    with open(chemin, "w", encoding="utf-8") as f:
        json.dump({"bilan": b, "essais": list(resultats)}, f, ensure_ascii=False, indent=1)
    return b
