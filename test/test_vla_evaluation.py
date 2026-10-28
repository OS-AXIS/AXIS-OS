"""
Évaluation VLA contre réflexes (axis_os/vla/evaluation.py) — sans GPU : vrai
VLAService, connecteur GR00T, façade et physique factices.
"""
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.policy.vla_service import EtatVLA, VLAService  # noqa: E402
from axis_os.store.sdk.ai.gr00t_connector import GR00TAction  # noqa: E402
from axis_os.vla.evaluation import (  # noqa: E402
    EvaluateurPrises, bilan, ecrire, texte_bilan, tirer_placements,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


ARMS = [f"left_j{i}" for i in range(7)] + [f"right_j{i}" for i in range(7)]
ROULIS = ("left_j1", "right_j1")
DT = 0.02

section("1 — Cartons tirés d'avance, identiques pour les deux modes")
a, b = tirer_placements(10, 1000), tirer_placements(10, 1000)
check("Même graine -> même suite de cartons", a == b)
check("Autre graine (celle de la collecte) -> autre suite", a != tirer_placements(10, 3))
check("Bornes de la collecte respectées (x 3.55–4.45, recul 0–5 cm, ±12°)",
      all(3.55 <= m["x"] <= 4.45 and 0 <= m["dy"] <= 0.05 and abs(m["lacet"]) <= 12
          for _, m in a))


class Monde:
    """Horloge + physique minimale : le carton est soulevé quand les bras ont
    atteint la cible de la politique (left_j0 >= 0.45)."""
    def __init__(self):
        self.t = 0.0
        self.bras = {j: 0.0 for j in ARMS}
        self.arret = False
        self.au_sol = False
        self.refus, self.arrets = 0, 0
        self.places, self.retraits, self.halts = [], 0, 0

    def horloge(self):
        return self.t

    def souleve(self):
        return self.bras["left_j0"] >= 0.45


class FauxConnecteur:
    arm_joints = ARMS

    def __init__(self):
        self.connecte, self.resets, self.consignes, self.erreur = True, 0, [], None
        self.cible = 0.5

    def is_connected(self):
        return self.connecte

    def reset(self):
        self.resets += 1

    def predict_action(self, image, instruction, state):
        self.consignes.append(instruction)
        if self.erreur:
            raise RuntimeError(self.erreur)
        return GR00TAction(target_angles={j: self.cible for j in ARMS},
                           base_velocity=(0.1, 0.0, 0.0), latency_s=0.05)


class FauxLoco:
    def __init__(self, m):
        self.m, self.v = m, []

    def set_velocity(self, vx, vy, wz):
        self.v.append((vx, vy, wz))

    def halt(self):
        self.m.halts += 1


class FausseFacade:
    """Réflexes : repère en 0.3 s, accoste en 3 s, prend en 2 s (soulève les
    bras au moment de TENU)."""
    def __init__(self, m, repere=True):
        self.m, self.repere = m, repere
        self.fin_nav, self.fin_manip, self.manip = None, None, "LIBRE"
        self.suspensions, self.retours = [], 0

    def chercher(self, couleur):
        from axis_os.apps.tri_colis.logique import Colis
        return [Colis(couleur, (4.0, 2.1, 0.9), 0.5)] if self.repere else []

    def accoster(self, colis):
        self.fin_nav = self.m.t + 3.0
        return True, ""

    def aller_au_poste(self, poste):
        self.retours += 1
        self.fin_nav = self.m.t + 2.0
        return True, ""

    def navigation(self):
        return "ARRIVE" if self.fin_nav is not None and self.m.t >= self.fin_nav else "EN_ROUTE"

    def prendre(self, colis):
        self.prises = getattr(self, "prises", 0) + 1
        self.fin_manip, self.manip = self.m.t + 2.0, "EN_COURS"
        return True, ""

    def manipulation(self):
        if self.manip == "EN_COURS" and self.m.t >= self.fin_manip:
            self.manip = "TENU"
            self.m.bras["left_j0"] = 0.5
        if self.manip == "POSE" and self.m.t >= self.fin_manip:
            self.manip = "LIBRE"
            self.m.bras["left_j0"] = 0.0
        return self.manip

    def poser(self):
        self.manip, self.fin_manip = "POSE", self.m.t + 1.0
        return True, ""

    def reinitialiser_manipulation(self):
        self.manip = "LIBRE"
        self.m.bras["left_j0"] = 0.0

    def suspendre(self, au_contact=False):
        self.suspensions.append(au_contact)

    def reprendre_bras(self):
        self.reprises = getattr(self, "reprises", 0) + 1


def monter(n=3, modes=("reflexes", "vla"), repere=True, duree_max_s=45.0, avec_loco=False,
           mesures_prise=None, **kw_vla):
    m = Monde()
    con = FauxConnecteur()
    loco = FauxLoco(m)
    vla = VLAService(con, m.bras, loco, lire_etat_bras=lambda: dict(m.bras),
                     lire_image=lambda: "image", delta_max_par_joint={j: 0.04 for j in ARMS},
                     horloge=m.horloge, arret_securite=lambda: m.arret, journal=lambda s: None,
                     **kw_vla)
    vla.start()
    f = FausseFacade(m, repere)
    ev = EvaluateurPrises(
        tirer_placements(n, 1000), modes,
        placer=lambda c, meta: m.places.append((c, meta)),
        retirer=lambda: setattr(m, "retraits", m.retraits + 1),
        horloge=m.horloge, facade=f, vla=vla, bras=m.bras,
        carton_souleve=m.souleve, carton_au_sol=lambda: m.au_sol,
        arret_securite=lambda: m.arret, halt=lambda: loco.halt(),
        compteurs=lambda: {"refus": m.refus, "arrets": m.arrets},
        journal=lambda s: None, duree_max_s=duree_max_s, roulis=ROULIS,
        mesures_prise=mesures_prise)
    return m, con, vla, f, ev


def tourner(m, ev, t_max=2000.0, chaque=None):
    while not ev.termine and m.t < t_max:
        ev.pas()
        if chaque:
            chaque(m, ev)
        m.t += DT


section("2 — Chaque carton est essayé par les deux modes, dans l'ordre")
m, con, vla, f, ev = monter(3)
rendus = []
tourner(m, ev, chaque=lambda m_, ev_: rendus.append((ev_.mode, ev_.sous_etat))
        if ev_.veut_rendu(m_.t + DT) else None)
check(f"Fin après 6 essais ({len(ev.resultats)})", ev.termine and len(ev.resultats) == 6)
check("Ordre : carton 0 réflexes, carton 0 VLA, carton 1 réflexes…",
      [(r["carton"], r["mode"]) for r in ev.resultats]
      == [(0, "reflexes"), (0, "vla"), (1, "reflexes"), (1, "vla"), (2, "reflexes"), (2, "vla")])
check("Les deux modes voient EXACTEMENT les mêmes cartons",
      m.places[0::2] == m.places[1::2] and len(m.places) == 6)
check(f"Tous réussis ({[r['succes'] for r in ev.resultats]})",
      all(r["succes"] for r in ev.resultats))
check("Consigne du VLA en anglais, d'après la couleur du carton",
      set(con.consignes) <= {"pick up the red box", "pick up the blue box"}
      and con.consignes[0] == f"pick up the {'red' if m.places[1][0] == 'rouge' else 'blue'} box")
check("Serveur GR00T remis à zéro à chaque essai VLA", con.resets == 3)
check("La façade lâche la base au début et à la fin de chaque essai VLA (fin : au contact)",
      f.suspensions == [False, True] * 3)
check("Rendu demandé : en recherche (réflexes), et ~10 fois/s pendant le VLA",
      {s for md, s in rendus if md == "reflexes"} == {"CHERCHER"}
      and 0 < sum(1 for md, _ in rendus if md == "vla") < 0.25 * vla.stats["ticks"] + 10)
check("Un retour au poste par essai", f.retours == 6)

section("3 — Critère de réussite : soulevé, entre les mains, 1 s d'affilée")
r_vla = [r for r in ev.resultats if r["mode"] == "vla"][0]
check(f"Durée VLA cohérente (bras 0 -> 0.45 à 1 rad/s, puis 1 s tenu : {r_vla['duree_s']} s)",
      1.3 <= r_vla["duree_s"] <= 1.8)
m, con, vla, f, ev = monter(1, modes=("vla",), duree_max_s=10.0)
bascule = [0]


def clignote(m_, ev_):
    # Carton soulevé 0.5 s, retombé 0.1 s, en boucle : jamais 1 s d'affilée.
    if ev_.etat == "ESSAI":
        bascule[0] += 1
        m_.bras["left_j0"] = 0.5 if bascule[0] % 30 < 25 else 0.0


tourner(m, ev, chaque=clignote)
check("Un carton qui retombe sans cesse n'est PAS une réussite (délai dépassé)",
      not ev.resultats[0]["succes"] and "délai" in ev.resultats[0]["raison"])

section("4 — Échecs : défaut du VLA, carton au sol, carton non repéré")
m, con, vla, f, ev = monter(1, modes=("vla",))
con.erreur = "serveur injoignable"
tourner(m, ev)
check(f"Serveur muet -> essai raté, raison donnée ({ev.resultats[0]['raison']!r})",
      not ev.resultats[0]["succes"] and "défaut" in ev.resultats[0]["raison"]
      and ev.resultats[0]["duree_s"] < 1.0)
m, con, vla, f, ev = monter(1, modes=("vla",))
con.cible = 0.3                                    # n'y arrive jamais
tourner(m, ev, chaque=lambda m_, ev_: setattr(m_, "au_sol", m_.t > 2.0 and ev_.etat == "ESSAI"))
check("Carton au sol -> raté", ev.resultats[0]["raison"] == "carton tombé au sol")
m, con, vla, f, ev = monter(1, modes=("reflexes",), repere=False)
tourner(m, ev)
check("Réflexes : carton non repéré par la caméra -> raté (pas de nouvel essai : même carton "
      "pour les deux modes)", ev.resultats[0]["raison"] == "carton non repéré par la caméra"
      and len(m.places) == 1)
con2 = FauxConnecteur()
con2.connecte = False
m, con, vla, f, ev = monter(1, modes=("vla",))
con.connecte = False
tourner(m, ev)
check("Connecteur non connecté -> raté sans bouger les bras",
      "non démarré" in ev.resultats[0]["raison"] and m.bras["left_j0"] == 0.0)

section("5 — Après le VLA : bras ramenés pas à pas, ouverture en dernier, gel de sécurité")
m, con, vla, f, ev = monter(1, modes=("vla",))
depart = None
pas_max, ordre, gele = [0.0], [], [True]
prec = {}


def surveiller(m_, ev_):
    global depart
    if depart is None and ev_._depart is not None:
        depart = dict(ev_._depart)
    if ev_.etat == "LIBERER" and not ev_._entree:
        for j in ARMS:
            if j in prec:
                pas_max[0] = max(pas_max[0], abs(m_.bras[j] - prec[j]))
        ordre.append((ev_.sous_etat, m_.bras["left_j0"], m_.bras["left_j1"]))
        # Arrêt de sécurité 0.5 s au milieu : rien ne doit bouger.
        m_.arret = 30 <= len(ordre) < 55
        if 31 <= len(ordre) < 55 and any(m_.bras[j] != prec.get(j) for j in ARMS):
            gele[0] = False
    prec.update(m_.bras)


tourner(m, ev, chaque=surveiller)
check(f"Pas maximal par tick ≤ 0.02 rad ({pas_max[0]:.3f})", pas_max[0] <= 0.02 + 1e-9)
desc = [o for o in ordre if o[0] == "DESCENDRE"]
check("Phase 1 : tout redescend SAUF l'ouverture (roulis tenu)",
      desc and all(abs(o[2] - desc[0][2]) < 1e-9 for o in desc) and desc[-1][1] < desc[0][1])
check("Phase 2 : ouverture, jusqu'à la posture de départ exacte",
      all(abs(m.bras[j] - depart[j]) < 1e-9 for j in ARMS))
check(f"Bras figés pendant l'arrêt de sécurité ({len(ordre)} ticks de libération, dont 24 gelés)", gele[0] and len(ordre) > 60)
check("VLA rendu inactif, base à l'arrêt", vla.etat is EtatVLA.INACTIF and m.halts >= 1)

section("5b — Mode vla_prise : approche codée, puis le VLA fait SEUL la prise, base immobile")
m, con, vla, f, ev = monter(2, modes=("reflexes", "vla_prise"))
vitesses_politique, rendus_p = [], []
loco_ = vla._loco


def suivre_p(m_, ev_):
    if ev_.etat == "ESSAI" and ev_.sous_etat == "POLITIQUE":
        vitesses_politique.append(len(loco_.v))
    if ev_.veut_rendu(m_.t + DT):
        rendus_p.append((ev_.mode, ev_.sous_etat))


tourner(m, ev, chaque=suivre_p)
rp = [r for r in ev.resultats if r["mode"] == "vla_prise"]
check(f"2 essais vla_prise réussis ({[(r['succes'], r['duree_s']) for r in rp]})",
      len(rp) == 2 and all(r["succes"] for r in rp))
check("Approche par les réflexes : recherche + accostage avant la politique (≥ 3.3 s)",
      all(r["duree_s"] >= 3.3 for r in rp))
check("La politique n'envoie AUCUNE vitesse de base (base tenue à l'arrêt)",
      vitesses_politique and len(set(vitesses_politique)) == 1)
check("La prise codée n'est PAS appelée en mode vla_prise (2 prises : celles des réflexes)",
      f.prises == 2)
check("Façade suspendue au contact à l'arrivée, puis manipulation resynchronisée après",
      f.suspensions.count(True) >= 2 and f.reprises == 2)
check("Rendu : recherche, puis inférences de la politique",
      {s_ for md, s_ in rendus_p if md == "vla_prise"} == {"CHERCHER", "POLITIQUE"})
bl_p = bilan(ev.resultats)
check("Bilan apparié réflexes contre vla_prise", bl_p["apparie"]["vla_prise"]["cartons"] == 2)

section("6 — Garde-fou et sécurité comptés PAR ESSAI ; bilan apparié")
m, con, vla, f, ev = monter(2)


def incidents(m_, ev_):
    if ev_.etat == "ESSAI" and ev_.mode == "vla" and ev_.k == 1 and m_.refus == 0:
        m_.refus, m_.arrets = 3, 1


tourner(m, ev, chaque=incidents)
check("Refus du garde-fou et arrêts attribués au bon essai",
      [r["refus_garde_fou"] for r in ev.resultats] == [0, 3, 0, 0]
      and [r["arrets_securite"] for r in ev.resultats] == [0, 1, 0, 0])
res = [{"carton": 0, "mode": "reflexes", "succes": True, "raison": "", "duree_s": 20.0,
        "refus_garde_fou": 0, "arrets_securite": 2},
       {"carton": 0, "mode": "vla", "succes": False, "raison": "délai de 45 s dépassé (POLITIQUE)",
        "duree_s": 45.0, "refus_garde_fou": 1, "arrets_securite": 0},
       {"carton": 1, "mode": "reflexes", "succes": False, "raison": "carton non repéré par la caméra",
        "duree_s": 1.0, "refus_garde_fou": 0, "arrets_securite": 0},
       {"carton": 1, "mode": "vla", "succes": True, "raison": "", "duree_s": 12.0,
        "refus_garde_fou": 0, "arrets_securite": 0},
       {"carton": 2, "mode": "reflexes", "succes": True, "raison": "", "duree_s": 24.0,
        "refus_garde_fou": 0, "arrets_securite": 0},
       {"carton": 2, "mode": "vla", "succes": True, "raison": "", "duree_s": 14.0,
        "refus_garde_fou": 0, "arrets_securite": 0}]
bl = bilan(res)
check(f"Taux par mode ({bl['par_mode']['reflexes']['taux']}, {bl['par_mode']['vla']['taux']})",
      bl["par_mode"]["reflexes"]["reussis"] == 2 and bl["par_mode"]["vla"]["reussis"] == 2)
check("Durée médiane des seules réussites (réflexes 22 s, VLA 13 s)",
      bl["par_mode"]["reflexes"]["duree_mediane_s"] == 22.0
      and bl["par_mode"]["vla"]["duree_mediane_s"] == 13.0)
check("Comparaison carton par carton : 1 les deux, 1 VLA seul, 1 réflexes seuls",
      bl["apparie"]["vla"] == {"cartons": 3, "les_deux": 1, "vla_seul": 1, "reflexes_seuls": 1,
                        "aucun": 0})
check("Échecs regroupés par cause", bl["par_mode"]["vla"]["echecs"] == {"délai de 45 s dépassé": 1})
d = Path(tempfile.mkdtemp())
ecrire(res, str(d / "eval.json"))
check("Résultats écrits en JSON", (d / "eval.json").exists())
print(texte_bilan(bl))

section("7 — Compteurs du VLA propres à chaque essai, total à part ; mesures de la prise")
relevé = {}
m, con, vla, f, ev = monter(2, mesures_prise=lambda: relevé["f"]())


def mesures_7():
    # Carton soulevé : 8° (22° pour le carton 1 aux réflexes) ; posé : 40°,
    # qui ne compte pas. Avant-bras sur le plateau pendant le carton 0 aux
    # réflexes (3 mm), à 4 cm sinon.
    lev = m.souleve()
    return {"inclinaison_deg": (22.0 if ev.k == 2 else 8.0) if lev else 40.0,
            "garde_avant_bras_m": 0.003 if (ev.k == 0 and lev) else 0.04}


relevé["f"] = mesures_7
tourner(m, ev)
inf = [r["vla"]["inferences"] for r in ev.resultats if r["mode"] == "vla"]
total = ev.vla_total()
check(f"Inférences du VLA par essai ({inf}) : leur somme est le total ({total['inferences']}), "
      f"aucune n'est un cumul", sum(inf) == total["inferences"]
      and all(0 < x < total["inferences"] for x in inf))
check("… de même pour les pas du VLA ; la latence maximale est celle de l'essai",
      sum(r["vla"]["ticks"] for r in ev.resultats if r["mode"] == "vla") == total["ticks"]
      and all(r["vla"]["latence_max_s"] <= total["latence_max_s"]
              for r in ev.resultats if r["mode"] == "vla"))
r0, r2 = ev.resultats[0], ev.resultats[2]
check("Inclinaison maximale relevée carton soulevé seulement (8° puis 22°, pas 40°)",
      r0["inclinaison_max_deg"] == 8.0 and r2["inclinaison_max_deg"] == 22.0)
check("Avant-bras sur le plateau : garde minimale et pas en contact, par essai",
      r0["garde_min_avant_bras_cm"] == 0.3 and r0["contacts_avant_bras"] > 0
      and r2["garde_min_avant_bras_cm"] == 4.0 and r2["contacts_avant_bras"] == 0)
b7 = bilan(ev.resultats)["par_mode"]["reflexes"]
check("Bilan : inclinaison max 22°, contact des avant-bras dans 1 essai sur 2",
      b7["inclinaison_max_deg"] == 22.0 and b7["essais_avec_contact_avant_bras"] == 1
      and b7["garde_min_avant_bras_cm"] == 0.3)
check("… et dans le texte du bilan", "penché au plus de 22.0°" in texte_bilan(bilan(ev.resultats)))
ecrire(ev.resultats, str(d / "eval7.json"), vla_total=total)
j7 = json.loads((d / "eval7.json").read_text(encoding="utf-8"))
check("JSON : total du VLA à part, compteurs par essai dans chaque essai",
      j7["vla_total"]["inferences"] == total["inferences"]
      and [e["vla"]["inferences"] for e in j7["essais"] if e["mode"] == "vla"] == inf)
m, con, vla, f, ev = monter(1, modes=("reflexes",))
f.prendre = lambda colis: (False, "prise refusée — hors de portée des bras : objet à 0.56 m "
                                  "devant le bassin et +0.18 m en hauteur")
tourner(m, ev)
check("Échec de la prise : la raison du service, détaillée et regroupée",
      bilan(ev.resultats)["par_mode"]["reflexes"]["echecs"]
      == {"prise refusée — hors de portée des bras": 1})

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
