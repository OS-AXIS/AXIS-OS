"""
Les lignes réelles de l'essai GPU T1 du 5 novembre 2026, 21 h 57 (une prise,
deux mains ; t1_prise_20261105_2157.csv réduit à ses colonnes utiles),
rejouées : prise parfaite, puis le carton glisse au port, l'OS le voit
(« le carton ne pèse plus sur les mains (3.5 N, 11.3 au levage) »), arrêt
propre.

Ce que disent les lignes :
  1. le serrage, fait pour 8 N estimés (6,3 N réels), ne laissait aucune
     marge au frottement : μ (N_G + N_D) / poids ≈ 0,9 avec μ = 0,8 ;
  2. au port, les faces mesurées bougent de ±2 cm, mais ENSEMBLE : leur écart
     ne varie que de 2 mm ; la marche balance mains et carton, les mains ne
     s'écartent pas ;
  3. la consigne de la main droite, elle, s'est desserrée de 5 mm : le tangage
     du poignet droit porte jusqu'à 2,45 N·m au port (2,74 au levage), surtout
     du poids ; à chaque pointe au-delà de son plafond (2,5 N·m, la moitié des
     5 N·m de l'URDF), _garder_sous_la_limite desserre d'un pas, sans jamais
     resserrer : 1,5 N de serrage perdus pour 0,07 N·m de moins au poignet ;
  4. « carton contre le torse » à 3 N : le carton était à 33 cm du bassin
     (face arrière), le torse attendu à 10 : hors d'atteinte, une traction.

Avec les corrections (service.py, prise.py) : serrage visé d'après le poids
(securite_serrage × poids / 2 μ + marge, atteint avant de ramener), consigne
gardée au port, pas d'effort vers un torse hors d'atteinte. Rejoué : les
couples des lignes plus ce que le serrage ajoute (Jᵀ, au point de contact),
les forces du juge plus le serrage ajouté et la consigne rendue.

Ce que le rejeu montre aussi : le serrage, au point de contact 11 cm le long
de la main, charge les POIGNETS (lacet surtout, ~0,15 N·m par newton). Avec
la référence de poids d'avant (le plus grand lu depuis le levage, 11,1 N,
9,8 en vrai), il fallait 13,9 N par main, au-delà de ce que les poignets
donnent dans leurs plafonds (2,5 N·m, inchangés) : « trop lourd », de peu.
Avec le poids pesé une fois, bras immobiles en haut du levage (essai de
18 h 21 : test_rejeu_port_t1821.py ; ici, les dernières lectures du levage :
10,0 N, à 2 % du vrai), il en faut 12,5 : les poignets les donnent, le
carton est porté, marge de frottement au-dessus de 2 pendant tout le port
rejoué.
"""
from types import SimpleNamespace
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import ManipulationService  # noqa: E402

FAIL = 0
R = ReglagesPrise()
CSV = Path(__file__).resolve().parent / "donnees" / "t1_prise_20261105_2157.csv"
ART = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
       "wrist_yaw")
# Plafond de chaque articulation (_limite_effort) : la moitié de l'effort de
# l'URDF (poignets en tangage et lacet : 5 N·m, les autres : 25), 10 au plus.
PLAFOND = {a: min(R.effort_max_nm, R.part_couple_garde * (5.0 if a in ("wrist_pitch", "wrist_yaw")
                                                            else 25.0)) for a in ART}
# Journal de l'essai, à l'arrêt « contre le torse » : « bleu réel (0.428,
# -0.006, 0.150) » (repère du bassin) ; carton de 20 cm de profondeur.
CARTON_X, PROFONDEUR = 0.428, 0.20


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def f(r, c):
    return float(r[c]) if r[c] else np.nan


LIGNES = list(csv.DictReader(open(CSV)))
PAS = {}                                   # t -> {main: ligne}
for r in LIGNES:
    PAS.setdefault(f(r, "t_s"), {})[r["main"]] = r
PAS = {t: d for t, d in sorted(PAS.items()) if len(d) == 2}
PHASE = {t: d["left"]["phase"] for t, d in PAS.items()}
TENU = [t for t in PAS if PHASE[t] == "TENU"]
LEVAGE = [t for t in PAS if PHASE[t] == "LEVAGE"]
HAUT = LEVAGE[-10:]                        # en haut du levage, avant de ramener
POIDS = np.nanmax([f(r, "poids_os_n") for r in LIGNES])     # référence d'avant : le plus grand
# Le poids pesé (service.py, en haut du levage, bras immobiles : médiane d'une
# fenêtre de filtre_force × 3 lectures) ; ici, les dernières du levage.
PESEE = float(np.median([f(PAS[t]["left"], "poids_os_n")
                         for t in [t for t in PAS if PAS[t]["left"]["phase"] == "LEVAGE"]
                         [-3 * R.filtre_force:]]))
MU, S = R.frottement_serrage, R.securite_serrage


def somme_juge(t):
    return f(PAS[t]["left"], "force_juge_n") + f(PAS[t]["right"], "force_juge_n")


section("1. Le constat : un serrage sans marge, le carton glisse au port")
n_haut = {c: np.mean([f(PAS[t][c], "force_juge_n") for t in HAUT]) for c in ("left", "right")}
check(f"Poids mesuré par l'OS : {np.nanmax([f(PAS[t]['left'], 'poids_os_n') for t in LEVAGE]):.1f} "
      f"N en haut du levage, {POIDS:.1f} au plus (carton de 1 kg : 9,8)", 10.0 <= POIDS <= 11.5)
check(f"Référence de poids de l'OS : le plus grand lu depuis le levage, {POIDS:.1f} N "
      f"({100 * (POIDS / 9.81 - 1):+.0f} % du vrai) ; pesé, bras presque immobiles en haut du "
      f"levage (médiane des {3 * R.filtre_force} dernières lectures) : {PESEE:.1f} N, à 5 % "
      f"du vrai", POIDS > 1.1 * 9.81 and abs(PESEE - 9.81) <= 0.05 * 9.81)
check(f"Serrage réel en haut du levage (juge) : G {n_haut['left']:.2f} N, D {n_haut['right']:.2f} N "
      f"— soit {POIDS / 2:.1f} N de poids par main à tenir par le seul frottement",
      all(6.0 <= v <= 6.8 for v in n_haut.values()))
marges = [MU * somme_juge(t) / POIDS for t in TENU]
check(f"Marge de frottement au port, μ (N_G + N_D) / poids, μ = {MU} : {min(marges):.2f} à "
      f"{max(marges):.2f}, loin du coefficient de sécurité ({S:.0f})", max(marges) < 1.2)
glisse = max(max(f(PAS[t][c], "contact_rel_z_m") for t in TENU)
             - f(PAS[TENU[0]][c], "contact_rel_z_m") for c in ("left", "right"))
check(f"… et le carton glisse : point de contact {100 * glisse:.1f} cm plus haut sur sa face "
      f"pendant le port (le carton descend dans les mains)", glisse >= 0.02)

section("2. Pourquoi : balancement, consigne desserrée, traction vers un torse absent")
ecarts = [f(PAS[t]["left"], "face_mesuree_m") - f(PAS[t]["right"], "face_mesuree_m") for t in TENU]
course = min(max(f(PAS[t][c], "face_mesuree_m") for t in TENU)
             - min(f(PAS[t][c], "face_mesuree_m") for t in TENU) for c in ("left", "right"))
check(f"Faces mesurées : chacune bouge de {100 * course:.1f} cm au moins, leur écart de "
      f"{1000 * (max(ecarts) - min(ecarts)):.0f} mm : les mains ne s'écartent pas, la marche les "
      f"balance avec le carton (d'où l'écart des DEUX mains au port, ReglagesPrise.ecart_face_port_m)",
      course >= 0.015 and max(ecarts) - min(ecarts) <= 0.003)
fc = {c: [f(PAS[t][c], "face_commandee_m") for t in TENU] for c in ("left", "right")}
desserre = abs(fc["right"][-1] - fc["right"][0])
check(f"Consigne : main D desserrée de {1000 * desserre:.0f} mm pendant le port, main G "
      f"{1000 * abs(fc['left'][-1] - fc['left'][0]):.0f} mm",
      0.004 <= desserre <= 0.006 and abs(fc["left"][-1] - fc["left"][0]) < 0.001)
MODELE = ModeleBras().au_contact(R.contact_main_m)


def jac_n(t, cote):
    """Couples (N·m) que 1 N de serrage de plus ajoute au bras `cote`, à la
    posture de la ligne : Jᵀ n au point de contact, n vers le carton."""
    r = PAS[t][cote]
    chaine = MODELE.gauche if cote == "left" else MODELE.droite
    joints = [f"{cote}_{a}_joint" for a in ART]
    q = {j: f(r, f"angle_{a}_rad") for j, a in zip(joints, ART)}
    n = np.array([0.0, -1.0 if cote == "left" else 1.0, 0.0])
    return chaine.jacobienne(q, joints).T @ n


k = ART.index("wrist_pitch")
pic = max(TENU, key=lambda t: abs(f(PAS[t]["right"], "couple_wrist_pitch_nm")))
tau_pic = abs(f(PAS[pic]["right"], "couple_wrist_pitch_nm"))
part = abs(jac_n(pic, "right")[k] * f(PAS[pic]["right"], "force_juge_n"))
raideur_d = f(PAS[pic]["right"], "force_juge_n") / abs(
    f(PAS[pic]["right"], "face_mesuree_m") - f(PAS[pic]["right"], "face_commandee_m"))
soulage = abs(jac_n(pic, "right")[k]) * raideur_d * desserre
check(f"… tangage du poignet D jusqu'à {tau_pic:.2f} N·m au port (plafond "
      f"{PLAFOND['wrist_pitch']}), dont {part:.2f} du serrage : surtout le poids ; les 5 mm de "
      f"desserrage l'allègent de {soulage:.2f} N·m, et ôtent {raideur_d * desserre:.1f} N au serrage",
      tau_pic > PLAFOND["wrist_pitch"] - 0.1 and part < 0.5 * tau_pic and soulage < 0.1)
arriere = CARTON_X - PROFONDEUR / 2.0
check(f"RAMENER : arrêté « contre le torse » à {max(f(PAS[t]['left'], 'poussee_os_n') for t in PAS if PHASE[t] == 'RAMENER'):.1f} "
      f"N, face arrière du carton à {100 * arriere:.0f} cm du bassin, le torse attendu à "
      f"{100 * R.distance_torse_m:.0f} : hors d'atteinte, c'était une traction",
      arriere - R.distance_torse_m > 0.15)
m = ManipulationService.__new__(ManipulationService)
m.r, m.charge = R, (POIDS, 3.0)
m._torse_possible = True
avant = ManipulationService._appui_torse(m)
m._torse_possible = arriere <= R.distance_torse_m
check("Désormais : torse hors d'atteinte, aucun « contre le torse » sur un effort vers le robot "
      "(rapproché en position seulement) ; atteignable, l'arrêt au toucher reste",
      avant.startswith("mains qui tirent de 3.0 N") and ManipulationService._appui_torse(m) == "",
      avant)

section("3. Avec les corrections : poids pesé, serrage d'après lui, marge au-dessus de 2")
requis = S * PESEE / (2.0 * MU)
vise = requis + R.marge_serrage_n
check(f"Serrage requis d'après le poids pesé : {S:.0f} × {PESEE:.1f} / (2 × {MU}) = {requis:.1f} N "
      f"par main ; visé avec {R.marge_serrage_n:.0f} N de marge : {vise:.1f}", 11.5 <= requis <= 13.5)
# Raideur au point de contact : force du juge sur l'avance de la consigne dans
# la face mesurée, en haut du levage.
raideur = {c: np.mean([f(PAS[t][c], "force_juge_n")
                       / abs(f(PAS[t][c], "face_mesuree_m") - f(PAS[t][c], "face_commandee_m"))
                       for t in HAUT]) for c in ("left", "right")}
avance = max((vise - n_haut[c]) / raideur[c] + abs(f(PAS[HAUT[-1]][c], "face_mesuree_m")
                                                   - f(PAS[HAUT[-1]][c], "face_commandee_m"))
             for c in ("left", "right"))
check(f"Raideur au contact {raideur['left']:.0f} et {raideur['right']:.0f} N/m : consigne à "
      f"{100 * avance:.1f} cm dans la face, sous la borne de {100 * R.penetration_poids_max_m:.0f} "
      f"(penetration_poids_max_m)", avance <= R.penetration_poids_max_m)
# Jusqu'où l'OS peut serrer, en haut du levage (couples des lignes plus Jᵀ n
# × le serrage ajouté) : aucune articulation des deux bras à moins de 0,3 N·m
# de son plafond (_marge_plafonds, rejoué tel quel).
m.mesures = SimpleNamespace(couple_max={f"{c}_{a}_joint": 2.0 * PLAFOND[a]
                                        for c in ("left", "right") for a in ART})


def couples(t, ajout_n):
    return {f"{c}_{a}_joint": f(PAS[t][c], f"couple_{a}_nm") + jac_n(t, c)[i] * ajout_n[c]
            for c in ("left", "right") for i, a in enumerate(ART)}


# Le serrage monte des deux côtés à la fois (le carton, en l'air, transmet la
# force d'une main à l'autre) : même ajout aux deux mains ; l'OS avance d'un
# pas à chaque instant où la porte est ouverte : il s'arrête là où elle se
# ferme sur la plupart des relevés (médiane), en haut du levage.
FIN = LEVAGE[-25:]                         # la dernière demi-seconde du levage


def porte(t, ajout):
    return ManipulationService._marge_plafonds(m, couples(t, {"left": ajout, "right": ajout}))


def jusqua(ajout_max):
    ajouts = []
    for t in FIN:
        a_ = 0.0
        while a_ < ajout_max and porte(t, a_ + 0.1):
            a_ += 0.1
        ajouts.append(a_)
    return {c: n_haut[c] + float(np.median(ajouts)) for c in n_haut}


avant = S * POIDS / (2.0 * MU)
bloque = jusqua(avant + R.marge_serrage_n - min(n_haut.values()))
check(f"Avec la référence d'avant ({POIDS:.1f} N), il fallait {avant:.1f} N par main : les "
      f"poignets arrêtent le serrage vers {min(bloque.values()):.1f} (plafonds inchangés) — "
      f"« trop lourd pour ces mains », de peu", min(bloque.values()) < avant)
atteint = jusqua(vise - min(n_haut.values()))
check(f"Avec le poids pesé : la porte laisse monter le serrage jusque vers G "
      f"{atteint['left']:.1f} N, D {atteint['right']:.1f} N par main, au moins le requis "
      f"({requis:.1f}) — le carton est porté", min(atteint.values()) >= requis)
poids_max = 2.0 * MU * min(atteint.values()) / S
check(f"… ces mains, dans leurs plafonds, porteraient {poids_max:.1f} N ainsi ; ce carton en pèse "
      f"{PESEE:.1f} (pesé)", poids_max >= PESEE)
serre = {c: min(atteint[c], vise) for c in atteint}
corrigees, par_main = [], []
for t in TENU:
    n = {c: f(PAS[t][c], "force_juge_n") + (serre[c] - n_haut[c])
         + raideur[c] * abs(f(PAS[t][c], "face_commandee_m") - f(PAS[HAUT[-1]][c],
                                                                 "face_commandee_m"))
         for c in ("left", "right")}
    corrigees.append(MU * (n["left"] + n["right"]) / PESEE)
    par_main.append(MU * min(n.values()) / (PESEE / 2.0))
check(f"Port rejoué (forces du juge, plus le serrage ajouté jusqu'à "
      f"{min(serre.values()):.1f} N par main, plus la consigne rendue), sur le poids pesé : marge "
      f"de frottement {min(corrigees):.2f} à {max(corrigees):.2f}, au-dessus de "
      f"{S:.0f} pendant tout le port (d'une main à l'autre, au pire instant du balancement : "
      f"{min(par_main):.2f})", min(corrigees) >= S)
check("Sans les corrections, le contrôle du port (_garder_serrage) aurait arrêté la base dès "
      f"l'entrée en port : marge {marges[0]:.2f} < {S:.0f}", marges[0] < S)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
