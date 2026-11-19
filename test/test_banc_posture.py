"""
Banc Isaac Sim de la posture GR00T WBC (lancer_posture.sh), sans GPU :

  1. Paliers : torse de 0,05 à 0,35 rad, bassin de 2,5 à 10 cm, puis les
     deux ensemble, croissants, dans les bornes de la consigne de l'OS.
  2. Premiers signes de perte d'équilibre : bassin incliné, qui tourne,
     pied qui glisse ; centrale inertielle illisible.
  3. Relevé par palier et verdict : un signe sur un palier accepté par
     l'OS, une chute, un refus du garde-fou, un carton lâché ou une série
     manquante font échouer le banc ; un palier refusé par l'OS, non.
  4. Le script Isaac (lecture, sans Isaac Sim) : chaque palier par
     LocomotionService.posture (jamais l'adaptateur directement : bornes,
     gardien et équilibre de l'OS toujours appliqués), carton de
     masse_max_kg dans le bilan d'équilibre, arrêt et retour droit au
     premier signe, relevé écrit ; axis_os/ n'importe pas le banc.
  5. Le lanceur : banc détaché, relevé et journal récupérés.
  6. L'app de tri T1 ne se penche pas tant que ce banc n'est pas passé
     (décision de revue).
"""
import math
import re
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE))
sys.path.insert(0, str(RACINE / "sim/validation"))

from axis_os.services.locomotion.service import BornesPosture  # noqa: E402
from banc_posture import (  # noqa: E402
    COLONNES, GLISSEMENT_MAX_M, INCLINAISON_MAX_DEG, VITESSE_MAX_RAD_S, Palier,
    inclinaison_deg, paliers, signe_de_perte, verdict,
)

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


section("1. Paliers")
b = BornesPosture()
series = paliers()
check("Trois séries : torse, bassin, les deux", [n for n, _ in series] == ["torse", "bassin",
                                                                          "les deux"])
tous = [v for _, vs in series for v in vs]
check("Chaque palier dans les bornes de la consigne (torse ≤ 0,35 rad, bassin ≤ 10 cm)",
      all(0.0 <= t <= b.tangage_max_rad and 0.0 <= a <= b.abaissement_max_m for t, a in tous))
check("Croissants dans chaque série, jusqu'aux bornes", all(
    all(v2 >= v1 for v1, v2 in zip(vs, vs[1:])) for _, vs in series)
    and max(t for t, _ in tous) == b.tangage_max_rad
    and max(a for _, a in tous) == b.abaissement_max_m)

section("2. Premiers signes de perte d'équilibre")


def quat_y(deg):
    a = math.radians(deg) / 2.0
    return (math.cos(a), 0.0, math.sin(a), 0.0)


check("Centrale inertielle : droit, 0° ; tangage de 12°, 12° ; quaternion non normé, idem",
      abs(inclinaison_deg((1, 0, 0, 0))) < 1e-9 and abs(inclinaison_deg(quat_y(12)) - 12) < 1e-6
      and abs(inclinaison_deg(tuple(2 * v for v in quat_y(12))) - 12) < 1e-6)
check("Rien d'anormal : aucun signe", signe_de_perte(2.0, 0.1, 0.002) is None
      and signe_de_perte(2.0, 0.1, None) is None)
for quoi, args in (("bassin incliné", (INCLINAISON_MAX_DEG + 0.5, 0.1, 0.0)),
                   ("qui tourne", (1.0, VITESSE_MAX_RAD_S + 0.1, 0.0)),
                   ("pied glissé", (1.0, 0.1, GLISSEMENT_MAX_M + 0.002)),
                   ("illisible", (math.nan, 0.1, 0.0))):
    s = signe_de_perte(*args)
    check(f"{quoi} : signe « {s} »", s is not None and quoi.split()[-1] in s, str(s))

section("3. Relevé par palier et verdict")
p = Palier("torse", 0.0, 1, 0.05, 0.0, accepte=True, marge_prevue_m=0.061)
for incl, vit, gl in ((1.0, 0.05, 0.001), (-3.0, 0.2, 0.004), (2.0, 0.1, 0.002)):
    p.suivre(incl, vit, gl)
check("Les plus grandes valeurs retenues (inclinaison signée, vitesse, glissement)",
      p.inclinaison_max_deg == -3.0 and p.vitesse_max == 0.2 and p.glissement_max_m == 0.004
      and p.signe is None)
check("La ligne du relevé a toutes ses colonnes", set(p.ligne()) == set(COLONNES)
      and p.ligne()["marge_prevue_cm"] == "6.10")


def releve(signe_sur=None, refuse_en_plus=False, lache=False):
    sortie = []
    for charge in (0.0, 1.2):
        for nom, valeurs in paliers():
            for k, (t, a) in enumerate(valeurs[:2], 1):
                q = Palier(nom, charge, k, t, a, accepte=True, marge_prevue_m=0.04,
                           carton_tenu=(not lache) if charge else None)
                if signe_sur == (nom, charge, k):
                    q.signe = "bassin incliné de +11.0° (au plus 10)"
                sortie.append(q)
            if refuse_en_plus:
                sortie.append(Palier(nom, charge, 3, 0.35, 0.0, accepte=False,
                                     marge_prevue_m=0.012, raison="posture refusée, équilibre"))
    return sortie


ok, raisons = verdict(releve(), None, 0, True)
check("Relevé propre : passé", ok, str(raisons))
ok, raisons = verdict(releve(refuse_en_plus=True), None, 0, True)
check("Paliers refusés par l'OS (équilibre prévu) : pas un échec, le modèle est prudent", ok,
      str(raisons))
for quoi, args in (("signe sur un palier accepté", (releve(("torse", 1.2, 2)), None, 0, True)),
                   ("chute", (releve(), "à 41.2 s", 0, True)),
                   ("refus du garde-fou", (releve(), None, 3, True)),
                   ("pas droit à la fin", (releve(), None, 0, False)),
                   ("carton lâché", (releve(lache=True), None, 0, True)),
                   ("série avec carton manquante",
                    ([q for q in releve() if q.charge_kg == 0.0], None, 0, True))):
    ok, raisons = verdict(*args)
    check(f"{quoi} : échoué ({raisons[0] if raisons else ''})", not ok and raisons)

section("4. Le script Isaac (lecture)")
SCRIPT = (RACINE / "sim/validation/test_isaac_w5_gr00t_posture.py").read_text(encoding="utf-8")
check("Chaque palier par la consigne de l'OS (loco.posture), jamais l'adaptateur directement",
      "loco.posture(tangage, abaissement" in SCRIPT
      and not re.search(r"adapt\w*\.posture\(", SCRIPT))
check("Carton de masse_max_kg, dans le bilan d'équilibre (charges=)",
      "ReglagesPrise().masse_max_kg" in SCRIPT and "charges=charge_aux_mains(tangage)" in SCRIPT)
check("Au premier signe : arrêt de la série et retour droit par l'OS (posture(0.0, 0.0))",
      "p.suivre(" in SCRIPT and "loco.posture(0.0, 0.0)" in SCRIPT and "retour droit" in SCRIPT)
check("Centrale inertielle (orientation, vitesse angulaire) et pieds (vérité de simulation)",
      "inclinaison_deg(st.quat_wxyz)" in SCRIPT and "st.ang_vel_body" in SCRIPT
      and "ankle_roll_link" in SCRIPT)
check("Relevé écrit (posture.csv, COLONNES), verdict et garde-fou au bilan",
      "posture.csv" in SCRIPT and "DictWriter(f, fieldnames=COLONNES)" in SCRIPT
      and "verdict(" in SCRIPT and 'loco.stats["refused"]' in SCRIPT)
importe = [str(f.relative_to(RACINE)) for f in (RACINE / "axis_os").rglob("*.py")
           if "banc_posture" in f.read_text(encoding="utf-8")]
check("axis_os/ n'importe pas le banc", not importe, str(importe))
# Essai GPU du 19 novembre 2026 : « chute à 0.00 s (Debout) », 0 tick, aucun
# palier essayé (la vidéo et l'initialisation avaient fait pencher le robot).
T1 = (RACINE / "sim/validation/test_isaac_t1_tri_usine.py").read_text(encoding="utf-8")
debout = SCRIPT.find("bridge.remettre_debout(")
check("Remis debout en dernier avant la boucle, après la vidéo, par le code commun des bancs "
      "(IsaacSimBridge.remettre_debout, celui du banc T1)",
      0 < SCRIPT.find("rec.start()") < debout < SCRIPT.find('etat["texte"] = "Debout"')
      and "bridge.remettre_debout(" in T1)
check("Aucune décision de chute avant le premier pas de physique de la boucle",
      re.search(r'FALLEN and etat\["chute"\] is None and etat\["k"\] > 0', SCRIPT) is not None)

section("5. Le lanceur")
LANCEUR = (RACINE / "deploy/machine_gpu/lancer_posture.sh").read_text(encoding="utf-8")
COPIER = (RACINE / "deploy/machine_gpu/copier_code.sh").read_text(encoding="utf-8")
check("lancer_posture.sh : banc détaché (docker exec -d), journal et relevé récupérés",
      "docker exec -d" in LANCEUR and "test_isaac_w5_gr00t_posture.py" in LANCEUR
      and "posture.csv" in LANCEUR and "posture.log" in LANCEUR and "--recuperer" in LANCEUR)
check("copier_code.sh copie le script et son module de banc dans le conteneur",
      "sim/validation/test_isaac_*.py" in COPIER and "sim/validation/banc_*.py" in COPIER)

section("6. L'app de tri T1 ne se penche pas tant que ce banc n'est pas passé")
SOURCES = sorted((RACINE / "axis_os/apps/tri_colis").glob("*.py")) + [
    RACINE / "sim/validation/test_isaac_t1_tri_usine.py"]
penche = [f"{f.relative_to(RACINE)} : {m}" for f in SOURCES
          for m in re.findall(r"plan_penche|changer_tangage|\.posture\(|pencher\s*=\s*True",
                              f.read_text(encoding="utf-8"))]
check(f"Ni plan_penche, ni changer_tangage, ni consigne de posture ({len(SOURCES)} fichiers : "
      f"app tri_colis, scène T1 Isaac)", not penche and len(SOURCES) > 3, "; ".join(penche))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
