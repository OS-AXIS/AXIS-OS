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
     gardien et équilibre de l'OS toujours appliqués), carton T1 d'1 kg au
     plus pris par l'OS avec la même prise que T1 (gains, raideur, mains
     caoutchouc, couples estimés, calibration), dans le bilan d'équilibre,
     scène éclairée comme T1, arrêt et retour droit au
     premier signe, relevé écrit ; axis_os/ n'importe pas le banc.
  5. Le lanceur : banc détaché, relevé et journal récupérés.
  6. (Avant : l'app de tri T1 ne se penchait pas tant que ce banc n'était
     pas passé. Il est passé, essai GPU du 20 novembre 2026 : l'app se
     penche et plie les jambes dans ses limites, vérifiées par
     test_posture_tri.py.)
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
# Essai GPU du 19 novembre 2026, 21 h 51 : le carton de masse_max_kg (1,2 kg)
# était « LÂCHÉ par la seule prise » ; désormais le carton T1, 1 kg au plus.
masse_carton = float(re.search(r"^MASSE_CARTON = ([0-9.]+)", SCRIPT, re.M).group(1))
check(f"Carton T1 de {masse_carton:g} kg (1 kg au plus, dans les moyens des mains), dans le bilan "
      f"d'équilibre (charges=)",
      masse_carton <= 1.0 and "mass=MASSE_CARTON" in SCRIPT
      and "charges=charge_aux_mains(tangage)" in SCRIPT and "(MASSE_CARTON, tuple(" in SCRIPT)
kp = re.search(r"^KP_BRAS = (\[.*\])$", SCRIPT, re.M).group(1)
kd = re.search(r"^KD_BRAS = (\[.*\])$", SCRIPT, re.M).group(1)
T1_ = (RACINE / "sim/validation/test_isaac_t1_tri_usine.py").read_text(encoding="utf-8")
graine_t1 = re.search(r'"--graine", type=int, default=(\d+)', T1_).group(1)
check("La même prise que T1 : ses gains de bras, raidis pendant la prise (gains_prise, à la "
      "demande de l'OS), ses couples estimés (même graine), mains caoutchouc liées avant le "
      "départ de la physique, et la prise de l'OS (ManipulationService.prendre)",
      f"KP_BRAS = {kp}" in T1_ and f"KD_BRAS = {kd}" in T1_
      and f"GRAINE_COUPLES = {graine_t1}" in SCRIPT
      and "BruitCouples(len(names), graine=GRAINE_COUPLES)" in SCRIPT
      and "g_prise = gains_prise(g)" in SCRIPT and "raideur=regler_raideur" in SCRIPT
      and "preparer_scene(world, stage" in SCRIPT and "preparer_scene(world, stage" in T1_
      and "manip.prendre(objet)" in SCRIPT and "ManipulationService(adapt.pose_bras" in SCRIPT)
check("… après la calibration des bras de T1 (biais des couples, mains), le carton posé sur un "
      "plateau devant le robot, jamais glissé entre les mains ; le plateau retiré une fois tenu",
      "manip.echantillon_calibration_couples(nom)" in SCRIPT
      and "manip.terminer_calibration_couples()" in SCRIPT and "calib.ajouter(" in SCRIPT
      and "def poser_carton():" in SCRIPT
      and "carton.set_world_pose(position=(pg + pd) / 2.0)" not in SCRIPT
      and "serrer=True" not in SCRIPT
      and SCRIPT.find("manip.etat is Etat.TENU") < SCRIPT.find("plateau.set_world_pose(position=np.array([10.0")
      and "ManipulationService.prendre" in SCRIPT)
BANC_PRISE = (RACINE / "sim/validation/banc_prise.py").read_text(encoding="utf-8")
prep = BANC_PRISE[BANC_PRISE.find("def preparer_scene("):BANC_PRISE.find("def demarrer_services(")]
prep = prep[prep.find('"""', prep.find('"""') + 3):]          # le code, après la docstring
check("Mains caoutchouc et éclairage de T1 (dôme et quatre lumières), posés après les corps "
      "rigides et avant le départ de la physique : par le code commun des bancs "
      "(banc_prise.preparer_scene), appelé par T1 et par ce banc après leur dernier corps rigide, "
      "jamais recopié",
      0 < prep.find("mains_caoutchouc(stage)") < prep.find("eclairer(stage)")
      < prep.find("avant_reset()") < prep.find("world.reset()")
      and all(0 < S_.rfind("world.scene.add(") < S_.find("    preparer_scene(world, stage")
              and "    world.reset()\n" not in S_ and "    mains_caoutchouc(stage)" not in S_
              and "in eclairer(stage):" not in S_ for S_ in (SCRIPT, T1_)))
check("Services de l'OS démarrés par le code commun (banc_prise.demarrer_services), dits au "
      "journal, dans les deux bancs",
      "demarrer_services([" in SCRIPT and "demarrer_services([" in T1_
      and "    loco.start()\n    modele" not in SCRIPT and "    manip.start()\n" not in SCRIPT)
check("Aucun objet posé devant le robot inconnu du site : le carton de ce banc sur son plateau "
      "déclaré (plateau_site, objets_inconnus) ; la masse de référence de T1 sur la table de prise "
      "(sd.OBSTACLES)",
      "objets_inconnus([(\"carton\"" in SCRIPT and "plateau_site((" in SCRIPT
      and "objets_inconnus([(\"masse de référence\"" in T1_ and "sd.OBSTACLES)" in T1_
      and "plateau_reference" not in T1_)
# Essai GPU du 25 novembre 2026, 16 h 43 : table de prise remplie avant la
# calibration de charge, la masse de référence posée sur le carton rouge de
# l'emplacement du milieu, éjectée au sol en (3.23, 0.41), dans la zone de
# marche ; le gardien l'a vue comme un obstacle inconnu, robot arrêté 90 s.
from banc_prise import chevauchements  # noqa: E402
from axis_os.apps.tri_colis import scene_demo as sd_  # noqa: E402
taille_ = (sd_.LARGEUR_CARTON, sd_.PROFONDEUR_CARTON)
table_ = [(c_, (x_, sd_.Y_CARTON)) for c_, x_ in zip(("bleu", "rouge", "rouge"), sd_.EMPLACEMENTS_X)]
parcs_ = [(f"stock {k}", (30.0 + 0.5 * k, 30.0)) for k in range(10)]
check("Deux objets jamais posés l'un dans l'autre (banc_prise.chevauchements) : la masse de "
      "référence sur l'emplacement du milieu, la table remplie comme à 16 h 43 → occupé par le "
      "carton rouge ; table vide → libre ; ses voisins (40 cm) ne la gênent pas ; son parc "
      "(36, 30) loin de ceux des cartons",
      chevauchements((sd_.EMPLACEMENTS_X[1], sd_.Y_CARTON), table_, taille_) == ["rouge"]
      and chevauchements((sd_.EMPLACEMENTS_X[1], sd_.Y_CARTON), [], taille_) == []
      and chevauchements((sd_.EMPLACEMENTS_X[1], sd_.Y_CARTON), [table_[0], table_[2]], taille_)
      == []
      and chevauchements((36.0, 30.0), parcs_, taille_) == [])
appel_ = T1_.find("\n            operateur(t, stats)")
check("Aucun objet posé par le banc T1 dans la zone de marche, inconnu du site : la table de "
      "prise remplie seulement une fois la calibration de charge finie (plus avant la boucle) ; "
      "la masse de référence posée sur un emplacement libre, sinon pas posée et dit ; tombée au "
      "sol, retirée par l'opérateur et dit (comme les cartons) ; les objets du banc de posture "
      "sur son plateau déclaré",
      "operateur(0.0, stats)" not in T1_ and T1_.count("    operateur(") == 1
      and 'etat_charge.get("fini")' in T1_[T1_.rfind("\n", 0, appel_):appel_]
      and 0 < T1_.find("occupe_ = chevauchements(") < T1_.find("if occupe_:")
      < T1_.find("reference.set_world_pose(position=np.array(\n                [X_REFERENCE")
      and "masse de référence tombée au sol" in T1_
      and T1_.find("masse de référence tombée au sol") < T1_.find("retirer_reference()\n            return True")
      and "objets_inconnus([(\"carton\"" in SCRIPT)
check("L'alerte « même cause » de la mission (ArretMemeCause, 20 s) aussi pendant la calibration "
      "de charge, qui attend la sécurité hors de la mission (16 h 43 : 90 s d'arrêt, aucune alerte)",
      "alerte_charge = ArretMemeCause(tri.cfg)" in T1_ and "alerte_charge.pas(" in T1_
      and "ALERTE (calibration de charge)" in T1_)
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

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
