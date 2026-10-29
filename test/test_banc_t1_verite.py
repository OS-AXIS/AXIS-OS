"""
Banc Isaac Sim T1 : l'OS ne reçoit que ce que le vrai G1 à mains caoutchouc
fournit (angles et vitesses articulaires, couples ESTIMÉS, centrale
inertielle, caméras, LiDAR) ; la vérité de la simulation ne sert qu'au juge
et aux vues, sans GPU ici.

  - couples estimés : BruitCouples, et pas de faux contact du bruit seul ;
  - hauteur du bassin (jambes + centrale), inclinaison vue des mains,
    « objet encore entre les mains » : estimations des signaux réels ;
  - gains de prise (poignets, coude) bornés au pilote du G1 ;
  - juge des contacts : vrais, manqués, faux ; compté par l'évaluation ;
  - axis_os/ n'importe ni le juge ni la vue du robot, ni aucun capteur de
    contact PhysX ;
  - vue du robot : un point 3D connu se projette au bon pixel ; calques
    dessinés ; incrustation ; /vue_robot.jpg sur la page de suivi ;
  - le script T1 branche tout cela (lecture du script).
"""
import math
import re
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE))
sys.path.insert(0, str(RACINE / "sim/validation"))

from PIL import Image  # noqa: E402

from axis_os.hal.core.frames import Quaternion, Transform, Vector3  # noqa: E402
from axis_os.hal.core.types import CameraIntrinsics  # noqa: E402
from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples  # noqa: E402
from axis_os.hal.drivers.isaac_sim.direct import SuiviDirect  # noqa: E402
from axis_os.services.manipulation.estimation import HauteurBassin, inclinaison_mains  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402
from axis_os.vla.evaluation import bilan, texte_bilan  # noqa: E402
from banc_prise import KP_MAX_PILOTE, JugeContacts, gains_prise  # noqa: E402
from banc_vue_robot import coins, dessiner, projeter  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


section("1. Couples estimés (bruit réaliste, prudent)")
vrai = np.linspace(-5.0, 5.0, 29)
b = BruitCouples(29, graine=7)
lectures = np.array([b(vrai) for _ in range(4000)])
diff = lectures[1:] - lectures[:-1]
check("Bruit blanc de 0,3 N·m : la différence de deux lectures a ~0,42 N·m d'écart type",
      abs(float(diff.std()) - 0.3 * math.sqrt(2)) < 0.03, f"{diff.std():.3f}")
ecart = lectures.mean(axis=0) - vrai
check("Erreur constante par articulation (gain ±5 %, biais ±0,5 N·m), jamais nulle partout",
      float(np.abs(ecart).max()) <= 0.5 + 0.05 * 5 + 0.05 and float(np.abs(ecart).max()) > 0.05)
check("Même graine, même robot : erreurs reproductibles",
      np.allclose(BruitCouples(29, graine=7)._biais, b._biais))
# Règle de contact par le couple : hausse ≥ 1,5 N·m sur 2 lectures de suite,
# au repos (aucun contact) : jamais déclenchée par le bruit seul.
b2 = BruitCouples(1, graine=11)
tau0 = b2([0.0])[0]
hausses = np.abs(np.array([b2([0.0])[0] for _ in range(200000)]) - tau0)
deux = int(np.sum((hausses[1:] >= 1.5) & (hausses[:-1] >= 1.5)))
check(f"200 000 lectures au repos : {deux} faux contact par le couple", deux == 0)

section("2. Estimations des signaux réels")
h = HauteurBassin()
debout = h({}, np.eye(3))
genoux = {f"{c}_{j}": v for c in ("left", "right")
          for j, v in (("hip_pitch_joint", -0.4), ("knee_joint", 0.8), ("ankle_pitch_joint", -0.4))}
plie = h(genoux, np.eye(3))
check(f"Jambes tendues : bassin à {debout:.3f} m (G1 debout ~0,79)", 0.75 < debout < 0.83)
check(f"Genoux pliés : bassin plus bas ({plie:.3f} m), mesuré sans la simulation",
      0.6 < plie < debout - 0.03)
check("Bassin penché (centrale inertielle) : hauteur cohérente, au cm près",
      abs(h(genoux, rot_x(math.radians(3))) - plie) < 0.02)
g, d = np.array([0.30, 0.15, 0.10]), np.array([0.30, -0.15, 0.10])
check("Mains à la même hauteur : objet droit (0°)", inclinaison_mains(g, d, np.eye(3)) < 1e-9)
check("Bassin roulé de 20° (centrale) : objet penché de 20°",
      abs(inclinaison_mains(g, d, rot_x(math.radians(20))) - 20.0) < 1e-6)
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))], time_scale=2.0) as r:
    m = r.manipulation
    check("Avant toute prise : « objet entre les mains » faux", not m.objet_entre_les_mains())
    time.sleep(0.2)
    m.prendre(Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20))
    t0 = time.monotonic()
    while m.etat not in (Etat.TENU, Etat.ECHEC) and time.monotonic() - t0 < 15:
        time.sleep(0.01)
    check("Carton tenu : objet entre les mains (cinématique sur les angles mesurés)",
          m.etat is Etat.TENU and m.objet_entre_les_mains(), m.raison)
    v = m.etat_visible()
    check("Vue de l'OS (lecture seule) : phase, plan, cibles ouverture/contact/borne, mains",
          v["phase"] == "TENU" and v["plan"] is not None and v["mains"] is not None
          and set(v["cibles"]) == {"ouverture", "contact", "borne"})

section("3. Gains de prise, bornés au pilote du G1")
normaux = {"left_wrist_yaw_joint": (40.0, 1.0), "right_wrist_pitch_joint": (40.0, 1.0),
           "left_elbow_joint": (60.0, 1.0), "left_shoulder_roll_joint": (100.0, 2.0),
           "left_knee_joint": (150.0, 4.0)}
pr = gains_prise(normaux)
check("Poignets : kp ×2 (40 → 80), kd ×√2", pr["left_wrist_yaw_joint"] == (80.0, math.sqrt(2))
      and pr["right_wrist_pitch_joint"][0] == 80.0)
check(f"Coude : borné au plus grand gain du pilote ({KP_MAX_PILOTE:.0f})",
      pr["left_elbow_joint"][0] == KP_MAX_PILOTE)
check("Épaules et jambes : gains inchangés", pr["left_shoulder_roll_joint"] == (100.0, 2.0)
      and pr["left_knee_joint"] == (150.0, 4.0))

section("4. Juge des contacts (vérité simulation)")
force = {"left": 0.0, "right": 0.0}
juge = JugeContacts(lambda c: force[c])
juge.debut()
juge.pas(1.00)
force["left"] = 12.0
juge.pas(1.04)
force["right"] = 9.0
juge.pas(1.30)
lignes = juge.fin({"left": {"t": 1.08, "signal": "géométrie"},
                   "right": None})
check("Main G : estimée 40 ms après le contact réel → vrai contact",
      "vrai contact" in lignes[0] and "géométrie" in lignes[0] and "12.0 N" in lignes[0], lignes[0])
check("Main D : contact réel jamais estimé → CONTACT MANQUÉ", "CONTACT MANQUÉ" in lignes[1])
force = {"left": 0.0, "right": 0.0}
juge.debut()
juge.pas(2.0)
lignes = juge.fin({"left": {"t": 2.0, "signal": "couple"}, "right": None})
check("Contact estimé sans contact réel → FAUX CONTACT ; rien des deux côtés → aucun",
      "FAUX CONTACT" in lignes[0] and "aucun contact" in lignes[1])
check("Compteurs : 1 vrai, 1 manqué, 1 faux", juge.compteurs == {
    "contacts_vrais": 1, "contacts_manques": 1, "faux_contacts": 1})
muet = JugeContacts(lambda c: None)
muet.debut()
muet.pas(0.0)
check("Capteur muet : « inconnu », jamais « non »",
      "inconnu" in muet.fin({"left": None, "right": None})[0])
res = [{"carton": 0, "mode": "reflexes", "succes": True, "raison": "", "duree_s": 9.0,
        "refus_garde_fou": 0, "arrets_securite": 0, "contacts_vrais": 2,
        "contacts_manques": 0, "faux_contacts": 0},
       {"carton": 1, "mode": "reflexes", "succes": False, "raison": "x", "duree_s": 9.0,
        "refus_garde_fou": 0, "arrets_securite": 0, "contacts_vrais": 0,
        "contacts_manques": 2, "faux_contacts": 1}]
bl = bilan(res)
check("Bilan d'évaluation : vrais contacts, manqués, faux, par mode",
      bl["par_mode"]["reflexes"]["contacts"] == {"contacts_vrais": 2, "contacts_manques": 2,
                                                 "faux_contacts": 1}
      and "2 vrais contacts, 2 manqués, 1 faux" in texte_bilan(bl))

section("5. axis_os/ ne lit pas la vérité de la simulation")
lus = []
for f in (RACINE / "axis_os").rglob("*.py"):
    t = f.read_text(encoding="utf-8")
    rel = f.relative_to(RACINE).as_posix()
    if re.search(r"^\s*(from|import)\s+(banc_prise|banc_vue_robot)\b", t, re.M) \
            or re.search(r"ContactSensor|isaacsim\.sensors\.physics|omni\.isaac\.sensor", t):
        lus.append(rel)
    # Poses PhysX : réservées aux pilotes de simulation (HAL), jamais aux services.
    if "get_world_pose" in t and not rel.startswith("axis_os/hal/drivers/"):
        lus.append(rel)
check("Aucun fichier d'axis_os/ n'importe le juge, la vue du robot ni un capteur de contact, "
      "aucun service ne lit une pose PhysX", not lus, str(lus))

section("6. Vue du robot : projection et calques")
K = CameraIntrinsics(fx=550.0, fy=550.0, cx=320.0, cy=240.0)
# Caméra en (1, 2, 1.2), regard vers +x monde, inclinée de 20° vers le bas :
# optique x à droite (-y monde), y en bas, z devant.
base = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])   # colonnes : x, y, z optiques
inc = math.radians(20)
R = np.array([[1, 0, 0], [0, math.cos(inc), math.sin(inc)], [0, -math.sin(inc), math.cos(inc)]])
ROT = base @ R.T
CAM = np.array([1.0, 2.0, 1.2])


def quaternion(m):
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0, m[1, 0] - m[0, 1])
    return Quaternion(w=w, x=x, y=y, z=z)


TF = Transform("world", "camera_optical", translation=Vector3(*CAM), rotation=quaternion(ROT))
axe = CAM + 2.0 * ROT[:, 2]
uv, ok = projeter(axe, K, TF)
check("Un point sur l'axe optique se projette au centre optique (320, 240)",
      ok[0] and np.allclose(uv[0], (320.0, 240.0), atol=1e-6), str(uv))
p_opt = np.array([0.3, -0.1, 1.5])
p = CAM + ROT @ p_opt
uv, ok = projeter(p, K, TF)
attendu = (550 * 0.3 / 1.5 + 320, 550 * -0.1 / 1.5 + 240)
check(f"Point 3D connu → pixel attendu {tuple(round(v, 2) for v in attendu)}",
      ok[0] and np.allclose(uv[0], attendu, atol=1e-6), str(uv))
ray = K.pixel_to_ray(100.0, 400.0)
d_ = 1.7
pw = TF.apply(Vector3(ray.x * d_, ray.y * d_, ray.z * d_))
uv, _ = projeter([pw.x, pw.y, pw.z], K, TF)
check("Aller-retour avec la perception : pixel → rayon (pixel_to_ray) → point → même pixel",
      np.allclose(uv[0], (100.0, 400.0), atol=1e-6), str(uv))
check("Point derrière la caméra : écarté", not projeter(CAM - ROT[:, 2], K, TF)[1][0])
c8 = coins((0.0, 0.0, 0.0), 0.24, 0.20, 0.20, math.radians(90))
check("Contour 3D : largeur le long de la normale du lacet",
      np.allclose(np.ptp(c8, axis=0), (0.24, 0.20, 0.20), atol=1e-9))
cible = CAM + 1.5 * ROT[:, 2]
couches = {"detections": [((300, 220, 40, 40), "rouge", 0.31)],
           "faces": [np.array([cible + 0.01 * i * ROT[:, 0] for i in range(10)])],
           "boite": coins(cible, 0.24, 0.20, 0.20, 0.0), "centre": cible,
           "cibles": {"borne": (cible + 0.1 * ROT[:, 0], cible - 0.1 * ROT[:, 0])},
           "mains": (cible + 0.12 * ROT[:, 0], cible - 0.12 * ROT[:, 0]),
           "phase": "FERMETURE", "contacts": {"left": True, "right": False},
           "verite": {"boite": coins(cible + 0.02, 0.24, 0.20, 0.20, 0.1),
                      "contacts": {"left": 14.0, "right": None}}}
noir = Image.new("RGB", (640, 480))
img = dessiner(noir.copy(), K, TF, couches)
a = np.asarray(img)
check("Calques dessinés : cadre de détection rouge, phase en haut à gauche",
      tuple(a[220, 320]) != (0, 0, 0) and a[8:40, 8:200].any())
sans = dessiner(noir.copy(), K, TF, couches, verite=False)
check("Réglage sans vérité : la vérité simulation (rose, pointillés) disparaît",
      int((np.asarray(sans) == (255, 80, 220)).all(axis=2).sum()) == 0
      and int((a == (255, 80, 220)).all(axis=2).sum()) > 0)

section("7. Page de suivi : vue du robot à côté de la vue extérieure")
s = SuiviDirect(tempfile.mkdtemp(), port=18093, images_par_s=100.0)
check("Le serveur démarre", s.demarrer())
with urllib.request.urlopen("http://127.0.0.1:18093/", timeout=5) as rep:
    page = rep.read().decode()
check("La page montre une deuxième image, /vue_robot.jpg", 'id="robot"' in page
      and "/vue_robot.jpg" in page)
try:
    urllib.request.urlopen("http://127.0.0.1:18093/vue_robot.jpg", timeout=5)
    absente = False
except urllib.error.HTTPError as e:
    absente = e.code == 404
check("Avant toute vue : 404", absente)
s.image_robot(img)
with urllib.request.urlopen("http://127.0.0.1:18093/vue_robot.jpg?1", timeout=5) as rep:
    corps = rep.read()
check("Vue publiée et servie en JPEG", corps[:2] == b"\xff\xd8")
s.arreter()

section("8. Banc T1 et outils GPU (lecture des scripts, sans Isaac Sim)")
T1 = (RACINE / "sim/validation/test_isaac_t1_tri_usine.py").read_text(encoding="utf-8")
check("Couples estimés : bruit branché sur le pont (garde-fou) et sur la prise",
      "bridge.bruit_couples = BruitCouples(" in T1
      and "e = bridge.bruit_couples(robot.get_applied_joint_efforts())" in T1)
check("Mains de l'OS : cinématique sur les angles mesurés (les prims USD : banc seulement)",
      "g_, d_ = mesures_bras.modele.mains({j: read_state().joint_pos[j] for j in chaine_bras})"
      in T1 and "def mains_reelles():" in T1)
check("Vérification de prise : objet encore entre les mains (cinématique), pas le carton PhysX",
      "ok_ = manip.objet_entre_les_mains()" in T1)
check("Hauteur du bassin et inclinaison : jambes, mains et centrale inertielle",
      "lambda: hauteur_bassin(angles_mesures(), rotation_imu())," in T1
      and "inclinaison=inclinaison_os," in T1)
check("Raideur de prise et arrêt d'urgence branchés", "raideur=regler_raideur," in T1
      and "arret_urgence=lambda: hal.estop_active," in T1)
check("Juge : prise par prise au journal, compteurs dans l'évaluation",
      "juge.fin(manip.contacts_estimes)" in T1 and "**(juge.compteurs if juge" in T1)
check("Vue du robot : vidéo, incrustation, page de suivi, réglage pour la démo",
      '"t1_vue_robot.mp4"' in T1 and "incrustation=derniere_vue" in T1
      and "suivi.image_robot(derniere_vue)" in T1 and "--vue-sans-calques" in T1)
REC = (RACINE / "deploy/machine_gpu/recuperer_t1.sh").read_text(encoding="utf-8")
CP = (RACINE / "deploy/machine_gpu/copier_code.sh").read_text(encoding="utf-8")
check("recuperer_t1.sh : vue du robot récupérée, réduite en 540p, dans le zip",
      "t1_vue_robot.mp4" in REC and 'scale=-2:540' in REC and '$( [ -f "$VUE" ]' in REC)
check("copier_code.sh : modules du banc copiés à côté des scripts",
      'sim/validation/banc_*.py' in CP)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
