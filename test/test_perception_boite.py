"""
Boîte orientée d'un carton d'après la caméra de profondeur (perception/boite.py),
sur des cartes de profondeur synthétiques (lancer de rayons : sol, table,
carton tourné, mains devant lui), avec la caméra de tête du banc T1.

Constat GPU (2026-10-30) : objet visé à x 0,310-0,318 et z 0,160, carton
réel à x 0,335-0,370 et z 0,137-0,139 (2,5 à 5 cm trop court, 2 cm trop
haut), lacet jamais estimé (toujours 0). Ici : carton qui sera pris à
0,34 m du bassin, vu pendant l'approche (de 1,20 à 0,85 m : plus près, la
caméra de tête ne le voit plus), tourné de 0 à 15° ; erreur exigée sous
1,5 cm sur le centre et 3° sur le lacet, et de même avec 30 % du carton
caché par les mains.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import CameraIntrinsics  # noqa: E402
from axis_os.services.perception.boite import (  # noqa: E402
    FusionCarton, ajuster_boite, plan_ransac, points_du_masque, rectangle_minimal,
)

FAIL = 0
K = CameraIntrinsics(fx=550.0, fy=550.0, cx=320.0, cy=240.0)
W, H = 640, 480
BASSIN_Z = 0.73
PROF, LARG, HAUT = 0.20, 0.24, 0.20         # le carton de la scène T1
Z_CENTRE = 0.90                              # sur un plateau à 0,80 m


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def camera(bassin_x):
    """Caméra de tête de T1 : 0,2 m devant et 0,4 m au-dessus du bassin,
    visant 1,5 m devant et 5 cm au-dessus de lui (robot face à +x)."""
    cam = np.array([bassin_x + 0.2, 0.0, BASSIN_Z + 0.4])
    f = np.array([bassin_x + 1.5, 0.0, BASSIN_Z + 0.05]) - cam
    f /= np.linalg.norm(f)
    r = np.cross(f, [0.0, 0.0, 1.0])
    r /= np.linalg.norm(r)
    return cam, np.stack([r, np.cross(f, r), f], 1)      # colonnes : x droite, y bas, z avant


def touche(c, d, demi):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (-demi - c) / d, (demi - c) / d
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    return np.where((tmax >= tmin) & (tmax > 0), np.where(tmin > 0, tmin, np.inf), np.inf)


def rendre(cam, rot, objets):
    """Profondeur (distance au plan image) et numéro de l'objet touché par
    pixel (-1 : sol) ; objets : (centre, demi-tailles, lacet)."""
    v, u = np.mgrid[0:H, 0:W]
    ray = np.stack([(u - K.cx) / K.fx, (v - K.cy) / K.fy, np.ones((H, W))], -1).reshape(-1, 3)
    d = ray @ rot.T
    s = np.full(len(d), np.inf)
    ident = np.full(len(d), -1)
    for i, (c, demi, lacet) in enumerate(objets):
        cz, sz = math.cos(lacet), math.sin(lacet)
        rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1.0]])
        t = touche((cam - np.asarray(c)) @ rz, d @ rz, np.asarray(demi))
        ident = np.where(t < s, i, ident)
        s = np.minimum(s, t)
    with np.errstate(divide="ignore", invalid="ignore"):
        sol = np.where(d[:, 2] < 0, -cam[2] / d[:, 2], np.inf)
    ident = np.where(sol < s, -1, ident)
    s = np.minimum(s, sol)
    return s.reshape(H, W), ident.reshape(H, W)


def scene(x_carton, lacet_deg, mains):
    """Carton (objet 0), plateau (1) et, si `mains`, deux mains devant les
    côtés du carton (2, 3), qui en cachent une partie."""
    a = math.radians(lacet_deg)
    objets = [((x_carton, 0.0, Z_CENTRE), (PROF / 2, LARG / 2, HAUT / 2), a),
              ((x_carton + 0.3 - PROF / 2, 0.0, 0.78), (0.3, 1.0, 0.02), 0.0)]
    if mains:
        for cote in (1, -1):
            objets.append(((x_carton - PROF / 2 - 0.05, cote * 0.09, Z_CENTRE - 0.03),
                           (0.03, 0.05, 0.06), 0.0))
    return objets


def estimer(x_carton, lacet_deg, mains, poses=(1.20, 1.00, 0.85), masque_ideal=True,
            bruit_relatif=0.0):
    """Boîte fusionnée sur les vues de l'approche ; rend (boîte, part cachée).
    `bruit_relatif` : écart type de la profondeur, en part de la distance."""
    fusion = FusionCarton()
    cache = []
    rng_ = np.random.default_rng(7)
    for i, dist in enumerate(poses):
        cam, rot = camera(x_carton - dist)
        prof, ident = rendre(cam, rot, scene(x_carton, lacet_deg, mains))
        if bruit_relatif:
            prof = prof * (1.0 + rng_.normal(0.0, bruit_relatif, prof.shape))
        vu = ident == 0
        if mains:
            _, ident_seul = rendre(cam, rot, scene(x_carton, lacet_deg, False))
            cache.append(1.0 - vu.sum() / max(1, (ident_seul == 0).sum()))
        if masque_ideal:
            masque = vu                          # ce que rend un masque (SAM 2)
        else:                                    # le cadre du détecteur, débordant
            ys, xs = np.nonzero(ident == 0)
            masque = np.zeros_like(vu)
            masque[max(0, ys.min() - 15):ys.max() + 15, max(0, xs.min() - 15):xs.max() + 15] = True
        fusion.ajouter(float(i), points_du_masque(prof, K, rot, cam, masque=masque))
    vers_cam = np.array([-1.0, 0.0, 0.0])
    return ajuster_boite(fusion.points(), (LARG, PROF, HAUT), vers_cam), (np.mean(cache)
                                                                          if cache else 0.0)


def ecarts(b, x_carton, lacet_deg):
    e_c = math.hypot(b.centre[0] - x_carton, b.centre[1])
    e_z = abs(b.centre[2] - Z_CENTRE)
    e_l = abs(((math.degrees(b.lacet) - lacet_deg) + 45) % 90 - 45)
    return e_c, e_z, e_l


print("\n  Briques : plan par RANSAC, rectangle minimal")
rng = np.random.default_rng(0)
plateau = np.c_[rng.uniform(0, 1, 400), rng.uniform(0, 1, 400), 0.80 + rng.normal(0, 0.002, 400)]
bruit = np.c_[rng.uniform(0, 1, 150), rng.uniform(0, 1, 150), rng.uniform(0.85, 1.2, 150)]
pl = plan_ransac(np.r_[plateau, bruit])
check("Plan de la table par RANSAC : hauteur à 2 mm près, points du plan seulement",
      pl is not None and abs(-pl[1] - 0.80) < 0.002 and pl[2][:400].mean() > 0.95
      and pl[2][400:].mean() < 0.05, str(pl and (-pl[1])))
a = math.radians(23.0)
coins = np.array([[0.1, 0.06], [-0.1, 0.06], [-0.1, -0.06], [0.1, -0.06]])
rot2 = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
nuage = (rng.uniform(-1, 1, (300, 2)) * [0.1, 0.06]) @ rot2.T + [0.5, 0.2]
c, ang, (l1, l2) = rectangle_minimal(np.r_[nuage, coins @ rot2.T + [0.5, 0.2]])
check("Rectangle minimal d'un rectangle tourné de 23° : angle, centre et côtés exacts",
      abs(math.degrees(ang) - 23.0) < 0.5 and np.allclose(c, [0.5, 0.2], atol=1e-3)
      and abs(max(l1, l2) - 0.2) < 1e-3 and abs(min(l1, l2) - 0.12) < 1e-3,
      f"{math.degrees(ang):.2f} {c} {l1:.3f} {l2:.3f}")

print("\n  Carton pris à 0,34 m, vu pendant l'approche (1,20 à 0,85 m), tourné de 0 à 15°")
X = 4.0
pire = np.zeros(3)
for lacet in (0.0, 5.0, 10.0, 15.0, -8.0):
    b, _ = estimer(X, lacet, mains=False)
    e = ecarts(b, X, lacet)
    pire = np.maximum(pire, e)
    print(f"    lacet {lacet:+5.1f}° : centre à {100 * e[0]:.2f} cm, hauteur à {100 * e[1]:.2f} cm, "
          f"lacet à {e[2]:.2f}° ; résidu {100 * b.residu_m:.2f} cm, couverture {b.couverture:.2f}")
check(f"Centre à moins de 1,5 cm ({100 * pire[0]:.2f} au pire)", pire[0] < 0.015)
check(f"Lacet à moins de 3° ({pire[2]:.2f}° au pire)", pire[2] < 3.0)
check(f"Hauteur du centre à moins de 1,5 cm ({100 * pire[1]:.2f} au pire ; avant : 2 cm trop haut)",
      pire[1] < 0.015)

print("\n  Même chose, 30 % du carton caché par les mains (masque du seul carton)")
pire, caches = np.zeros(3), []
for lacet in (0.0, 5.0, 10.0, 15.0):
    b, cache = estimer(X, lacet, mains=True)
    caches.append(cache)
    e = ecarts(b, X, lacet)
    pire = np.maximum(pire, e)
    print(f"    lacet {lacet:+5.1f}° : {100 * cache:.0f} % caché ; centre à {100 * e[0]:.2f} cm, "
          f"lacet à {e[2]:.2f}° ; résidu {100 * b.residu_m:.2f} cm, couverture {b.couverture:.2f}")
check(f"Part du carton cachée par les mains : {100 * min(caches):.0f} à {100 * max(caches):.0f} %",
      0.25 <= min(caches) and max(caches) <= 0.35)
check(f"Centre à moins de 1,5 cm ({100 * pire[0]:.2f} au pire)", pire[0] < 0.015)
check(f"Lacet à moins de 3° ({pire[2]:.2f}° au pire)", pire[2] < 3.0)

print("\n  Profondeur bruitée (caméra réelle : ~5 mm à 1 m), 30 % caché")
pire = np.zeros(3)
for lacet in (0.0, 8.0, 15.0):
    b, _ = estimer(X, lacet, mains=True, bruit_relatif=0.005)
    e = ecarts(b, X, lacet)
    pire = np.maximum(pire, e)
    print(f"    lacet {lacet:+5.1f}° : centre à {100 * e[0]:.2f} cm, lacet à {e[2]:.2f}° ; "
          f"résidu {100 * b.residu_m:.2f} cm, couverture {b.couverture:.2f}")
check(f"Centre à moins de 1,5 cm ({100 * pire[0]:.2f} au pire)", pire[0] < 0.015)
check(f"Lacet à moins de 3° ({pire[2]:.2f}° au pire)", pire[2] < 3.0)

print("\n  Mal vu : jamais deviné")
b, _ = estimer(X, 10.0, mains=True, masque_ideal=False)
check(f"Cadre du détecteur débordant sur les mains et le fond : résidu {100 * b.residu_m:.1f} cm, "
      f"au-delà de 1,5 cm (prise refusée par l'appelant)", b.residu_m > 0.015, str(b.residu_m))
b, _ = estimer(X, 10.0, mains=False, masque_ideal=False)
check(f"Même cadre, sans mains : le sol vu autour de la table n'est pas pris pour elle ; "
      f"boîte juste (résidu {100 * b.residu_m:.2f} cm, table à {b.hauteur_table:.3f} m)",
      b.residu_m < 0.015 and abs(b.hauteur_table - 0.80) < 0.005
      and ecarts(b, X, 10.0)[0] < 0.015, f"{b.residu_m} {b.hauteur_table}")
b, _ = estimer(X, 10.0, mains=False, poses=(0.85,))
check(f"Une seule vue : ajustement quand même ({b.n_points} points)", b is not None)
check("Trop peu de points : None, jamais une boîte par défaut",
      ajuster_boite(np.zeros((5, 3)), (LARG, PROF, HAUT), np.array([-1.0, 0, 0])) is None)

print("\n  Application de tri : la façade ajuste la boîte et refuse un carton mal vu")
from types import SimpleNamespace  # noqa: E402

from axis_os.apps.tri_colis.facade import ConfigFacade, FacadeServices  # noqa: E402
from axis_os.hal.core.frames import Vector3  # noqa: E402
from axis_os.services.navigation.service import Pose2D  # noqa: E402
from axis_os.services.perception.service import Detection  # noqa: E402


class PerceptionFactice:
    """Ce que rend PerceptionService pour une vue : une détection et les
    points 3D de son masque (derniers_points)."""

    def __init__(self):
        self.derniers_points, self.vue = {}, None

    def montrer(self, x_carton, lacet_deg, dist, masque_ideal=True, mains=False):
        cam, rot = camera(x_carton - dist)
        prof, ident = rendre(cam, rot, scene(x_carton, lacet_deg, mains))
        ys, xs = np.nonzero(ident == 0)
        bbox = (int(xs.min()), int(ys.min()), int(np.ptp(xs)), int(np.ptp(ys)))
        masque = ident == 0 if masque_ideal else None
        self.derniers_points = {bbox: points_du_masque(prof, K, rot, cam, masque=masque,
                                                       bbox=(bbox[0] - 15, bbox[1] - 15,
                                                             bbox[2] + 30, bbox[3] + 40))}
        self.vue = Detection(label="rouge", confidence=0.4, bbox=bbox,
                             position_3d=Vector3(x_carton - PROF / 2, 0.0, Z_CENTRE))

    def find(self, query, now_ns=None, frame=None):
        return SimpleNamespace(ok=True, value=[self.vue], reason=None)


class BrasFactices:
    def __init__(self):
        self.objets, self.mesures = [], None

    def prendre(self, objet):
        self.objets.append(objet)
        return SimpleNamespace(ok=True, reason=None)


for lacet in (0.0, 12.0):
    percep, bras, journal = PerceptionFactice(), BrasFactices(), []
    pose = [Pose2D(X - 1.2, 0.0, 0.0)]
    fac = FacadeServices(None, None, bras, percep, lambda: pose[0], lambda: 0.0, 0.0,
                         journal=journal.append, cfg=ConfigFacade(requete="{couleur}"))
    for dist in (1.20, 1.00, 0.85):
        pose[0] = Pose2D(X - dist, 0.0, 0.0)
        percep.montrer(X, lacet, dist)
        vus = fac.chercher("rouge")
    c = vus[0]
    e_c = math.hypot(c.position[0] - X, c.position[1])
    e_l = abs(((math.degrees(c.lacet) - lacet) + 45) % 90 - 45)
    ok, _ = fac.prendre(c)
    check(f"Carton tourné de {lacet:.0f}° : centre à {100 * e_c:.2f} cm, lacet à {e_l:.2f}° "
          f"(3 images fusionnées), transmis à la prise avec son lacet", ok and e_c < 0.015
          and e_l < 3.0 and not c.mal_vu and bras.objets
          and abs(bras.objets[0].lacet - c.lacet) < 1e-12 and "3 image(s)" in c.vu, c.vu)
    check("… journal : centre, lacet, résidu, couverture",
          any(l.startswith("carton rouge vu : centre (") for l in journal), str(journal))
for mains, attendu in ((False, "pris"), (True, "mal vu")):
    percep, bras, journal = PerceptionFactice(), BrasFactices(), []
    fac = FacadeServices(None, None, bras, percep, lambda: Pose2D(X - 0.85, 0.0, 0.0),
                         lambda: 0.0, 0.0, journal=journal.append,
                         cfg=ConfigFacade(requete="{couleur}"))
    percep.montrer(X, 10.0, 0.85, masque_ideal=False, mains=mains)
    vus = fac.chercher("rouge")
    ok, raison = fac.prendre(vus[0])
    if attendu == "pris":
        check("Sans masque (SAM absent), cadre débordant sur la table et le sol : la table est "
              "écartée, la boîte reste juste, le carton est pris", ok and bras.objets
              and math.hypot(vus[0].position[0] - X, vus[0].position[1]) < 0.015, raison)
    else:
        check("Sans masque, cadre débordant sur des mains : « carton mal vu », prise refusée "
              "sans toucher aux bras", not ok and "carton mal vu" in raison and not bras.objets,
              raison)

print("\n  Détecteur OWLv2 et masque SAM 2.1 : poids épinglés, sinon refusés")
import tempfile  # noqa: E402

from axis_os.hal.core.types import CameraFrame  # noqa: E402
from axis_os.services.perception.owlvit_detector import (  # noqa: E402
    MODELE_OWLV2, OWLV2_REVISION, Owlv2Detector, OwlViTDetector,
)
from axis_os.services.perception.sam2_segmenteur import Sam2Segmenteur  # noqa: E402

image = CameraFrame(width=W, height=H, encoding="rgb8", data=bytes(W * H * 3))
o2 = Owlv2Detector()
check("OWLv2 par défaut : google/owlv2-base-patch16-ensemble à sa révision épinglée",
      o2._model_name == MODELE_OWLV2 and o2._revision == OWLV2_REVISION)
for det, nom in ((Owlv2Detector(revision="main"), "révision « main »"),
                 (Owlv2Detector(model_name="autre/modele"), "autre modèle sans révision")):
    check(f"OWLv2, {nom} : chargement refusé, aucune détection", not det.load()
          and "NON épinglés" in (det.stats["load_error"] or "") and det.detect(image, "a box") == [],
          det.stats["load_error"])
check("Boîtes d'OWLv2 rapportées au carré complété (640 × 640), celles d'OWL-ViT à l'image",
      o2._taille_cible(image) == [640, 640] and OwlViTDetector()._taille_cible(image) == [480, 640])
seg = Sam2Segmenteur(revision="main")
check("SAM 2.1, révision non épinglée : chargement refusé", not seg.load()
      and "NON épinglés" in (seg.erreur or ""), seg.erreur)
check("SAM 2.1 sans modèle : pas de masque (la perception garde le cadre)",
      Sam2Segmenteur().masque(image, (10, 10, 50, 50)) is None)

print("\n  Banc : images enregistrées, comparaison de deux détecteurs (factices)")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "validation"))
import banc_perception as bp  # noqa: E402

LACET_BANC = 10.0
vues = []
for dist in (1.20, 1.00, 0.85):
    cam, rot = camera(X - dist)
    prof, ident = rendre(cam, rot, scene(X, LACET_BANC, False))
    rgb = np.full((H, W, 3), 60, dtype=np.uint8)
    rgb[ident == 1] = (150, 150, 150)
    rgb[ident == 0] = (200, 40, 40)
    ys, xs = np.nonzero(ident == 0)
    vues.append((cam, rot, prof, rgb, ident == 0,
                 (int(xs.min()), int(ys.min()), int(np.ptp(xs) + 1), int(np.ptp(ys) + 1))))
courante = [0]
dossier = tempfile.mkdtemp()
enr = bp.EnregistreurImages(
    dossier,
    SimpleNamespace(capture=lambda: CameraFrame(W, H, "rgb8", vues[courante[0]][3].tobytes()),
                    get_intrinsics=lambda: K),
    SimpleNamespace(capture_depth=lambda: vues[courante[0]][2]),
    lambda: (vues[courante[0]][1], vues[courante[0]][0]),
    lambda: [("rouge", (X, 0.0, Z_CENTRE), math.radians(LACET_BANC), (PROF / 2, LARG / 2, HAUT / 2))])
pris = []
for i, t in enumerate((0.0, 0.5, 1.0, 2.0)):
    courante[0] = min(i, 2) if i < 2 else i - 1
    pris.append(enr.prendre(t))
images = bp.charger(dossier)
check("Au plus une image par seconde simulée : 3 images sur 4 demandes, vérité comprise",
      pris == [True, False, True, True] and len(images) == 3
      and images[0]["couleurs"].tolist() == ["rouge"] and images[0]["rgb"].shape == (H, W, 3))
vrais = bp.cartons_a_voir(images[0])
check("Cadre vrai : la projection de la boîte vraie recouvre les pixels du carton (IoU ≥ 0,8)",
      len(vrais) == 1 and bp.iou(vrais[0]["cadre"], vues[0][5]) >= 0.8,
      str(vrais and (vrais[0]["cadre"], vues[0][5])))
par_image = {v[3].tobytes(): i for i, v in enumerate(vues)}


class DetecteurFactice:
    """Le carton à 0,3 (doublon à 0,25 sur l'autre requête), une fausse
    détection à 0,08 ; `rate` : images où il ne voit rien."""

    def __init__(self, rate=()):
        self.rate, self.n = set(rate), {}

    def detect(self, frame, q):
        k_ = par_image[frame.data]
        v = vues[k_]
        if k_ in self.rate:
            return []
        if q == "a box":
            return [Detection("a box", 0.30, v[5]), Detection("a box", 0.08, (5, 5, 60, 40))]
        return [Detection(q, 0.25, v[5])]


class SegmenteurFactice:
    def masque(self, frame, bbox):
        return vues[par_image[frame.data]][4]


bilans = {"bon": bp.evaluer(images, DetecteurFactice(), [0.05, 0.10, 0.30], (LARG, PROF, HAUT),
                            0.10, SegmenteurFactice()),
          "myope": bp.evaluer(images, DetecteurFactice(rate=(0,)), [0.05, 0.10, 0.30],
                              (LARG, PROF, HAUT), 0.10, SegmenteurFactice())}
b = bilans["bon"]
check("Au seuil 0,05 : 3 cartons vus sur 3, 3 fausses détections ; à 0,10 : aucune fausse, "
      "doublons retirés", b["a_voir"] == 3 and b["seuils"][0.05]["vus"] == 3
      and b["seuils"][0.05]["fausses"] == 3 and b["seuils"][0.10]["vus"] == 3
      and b["seuils"][0.10]["fausses"] == 0, str(b["seuils"]))
em = b["boites"]["masque"]
check(f"Boîte ajustée sur une seule image, avec le masque : centre à "
      f"{100 * max(e['centre_m'] for e in em):.2f} cm au plus, lacet à "
      f"{max(e['lacet_deg'] for e in em):.2f}°, aucune refusée", len(em) == 3
      and max(e["centre_m"] for e in em) < 0.015 and max(e["lacet_deg"] for e in em) < 3.0
      and not any(e["refus"] for e in em), str(em))
check("… et avec le cadre seul (débordant sur la table) : 3 boîtes aussi",
      len(b["boites"]["cadre"]) == 3)
texte = bp.rapport(bilans, {"bon": 0.10, "myope": 0.10})
check("Rapport : tableau par seuil, boîte ajustée, verdict pour le détecteur qui voit le plus",
      "vus 3/3" in texte and "vus 2/3" in texte and "masque SAM 2.1" in texte
      and "Verdict : garder bon" in texte, texte)

print(f"\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}")
sys.exit(1 if FAIL else 0)
