"""
Service « personnes » : apparence, suivi multi-personnes, ré-identification
courte, position au sol, mémoire bornée, détecteurs (sans aucun modèle
téléchargé : détections factices).
"""
import os
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import (  # noqa: E402
    FRAME_BASE, FRAME_CAMERA, FRAME_WORLD, FrameTree, Transform, camera_optical_to_link,
)
from axis_os.hal.core.types import CameraFrame, CameraIntrinsics, HALStatus, Vector3  # noqa: E402
from axis_os.services.people.appearance import (  # noqa: E402
    DESCRIPTOR_SIZE, HEAD_FRACTION, appearance_descriptor, appearance_distance,
    synthetic_person_image,
)
from axis_os.services.people.detector import (  # noqa: E402
    FakePersonDetector, NullPersonDetector, ObjectDetectorPersons, PersonDetection,
)
from axis_os.services.people.rtdetr_detector import (  # noqa: E402
    MODELE_RTDETR, RTDETR_REVISION, RTDETRPersonDetector,
)
from axis_os.services.people.service import PeopleService  # noqa: E402
from axis_os.services.people.tracker import (  # noqa: E402
    MAX_TRACKS, REID_MAX_S, Observation, PersonTracker,
)

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


rng = np.random.default_rng(1)
ROUGE, BLEU, NOIR, GRIS = (200, 30, 30), (30, 50, 200), (30, 30, 30), (120, 120, 120)


def app(torse, jambes=NOIR, bruit=6.0, **kw):
    img = synthetic_person_image(torse, jambes, noise=bruit, rng=rng, **kw)
    return appearance_descriptor(img, (0, 0, img.shape[1], img.shape[0]))


section("1. Apparence : les vêtements, jamais la tête")
a1, a2, b = app(ROUGE), app(ROUGE), app(BLEU)
check(f"Un petit vecteur de {DESCRIPTOR_SIZE} nombres", a1.shape == (DESCRIPTOR_SIZE,))
check("Même tenue, éclairage bruité : apparences proches (< 0.15)",
      appearance_distance(a1, a2) < 0.15, str(appearance_distance(a1, a2)))
check("Haut rouge contre haut bleu : apparences éloignées (> 0.4)",
      appearance_distance(a1, b) > 0.4, str(appearance_distance(a1, b)))
img = synthetic_person_image(ROUGE, NOIR)
img2 = img.copy()
img2[:int(img.shape[0] * HEAD_FRACTION)] = rng.integers(0, 255, img2[:int(img.shape[0] * HEAD_FRACTION)].shape)
d_tete = appearance_distance(appearance_descriptor(img, (0, 0, 40, 100)),
                             appearance_descriptor(img2, (0, 0, 40, 100)))
check("La tête (le visage) n'est jamais lue : la changer ne change rien",
      d_tete == 0.0, str(d_tete))
check("Boîte trop petite, ou hors de l'image : pas de descripteur",
      appearance_descriptor(img, (0, 0, 4, 8)) is None
      and appearance_descriptor(img, (500, 500, 40, 100)) is None)
check("Boîte non finie : pas de descripteur",
      appearance_descriptor(img, (float("nan"), 0, 40, 100)) is None)
check("Apparence absente : la distance ne dit rien (None)",
      appearance_distance(a1, None) is None)

section("2. Suivi : numéro stable, filtre de mouvement, association")
tr = PersonTracker()
t = 0.0
for k in range(30):                     # une personne marche à 1 m/s
    t = k * 0.1
    tr.update([Observation((1.0 + 1.0 * t + rng.normal(0, 0.05), 0.0), 0.9, a1)], t)
snaps = tr.snapshots()
check("Une personne qui marche : une seule piste, confirmée", len(snaps) == 1, str(snaps))
check("… sa vitesse estimée est d'environ 1 m/s", abs(snaps[0].vx - 1.0) < 0.3, str(snaps[0].vx))
check("Une détection isolée ne devient pas une personne (pas confirmée)",
      len(PersonTracker().snapshots()) == 0)
tr2 = PersonTracker()
tr2.update([Observation((2.0, 0.0), 0.9, a1)], 0.0)
check("… après une seule détection, aucune piste confirmée", tr2.snapshots() == [])
tr2.update([Observation((2.0, 0.0), 0.9, a1)], 0.1)
check("… après deux, confirmée", len(tr2.snapshots()) == 1)

# Deux personnes qui se croisent, habillées différemment.
tr = PersonTracker()
ids = {}
for k in range(41):
    t = k * 0.1
    ya, yb = -1.0 + 0.05 * k, 1.0 - 0.05 * k       # elles se croisent à k = 20
    tr.update([Observation((3.0, ya), 0.9, app(ROUGE)), Observation((3.0, yb), 0.9, app(BLEU))], t)
    for s in tr.snapshots():
        ids.setdefault(s.track_id, []).append(round(s.y, 2))
fin = {s.track_id: s.y for s in tr.snapshots()}
rouge = min(ids, key=lambda i: ids[i][0])
check("Deux personnes d'apparence différente qui se croisent gardent chacune leur numéro",
      len(fin) == 2 and fin[rouge] > 0.5, str(fin))

section("3. Ré-identification courte, jamais dans le doute")
tr = PersonTracker()
for k in range(10):
    tr.update([Observation((3.0, 0.0), 0.9, app(ROUGE))], k * 0.1)
pid = tr.snapshots()[0].track_id
for k in range(10, 30):                 # cachée 2 s (derrière un poteau)
    tr.update([], k * 0.1)
tr.update([Observation((3.6, 0.0), 0.9, app(ROUGE))], 3.0)
s = tr.snapshots()
check("Cachée 2 s, revue 60 cm plus loin, même tenue : même numéro (reprise)",
      len(s) == 1 and s[0].track_id == pid and s[0].reid_count == 1, str(s))
tr = PersonTracker()
for k in range(10):
    tr.update([Observation((3.0, 0.0), 0.9, app(ROUGE))], k * 0.1)
for k in range(10, 30):
    tr.update([], k * 0.1)
tr.update([Observation((3.6, 0.0), 0.9, app(BLEU))], 3.0)
check("… revue en bleu : pas de reprise (autre personne, autre numéro)",
      tr.stats["reid"] == 0 and tr.stats["tracks_created"] == 2)
tr = PersonTracker()
for k in range(10):
    tr.update([Observation((3.0, 0.0), 0.9, app(ROUGE))], k * 0.1)
for k in range(10, 30):
    tr.update([], k * 0.1)
tr.update([Observation((3.5, 0.3), 0.9, app(ROUGE)), Observation((3.5, -0.3), 0.9, app(ROUGE))], 3.0)
check("… deux personnes en rouge reparaissent : aucune reprise (ambiguë)",
      tr.stats["reid"] == 0 and tr.stats["reid_ambiguous"] >= 1, str(tr.stats))
tr = PersonTracker()
for k in range(10):
    tr.update([Observation((3.0, 0.0), 0.9, app(ROUGE))], k * 0.1)
tr.update([], 0.9 + REID_MAX_S + 0.2)
check(f"Non vue depuis plus de {REID_MAX_S:g} s : piste et apparence oubliées",
      len(tr) == 0 and tr.stats["tracks_lost"] == 1)
tr.update([Observation((3.0, 0.0), 0.9, app(ROUGE))], 7.0)
tr.update([Observation((3.0, 0.0), 0.9, app(ROUGE))], 7.1)
check("… revue ensuite : un NOUVEAU numéro (jamais réutilisé)",
      tr.snapshots()[0].track_id != pid)
tr = PersonTracker()
tr.update([Observation((3.0 + 9.0, 0.0), 0.9, app(ROUGE))], 0.0)
for k in range(10):
    tr.update([Observation((3.0, 0.0), 0.9, app(ROUGE))], 0.1 * k)
for k in range(10, 22):
    tr.update([], 0.1 * k)
tr.update([Observation((3.0 + 20 * 0.2 + 3.0, 0.0), 0.9, app(ROUGE))], 2.2)
check("Reprise impossible pour un humain (7 m en 1.3 s) : refusée",
      all(x.reid_count == 0 for x in tr.snapshots()))

section("4. Mémoire bornée, entrées invalides")
tr = PersonTracker()
for k in range(2):
    tr.update([Observation((float(i), 5.0), 0.9, None) for i in range(MAX_TRACKS + 10)], k * 0.1)
check(f"Au plus {MAX_TRACKS} pistes", len(tr) <= MAX_TRACKS and tr.stats["saturated"] > 0)
tr = PersonTracker()
tr.update([Observation((float("nan"), 0.0), 0.9, None),
           Observation((float("inf"), 0.0), 0.9, None)], 0.0)
check("Position non finie : ignorée", len(tr) == 0)
tr.forget_all()
check("forget_all : plus aucune piste", len(tr) == 0)

section("5. Service : il ne regarde que sur demande, et oublie")
horloge = [0.0]
det = FakePersonDetector([PersonDetection((0, 0, 40, 100), 0.9, (2.0, 0.0, 0.0),
                                          tuple(app(ROUGE)))])
svc = PeopleService(det, source_pose=lambda: (0.0, 0.0, 0.0), horloge=lambda: horloge[0])
svc.start()
svc.tick()
check("Sans demande d'une app : aucune détection lancée", det.calls == 0)
check("… demander() réveille le service (rend True)", svc.demander() is True)
for k in range(5):
    horloge[0] += 0.1
    svc.tick()
check("… puis il détecte et suit", det.calls == 5 and len(svc.snapshots()) == 1)
rel = svc.relative(svc.snapshots()[0])
check("Distance et direction par rapport au robot", abs(rel[0] - 2.0) < 1e-6 and abs(rel[1]) < 1e-6)
m = svc.metrics
check("Mesures : fréquence de détection, latence, pertes et reprises",
      abs(m["detection_hz"] - 10.0) < 0.5 and m["latency_ms_last"] is not None
      and {"tracks_lost", "reidentified", "reid_ambiguous", "unlocated"} <= set(m), str(m))
horloge[0] += PeopleService.DEMAND_S + 1.0
svc.tick()
check(f"Plus de demande depuis {PeopleService.DEMAND_S:g} s : plus de détection, tout est oublié",
      det.calls == 5 and len(svc.snapshots()) == 0 and len(svc._tracker) == 0)


class Panne(FakePersonDetector):
    @property
    def stats(self):
        return {"last_inference_error": "CUDA hors mémoire"}


svc = PeopleService(Panne([]), source_pose=lambda: (0, 0, 0), horloge=lambda: horloge[0])
svc.start()
svc.demander()
r = svc.tick()
check("Détecteur en panne : signalé, jamais pris pour « personne »",
      not r.ok and "panne" in r.reason and not svc.health().healthy, str(r))
svc = PeopleService(NullPersonDetector())
svc.start()
check("Sans détecteur : le service le dit", "aucun détecteur" in svc.health().detail)

section("6. Position au sol par la géométrie de la caméra")


class Camera:
    def __init__(self, image):
        self.image = image

    def initialize(self):
        return HALStatus.OK

    def is_healthy(self):
        return True

    def get_intrinsics(self):
        return CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)

    def capture(self):
        return CameraFrame(640, 480, "rgb8", self.image.tobytes())


arbre = FrameTree()
arbre.set(Transform(FRAME_WORLD, FRAME_BASE, Vector3(0.0, 0.0, 0.0)))
arbre.set(Transform(FRAME_BASE, FRAME_CAMERA, Vector3(0.1, 0.0, 1.2)))
arbre.set(camera_optical_to_link())
# Pieds d'une personne à 3 m devant le robot : pixel v = 240 + 500 * 1.2 / 2.9.
v_pieds = 240 + 500 * 1.2 / 2.9
h = 160.0
image = np.full((480, 640, 3), 120, dtype=np.uint8)
silhouette = synthetic_person_image(ROUGE, NOIR, width=40, height=int(h))
y0 = int(round(v_pieds - h))
image[y0:y0 + int(h), 300:340] = silhouette
det = FakePersonDetector([PersonDetection((300.0, v_pieds - h, 40.0, h), 0.9)])
svc = PeopleService(det, camera=Camera(image), frame_tree=arbre,
                    source_pose=lambda: (0.0, 0.0, 0.0), horloge=lambda: horloge[0])
svc.start()
svc.demander()
obs = svc._situer(det.detect(None), Camera(image).capture())
check("Pieds dans l'image : position au sol retrouvée (3 m devant, à 5 cm près)",
      len(obs) == 1 and abs(obs[0].xy[0] - 3.0) < 0.05 and abs(obs[0].xy[1]) < 0.05, str(obs))
check("… et l'apparence est calculée sur l'image (haut rouge)",
      obs and appearance_distance(obs[0].appearance, app(ROUGE)) < 0.2)
coupee = FakePersonDetector([PersonDetection((300.0, 400.0, 40.0, 80.0), 0.9)])
obs = svc._situer(coupee.detect(None), Camera(image).capture())
check("Pieds hors de l'image : pas de position inventée (comptée « unlocated »)",
      obs == [] and svc.stats["unlocated"] == 1)


class Profondeur:
    def capture_depth(self):
        return np.full((480, 640), 2.4)


svc_p = PeopleService(det, camera=Camera(image), frame_tree=arbre, depth_camera=Profondeur(),
                      source_pose=lambda: (0.0, 0.0, 0.0), horloge=lambda: horloge[0])
obs = svc_p._situer([PersonDetection((300.0, 100.0, 40.0, 380.0), 0.9)], Camera(image).capture())
check("Caméra de profondeur : position par la surface du torse (2.5 m environ)",
      len(obs) == 1 and abs(obs[0].xy[0] - 2.5) < 0.15, str(obs))
sans_arbre = PeopleService(det, camera=Camera(image), source_pose=lambda: (0, 0, 0))
check("Sans arbre de repères : aucune position (jamais inventée)",
      sans_arbre._situer(det.detect(None), Camera(image).capture()) == [])

section("7. Vie privée : rien sur disque, aucune image gardée")
avant_cwd, avant_tmp = set(os.listdir(".")), set(os.listdir(tempfile.gettempdir()))
svc = PeopleService(FakePersonDetector([PersonDetection((300.0, v_pieds - h, 40.0, h), 0.9)]),
                    camera=Camera(image), frame_tree=arbre, source_pose=lambda: (0, 0, 0),
                    horloge=lambda: horloge[0])
svc.start()
svc.demander()
for k in range(5):
    horloge[0] += 0.1
    svc.tick()
check("Aucun fichier écrit (dossier courant, dossier temporaire)",
      set(os.listdir(".")) == avant_cwd
      and set(os.listdir(tempfile.gettempdir())) - avant_tmp == set())
gros = [k for k, v in vars(svc).items() if isinstance(v, (bytes, np.ndarray)) and len(v) > 1000]
check("Le service ne garde aucune image après le cycle", gros == [], str(gros))
check("Les pistes ne gardent qu'un petit descripteur (pas d'image)",
      all(v.appearance is None or v.appearance.size == DESCRIPTOR_SIZE
          for v in svc._tracker._tracks.values()))

section("8. Détecteurs : RT-DETR épinglé, solution de secours OWL-ViT")
check("RT-DETR : modèle Apache 2.0 des auteurs, révision épinglée (40 caractères)",
      MODELE_RTDETR == "PekingU/rtdetr_r18vd" and len(RTDETR_REVISION) == 40
      and all(c in "0123456789abcdef" for c in RTDETR_REVISION))
for rev in (None, "main", "v1.0"):
    d = RTDETRPersonDetector(revision=rev)
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        ok = d.load()
    check(f"… révision {rev!r} : chargement REFUSÉ avant tout téléchargement",
          ok is False and "NON épinglés" in d.stats["load_error"])
check("… un autre modèle n'hérite pas de la révision par défaut",
      RTDETRPersonDetector(model_name="autre/modele").stats["revision"] == "NON ÉPINGLÉE")
check("… modèle non chargé : aucune détection, sans erreur",
      RTDETRPersonDetector().detect(CameraFrame(2, 2, "rgb8", b"\0" * 12)) == [])
check("… la classe « person » est repérée par son nom",
      RTDETRPersonDetector._ids_personne({0: "person", 1: "bicycle", "2": "Person"}) == {0, 2})


class Boite(list):
    def tolist(self):
        return list(self)


d = RTDETRPersonDetector()
d._person_ids = {0}
res = {"scores": [0.9, 0.8, 0.7], "labels": [0, 2, 0],
       "boxes": [Boite([10, 20, 50, 220]), Boite([0, 0, 5, 5]), Boite([100, 20, 130, 200])]}
dets = d._personnes(res, 123)
check("… post-traitement : seules les personnes, boîtes (x, y, largeur, hauteur)",
      len(dets) == 2 and dets[0].bbox == (10.0, 20.0, 40.0, 200.0) and dets[0].timestamp_ns == 123,
      str(dets))
check("… le module se charge sans torch ni transformers (CI sans modèle)",
      "transformers" not in sys.modules)


class Owl:
    def detect(self, frame, query=None):
        from axis_os.services.perception.service import Detection
        return [Detection("a person", 0.3, (1, 2, 3, 4)), Detection("a person", 0.05, (0, 0, 1, 1))]


secours = ObjectDetectorPersons(Owl())
dets = secours.detect(CameraFrame(2, 2, "rgb8", b"\0" * 12))
check("Solution de secours OWL-ViT : « a person », au-dessus du seuil",
      len(dets) == 1 and dets[0].bbox == (1.0, 2.0, 3.0, 4.0))

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
