"""
Pose d'attente qui cadre tout le carton, filtre anti-fantôme géométrique et
raison exacte de chaque image d'observation écartée.

Constat GPU (essai T1 de main, novembre 2026) : à chaque accostage, « carton
… pas revu de façon cohérente après l'arrêt en 4 s : 34 image(s) neuve(s),
0 retenue(s) … rejets : carton pas vu ». À la pose d'attente (0.28 m fixe
derrière la pose de prise), le carton proche remplissait le bas de l'image
et en sortait ; sa détection était « écartée : trop grande » (plus de 25 %
de l'image) ; aucune prise n'a été tentée.

Désormais (axis_os/apps/tri_colis/facade.py, couleur.py) :
  - la pose d'attente est calculée : la plus proche d'où les 8 coins du
    carton tombent dans l'image de la caméra de tête, à 5 % du bord ;
  - une détection est écartée si son cadre touche les quatre bords de
    l'image, ou si sa taille métrique (masque et profondeur) ne peut pas
    être celle d'un carton ; un carton proche qui remplit 40 % de l'image
    est gardé ;
  - chaque image d'observation écartée l'est avec sa raison exacte ;
  - la recherche du carton ne part qu'une fois la base arrêtée depuis
    duree_visee_s (le critère de la visée) : deux recherches lancées tête
    encore en rotation l'avaient situé à 2 et 3 cm de côté.

Banc synthétique : la vraie chaîne (PerceptionService, PerceptionCouleur,
FacadeServices), la caméra de tête du banc T1 (0,2 m devant et 0,4 m
au-dessus du bassin, 15° vers le bas, 640 × 480, focale 550 pixels), la
scène T1 (table de prise, carton rouge de 24 × 20 × 20 cm), rendue par lancer
de rayons (profondeur, couleur), un détecteur et un masque idéaux.
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis.couleur import PerceptionCouleur  # noqa: E402
from axis_os.apps.tri_colis.facade import ConfigFacade, FacadeServices  # noqa: E402
from axis_os.apps.tri_colis.logique import Colis  # noqa: E402
from axis_os.hal.core.frames import (  # noqa: E402
    FRAME_BASE, FRAME_CAMERA, FRAME_WORLD, FrameTree, Transform, camera_optical_to_link,
)
from axis_os.hal.core.types import CameraFrame, CameraIntrinsics, Quaternion, Vector3  # noqa: E402
from axis_os.services.base import ServiceResult  # noqa: E402
from axis_os.services.manipulation.service import Etat  # noqa: E402
from axis_os.services.navigation.service import EtatNavigation, Pose2D  # noqa: E402
from axis_os.services.perception.service import Detection, PerceptionService  # noqa: E402

FAIL = 0
K = CameraIntrinsics(fx=550.0, fy=550.0, cx=320.0, cy=240.0)
W, H = 640, 480
BASSIN_Z = 0.748                              # « hauteur du bassin, robot posé » (journal T1)
MONTAGE, VISEE = np.array([0.2, 0.0, 0.4]), np.array([1.5, 0.0, 0.05])
LARG, PROF, HAUT = 0.24, 0.20, 0.20
CAP = math.pi / 2                             # face à +y, comme dans la scène T1
CARTON = np.array([4.0, 2.10, 0.90])          # centre, sur la table de prise (plateau 0,80)
ROUGE, GRIS, SOL = (204, 26, 26), (150, 150, 155), (115, 179, 217)


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def camera(x, y, cap):
    """Pose de la caméra de tête (centre, rotation optique -> monde) pour la
    base en (x, y, cap) : calcul du test, indépendant de celui de l'OS."""
    b = np.array([x, y, BASSIN_Z])
    cam, vise = b + rz(cap) @ MONTAGE, b + rz(cap) @ VISEE
    f = (vise - cam) / np.linalg.norm(vise - cam)
    r = np.cross(f, [0.0, 0.0, 1.0])
    r /= np.linalg.norm(r)
    return cam, np.stack([r, np.cross(f, r), f], 1)


def coins(centre=CARTON):
    return np.array([centre + rz(CAP) @ (i * PROF / 2, j * LARG / 2, k * HAUT / 2)
                     for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])


def marge_coins(x, y, cap, centre=CARTON):
    """Marge (part de l'image) du coin du carton le plus près d'un bord."""
    cam, rot = camera(x, y, cap)
    p = (coins(centre) - cam) @ rot
    u, v = K.fx * p[:, 0] / p[:, 2] + K.cx, K.fy * p[:, 1] / p[:, 2] + K.cy
    return float(np.min([u / W, (W - 1 - u) / W, v / H, (H - 1 - v) / H]))


def touche(c, d, demi):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (-demi - c) / d, (demi - c) / d
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    return np.where((tmax >= tmin) & (tmax > 0), np.where(tmin > 0, tmin, np.inf), np.inf)


OBJETS = [(CARTON, np.array([PROF / 2, LARG / 2, HAUT / 2]), CAP),          # 0 : carton
          (np.array([4.0, 2.30, 0.78]), np.array([0.8, 0.3, 0.02]), 0.0)]   # 1 : plateau
_rendus = {}


def rendre(x, y, cap):
    """(profondeur, numéro de l'objet par pixel : -1 sol, 0 carton, 1 table)."""
    cle = (round(x, 4), round(y, 4), round(cap, 4))
    if cle not in _rendus:
        cam, rot = camera(x, y, cap)
        v, u = np.mgrid[0:H, 0:W]
        ray = np.stack([(u - K.cx) / K.fx, (v - K.cy) / K.fy, np.ones((H, W))], -1).reshape(-1, 3)
        d = ray @ rot.T
        s, ident = np.full(len(d), np.inf), np.full(len(d), -1)
        for i, (c, demi, lacet) in enumerate(OBJETS):
            t = touche((cam - c) @ rz(lacet), d @ rz(lacet), demi)
            ident = np.where(t < s, i, ident)
            s = np.minimum(s, t)
        with np.errstate(divide="ignore", invalid="ignore"):
            sol = np.where(d[:, 2] < 0, -cam[2] / d[:, 2], np.inf)
        ident = np.where(sol < s, -1, ident)
        _rendus[cle] = (np.minimum(s, sol).reshape(H, W), ident.reshape(H, W))
    return _rendus[cle]


class Banc:
    """Robot immobile en (x, y, cap) : caméras (couleur, profondeur),
    arbre de repères, détecteur et masque idéaux ; `mode` du détecteur :
    « carton » (son cadre, élargi de `large` pixels de chaque côté, comme
    ceux, un peu larges, d'un vrai détecteur), « fantome » (un cadre sur
    toute l'image), ou « rien »."""

    def __init__(self, x, y, cap=CAP, mode="carton", large=0):
        self.t, self.mode, self.large = 0.0, mode, large
        self.tree = FrameTree()
        a = math.atan2(-(VISEE[2] - MONTAGE[2]), VISEE[0] - MONTAGE[0])
        self.tree.set(Transform(FRAME_BASE, FRAME_CAMERA, translation=Vector3(*map(float, MONTAGE)),
                                rotation=Quaternion(w=math.cos(a / 2), x=0.0, y=math.sin(a / 2),
                                                    z=0.0)))
        self.tree.set(camera_optical_to_link())
        self.placer(x, y, cap)
        banc = self

        class Camera:
            def is_healthy(self):
                return True

            def initialize(self):
                return None

            def get_intrinsics(self):
                return K

            def capture(self):
                _, ident = rendre(*banc.pose)
                img = np.empty((H, W, 3), dtype=np.uint8)
                img[...] = SOL
                img[ident == 1] = GRIS
                img[ident == 0] = ROUGE
                return CameraFrame(W, H, "rgb8", img.tobytes(), timestamp_ns=int(banc.t * 1e9))

            def capture_depth(self):
                return rendre(*banc.pose)[0]

        class Detecteur:
            is_open_vocabulary = True
            stats = {}

            def detect(self, frame, query=None):
                _, ident = rendre(*banc.pose)
                if banc.mode == "rien":
                    return []
                if banc.mode == "fantome":
                    return [Detection(query, 0.2, (0, 0, W, H))]
                ys, xs = np.nonzero(ident == 0)
                x0, y0 = max(0, xs.min() - banc.large), max(0, ys.min() - banc.large)
                x1, y1 = min(W - 1, xs.max() + banc.large), min(H - 1, ys.max() + banc.large)
                return [Detection(query, 0.4, (int(x0), int(y0), int(x1 - x0 + 1),
                                               int(y1 - y0 + 1)))]

        class Masque:                        # ce que rend SAM 2.1, en mieux
            def masque(self, image, bbox):
                _, ident = rendre(*banc.pose)
                x, y, w, h = (int(v) for v in bbox)
                dans = np.zeros((H, W), dtype=bool)
                dans[y:y + h, x:x + w] = True
                # Cadre sur toute l'image : le plus grand segment, le sol.
                return dans & (ident == (-1 if banc.mode == "fantome" else 0))

        self.cam = Camera()
        self.service = PerceptionService(self.cam, self.tree, Detecteur(),
                                         depth_camera=self.cam, segmenteur=Masque())
        self.service.start()
        self.percep = PerceptionCouleur(self.service, self.cam, ["rouge", "bleu"],
                                        requetes=("a box",), dims=(LARG, PROF, HAUT))
        self.journal = []
        nav = SimpleNamespace(etat=EtatNavigation.ARRIVE, carte=None, annuler=lambda: None,
                              aller_vers=lambda dest: ServiceResult(ok=True))
        loco = SimpleNamespace(halt=lambda: None, set_velocity=lambda *a: None,
                               dernier_verdict=None, bande_morte=0.05)
        manip = SimpleNamespace(etat=Etat.LIBRE, mesures=None, reinitialiser=lambda *a: None)
        self.facade = FacadeServices(nav, loco, manip, self.percep, lambda: Pose2D(*self.pose),
                                     lambda: self.t, CAP, journal=self.journal.append,
                                     cfg=ConfigFacade(requete="{couleur}"))

    def placer(self, x, y, cap=CAP):
        self.pose = (x, y, cap)
        self.tree.set(Transform(FRAME_WORLD, FRAME_BASE, translation=Vector3(x, y, BASSIN_Z),
                                rotation=Quaternion(w=math.cos(cap / 2), x=0.0, y=0.0,
                                                    z=math.sin(cap / 2))))


section("1. Pose d'attente : le carton entier dans l'image")
b = Banc(4.0, 1.0)                                      # poste de prise de la scène T1
vus = b.facade.chercher("rouge")
check("Vu depuis le poste de prise", len(vus) == 1 and math.dist(vus[0].position, CARTON) < 0.02,
      str(vus))
colis = Colis("rouge", tuple(CARTON), 0.4)
cible, _ = b.facade._cible_prise(colis.position)
ok, _ = b.facade.accoster(colis)
attente = b.facade._apres_nav
recul = math.hypot(attente.x - cible.x, attente.y - cible.y)
m = marge_coins(attente.x, attente.y, attente.cap)
check(f"Pose d'attente calculée ({recul:.2f} m derrière la pose de prise) : les 8 coins du carton "
      f"dans l'image, à 5 % du bord au moins (calcul du test, indépendant)", ok and m >= 0.05,
      f"{recul:.2f} m, marge {m:.3f}")
c, s = math.cos(CAP), math.sin(CAP)
m1 = marge_coins(attente.x + 0.01 * c, attente.y + 0.01 * s, CAP)
check("… et la plus proche : 1 cm plus près, un coin passe à moins de 5 % du bord", m1 < 0.05,
      f"{m1:.3f}")
_, ident = rendre(attente.x, attente.y, CAP)
bord = np.concatenate([ident[0], ident[-1], ident[:, 0], ident[:, -1]])
check("… dans l'image rendue de cette pose, le carton ne touche aucun bord",
      (ident == 0).any() and not (bord == 0).any())
m28 = marge_coins(cible.x - 0.28 * c, cible.y - 0.28 * s, CAP)
check(f"Avant (recul fixe de 0.28 m, l'essai GPU) : des coins HORS de l'image (marge {m28:.2f})",
      m28 < 0.0)
check("… le journal dit où et pourquoi", any(l.startswith("pose d'attente à") and "8 coins" in l
                                            for l in b.journal), str(b.journal))
pre = b.facade._dest
check("… pré-pose jamais plus près que la pose d'attente",
      math.hypot(pre.x - cible.x, pre.y - cible.y) >= recul - 1e-9, f"{pre} {attente}")

section("2. Fantômes : géométrie, pas fraction d'image")
# Le plus près où il remplit le plus l'image : 0,48 m (bassin -> centre),
# carton coupé par le bord bas ; cadre du détecteur élargi de 14 pixels.
b = Banc(CARTON[0], CARTON[1] - 0.48, large=14)
r = b.percep.find("rouge", now_ns=0)
e = b.percep.diagnostic["detections"][0]
check(f"Carton proche, cadre sur {100 * e['part_image']:.0f} % de l'image, coupé par le bord "
      f"{', '.join(e['bords'])} : GARDÉ (avant : « écartée : trop grande », au-delà de 25 %)",
      e["part_image"] >= 0.40 and len(r.value) == 1 and not e.get("rejet"), str(e))
b = Banc(4.0, 1.0, mode="fantome")
r = b.percep.find("bleu", now_ns=0)
e = b.percep.diagnostic["detections"][0]
check("Cadre sur toute l'image, sol bleu (le fantôme constaté sur GPU) : écarté, pas de carton "
      "bleu fantôme", r.ok and r.value == [] and "quatre bords" in e["couleur"], str(e))
# Même fantôme, cadre retenu d'un pixel sur chaque bord : sa taille métrique
# (le sol entier, des mètres) le trahit.
b.service._detector.detect = lambda frame, query=None: [Detection(query, 0.2, (2, 2, W - 4, H - 4))]
r = b.percep.find("bleu", now_ns=1)
e = b.percep.diagnostic["detections"][0]
check("… cadre décollé des bords : écarté par sa taille métrique (masque et profondeur)",
      r.value == [] and "trop grand pour un carton" in e["couleur"], str(e))

section("3. Raison exacte de chaque image d'observation écartée")


def observer(banc, couleur="rouge", duree=5.0):
    f = banc.facade
    f._colis_vise = Colis(couleur, tuple(CARTON), 0.4, CAP, vu="visée")
    f._observation_requise = True
    f._insertion, _ = f._cible_prise(tuple(CARTON))
    f._phase, f._etat_accostage = "OBSERVATION", "EN_ROUTE"
    banc.t += 0.05
    f._debut_observation()
    while banc.t < duree and f._etat_accostage == "EN_ROUTE":
        banc.t += 0.05
        f.tick()
    return f._etat_accostage


for mode, attendu in (("rien", "n°1–"),
                      ("fantome", "écartée : cadre sur les quatre bords de l'image")):
    b = Banc(attente.x, attente.y, mode=mode)
    etat = observer(b)
    ligne = next((l for l in b.journal if "pas revu de façon cohérente" in l), "")
    check(f"Détecteur « {mode} » : observation en échec, chaque image écartée avec sa raison",
          etat == "ECHEC" and attendu in ligne and "carton pas vu" not in ligne, ligne)
b = Banc(attente.x, attente.y, mode="rien")
observer(b)
ligne = next((l for l in b.journal if "pas revu de façon cohérente" in l), "")
check("… « aucune détection », images consécutives regroupées (n°1–N)",
      "images écartées : n°1–" in ligne and ": aucune détection" in ligne, ligne)
b = Banc(attente.x, attente.y)
observer(b, couleur="bleu")
ligne = next((l for l in b.journal if "pas revu de façon cohérente" in l), "")
check("… carton d'une autre couleur : « couleur lue : rouge », avec la mesure des pixels",
      "couleur lue : rouge (RGB moyen" in ligne, ligne)
b = Banc(attente.x, attente.y)
etat = observer(b)
check("Carton bien cadré, détecteur et masque idéaux : revu après l'arrêt (3 images), à 1 cm "
      "près", b.facade._observe is not None and "3 images prises après l'arrêt"
      in b.facade._observe.vu and math.dist(b.facade._observe.position, CARTON) < 0.01,
      f"{etat} {b.journal}")

section("4. La recherche du carton ne part qu'une fois la base immobile")
# Constat GPU (essai T1 de main) : deux recherches lancées dès l'arrivée de la
# navigation au poste, tête encore en rotation, ont situé le carton à +2.1 et
# -3.3 cm de côté ; la visée, robot arrêté depuis 0.6 s, les a corrigées.
from axis_os.apps.tri_colis.logique import ConfigTri, TriColis  # noqa: E402

b = Banc(4.0, 1.0)
f = b.facade
moments = {"nav": None, "recherches": [], "consignes": [], "arrets": []}
f.loco.set_velocity = lambda *a: moments["consignes"].append(b.t)
f.loco.halt = lambda: moments["arrets"].append(b.t)
aller, trouver = f.aller_au_poste, b.percep.find


def aller_au_poste(poste):
    moments["nav"] = b.t                      # la navigation (factice) arrive aussitôt
    return aller(poste)


def find(*a, **k):
    moments["recherches"].append((b.t, f.veut_percevoir))
    return trouver(*a, **k)


f.aller_au_poste, b.percep.find = aller_au_poste, find
tri = TriColis(f, ConfigTri(regles={"rouge": "A"}, duree_mission_s=1e6))
percevoir = []
while b.t < 3.0 and not moments["recherches"]:
    b.t += 0.02
    tri.pas()
    f.tick()
    percevoir.append((b.t, f._phase, f.veut_percevoir))
t_nav, t_rech = moments["nav"], moments["recherches"][0][0] if moments["recherches"] else None
duree = ConfigFacade().duree_visee_s
check(f"Arrivée au poste de prise : la recherche attend {duree} s, base arrêtée (le critère de "
      f"la visée)", t_nav is not None and t_rech is not None
      and t_rech - t_nav >= duree - 1e-9, f"navigation à {t_nav}, recherche à {t_rech}")
check("… pendant l'attente : aucune consigne de marche, la base mise à l'arrêt à chaque pas",
      t_rech is not None and not [t for t in moments["consignes"] if t_nav <= t < t_rech]
      and len([t for t in moments["arrets"] if t_nav < t < t_rech]) >= 25,
      str(moments["consignes"][:3]))
arret = [(t, v) for t, ph, v in percevoir if ph == "ARRET"]
check("… et images neuves demandées (veut_percevoir) tout l'arrêt, assez longtemps",
      arret and all(v for _, v in arret) and arret[-1][0] - arret[0][0] >= duree - 0.05,
      str(percevoir[:5]))
check("… puis la recherche trouve le carton, et l'accostage part", tri.colis is not None
      and tri.etat == "ACCOSTER", f"{tri.etat} {tri.colis}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
