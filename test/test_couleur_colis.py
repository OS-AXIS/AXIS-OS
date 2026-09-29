"""
Couleur des colis : lue sur les pixels, jamais devinée.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis.couleur import PerceptionCouleur, couleur_boite, couleur_de
from axis_os.apps.tri_colis.scene_demo import COULEURS_RGB
from axis_os.services.base import ServiceResult
from axis_os.services.perception.service import Detection

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


section("1 — Teintes")
for nom, rgb in COULEURS_RGB.items():
    for eclairage in (1.0, 0.6, 0.35):          # ombre, lumière faible
        c = couleur_de(tuple(v * eclairage for v in rgb))
        check(f"{nom} de la scène, éclairage ×{eclairage} -> {c}", c == nom)
check("Constaté sur GPU : rouge DÉLAVÉ par l'éclairage (rose) -> rouge",
      couleur_de((1.0, 0.62, 0.62)) == "rouge")
check("… bleu délavé (bleu ciel) -> bleu", couleur_de((0.6, 0.78, 1.0)) == "bleu")
check("Blanc bleuté, gris clair : toujours rien",
      couleur_de((0.88, 0.92, 0.97)) is None and couleur_de((0.85, 0.85, 0.9)) is None)
check("Carton brun (0.72, 0.52, 0.30) : PAS une couleur de tri",
      couleur_de((0.72, 0.52, 0.30)) is None)
check("Gris de table : rien", couleur_de((0.45, 0.45, 0.50)) is None)
check("Noir : rien", couleur_de((0.05, 0.02, 0.02)) is None)


def image(l, h, fond, boites):
    """Image rgb8 : fond uni, rectangles colorés."""
    px = bytearray()
    for y in range(h):
        for x in range(l):
            c = fond
            for (bx, by, bw, bh), col in boites:
                if bx <= x < bx + bw and by <= y < by + bh:
                    c = col
            px += bytes(int(v * 255) for v in c)
    return bytes(px)


GRIS = (0.45, 0.45, 0.50)
L, H = 160, 120
img = image(L, H, GRIS, [((20, 40, 40, 40), COULEURS_RGB["rouge"]),
                         ((100, 40, 40, 40), COULEURS_RGB["bleu"])])

section("2 — Boîte englobante")
check("Boîte serrée sur le rouge -> rouge", couleur_boite(img, L, H, (20, 40, 40, 40)) == "rouge")
check("Boîte trop large (déborde sur la table) : le centre suffit -> rouge",
      couleur_boite(img, L, H, (5, 25, 70, 70)) == "rouge")
check("Boîte à cheval sur deux cartons : pas de majorité -> None",
      couleur_boite(img, L, H, (40, 40, 80, 40)) is None)
check("Boîte sur la table grise -> None", couleur_boite(img, L, H, (60, 0, 30, 30)) is None)
NOIR = (0.03, 0.03, 0.03)
img2 = image(L, H, GRIS, [((20, 30, 40, 8), COULEURS_RGB["rouge"]),   # rebord éclairé
                          ((20, 38, 40, 42), NOIR),                     # face à l'ombre
                          ((100, 30, 40, 8), COULEURS_RGB["bleu"]),
                          ((100, 38, 40, 42), NOIR)])
check("Constaté sur GPU : face à l'OMBRE (noire), seul le rebord coloré -> rouge",
      couleur_boite(img2, L, H, (20, 30, 40, 50)) == "rouge")
check("… et bleu pour l'autre", couleur_boite(img2, L, H, (100, 30, 40, 50)) == "bleu")
check("Objet entièrement noir : pas de couleur de tri -> None",
      couleur_boite(img2, L, H, (20, 40, 40, 38)) is None)
check("Boîte hors de l'image -> None", couleur_boite(img, L, H, (500, 500, 20, 20)) is None)

section("3 — PerceptionCouleur : même interface que le service")


class PerceptionFactice:
    def __init__(self):
        self.appels = 0

    def find(self, query, now_ns=None, frame=None):
        self.appels += 1
        return ServiceResult(ok=True, value=[
            Detection(query, 0.4, (20, 40, 40, 40)),
            Detection(query, 0.6, (100, 40, 40, 40)),
            Detection(query, 0.3, (60, 0, 30, 30))])       # table : sans couleur


cam = SimpleNamespace(capture=lambda: SimpleNamespace(data=img, width=L, height=H))
pf = PerceptionFactice()
pc = PerceptionCouleur(pf, cam, ["rouge", "bleu", "vert"])
r = pc.find("rouge", now_ns=1)
check("find('rouge') : une détection, la bonne", r.ok and len(r.value) == 1
      and r.value[0].bbox == (20, 40, 40, 40))
r = pc.find("bleu", now_ns=1)
check("find('bleu') : la bleue", len(r.value) == 1 and r.value[0].bbox == (100, 40, 40, 40))
check("Même image, deux couleurs : l'IA n'a tourné qu'une fois par requête",
      pf.appels == 2)
check("find('vert') : rien — il n'y a pas de vert", pc.find("vert", now_ns=1).value == [])
pc.find("rouge", now_ns=2)
check("Image suivante : nouvelle inférence", pf.appels == 4)


class PerceptionEnPanne:
    def find(self, *a, **k):
        return ServiceResult(ok=False, reason="détecteur en panne")


SOL = (0.45, 0.70, 0.85)                 # le sol bleu de la scène GPU
img3 = image(L, H, SOL, [((60, 40, 40, 40), COULEURS_RGB["rouge"])])


class PerceptionImageEntiere:
    def find(self, query, now_ns=None, frame=None):
        return ServiceResult(ok=True, value=[Detection(query, 0.2, (0, 0, L, H)),
                                             Detection(query, 0.3, (60, 40, 40, 40))])


cam3 = SimpleNamespace(capture=lambda: SimpleNamespace(data=img3, width=L, height=H))
pc3 = PerceptionCouleur(PerceptionImageEntiere(), cam3, ["rouge", "bleu"])
check("Détection couvrant toute l'image (sol bleu) : écartée, PAS de carton bleu fantôme",
      pc3.find("bleu", now_ns=1).value == [])
check("… le vrai carton rouge, lui, est gardé", len(pc3.find("rouge", now_ns=1).value) == 1)

r = PerceptionCouleur(PerceptionEnPanne(), cam, ["rouge"]).find("rouge", now_ns=1)
from axis_os.apps.tri_colis.couleur import sans_doublons
d = sans_doublons([Detection("a", 0.3, (0, 0, 40, 40)), Detection("b", 0.5, (2, 1, 40, 40)),
                   Detection("c", 0.2, (80, 0, 40, 40))])
check("Doublons (deux formulations, même boîte) : un seul gardé, le plus sûr",
      len(d) == 2 and d[0].confidence == 0.5)
check("Détecteur en panne : l'échec REMONTE (zéro détection ≠ scène vide)",
      not r.ok and "panne" in r.reason)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
