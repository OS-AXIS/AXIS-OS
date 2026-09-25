"""
La copie de l'URDF 29 DOF avec collision des mains doit rester FIDÈLE à
l'officiel : seules les deux mains diffèrent, et elle est à jour.

Une copie dérivée qui dériverait en silence de sa source serait pire
qu'un fichier modifié à la main : on la croirait conforme.
"""
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from axis_os.hal.drivers.unitree_g1.urdf.deriver_mains_collision import (
    CIBLE, MAINS, SOURCE, deriver,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


off = ET.parse(SOURCE).getroot()
der = ET.parse(CIBLE).getroot()

check("Dans l'URDF OFFICIEL, les mains n'ont aucune collision — c'est le "
      "défaut que corrige la copie",
      all(l.find("collision") is None for l in off.findall("link")
          if l.get("name") in MAINS))
check("Dans la copie, les deux mains ont une collision",
      all(l.find("collision") is not None for l in der.findall("link")
          if l.get("name") in MAINS))


def hors_mains(r):
    return [ET.tostring(x) for x in r
            if not (x.tag == "link" and x.get("name") in MAINS)]


check("Tout le reste est STRICTEMENT identique à l'officiel",
      hors_mains(off) == hors_mains(der))

with tempfile.TemporaryDirectory() as d:
    tmp = Path(d) / "regen.urdf"
    deriver(SOURCE, tmp)
    check("La copie versionnée est À JOUR : la régénérer donne le même "
          "résultat", ET.tostring(ET.parse(tmp).getroot())
          == ET.tostring(der))

print(f"\n  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
sys.exit(0 if FAIL == 0 else 1)
