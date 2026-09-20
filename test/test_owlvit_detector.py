"""
OwlViTDetector — contrat et dégradation, SANS le modèle.

Ce que ce test prouve : que le détecteur respecte IObjectDetector et
qu'il ne casse rien quand torch/transformers sont absents ou que le
modèle n'est pas chargé.

Ce qu'il ne prouve PAS : qu'OWL-ViT reconnaît quoi que ce soit. Cela
demande le modèle et de vraies images — voir
test_isaac_p2_detection.py.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import CameraFrame
from axis_os.services.perception.owlvit_detector import OwlViTDetector
from axis_os.services.perception.service import (
    IObjectDetector, NullObjectDetector, PerceptionService,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


section("1 — Contrat IObjectDetector")

d = OwlViTDetector()
check("OwlViTDetector implémente IObjectDetector",
      isinstance(d, IObjectDetector))
check("Aucune méthode abstraite restante",
      not getattr(d, "__abstractmethods__", None))
check("is_open_vocabulary est True — c'est tout l'intérêt du modèle",
      d.is_open_vocabulary is True)
check("is_loaded est False avant load()", d.is_loaded is False)

section("2 — Le chargement est SÉPARÉ du constructeur")

# Construire ne doit rien télécharger ni charger : sinon impossible
# d'inspecter la configuration d'un service sans payer plusieurs
# secondes et un téléchargement de poids.
d2 = OwlViTDetector(threshold=0.25, max_detections=5)
check("Construire un détecteur ne charge PAS le modèle",
      d2.is_loaded is False)
check("La configuration est lisible sans chargement",
      d2.stats["model"] == "google/owlvit-base-patch32")

section("3 — Dégradation sans dépendances ni modèle")

loaded = d.load()
if loaded:
    check("Modèle chargé (torch/transformers présents)", True)
else:
    check("load() rend False proprement, sans exception", loaded is False)
    check("L'erreur de chargement est explicite",
          d.stats["load_error"] is not None)

check("detect() sans modèle rend [] plutôt que de lever — une raison "
      "d'infrastructure ne doit pas faire échouer une mission",
      d.detect(None) == [])

frame = CameraFrame(64, 48, "rgb8", b"\x00" * (64 * 48 * 3),
                    timestamp_ns=1)
check("detect() sur une image valide sans modèle rend [] aussi",
      d.detect(frame) == [])

section("4 — Encodage non supporté : refuser, pas deviner")

depth = CameraFrame(64, 48, "depth16", b"\x00" * (64 * 48 * 2),
                    timestamp_ns=1)
check("Une image depth16 n'est PAS interprétée comme du RGB — des "
      "détections absurdes présentées comme valides seraient pires "
      "que rien",
      d.detect(depth) == [])

section("5 — Interchangeabilité avec les autres détecteurs")

owl_api = {m for m in dir(d) if not m.startswith("_")}
null_api = {m for m in dir(NullObjectDetector()) if not m.startswith("_")}
manquant = (null_api & {"detect", "is_open_vocabulary"}) - owl_api
check(f"Expose tout le contrat de NullObjectDetector "
      f"(manquant: {manquant or 'aucun'})", not manquant)

# Le service doit accepter le détecteur sans rien savoir de lui.
from axis_os.hal.drivers.sim_common.sim_perception import FakeCamera
svc = PerceptionService(FakeCamera("c", Path("/tmp/axisos_inexistant")),
                        None, d)
check("PerceptionService accepte OwlViTDetector sans modification",
      svc._detector is d)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
if FAIL == 0:
    print("\n  ✓ Contrat IObjectDetector respecté, vocabulaire ouvert")
    print("  ✓ Dégradation propre sans torch/transformers")
    print("  ⚠ La reconnaissance réelle reste à valider avec le modèle")

sys.exit(0 if FAIL == 0 else 1)
