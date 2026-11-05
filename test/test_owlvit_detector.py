"""
OwlViTDetector — contrat et dégradation, SANS le modèle.

Ce que ce test prouve : que le détecteur respecte IObjectDetector et
qu'il ne casse rien quand torch/transformers sont absents ou que le
modèle n'est pas chargé.

Ce qu'il ne prouve PAS : qu'OWL-ViT reconnaît quoi que ce soit. Cela
demande le modèle et de vraies images — voir
sim/validation/test_isaac_p2_detection.py.
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

section("6 — Détecteur des cartons : OWLv2 par défaut, OWL-ViT en repli")
# Verdict du banc de perception (essai GPU T1, novembre 2026) : OWLv2 au
# seuil 0,10 (30 cartons sur 30, 47 fausses détections), contre OWL-ViT à
# 0,06 (30 sur 30, 66). Chargement simulé : pas de poids ici.
from axis_os.services.perception.owlvit_detector import (  # noqa: E402
    Owlv2Detector, detecteur_carton)


def charge(ok):
    def load(self):
        self._load_error = None if ok else "poids absents"
        return ok
    return load


charges = {OwlViTDetector: OwlViTDetector.load, Owlv2Detector: Owlv2Detector.load}
messages = []
try:
    Owlv2Detector.load, OwlViTDetector.load = charge(True), charge(True)
    det, nom, seuil = detecteur_carton(journal=messages.append)
    check("Par défaut : OWLv2, seuil 0,10 (celui du banc)",
          type(det) is Owlv2Detector and nom == "owlv2" and seuil == 0.10 and det._threshold == 0.10)
    det, nom, seuil = detecteur_carton(seuil=0.12, journal=messages.append)
    check("… seuil donné : appliqué à OWLv2", seuil == 0.12 and det._threshold == 0.12)
    Owlv2Detector.load = charge(False)
    det, nom, seuil = detecteur_carton(seuil=0.12, journal=messages.append)
    check("OWLv2 ne se charge pas : repli sur OWL-ViT, à SON seuil validé (0,06), et on le dit",
          type(det) is OwlViTDetector and nom == "owlvit" and seuil == 0.06
          and det._threshold == 0.06 and any("repli sur owlvit" in m and "poids absents" in m
                                             for m in messages))
    OwlViTDetector.load = charge(False)
    det, nom, seuil = detecteur_carton(journal=messages.append)
    check("Aucun ne se charge : le dernier essayé, non chargé, avec sa raison",
          nom == "owlvit" and det.stats["load_error"] == "poids absents")
finally:
    OwlViTDetector.load, Owlv2Detector.load = charges[OwlViTDetector], charges[Owlv2Detector]

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
if FAIL == 0:
    print("\n  ✓ Contrat IObjectDetector respecté, vocabulaire ouvert")
    print("  ✓ Dégradation propre sans torch/transformers")
    print("  ⚠ La reconnaissance réelle reste à valider avec le modèle")

sys.exit(0 if FAIL == 0 else 1)
