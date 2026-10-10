"""
OWL-ViT — poids épinglés et au format safetensors (audit du 2026-10-10, I7).

Sans réseau ni modèle : de faux modules torch et transformers notent les
arguments de from_pretrained. Vérifié :
  - par défaut : révision OWLVIT_REVISION et use_safetensors=True ;
  - sans révision, ou avec un nom de branche : chargement REFUSÉ, avant
    tout téléchargement ;
  - un autre modèle n'hérite pas de la révision par défaut ;
  - le refus se lève seulement en simulation, explicitement, et le dit.
"""
import contextlib
import io
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


APPELS = []


class _FauxModele:
    def to(self, device):
        return self

    def eval(self):
        return self


class _FauxChargeur:
    def __init__(self, nom):
        self.nom = nom

    def from_pretrained(self, modele, **kw):
        APPELS.append((self.nom, modele, dict(kw)))
        return _FauxModele()


faux_torch = types.ModuleType("torch")
faux_torch.cuda = types.SimpleNamespace(is_available=lambda: False)
faux_tf = types.ModuleType("transformers")
faux_tf.OwlViTProcessor = _FauxChargeur("processor")
faux_tf.OwlViTForObjectDetection = _FauxChargeur("modele")
sys.modules["torch"] = faux_torch
sys.modules["transformers"] = faux_tf

from axis_os.services.perception.owlvit_detector import (  # noqa: E402
    MODELE_OWLVIT, OWLVIT_REVISION, OwlViTDetector,
)


def charger(**kw):
    APPELS.clear()
    d = OwlViTDetector(device="cpu", **kw)
    with contextlib.redirect_stdout(io.StringIO()) as sortie:
        ok = d.load()
    return d, ok, sortie.getvalue()


print("\n  OWL-ViT : poids épinglés\n")
check(f"Révision épinglée : commit complet ({OWLVIT_REVISION[:12]}…)",
      len(OWLVIT_REVISION) == 40 and OWLVIT_REVISION == "85fa9bec488a03a73b7f29206b1550551d0a7556")

d, ok, sortie = charger()
modele = [a for a in APPELS if a[0] == "modele"]
proc = [a for a in APPELS if a[0] == "processor"]
check("Par défaut : chargé", ok is True)
check("Processor et modèle chargés à la révision épinglée",
      proc and modele and proc[0][2].get("revision") == OWLVIT_REVISION
      and modele[0][2].get("revision") == OWLVIT_REVISION)
check("Poids du modèle au format safetensors (use_safetensors=True)",
      modele[0][2].get("use_safetensors") is True)
check("La révision est visible dans les stats et le journal",
      d.stats["revision"] == OWLVIT_REVISION and OWLVIT_REVISION in sortie)

for nom, rev in {"None": None, "« main » (branche)": "main", "hash court": "85fa9bec",
                 "hash invalide": "Z" * 40}.items():
    d, ok, sortie = charger(revision=rev)
    check(f"Révision {nom} : chargement REFUSÉ, rien téléchargé",
          ok is False and APPELS == [] and "NON épinglés" in (d.stats["load_error"] or ""))

d, ok, _ = charger(model_name="google/owlvit-large-patch14")
check("Autre modèle sans révision donnée : refusé (la révision par défaut n'est pas "
      "la sienne)", ok is False and APPELS == [])

d, ok, sortie = charger(revision=None, autoriser_non_epingle_SIMULATION=True)
check("Non épinglé autorisé EXPLICITEMENT en simulation : chargé, avec avertissement",
      ok is True and "SIMULATION" in sortie and "NON épinglés" in sortie)
check("… toujours en safetensors", [a for a in APPELS if a[0] == "modele"][0][2]
      .get("use_safetensors") is True)
check("Le modèle par défaut reste celui de Google", MODELE_OWLVIT == "google/owlvit-base-patch32")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
