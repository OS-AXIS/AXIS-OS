"""
Collecteur pytest pour les scripts de test d'AXIS-OS.

Pourquoi ce fichier : chaque test/test_*.py est un script autonome. Il
affiche ses vérifications une à une ([✓] ou [✗]) et se termine par
sys.exit(0) si tout passe, sys.exit(1) sinon. Importé par pytest, ce
sys.exit arrêterait la collecte de toute la suite. Ici, chaque script est
lancé dans son propre processus, depuis la racine du dépôt, exactement comme
avec « python3 test/test_x.py » ; son code de sortie décide du résultat.

Un script = un test pytest. La sortie du script est jointe au rapport : en
cas d'échec, pytest montre les lignes [✗] qui ont échoué.

Ressources rares : un script qui demande Isaac Sim, un GPU ou un gros modèle
le déclare dans une ligne de commentaire, parmi ses 40 premières lignes :

    # pytest-marqueurs : isaac, gpu

Sans la ressource, le test est SAUTÉ (avec la raison), pas compté comme
réussi. Aucun script de test/ n'en déclare aujourd'hui : les 87 tournent sur
une machine sans GPU, et la CI les exécute tous.
"""
import importlib.util
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

RACINE = Path(__file__).resolve().parents[1]

# Durée maximale d'un script. Le plus long (test_heavy_integration.py) prend
# environ 2 minutes 20 ; la marge couvre une machine de CI plus lente sans
# laisser un script bloqué occuper la CI pendant des heures.
DELAI_MAX_S = 900

LIGNE_MARQUEURS = re.compile(r"^#\s*pytest-marqueurs\s*:\s*(.+)$")
MARQUEURS_CONNUS = ("isaac", "gpu", "gros_modele")


def _raison_de_sauter(marqueur):
    """Rend la raison de sauter le test si la ressource manque, sinon None."""
    if marqueur == "isaac" and importlib.util.find_spec("isaacsim") is None:
        return "Isaac Sim absent (module isaacsim introuvable)"
    if marqueur == "gpu" and shutil.which("nvidia-smi") is None:
        return "pas de GPU NVIDIA (nvidia-smi introuvable)"
    if marqueur == "gros_modele" and os.environ.get("AXIS_OS_GROS_MODELES") != "1":
        return "gros modèle non activé (AXIS_OS_GROS_MODELES=1 pour le lancer)"
    return None


def _marqueurs_declares(chemin):
    with open(chemin, encoding="utf-8") as f:
        for _, ligne in zip(range(40), f):
            m = LIGNE_MARQUEURS.match(ligne.strip())
            if m:
                noms = [n.strip() for n in m.group(1).split(",") if n.strip()]
                inconnus = [n for n in noms if n not in MARQUEURS_CONNUS]
                if inconnus:
                    # Une faute de frappe ne doit pas faire sauter un test en
                    # silence, ni le faire tourner là où il ne peut pas.
                    raise pytest.UsageError(
                        f"{chemin.name} : marqueur(s) inconnu(s) {inconnus}, "
                        f"connus : {MARQUEURS_CONNUS}")
                return noms
    return []


class EchecDuScript(Exception):
    """Le script s'est terminé avec un code de sortie non nul."""


class ScriptDeTest(pytest.Item):
    def __init__(self, *, marqueurs, **kwargs):
        super().__init__(**kwargs)
        for nom in marqueurs:
            self.add_marker(nom)
        self.marqueurs = marqueurs

    def setup(self):
        for nom in self.marqueurs:
            raison = _raison_de_sauter(nom)
            if raison:
                pytest.skip(raison)

    def runtest(self):
        try:
            r = subprocess.run([sys.executable, str(self.path)], cwd=str(RACINE),
                               capture_output=True, text=True, timeout=DELAI_MAX_S)
        except subprocess.TimeoutExpired as e:
            sortie = e.stdout.decode("utf-8", "replace") if isinstance(e.stdout, bytes) \
                else (e.stdout or "")
            self.add_report_section("call", "stdout", sortie)
            raise EchecDuScript(f"arrêté après {DELAI_MAX_S} s (délai dépassé)") from None
        self.add_report_section("call", "stdout", r.stdout)
        if r.stderr:
            self.add_report_section("call", "stderr", r.stderr)
        if r.returncode != 0:
            echecs = [l.strip() for l in r.stdout.splitlines() if "[✗]" in l]
            raise EchecDuScript(f"code de sortie {r.returncode}" + (
                "\n" + "\n".join(echecs[:20]) if echecs else ""))

    def repr_failure(self, excinfo):
        if excinfo.errisinstance(EchecDuScript):
            return f"{self.path.name} : {excinfo.value}"
        return super().repr_failure(excinfo)

    def reportinfo(self):
        return self.path, 0, f"script {self.path.name}"


class FichierScript(pytest.File):
    def collect(self):
        yield ScriptDeTest.from_parent(self, name=self.path.stem,
                                       marqueurs=_marqueurs_declares(self.path))


def pytest_pycollect_makemodule(module_path, parent):
    """Remplace le module Python ordinaire par un FichierScript.

    Ce crochet est appelé pour chaque test_*.py trouvé, et aussi pour un
    fichier nommé directement (« pytest test/test_proximite.py ») : les deux
    cas lancent donc le script dans son processus, sans jamais l'importer.
    """
    if module_path.parent == Path(__file__).parent and module_path.name.startswith("test_"):
        return FichierScript.from_parent(parent, path=module_path)
    return None
