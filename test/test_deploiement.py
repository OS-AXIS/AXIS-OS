"""
Scripts de déploiement (deploy/) — vérifications statiques, sans GPU.

Ces scripts tournent sur des VM Vast.ai (Ubuntu 22.04) : on ne peut pas les
exécuter ici. On vérifie ce qui peut l'être sans machine :
  - chaque script est syntaxiquement valide pour bash ;
  - les versions épinglées concordent avec requirements.txt — une version
    épinglée à un endroit et libre à un autre, c'est la version du jour sur
    chaque nouvelle machine (cause de la panne de perception de transformers
    5.x, audit du 2026-10-10).
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE))

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


SCRIPTS = sorted((RACINE / "deploy").rglob("*.sh"))
INSTALLER = (RACINE / "deploy/machine_gpu/installer.sh").read_text(encoding="utf-8")
REQUIREMENTS = (RACINE / "requirements.txt").read_text(encoding="utf-8")


def version_requirements(paquet):
    m = re.search(rf"^{re.escape(paquet)}==([\w.]+)\s*$", REQUIREMENTS, re.M)
    return m.group(1) if m else None


section("1 — Syntaxe bash de chaque script")

if shutil.which("bash"):
    check(f"{len(SCRIPTS)} scripts trouvés dans deploy/", len(SCRIPTS) >= 10)
    for s in SCRIPTS:
        r = subprocess.run(["bash", "-n", str(s)], capture_output=True, text=True)
        check(f"bash -n {s.relative_to(RACINE)}", r.returncode == 0)
else:
    print("  [—] bash absent : syntaxe non vérifiable ici")

section("2 — transformers : même version épinglée partout")

v = version_requirements("transformers")
check(f"requirements.txt épingle transformers exactement ({v})", v is not None)
installs = re.findall(r"pip install[^\n]*(?:\\\n[^\n]*)*", INSTALLER)
mentions = [m for i in installs for m in re.findall(r"transformers[^\s\"']*", i)]
check(f"installer.sh installe transformers=={v} ({mentions})",
      mentions == [f"transformers=={v}"])

section("3 — uv : version épinglée, installeur vérifié avant exécution")

ENTRAINER = (RACINE / "deploy/machine_gpu/entrainer_vla.sh").read_text(encoding="utf-8")
tous = "\n".join(f.read_text(encoding="utf-8") for f in SCRIPTS)
check("Plus de « astral.sh/uv/install.sh | sh » (version du jour, sans contrôle)",
      "astral.sh/uv/install.sh" not in tous)
v_uv = re.search(r'^UV_VERSION="(\d+\.\d+\.\d+)"', ENTRAINER, re.M)
h_uv = re.search(r'^UV_INSTALLEUR_SHA256="([0-9a-f]{64})"', ENTRAINER, re.M)
check(f"Version de uv épinglée ({v_uv.group(1) if v_uv else None})", v_uv is not None)
check("Empreinte SHA-256 de l'installeur présente", h_uv is not None)
check("L'installeur est VÉRIFIÉ (sha256sum -c) avant d'être exécuté",
      ENTRAINER.find("sha256sum -c") != -1
      and ENTRAINER.find("sha256sum -c") < ENTRAINER.find("sh /tmp/uv-installer.sh"))
check("… et son URL porte la version épinglée",
      "releases/download/$UV_VERSION/uv-installer.sh" in ENTRAINER)
check("Aucun « curl … | sh » non commenté, hors get.docker.com (script officiel, gardé "
      "et justifié)",
      all("get.docker.com" in l for l in tous.splitlines()
          if re.search(r"curl[^|#]*\|\s*(sudo\s+)?sh\b", l) and not l.lstrip().startswith("#")))
check("get.docker.com : le pourquoi est écrit à côté",
      "script d'installation OFFICIEL de Docker" in INSTALLER)

section("4 — Paquets pip épinglés là où c'est possible sans casser")

EVALUER = (RACINE / "deploy/machine_gpu/evaluer_vla.sh").read_text(encoding="utf-8")
RECUPERER = (RACINE / "deploy/machine_gpu/recuperer_vla.sh").read_text(encoding="utf-8")
for texte, nom, paquets in (
        (EVALUER, "evaluer_vla.sh", ("pyzmq", "msgpack", "msgpack-numpy")),
        (RECUPERER, "recuperer_vla.sh", ("pandas", "pyarrow", "imageio[ffmpeg]")),
        (INSTALLER, "installer.sh", ("transformers", "imageio[ffmpeg]", "imageio-ffmpeg"))):
    for paq in paquets:
        check(f"{nom} : {paq} en version exacte",
              re.search(rf'"{re.escape(paq)}==[\d.]+"', texte) is not None)
check("recuperer_vla.sh : versions compatibles avec le Python 3.10 d'Ubuntu 22.04 "
      "(pandas 2.x : la 3.x exige 3.11)", '"pandas==2.' in RECUPERER)
check("installer.sh : pillow en plancher seulement (celui d'Isaac Sim n'est pas remplacé)",
      '"pillow>=10.0.0"' in INSTALLER)
check("installer.sh : torch et onnxruntime non épinglés, et le pourquoi est écrit",
      "torch NON épinglé" in INSTALLER and "onnxruntime-gpu : NON épinglé" in INSTALLER)
check("installer.sh : --network=host gardé, avec sa raison",
      "--network=host GARDÉ" in INSTALLER and "--network=host" in INSTALLER)

section("5 — OWL-ViT : même révision épinglée au pré-téléchargement et au chargement")

from axis_os.services.perception.owlvit_detector import OWLVIT_REVISION  # noqa: E402

r_inst = re.search(r'^OWLVIT_REVISION="([0-9a-f]{40})"', INSTALLER, re.M)
check(f"installer.sh épingle la révision ({r_inst.group(1)[:12] if r_inst else None}…)",
      r_inst is not None)
check("… identique à OWLVIT_REVISION du détecteur",
      r_inst is not None and r_inst.group(1) == OWLVIT_REVISION)
pre = INSTALLER[INSTALLER.find("Pré-télécharge OWL-ViT"):]
check("Le pré-téléchargement passe la révision et use_safetensors=True",
      "revision=r" in pre and "use_safetensors=True" in pre)

section("5 — gh récent depuis le dépôt officiel, clé vérifiée ; vidéo réduite")
gh = INSTALLER[INSTALLER.find("GH_EMPREINTES="):INSTALLER.find('etape "3/9')]
check("installer.sh : gh depuis cli.github.com, plus le gh d'Ubuntu",
      "https://cli.github.com/packages stable main" in gh
      and "apt-get install -y -qq gh || true" not in INSTALLER)
check("… empreinte de la clé comparée AVANT de l'installer dans les trousseaux d'apt",
      0 <= gh.find("GH_EMPREINTES") < gh.find("gpg --show-keys") < gh.find("clé inattendue")
      < gh.find("/etc/apt/keyrings/githubcli-archive-keyring.gpg"))
check("… et la version installée est vérifiée (2.40 au moins)",
      "ge 2.40" in gh and gh.count("gh_recent") >= 3)
RECUP_T1 = (RACINE / "deploy/machine_gpu/recuperer_t1.sh").read_text(encoding="utf-8")
check("recuperer_t1.sh : vidéo réduite en 540p, zippée avec le journal",
      'scale=-2:540' in RECUP_T1 and "zipfile -c" in RECUP_T1 and 't1_$H.log' in RECUP_T1)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
