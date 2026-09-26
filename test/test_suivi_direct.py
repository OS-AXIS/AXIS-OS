"""
Suivi en direct d'un test Isaac Sim : page, journal, image, arrêt propre.
Sans GPU : une image factice remplace la caméra.
"""
import json
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PIL import Image

from axis_os.hal.drivers.isaac_sim.direct import SuiviDirect

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def lire(url, data=None):
    req = urllib.request.Request(url, data=data, method="POST" if data is not None else "GET")
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.status, r.headers.get("Content-Type"), r.read()


d = tempfile.mkdtemp()
s = SuiviDirect(d, port=18080, images_par_s=100.0)
check("Le serveur démarre", s.demarrer())
base = "http://127.0.0.1:18080"
code, typ, corps = lire(base + "/")
check("Page servie, en HTML, avec le bouton d'arrêt",
      code == 200 and "text/html" in typ and "Arrêter proprement".encode() in corps)
s.journal("[   12.0s] colis rouge LIVRÉ au poste A (total 1)")
s.resume("00:12 simulé · 1 trié")
e = json.loads(lire(base + "/etat")[2])
check("État : résumé et journal", e["resume"].startswith("00:12")
      and "LIVRÉ" in e["journal"][-1] and e["image"] == 0)
s.image(Image.new("RGB", (64, 36), (200, 30, 30)))
e = json.loads(lire(base + "/etat")[2])
code, typ, corps = lire(base + "/image.jpg?1")
check("Image publiée et servie en JPEG", e["image"] == 1 and code == 200
      and typ == "image/jpeg" and corps[:2] == b"\xff\xd8")
img_hd = Image.new("RGB", (1920, 1080), (30, 30, 200))
time.sleep(0.02)
s.image(img_hd)
corps = lire(base + "/image.jpg")[2]
from io import BytesIO
check("Image 1080p réduite pour le flux (960 px de large)",
      Image.open(BytesIO(corps)).size == (960, 540))
import socket
c = socket.create_connection(("127.0.0.1", 18080), timeout=5)
c.sendall(b"GET /flux HTTP/1.1\r\nHost: x\r\n\r\n")
recu = b""
t0 = time.time()
while recu.count(b"--image") < 1 and time.time() - t0 < 3:
    recu += c.recv(65536)
time.sleep(0.02)
s.image(Image.new("RGB", (64, 36), (30, 200, 30)))
while recu.count(b"--image") < 2 and time.time() - t0 < 3:
    recu += c.recv(65536)
c.close()
check("Flux continu : l'image suivante est POUSSÉE au navigateur (MJPEG)",
      b"multipart/x-mixed-replace" in recu and recu.count(b"--image") >= 2)
s2 = SuiviDirect(tempfile.mkdtemp(), port=18081, images_par_s=1.0)
s2.image(Image.new("RGB", (8, 8)))
s2.image(Image.new("RGB", (8, 8)))
check("Cadence limitée : 2 images coup sur coup, 1 seule écrite", s2._image_n == 1)
check("Pas d'arrêt demandé au départ", not s.arret_demande)
lire(base + "/arreter", data=b"")
check("Bouton : l'arrêt est demandé, et journalisé",
      s.arret_demande and "ARRÊT DEMANDÉ" in s.etat()["journal"][-1])
try:
    lire(base + "/../etc/passwd")
    check("Chemin arbitraire refusé", False)
except urllib.error.HTTPError as err:
    check("Chemin arbitraire refusé (aucun fichier du disque servi)", err.code == 404)
check("N'écoute que sur localhost par défaut (accès par tunnel SSH)", s.hote == "127.0.0.1")
s.arreter()
occupe = SuiviDirect(d, port=18082)
occupe.demarrer()
double = SuiviDirect(d, port=18082)
check("Port déjà pris : pas de plantage du test, suivi désactivé", double.demarrer() is False)
occupe.arreter()

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
