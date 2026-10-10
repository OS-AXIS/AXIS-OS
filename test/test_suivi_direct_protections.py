"""
Page de suivi en direct — protections contre les sites tiers (audit du
2026-10-10, M1).

Le tunnel SSH ouvert, n'importe quelle page web visitée dans le même
navigateur peut envoyer des requêtes à http://localhost:8080. Vérifié ici :
  - Host étranger (rebinding DNS) : refusé ;
  - Origin étrangère : refusée ;
  - POST /arreter sans l'en-tête X-AXIS-Arret (formulaire, fetch simple
    d'un site tiers) : refusé, le test n'est PAS arrêté ;
  - page interdite dans un cadre ;
  - flux simultanés limités ;
  - rien de tout cela ne gêne l'usage normal par le tunnel.
"""
import socket
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.isaac_sim.direct import ENTETE_ARRET, SuiviDirect  # noqa: E402

FAIL = 0
PORT = 18090


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def brut(methode, chemin, entetes):
    """Requête HTTP écrite à la main : on choisit EXACTEMENT les en-têtes
    (urllib impose Host). Rend (code, en-têtes en minuscules, corps)."""
    c = socket.create_connection(("127.0.0.1", PORT), timeout=5)
    lignes = [f"{methode} {chemin} HTTP/1.1"] + [f"{k}: {v}" for k, v in entetes.items()]
    if methode == "POST":
        lignes.append("Content-Length: 0")
    c.sendall(("\r\n".join(lignes) + "\r\n\r\n").encode())
    recu = b""
    t0 = time.time()
    while b"\r\n\r\n" not in recu and time.time() - t0 < 3:
        recu += c.recv(65536)
    tete, _, corps = recu.partition(b"\r\n\r\n")
    premiere, *autres = tete.decode("latin-1").split("\r\n")
    h = {l.split(":", 1)[0].strip().lower(): l.split(":", 1)[1].strip()
         for l in autres if ":" in l}
    # Corps lu EN ENTIER (Content-Length) : les en-têtes peuvent arriver
    # sans lui, et un test qui le lirait à moitié échouerait au hasard.
    attendu = int(h.get("content-length", 0) or 0)
    while len(corps) < attendu and time.time() - t0 < 3:
        morceau = c.recv(65536)
        if not morceau:
            break
        corps += morceau
    c.close()
    return int(premiere.split()[1]), h, corps


LOCAL = {"Host": f"localhost:{PORT}"}
s = SuiviDirect(tempfile.mkdtemp(), port=PORT, flux_max=2, hotes_autorises=["relais.example"])
check("Le serveur démarre", s.demarrer())

section("1 — Usage normal par le tunnel : inchangé")

for hote in (f"localhost:{PORT}", f"127.0.0.1:{PORT}", f"[::1]:{PORT}", "localhost:9090"):
    code, _, _ = brut("GET", "/", {"Host": hote})
    check(f"Host {hote} : page servie", code == 200)
code, _, _ = brut("GET", "/etat", {"Host": f"localhost:{PORT}",
                                    "Origin": f"http://localhost:{PORT}"})
check("Origin locale : acceptée", code == 200)
code, _, _ = brut("GET", "/", {"Host": "relais.example"})
check("Hôte ajouté explicitement (hotes_autorises) : accepté", code == 200)

section("2 — Rebinding DNS : Host étranger refusé")

for hote in ("evil.example:8080", "evil.example", "localhost.evil.example", ""):
    code, _, _ = brut("GET", "/etat", {"Host": hote} if hote else {})
    check(f"Host « {hote} » : refusé (403)", code == 403)
check("Refus comptés", s.refus["hote"] == 4)

section("3 — Origin étrangère refusée")

for origine in ("http://evil.example", "null", "http://localhost.evil.example"):
    code, _, _ = brut("GET", "/etat", {**LOCAL, "Origin": origine})
    check(f"Origin « {origine} » : refusée (403)", code == 403)

section("4 — POST /arreter : en-tête dédié obligatoire")

code, _, _ = brut("POST", "/arreter", LOCAL)
check("Sans X-AXIS-Arret (formulaire ou fetch simple d'un site tiers) : 403",
      code == 403 and not s.arret_demande)
code, _, _ = brut("POST", "/arreter", {**LOCAL, "Origin": "http://evil.example",
                                        ENTETE_ARRET: "oui"})
check("Avec l'en-tête mais depuis une origine étrangère : 403",
      code == 403 and not s.arret_demande)
code, _, _ = brut("POST", "/arreter", {"Host": "evil.example", ENTETE_ARRET: "oui"})
check("Avec l'en-tête mais Host étranger : 403", code == 403 and not s.arret_demande)
code, _, _ = brut("OPTIONS", "/arreter", {**LOCAL, "Origin": "http://evil.example"})
check(f"Requête préalable OPTIONS d'un site tiers : non acceptée ({code})",
      code >= 400)
code, _, _ = brut("POST", "/arreter", {**LOCAL, "Origin": f"http://localhost:{PORT}",
                                        ENTETE_ARRET: "oui"})
check("Depuis la page elle-même (Origin locale + en-tête) : arrêt demandé",
      code == 200 and s.arret_demande)
page = brut("GET", "/", LOCAL)[2]
check("Le bouton de la page envoie bien l'en-tête", ENTETE_ARRET.encode() in page)

section("5 — Jamais dans un cadre")

code, h, _ = brut("GET", "/", LOCAL)
check("X-Frame-Options: DENY", h.get("x-frame-options") == "DENY")
check("Content-Security-Policy: frame-ancestors 'none'",
      "frame-ancestors 'none'" in h.get("content-security-policy", ""))
check("X-Content-Type-Options: nosniff", h.get("x-content-type-options") == "nosniff")

section("6 — Flux simultanés limités")

flux = []
for _ in range(2):
    c = socket.create_connection(("127.0.0.1", PORT), timeout=5)
    c.sendall(f"GET /flux HTTP/1.1\r\nHost: localhost:{PORT}\r\n\r\n".encode())
    flux.append(c)
time.sleep(0.3)
code, _, _ = brut("GET", "/flux", LOCAL)
check(f"3e flux avec flux_max=2 : 503 ({code}) — la page passe en images successives",
      code == 503 and s.refus["flux"] == 1)
for c in flux:
    c.close()
from PIL import Image  # noqa: E402

# Un flux fermé côté navigateur n'est constaté qu'à l'écriture suivante :
# on pousse des images jusqu'à ce que les fils d'exécution s'en aperçoivent.
t0 = time.time()
while s._flux_ouverts and time.time() - t0 < 8:
    time.sleep(0.07)
    s.image(Image.new("RGB", (8, 8)))
check("Flux fermés : la place est rendue", s._flux_ouverts == 0)

s.arreter()

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
