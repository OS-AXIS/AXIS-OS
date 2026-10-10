"""
Suivi EN DIRECT d'un test Isaac Sim, depuis un navigateur.

Attendre la fin d'un essai de 30 minutes pour découvrir qu'il a échoué à
la deuxième minute, c'est perdre des heures GPU. Ce module sert, pendant
le test :
  - la dernière image de la caméra de suivi, tableau de bord incrusté ;
  - les dernières lignes du journal (colis livrés, alertes, arrêts) ;
  - un bouton « Arrêter proprement » : la boucle s'interrompt, la vidéo
    et le bilan sont écrits normalement (un Ctrl+C perdrait la vidéo).

L'image est un FLUX CONTINU (MJPEG, comme une caméra de surveillance :
le navigateur l'affiche comme une vidéo), jusqu'à 15 images par seconde.
Limite honnête : la fluidité ne peut dépasser le rythme auquel la
simulation produit des images, et la simulation est plus lente que le
temps réel — on voit le robot au ralenti, mais sans attendre.

Serveur HTTP minimal, dans un fil d'exécution à part, qui ne touche
JAMAIS Isaac Sim : il ne lit que la dernière image et une file de textes. Il
écoute sur localhost par défaut : on y accède par un tunnel SSH, sans
exposer la machine sur Internet.

    ssh -L 8080:localhost:8080 <connexion SSH de la machine>
    puis http://localhost:8080

PROTECTIONS (audit du 2026-10-10). Écouter sur localhost ne suffit pas : le
tunnel ouvert, N'IMPORTE QUELLE page web visitée dans le même navigateur
peut envoyer des requêtes à http://localhost:8080. Sans protection, elle
pouvait arrêter le test (un POST « simple » ne demande aucune permission au
navigateur), l'afficher dans un cadre invisible pour faire cliquer le
bouton, ou lire le journal par « rebinding » DNS (un nom de domaine qui se
met à pointer vers 127.0.0.1). D'où :
  - en-tête Host vérifié : seulement localhost / 127.0.0.1 / [::1] (plus
    les noms ajoutés explicitement par hotes_autorises) — déjoue le
    rebinding DNS, où Host porte le nom du site malveillant ;
  - en-tête Origin, s'il est présent, vérifié de même ;
  - POST /arreter exige l'en-tête X-AXIS-Arret : un site tiers ne peut pas
    l'ajouter sans une requête préalable (OPTIONS) que ce serveur refuse ;
  - page interdite dans un cadre (X-Frame-Options, frame-ancestors) ;
  - flux d'images simultanés limités (flux_max) : chaque flux occupe un fil
    d'exécution ; au-delà, 503, et la page passe en images successives.
Aucune de ces protections ne touche la simulation : au pire, le suivi est
refusé, le test continue.
"""
from __future__ import annotations

import io
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Deque, Optional, Sequence
from urllib.parse import urlsplit

# Noms sous lesquels un navigateur joint ce serveur par le tunnel SSH.
_HOTES_LOCAUX = ("localhost", "127.0.0.1", "[::1]")
# En-tête exigé sur POST /arreter : un site tiers ne peut pas l'ajouter sans
# requête OPTIONS préalable, que ce serveur ne sait pas traiter (donc refuse).
ENTETE_ARRET = "X-AXIS-Arret"


def _nom_hote(valeur: str) -> str:
    """'localhost:8080' -> 'localhost' ; '[::1]:9090' -> '[::1]' ; minuscules."""
    v = (valeur or "").strip().lower()
    if v.startswith("["):
        fin = v.find("]")
        return v[:fin + 1] if fin > 0 else v
    return v.rsplit(":", 1)[0] if v.count(":") == 1 else v

_PAGE = """<!doctype html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AXIS-OS — en direct</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; background: #0f141e; color: #e8e8e8;
         font: 15px/1.4 system-ui, sans-serif; }
  header { padding: 10px 16px; display: flex; gap: 12px; align-items: center;
           flex-wrap: wrap; }
  h1 { font-size: 17px; margin: 0; color: #78c8ff; }
  #etat { color: #aab; }
  main { display: grid; grid-template-columns: minmax(0, 3fr) minmax(260px, 1fr);
         gap: 12px; padding: 0 16px 16px; }
  @media (max-width: 800px) { main { grid-template-columns: 1fr; } }
  img { width: 100%; border-radius: 6px; background: #000; }
  pre { margin: 0; white-space: pre-wrap; font-size: 13px; background: #161c28;
        padding: 10px; border-radius: 6px; max-height: 75vh; overflow: auto; }
  button { background: #c8322d; color: #fff; border: 0; border-radius: 6px;
           padding: 8px 14px; font-size: 14px; cursor: pointer; }
  button:disabled { background: #555; cursor: default; }
</style></head><body>
<header><h1>AXIS-OS — test en direct</h1><span id="etat">connexion…</span>
<button id="stop">Arrêter proprement</button></header>
<main><img id="img" src="/flux" alt="image du test en direct"><pre id="journal"></pre></main>
<script>
const img = document.getElementById('img'), j = document.getElementById('journal'),
      etat = document.getElementById('etat'), stop = document.getElementById('stop');
async function maj() {
  try {
    const r = await fetch('/etat', {cache: 'no-store'}); const e = await r.json();
    etat.textContent = e.resume;
    j.textContent = e.journal.join('\\n'); j.scrollTop = j.scrollHeight;
    if (e.arret_demande) { stop.disabled = true; stop.textContent = 'Arrêt demandé'; }
  } catch (_) { etat.textContent = 'test terminé ou injoignable'; }
}
// Certains tunnels (ssheasy.com, proxys web) attendent la FIN d'une
// réponse avant de la transmettre : un flux continu n'arrive alors jamais.
// Si aucune image n'est affichée après 3 s, on passe en images
// successives, qui traversent n'importe quel intermédiaire.
let modeImages = false;
function imagesSuccessives() {
  if (modeImages) return; modeImages = true;
  const suivante = () => {
    const i = new Image();
    i.onload = () => { img.src = i.src; setTimeout(suivante, 200); };
    i.onerror = () => setTimeout(suivante, 1000);
    i.src = '/image.jpg?' + Date.now();
  };
  suivante();
}
setTimeout(() => { if (!img.naturalWidth) { img.removeAttribute('src'); imagesSuccessives(); } }, 3000);
img.onerror = () => imagesSuccessives();
stop.onclick = async () => {
  if (!confirm('Arrêter le test ? La vidéo et le bilan seront écrits.')) return;
  await fetch('/arreter', {method: 'POST', headers: {'X-AXIS-Arret': 'oui'}}); maj();
};
setInterval(maj, 1000); maj();
</script></body></html>
"""


class SuiviDirect:
    def __init__(self, dossier: str, port: int = 8080, hote: str = "127.0.0.1",
                 lignes_max: int = 200, images_par_s: float = 15.0,
                 largeur_flux: int = 960, hotes_autorises: Sequence[str] = (),
                 flux_max: int = 4):
        """
        hotes_autorises : noms d'hôte acceptés EN PLUS de localhost /
            127.0.0.1 / [::1] (en-têtes Host et Origin) — par exemple le nom
            d'un relais web de tunnel SSH. Rien par défaut.
        flux_max : flux MJPEG simultanés au plus (un fil d'exécution chacun).
        """
        self.dossier = Path(dossier)
        self.hotes_autorises = frozenset(_HOTES_LOCAUX) | {
            _nom_hote(h) for h in hotes_autorises if h}
        self.flux_max = flux_max
        self._flux_ouverts = 0
        self._verrou_flux = threading.Lock()
        self.refus = {"hote": 0, "origine": 0, "entete": 0, "flux": 0}
        self._hotes_signales: set = set()
        self.dossier.mkdir(parents=True, exist_ok=True)
        self.port, self.hote = port, hote
        self._journal: Deque[str] = deque(maxlen=lignes_max)
        self._resume = "démarrage…"
        self._image_n = 0
        self._periode_image = 1.0 / images_par_s
        self._t_image = 0.0
        self._verrou = threading.Lock()
        self._largeur = largeur_flux
        self._jpeg: Optional[bytes] = None
        self._nouvelle = threading.Condition()
        self.arret_demande = False
        self._serveur: Optional[ThreadingHTTPServer] = None

    # ---- côté test -------------------------------------------------------
    def demarrer(self) -> bool:
        suivi = self

        class Gestionnaire(BaseHTTPRequestHandler):
            def log_message(self, *a):          # silencieux
                pass

            def _entetes_securite(self):
                # Jamais dans un cadre : sinon une page tierce pourrait
                # superposer le bouton « Arrêter » à autre chose.
                self.send_header("X-Frame-Options", "DENY")
                self.send_header("Content-Security-Policy", "frame-ancestors 'none'")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "no-referrer")

            def _envoyer(self, code, corps: bytes, type_: str):
                self.send_response(code)
                self.send_header("Content-Type", type_)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(corps)))
                self._entetes_securite()
                self.end_headers()
                self.wfile.write(corps)

            def _refuser(self, cle: str, detail: str) -> bool:
                suivi.refus[cle] += 1
                self._envoyer(403, f"refusé : {detail}".encode(), "text/plain; charset=utf-8")
                return False

            def _autorise(self) -> bool:
                """Host (et Origin s'il est là) doivent désigner ce serveur
                par un nom local — voir l'en-tête du module."""
                hote = _nom_hote(self.headers.get("Host", ""))
                if hote not in suivi.hotes_autorises:
                    if len(suivi._hotes_signales) < 10 and hote not in suivi._hotes_signales:
                        suivi._hotes_signales.add(hote)
                        print(f"  [direct] requête refusée : hôte « {hote[:60]} » (seuls "
                              f"{sorted(suivi.hotes_autorises)} sont acceptés ; un relais "
                              f"de tunnel s'ajoute par hotes_autorises)")
                    return self._refuser("hote", "hôte non autorisé")
                origine = self.headers.get("Origin")
                if origine is not None and _nom_hote(urlsplit(origine).netloc) \
                        not in suivi.hotes_autorises:
                    return self._refuser("origine", "origine non autorisée")
                return True

            def do_GET(self):
                if not self._autorise():
                    return
                if self.path in ("/", "/index.html"):
                    self._envoyer(200, _PAGE.encode(), "text/html; charset=utf-8")
                elif self.path == "/etat":
                    self._envoyer(200, json.dumps(suivi.etat()).encode(),
                                  "application/json")
                elif self.path.startswith("/image.jpg"):
                    j = suivi._jpeg
                    if j is not None:
                        self._envoyer(200, j, "image/jpeg")
                    else:
                        self._envoyer(404, b"", "text/plain")
                elif self.path.split("?")[0] in ("/tete.jpg", "/tete_ok.jpg"):
                    # vue de la tête du robot : dernière recherche vide, et
                    # dernière recherche RÉUSSIE (pour comparer)
                    p = suivi.dossier / self.path.split("?")[0].lstrip("/")
                    if p.exists():
                        self._envoyer(200, p.read_bytes(), "image/jpeg")
                    else:
                        self._envoyer(404, b"", "text/plain")
                elif self.path.startswith("/flux"):
                    with suivi._verrou_flux:
                        plein = suivi._flux_ouverts >= suivi.flux_max
                        if not plein:
                            suivi._flux_ouverts += 1
                    if plein:
                        # 503 : la page passe d'elle-même en images successives.
                        suivi.refus["flux"] += 1
                        self._envoyer(503, b"trop de flux ouverts", "text/plain")
                        return
                    try:
                        self._flux()
                    finally:
                        with suivi._verrou_flux:
                            suivi._flux_ouverts -= 1
                else:
                    self._envoyer(404, b"", "text/plain")

            def _flux(self):
                """MJPEG : chaque nouvelle image est poussée au navigateur."""
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=image")
                self.send_header("Cache-Control", "no-store")
                self._entetes_securite()
                self.end_headers()
                vue = -1
                try:
                    while suivi._serveur is not None:
                        with suivi._nouvelle:
                            suivi._nouvelle.wait_for(
                                lambda: suivi._image_n != vue or suivi._serveur is None,
                                timeout=5.0)
                        j, vue = suivi._jpeg, suivi._image_n
                        if j is None:
                            continue
                        self.wfile.write(b"--image\r\nContent-Type: image/jpeg\r\n"
                                         + f"Content-Length: {len(j)}\r\n\r\n".encode()
                                         + j + b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass                         # onglet fermé : rien de grave

            def do_POST(self):
                if not self._autorise():
                    return
                if self.path == "/arreter":
                    if not self.headers.get(ENTETE_ARRET):
                        # Un formulaire ou un fetch « simple » d'un site tiers
                        # n'a pas cet en-tête : l'arrêt est refusé.
                        self._refuser("entete", f"en-tête {ENTETE_ARRET} manquant")
                        return
                    suivi.arret_demande = True
                    suivi.journal("ARRÊT DEMANDÉ depuis la page de suivi")
                    self._envoyer(200, b"ok", "text/plain")
                else:
                    self._envoyer(404, b"", "text/plain")

        try:
            self._serveur = ThreadingHTTPServer((self.hote, self.port), Gestionnaire)
        except OSError as exc:
            print(f"  [direct] port {self.port} indisponible ({exc}) — pas de suivi")
            return False
        threading.Thread(target=self._serveur.serve_forever, daemon=True).start()
        print(f"  [direct] suivi en direct : http://localhost:{self.port} "
              f"(tunnel : ssh -L {self.port}:localhost:{self.port} <machine>)")
        return True

    def arreter(self) -> None:
        if self._serveur is not None:
            srv, self._serveur = self._serveur, None
            with self._nouvelle:
                self._nouvelle.notify_all()      # libère les flux en cours
            srv.shutdown()

    def journal(self, ligne: str) -> None:
        with self._verrou:
            self._journal.append(ligne)

    def resume(self, texte: str) -> None:
        self._resume = texte

    def image(self, img_pil) -> None:
        """Dernière image composée (PIL), réduite et encodée en mémoire.
        Limitée à `images_par_s` par seconde réelle : l'encodage se fait
        dans la boucle de simulation, il ne doit pas la ralentir."""
        t = time.monotonic()
        if t - self._t_image < self._periode_image:
            return
        self._t_image = t
        if img_pil.width > self._largeur:
            # reduce() par un facteur entier : ~5x plus rapide qu'un
            # resize() bicubique (4.5 ms contre 20 ms en 1080p, mesuré).
            img_pil = img_pil.reduce(-(-img_pil.width // self._largeur))
        tampon = io.BytesIO()
        img_pil.save(tampon, "JPEG", quality=75)
        with self._nouvelle:
            self._jpeg = tampon.getvalue()
            self._image_n += 1
            self._nouvelle.notify_all()

    def etat(self) -> dict:
        with self._verrou:
            j = list(self._journal)
        return {"resume": self._resume, "journal": j,
                "image": self._image_n, "arret_demande": self.arret_demande}
