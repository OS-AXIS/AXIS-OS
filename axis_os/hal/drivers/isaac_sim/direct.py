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
"""
from __future__ import annotations

import io
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Deque, Optional

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
  await fetch('/arreter', {method: 'POST'}); maj();
};
setInterval(maj, 1000); maj();
</script></body></html>
"""


class SuiviDirect:
    def __init__(self, dossier: str, port: int = 8080, hote: str = "127.0.0.1",
                 lignes_max: int = 200, images_par_s: float = 15.0,
                 largeur_flux: int = 960):
        self.dossier = Path(dossier)
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

            def _envoyer(self, code, corps: bytes, type_: str):
                self.send_response(code)
                self.send_header("Content-Type", type_)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(corps)))
                self.end_headers()
                self.wfile.write(corps)

            def do_GET(self):
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
                elif self.path.startswith("/flux"):
                    self._flux()
                else:
                    self._envoyer(404, b"", "text/plain")

            def _flux(self):
                """MJPEG : chaque nouvelle image est poussée au navigateur."""
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=image")
                self.send_header("Cache-Control", "no-store")
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
                if self.path == "/arreter":
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
