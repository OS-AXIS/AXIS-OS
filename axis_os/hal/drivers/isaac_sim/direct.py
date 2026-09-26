"""
Suivi EN DIRECT d'un test Isaac Sim, depuis un navigateur.

Attendre la fin d'un essai de 30 minutes pour découvrir qu'il a échoué à
la deuxième minute, c'est perdre des heures GPU. Ce module sert, pendant
le test :
  - la dernière image de la caméra de suivi, tableau de bord incrusté ;
  - les dernières lignes du journal (colis livrés, alertes, arrêts) ;
  - un bouton « Arrêter proprement » : la boucle s'interrompt, la vidéo
    et le bilan sont écrits normalement (un Ctrl+C perdrait la vidéo).

Serveur HTTP minimal, dans un fil d'exécution à part, qui ne touche
JAMAIS Isaac Sim : il ne lit que des fichiers et une file de textes. Il
écoute sur localhost par défaut : on y accède par un tunnel SSH, sans
exposer la machine sur Internet.

    ssh -L 8080:localhost:8080 <connexion SSH de la machine>
    puis http://localhost:8080
"""
from __future__ import annotations

import json
import os
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
<main><img id="img" alt="dernière image du test"><pre id="journal"></pre></main>
<script>
const img = document.getElementById('img'), j = document.getElementById('journal'),
      etat = document.getElementById('etat'), stop = document.getElementById('stop');
async function maj() {
  try {
    const r = await fetch('/etat', {cache: 'no-store'}); const e = await r.json();
    etat.textContent = e.resume;
    j.textContent = e.journal.join('\\n'); j.scrollTop = j.scrollHeight;
    if (e.image) img.src = '/image.jpg?' + e.image;
    if (e.arret_demande) { stop.disabled = true; stop.textContent = 'Arrêt demandé'; }
  } catch (_) { etat.textContent = 'test terminé ou injoignable'; }
}
stop.onclick = async () => {
  if (!confirm('Arrêter le test ? La vidéo et le bilan seront écrits.')) return;
  await fetch('/arreter', {method: 'POST'}); maj();
};
setInterval(maj, 1000); maj();
</script></body></html>
"""


class SuiviDirect:
    def __init__(self, dossier: str, port: int = 8080, hote: str = "127.0.0.1",
                 lignes_max: int = 200, images_par_s: float = 2.0):
        self.dossier = Path(dossier)
        self.dossier.mkdir(parents=True, exist_ok=True)
        self.port, self.hote = port, hote
        self._journal: Deque[str] = deque(maxlen=lignes_max)
        self._resume = "démarrage…"
        self._image_n = 0
        self._periode_image = 1.0 / images_par_s
        self._t_image = 0.0
        self._verrou = threading.Lock()
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
                    p = suivi.dossier / "direct.jpg"
                    if p.exists():
                        self._envoyer(200, p.read_bytes(), "image/jpeg")
                    else:
                        self._envoyer(404, b"", "text/plain")
                else:
                    self._envoyer(404, b"", "text/plain")

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
            self._serveur.shutdown()
            self._serveur = None

    def journal(self, ligne: str) -> None:
        with self._verrou:
            self._journal.append(ligne)

    def resume(self, texte: str) -> None:
        self._resume = texte

    def image(self, img_pil) -> None:
        """Dernière image composée (PIL). Écriture ATOMIQUE, limitée à
        `images_par_s` par seconde réelle : l'encodage coûte du temps."""
        t = time.monotonic()
        if t - self._t_image < self._periode_image:
            return
        self._t_image = t
        tmp = self.dossier / "direct.tmp.jpg"
        img_pil.save(tmp, "JPEG", quality=80)
        os.replace(tmp, self.dossier / "direct.jpg")
        self._image_n += 1

    def etat(self) -> dict:
        with self._verrou:
            j = list(self._journal)
        return {"resume": self._resume, "journal": j,
                "image": self._image_n, "arret_demande": self.arret_demande}
