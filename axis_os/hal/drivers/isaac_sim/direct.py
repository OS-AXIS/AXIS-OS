"""
Suivi en direct d'un test Isaac Sim, depuis un navigateur.

Pourquoi : attendre la fin d'un essai de 30 minutes dans Isaac Sim (le
simulateur robotique de NVIDIA) pour découvrir qu'il a échoué à la
deuxième minute, c'est perdre des heures de calcul sur GPU (la carte
graphique qui fait tourner la simulation). Pendant le test, ce module
sert une page web qui montre :
  - la dernière image de la caméra de suivi, tableau de bord incrusté
    (voir video.py) ;
  - les dernières lignes du journal (colis livrés, alertes, arrêts) ;
  - un bouton « Arrêter proprement » : la boucle de test s'interrompt,
    la vidéo et le bilan sont écrits normalement, alors qu'un Ctrl+C
    perdrait la vidéo.

L'image arrive en flux continu au format MJPEG (une suite d'images JPEG
envoyées dans une seule réponse, comme une caméra de surveillance : le
navigateur l'affiche comme une vidéo), jusqu'à 15 images par seconde. Si
un intermédiaire bloque ce flux, la page passe d'elle-même en images
successives. Limite connue : la fluidité ne peut pas dépasser le rythme
auquel la simulation produit des images, et la simulation est plus lente
que le temps réel. On voit donc le robot au ralenti, mais sans attendre
la fin du test.

Le serveur HTTP est minimal et tourne dans un fil d'exécution (thread) à
part. Il ne touche jamais Isaac Sim : il ne lit que la dernière image et
une file de textes, déposées par la boucle de test. Il écoute par défaut
sur localhost (adresse qui ne désigne que la machine elle-même,
injoignable depuis le réseau) : on y accède par un tunnel SSH (connexion
chiffrée qui relaie un port de la machine de simulation vers le poste de
l'utilisateur), sans exposer la machine sur Internet.

    ssh -L 8080:localhost:8080 <connexion SSH de la machine>
    puis http://localhost:8080

Son usage pendant la démonstration de tri en usine est décrit dans
docs/demo_tri_usine.md.

Protections contre les sites tiers. Un audit de sécurité (2026-10-10) a
montré qu'écouter sur localhost ne suffit pas : une fois le tunnel
ouvert, n'importe quelle page web visitée dans le même navigateur peut
envoyer des requêtes à http://localhost:8080. Sans protection, une telle
page pouvait arrêter le test (une requête POST « simple » ne demande
aucune permission au navigateur), afficher la page de suivi dans un
cadre invisible pour faire cliquer le bouton à l'insu de l'utilisateur,
ou lire le journal par « rebinding DNS » (le DNS est l'annuaire qui
traduit un nom de domaine en adresse ; ici, un nom de domaine se met à
pointer vers 127.0.0.1, l'adresse de la machine locale). D'où les
défenses suivantes, vérifiées par test/test_suivi_direct_protections.py :
  - en-tête Host (le nom de serveur que le navigateur dit joindre)
    vérifié : seulement localhost, 127.0.0.1 ou [::1] (adresse locale en
    IPv6), plus les noms ajoutés explicitement par hotes_autorises. Cela
    déjoue le rebinding DNS, où Host porte le nom du site malveillant ;
  - en-tête Origin (le site d'où part la requête), s'il est présent,
    vérifié de la même façon ;
  - POST /arreter exige l'en-tête X-AXIS-Arret : un site tiers ne peut
    pas l'ajouter sans une requête de vérification préalable (OPTIONS,
    dite « preflight » dans CORS, le contrôle par le navigateur des
    requêtes entre sites) que ce serveur refuse ;
  - page interdite d'affichage dans un cadre (en-têtes X-Frame-Options et
    Content-Security-Policy « frame-ancestors ») ;
  - nombre de flux d'images simultanés limité (flux_max) : chaque flux
    occupe un fil d'exécution ; au-delà, le serveur répond 503 (service
    indisponible) et la page passe en images successives.
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

# Noms sous lesquels un navigateur joint ce serveur par le tunnel SSH
# (nom local, adresse IPv4, adresse IPv6).
_HOTES_LOCAUX = ("localhost", "127.0.0.1", "[::1]")
# En-tête exigé sur POST /arreter : un site tiers ne peut pas l'ajouter
# sans requête OPTIONS préalable, que ce serveur ne sait pas traiter (il la
# refuse donc).
ENTETE_ARRET = "X-AXIS-Arret"


def _nom_hote(valeur: str) -> str:
    """Extrait le nom d'hôte d'un en-tête Host (ou de la partie réseau
    d'une Origin), sans le port et en minuscules.

        'localhost:8080' -> 'localhost' ; '[::1]:9090' -> '[::1]'

    Une adresse IPv6 s'écrit entre crochets, d'où le traitement à part ;
    une valeur sans crochets qui contient plusieurs « : » est laissée
    telle quelle.
    """
    v = (valeur or "").strip().lower()
    if v.startswith("["):
        fin = v.find("]")
        return v[:fin + 1] if fin > 0 else v
    return v.rsplit(":", 1)[0] if v.count(":") == 1 else v

# Page de suivi, servie telle quelle. Son script interroge /etat chaque
# seconde, affiche le flux /flux et passe en images successives
# (/image.jpg) si le flux n'arrive pas, par exemple derrière un tunnel qui
# attend la fin d'une réponse avant de la transmettre.
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
  .vues { display: grid; grid-template-columns: minmax(0, 3fr) minmax(0, 2fr); gap: 8px;
          align-items: start; }
  @media (max-width: 800px) { .vues { grid-template-columns: 1fr; } }
  #robot:not([src]) { display: none; }
  pre { margin: 0; white-space: pre-wrap; font-size: 13px; background: #161c28;
        padding: 10px; border-radius: 6px; max-height: 75vh; overflow: auto; }
  button { background: #c8322d; color: #fff; border: 0; border-radius: 6px;
           padding: 8px 14px; font-size: 14px; cursor: pointer; }
  button:disabled { background: #555; cursor: default; }
</style></head><body>
<header><h1>AXIS-OS — test en direct</h1><span id="etat">connexion…</span>
<button id="stop">Arrêter proprement</button></header>
<main><div class="vues"><img id="img" src="/flux" alt="image du test en direct">
<img id="robot" alt="vue de la caméra de tête du robot"></div><pre id="journal"></pre></main>
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
// Vue du robot (caméra de tête, ce que lit la perception), à côté de la
// vue extérieure : images successives, absentes tant que le test n'en
// envoie pas.
const robot = document.getElementById('robot');
function vueRobot() {
  const i = new Image();
  i.onload = () => { robot.src = i.src; setTimeout(vueRobot, 250); };
  i.onerror = () => setTimeout(vueRobot, 2000);
  i.src = '/vue_robot.jpg?' + Date.now();
}
vueRobot();
setInterval(maj, 1000); maj();
</script></body></html>
"""


class SuiviDirect:
    """Serveur de suivi en direct d'un test, piloté par la boucle de test.

    Côté test : demarrer(), puis au fil de la boucle image() (souvent par
    SceneRecorder.direct, voir video.py), journal() et resume() ; lire
    arret_demande pour s'arrêter proprement quand l'utilisateur le
    demande ; arreter() à la fin.
    """

    def __init__(self, dossier: str, port: int = 8080, hote: str = "127.0.0.1",
                 lignes_max: int = 200, images_par_s: float = 15.0,
                 largeur_flux: int = 960, hotes_autorises: Sequence[str] = (),
                 flux_max: int = 4):
        """
        dossier : répertoire d'où sont servies les vues de la tête du
            robot (tete.jpg, tete_ok.jpg), déposées par le test ; créé
            s'il n'existe pas.
        port, hote : adresse d'écoute ; 127.0.0.1 par défaut, pour n'être
            joignable que depuis la machine elle-même (donc par le tunnel).
        lignes_max : lignes de journal conservées ; les plus anciennes
            sont oubliées.
        images_par_s : nombre maximal d'images encodées par seconde réelle
            pour la page.
        largeur_flux : largeur maximale, en pixels, des images envoyées.
        hotes_autorises : noms d'hôte acceptés en plus de localhost,
            127.0.0.1 et [::1] (en-têtes Host et Origin), par exemple le
            nom d'un relais web de tunnel SSH. Aucun par défaut.
        flux_max : nombre maximal de flux MJPEG simultanés (chacun occupe
            un fil d'exécution).
        """
        self.dossier = Path(dossier)
        self.hotes_autorises = frozenset(_HOTES_LOCAUX) | {
            _nom_hote(h) for h in hotes_autorises if h}
        self.flux_max = flux_max
        self._flux_ouverts = 0
        self._verrou_flux = threading.Lock()
        # Requêtes refusées, comptées par motif (utile au diagnostic).
        self.refus = {"hote": 0, "origine": 0, "entete": 0, "flux": 0}
        # Hôtes refusés déjà signalés dans la console : chacun ne l'est
        # qu'une fois, et dix au plus, pour ne pas inonder la console.
        self._hotes_signales: set = set()
        self.dossier.mkdir(parents=True, exist_ok=True)
        self.port, self.hote = port, hote
        # Journal : écrit par la boucle de test, lu par le serveur, d'où
        # le verrou (_verrou).
        self._journal: Deque[str] = deque(maxlen=lignes_max)
        self._resume = "démarrage…"
        self._image_n = 0
        self._periode_image = 1.0 / images_par_s
        self._t_image = 0.0
        self._verrou = threading.Lock()
        self._largeur = largeur_flux
        # Dernière image encodée en JPEG, et condition qui réveille les
        # flux MJPEG à chaque nouvelle image (_image_n en est le numéro).
        self._jpeg: Optional[bytes] = None
        # Dernière image de la vue du robot (image_robot), en JPEG.
        self._jpeg_robot: Optional[bytes] = None
        self._t_image_robot = 0.0
        self._nouvelle = threading.Condition()
        self.arret_demande = False
        self._serveur: Optional[ThreadingHTTPServer] = None

    # ---- côté test -------------------------------------------------------
    def demarrer(self) -> bool:
        """Lance le serveur dans un fil d'exécution en arrière-plan.

        Renvoie False, avec un message, si le port est indisponible : il
        n'y a alors pas de suivi en direct.
        """
        # Le serveur crée un gestionnaire par requête ; celui-ci retrouve
        # l'objet de suivi par cette variable.
        suivi = self

        class Gestionnaire(BaseHTTPRequestHandler):
            """Traite chaque requête HTTP, après vérification des en-têtes
            (voir _autorise)."""

            # Pas de ligne de console à chaque requête.
            def log_message(self, *a):          # silencieux
                pass

            def _entetes_securite(self):
                # Jamais dans un cadre : sinon une page tierce pourrait
                # superposer le bouton « Arrêter » à autre chose et le
                # faire cliquer à l'insu de l'utilisateur. nosniff : le
                # navigateur s'en tient au type de contenu annoncé ;
                # no-referrer : l'adresse de la page n'est pas transmise
                # aux autres sites.
                self.send_header("X-Frame-Options", "DENY")
                self.send_header("Content-Security-Policy", "frame-ancestors 'none'")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "no-referrer")

            def _envoyer(self, code, corps: bytes, type_: str):
                """Envoie une réponse complète, jamais mise en cache, avec
                les en-têtes de sécurité."""
                self.send_response(code)
                self.send_header("Content-Type", type_)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(corps)))
                self._entetes_securite()
                self.end_headers()
                self.wfile.write(corps)

            def _refuser(self, cle: str, detail: str) -> bool:
                """Répond 403 (accès interdit), compte le refus sous le
                motif cle et renvoie False."""
                suivi.refus[cle] += 1
                self._envoyer(403, f"refusé : {detail}".encode(), "text/plain; charset=utf-8")
                return False

            def _autorise(self) -> bool:
                """Vérifie que Host, et Origin s'il est présent,
                désignent ce serveur par un nom autorisé (voir l'en-tête
                du module).

                Sinon, répond 403 et renvoie False. Un hôte refusé est
                signalé une fois dans la console (dix hôtes différents au
                plus), avec la marche à suivre pour un relais de tunnel
                légitime.
                """
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
                """Routes : / ou /index.html (page), /etat (résumé et
                journal au format JSON, un format texte de données
                structurées), /image.jpg (dernière image), /vue_robot.jpg
                (dernière vue de la caméra de tête), /tete.jpg et
                /tete_ok.jpg (vues de la tête du robot), /flux (flux
                MJPEG). Toute autre adresse reçoit 404 (introuvable)."""
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
                elif self.path.startswith("/vue_robot.jpg"):
                    j = suivi._jpeg_robot
                    if j is not None:
                        self._envoyer(200, j, "image/jpeg")
                    else:
                        self._envoyer(404, b"", "text/plain")
                elif self.path.split("?")[0] in ("/tete.jpg", "/tete_ok.jpg"):
                    # Vue de la caméra de tête du robot lors de la dernière
                    # recherche de colis restée vide, et lors de la dernière
                    # recherche réussie, pour comparer. Images déposées par
                    # sim/validation/test_isaac_t1_tri_usine.py (essai de
                    # tri de colis en usine). Seuls ces deux noms fixes sont
                    # servis depuis le dossier.
                    p = suivi.dossier / self.path.split("?")[0].lstrip("/")
                    if p.exists():
                        self._envoyer(200, p.read_bytes(), "image/jpeg")
                    else:
                        self._envoyer(404, b"", "text/plain")
                elif self.path.startswith("/flux"):
                    # Flux limités à flux_max : chacun occupe un fil
                    # d'exécution tant que l'onglet reste ouvert.
                    with suivi._verrou_flux:
                        plein = suivi._flux_ouverts >= suivi.flux_max
                        if not plein:
                            suivi._flux_ouverts += 1
                    if plein:
                        # 503 : la page passe d'elle-même en images
                        # successives.
                        suivi.refus["flux"] += 1
                        self._envoyer(503, b"trop de flux ouverts", "text/plain")
                        return
                    # finally : la place est rendue même si la connexion
                    # se coupe.
                    try:
                        self._flux()
                    finally:
                        with suivi._verrou_flux:
                            suivi._flux_ouverts -= 1
                else:
                    self._envoyer(404, b"", "text/plain")

            def _flux(self):
                """Flux MJPEG : chaque nouvelle image est poussée au
                navigateur dès qu'elle existe, dans une seule réponse HTTP
                sans fin où chaque image remplace la précédente.

                L'attente d'une image dure au plus 5 s, pour remarquer
                l'arrêt du serveur même sans nouvelle image. Le flux se
                termine à l'arrêt du serveur ou à la fermeture de l'onglet.
                """
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=image")
                self.send_header("Cache-Control", "no-store")
                self._entetes_securite()
                self.end_headers()
                vue = -1                # dernière image envoyée
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
                """Seule route : POST /arreter, qui demande l'arrêt propre
                du test (la boucle de test lit arret_demande)."""
                if not self._autorise():
                    return
                if self.path == "/arreter":
                    if not self.headers.get(ENTETE_ARRET):
                        # Un formulaire ou une requête « simple » (sans
                        # en-tête personnalisé) venue d'un site tiers n'a pas
                        # cet en-tête : l'arrêt est refusé.
                        self._refuser("entete", f"en-tête {ENTETE_ARRET} manquant")
                        return
                    suivi.arret_demande = True
                    suivi.journal("ARRÊT DEMANDÉ depuis la page de suivi")
                    self._envoyer(200, b"ok", "text/plain")
                else:
                    self._envoyer(404, b"", "text/plain")

        # Un fil d'exécution par requête : un flux ouvert ne bloque pas les
        # autres requêtes. Le fil du serveur est « daemon » : il n'empêche
        # pas le programme de se terminer.
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
        """Arrête le serveur ; les flux en attente sont réveillés et se
        terminent d'eux-mêmes."""
        if self._serveur is not None:
            srv, self._serveur = self._serveur, None
            with self._nouvelle:
                self._nouvelle.notify_all()      # libère les flux en cours
            srv.shutdown()

    def journal(self, ligne: str) -> None:
        """Ajoute une ligne au journal affiché sur la page."""
        with self._verrou:
            self._journal.append(ligne)

    def resume(self, texte: str) -> None:
        """Remplace le résumé affiché en haut de la page."""
        self._resume = texte

    def image(self, img_pil) -> None:
        """Reçoit la dernière image composée (objet PIL, la bibliothèque
        d'images de Python), la réduit et l'encode en JPEG en mémoire.

        La cadence est limitée à images_par_s par seconde réelle (horloge
        du système, pas celle de la simulation), car l'encodage se fait
        dans la boucle de simulation et ne doit pas la ralentir. Les
        images en trop sont simplement ignorées.
        """
        t = time.monotonic()
        if t - self._t_image < self._periode_image:
            return
        self._t_image = t
        if img_pil.width > self._largeur:
            # reduce() divise la taille par un facteur entier (arrondi au
            # supérieur, pour ne pas dépasser largeur_flux) : environ 5 fois
            # plus rapide qu'un resize() bicubique (4,5 ms contre 20 ms en
            # 1080p, mesuré).
            img_pil = img_pil.reduce(-(-img_pil.width // self._largeur))
        tampon = io.BytesIO()
        img_pil.save(tampon, "JPEG", quality=75)
        with self._nouvelle:
            self._jpeg = tampon.getvalue()
            self._image_n += 1
            self._nouvelle.notify_all()      # réveille les flux en attente

    def image_robot(self, img_pil) -> None:
        """Reçoit la vue de la caméra de tête du robot (objet PIL), servie
        par /vue_robot.jpg ; même cadence maximale que image()."""
        t = time.monotonic()
        if t - self._t_image_robot < self._periode_image:
            return
        self._t_image_robot = t
        tampon = io.BytesIO()
        img_pil.convert("RGB").save(tampon, "JPEG", quality=75)
        self._jpeg_robot = tampon.getvalue()

    def etat(self) -> dict:
        """Instantané servi par /etat : résumé, journal, numéro de la
        dernière image et arrêt demandé ou non."""
        with self._verrou:
            j = list(self._journal)
        return {"resume": self._resume, "journal": j,
                "image": self._image_n, "arret_demande": self.arret_demande}
