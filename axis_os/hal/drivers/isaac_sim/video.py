"""
AXIS-OS — Enregistrement vidéo des démonstrations en simulation.

Pourquoi ce module : une vidéo où un robot marche puis s'arrête ne
montre rien d'utile, car le spectateur ne sait pas pourquoi il s'arrête.
Ce qui rend une démonstration compréhensible, c'est l'état du système
affiché en même temps que le robot : l'app (application AXIS-OS) en
cours, la consigne reçue, l'état du service, les refus au moment où ils
arrivent.

La simulation tourne dans Isaac Sim, le simulateur robotique de NVIDIA.
Le module fournit deux choses :
  - une caméra « spectateur », distincte de la caméra embarquée du
    robot, qui filme la scène de trois quarts (fixe, ou qui suit le
    robot, voir SceneRecorder.suivre) ;
  - une incrustation : un panneau d'état permanent, et un bandeau
    d'événement quand quelque chose d'important se produit.
Il peut aussi ajouter une piste son (la parole captée par le micro, par
exemple) et transmettre chaque image composée au suivi en direct par
navigateur (voir direct.py).

Le fichier MP4 (format vidéo courant) est écrit au fil de l'eau, image
par image, avec imageio (bibliothèque Python d'écriture d'images et de
vidéos) et ffmpeg (l'outil d'encodage vidéo qu'elle utilise), plutôt
qu'en milliers d'images séparées sur disque : une minute en 720p
(1280 x 720 pixels), c'est 1500 images et plusieurs gigaoctets au
format PNG.

Le temps de la vidéo est celui de la simulation : la simulation est plus
lente que le temps réel, mais la vidéo se regarde à vitesse normale.
"""
from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence, Tuple


# Caractères accentués qu'une police doit savoir dessiner pour être
# retenue (voir _couvre_accents).
_TEST_ACCENTS = "éèàêôçÉ"


def _rendu(police, c: str) -> bytes:
    """Dessine un caractère dans une petite image en niveaux de gris et
    renvoie les octets de cette image.

    Pourquoi passer par une image : le résultat se compare de la même
    façon d'une version de Pillow (la bibliothèque d'images de Python) à
    l'autre, contrairement à getmask(), dont l'objet renvoyé n'a pas de
    méthode tobytes() dans toutes les versions.
    """
    from PIL import Image, ImageDraw
    img = Image.new("L", (64, 64), 0)
    ImageDraw.Draw(img).text((4, 4), c, font=police, fill=255)
    return img.tobytes()


def _couvre_accents(police) -> bool:
    """Indique si la police dessine vraiment les caractères accentués.

    Le piège : une police à laquelle il manque un glyphe (le dessin d'un
    caractère) affiche à sa place un rectangle vide, sans lever d'erreur.
    Constaté dans la première vidéo de démonstration : « ex□cution »,
    « demand□e ». Pour le détecter, on compare le rendu de chaque
    caractère accentué à celui d'un caractère qu'aucune police ne dessine
    (U+10FFFD, réservé à un usage privé) et à une image vide : s'il est
    identique à l'un des deux, le glyphe manque.

    Une première version de ce test était fausse, sans que rien ne le
    signale : elle comparait getmask(...).tobytes(), méthode absente dans
    la version de Pillow installée. L'exception était avalée et toutes
    les polices étaient déclarées sans accents, y compris DejaVu, qui les
    possède. Désormais, si le test échoue, l'erreur est affichée et la
    police écartée.
    """
    try:
        absent = _rendu(police, "\U0010FFFD")
        vide = bytes(64 * 64)
        return all(_rendu(police, c) not in (absent, vide)
                   for c in _TEST_ACCENTS)
    except Exception as exc:            # noqa: BLE001
        print(f"  [vidéo] test de police impossible : {exc}")
        return False


def _candidats_polices():
    """Énumère les fichiers de police à essayer, dans l'ordre : chemins
    système connus, police fournie avec matplotlib, puis recherche dans
    les dossiers de polices du système et d'Isaac Sim."""
    connus = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
    ]
    for c in connus:
        yield Path(c)
    # matplotlib (bibliothèque de graphiques scientifiques) embarque la
    # police DejaVu Sans, qui couvre les accents. C'est la source la plus
    # sûre dans un environnement scientifique comme celui d'Isaac Sim, où
    # aucune police système n'est garantie.
    try:
        import matplotlib
        yield (Path(matplotlib.get_data_path()) / "fonts" / "ttf"
               / "DejaVuSans.ttf")
    except Exception:                   # noqa: BLE001
        pass
    # Dernier recours : polices fournies par Isaac Sim ou par le système.
    # La recherche est bornée (environ 400 fichiers par dossier), car
    # l'arborescence d'Isaac Sim est immense.
    for racine in ("/usr/share/fonts", "/isaac-sim/kit/resources",
                   "/isaac-sim/exts"):
        base = Path(racine)
        if not base.exists():
            continue
        vus = 0
        for f in base.rglob("*.ttf"):
            yield f
            vus += 1
            if vus > 400:
                break


# Police retenue, cherchée une seule fois par processus. Clé « chemin » :
# chemin du fichier, ou None si aucune police ne convient.
_POLICE_CHOISIE = {}


def _police(taille: int):
    """Renvoie une police de la taille demandée qui dessine réellement les
    accents.

    Si aucune n'est trouvée, on se rabat sur la police par défaut de
    Pillow en l'annonçant clairement dans la console : jamais une vidéo
    aux accents cassés sans avertissement. La recherche n'a lieu qu'au
    premier appel ; les appels suivants réutilisent le fichier retenu.
    """
    from PIL import ImageFont
    if "chemin" not in _POLICE_CHOISIE:
        _POLICE_CHOISIE["chemin"] = None
        for chemin in _candidats_polices():
            try:
                f = ImageFont.truetype(str(chemin), 24)
            except Exception:           # noqa: BLE001
                continue
            if _couvre_accents(f):
                _POLICE_CHOISIE["chemin"] = str(chemin)
                print(f"  [vidéo] police : {chemin}")
                break
        if _POLICE_CHOISIE["chemin"] is None:
            print("  [vidéo] AVERTISSEMENT : aucune police avec accents "
                  "trouvée — `apt-get install -y fonts-dejavu-core` dans le "
                  "conteneur. Les accents seront illisibles.")
    if _POLICE_CHOISIE["chemin"]:
        return ImageFont.truetype(_POLICE_CHOISIE["chemin"], taille)
    # Les anciennes versions de Pillow n'acceptent pas l'argument size.
    try:
        return ImageFont.load_default(size=taille)
    except TypeError:
        return ImageFont.load_default()


class SceneRecorder:
    """Filme une scène avec une caméra spectateur et incruste l'état du
    système dans chaque image.

        rec = SceneRecorder(world, "/isaac-sim/videos/demo.mp4",
                            position=(2.0, -3.5, 1.8), cible=(1.2, 0, 0.6))
        rec.start()
        ...
        rec.evenement("L'app a cessé de répondre", t_s=t, duree_s=3.0)
        rec.image(etat=["Service : RUNNING", "Consigne : 0.5 m/s"], t_s=t)
        ...
        rec.terminer()

    start() crée la caméra et ouvre le fichier ; image() est appelée à
    chaque image voulue dans la boucle de test ; terminer() ferme le
    fichier et y ajoute la piste son s'il y en a une. position et cible
    règlent la caméra fixe ; suivre() la fait accompagner le robot.
    """

    def __init__(self, world, chemin_mp4: str,
                 position: Tuple[float, float, float] = (2.0, -3.5, 1.8),
                 cible: Tuple[float, float, float] = (1.2, 0.0, 0.6),
                 largeur: int = 1920, hauteur: int = 1080, fps: int = 25,
                 titre: str = "AXIS-OS", qualite: int = 9,
                 panneau: str = "gauche"):
        self._world = world
        self._chemin = Path(chemin_mp4)
        self._position = position
        self._cible = cible
        self._w, self._h = largeur, hauteur
        self.fps = fps
        self._titre = titre
        self._cam = None
        self._writer = None
        self._bandeau: Optional[str] = None
        self._bandeau_fin_s = 0.0
        self._bandeau_couleur = (220, 60, 50)
        self.images = 0
        self._qualite = qualite
        # Un texte calibré pour du 720p devient trop petit, illisible sur
        # un téléphone, dans une vidéo en 1080p (1920 x 1080) : textes et
        # marges sont donc proportionnels à la hauteur de l'image
        # (référence : 720 pixels).
        self._k = hauteur / 720.0
        self._f_titre = _police(int(24 * self._k))
        self._f_etat = _police(int(18 * self._k))
        self._f_bandeau = _police(int(30 * self._k))
        # Suivi du sujet (voir suivre()) : None tant que la caméra reste
        # fixe. _pos_lissee : position du sujet lissée d'une image à
        # l'autre.
        self._suivi = None
        self._pos_lissee = None
        # Panneau d'état à gauche ou à droite, selon l'endroit où se
        # trouve le sujet, pour ne pas le masquer.
        self._panneau = panneau
        # Sons à mixer dans la vidéo : liste de (instant de fin en
        # secondes, échantillons à 16 kHz, soit 16 000 par seconde).
        self._sons = []
        # Suivi en direct par navigateur (direct.SuiviDirect), optionnel :
        # s'il est branché, il reçoit chaque image composée au moment où
        # elle est produite.
        self.direct = None

    # ---- cycle de vie -------------------------------------------------

    def suivre(self, position_sujet, decalage=(2.4, -2.8, 0.5),
               hauteur_visee: float = 0.75, lissage: float = 0.12,
               decalage_cadre=(-0.45, -0.40)) -> None:
        """Fait accompagner le sujet par la caméra, comme un travelling au
        cinéma.

        Pourquoi : avec une caméra fixe, se rapprocher fait sortir le
        robot du champ pendant sa marche ; la première vidéo, cadrée large
        pour l'éviter, montrait un robot minuscule au milieu d'un sol vide.

        position_sujet : fonction qui renvoie la position (x, y, z) du
                         sujet.
        decalage       : position de la caméra relative au sujet ; par
                         défaut devant et sur le côté (vue de trois quarts
                         avant).
        hauteur_visee  : hauteur fixe du point visé ; la caméra se place à
                         cette hauteur plus decalage[2]. Seule la position
                         au sol (x, y) du sujet est suivie.
        lissage        : fraction de l'écart restant parcourue à chaque
                         image (0.12 = 12 %). Sans lissage, la caméra
                         reproduit chaque oscillation du bassin pendant la
                         marche, et l'image tremble.
        decalage_cadre : décalage (x, y) du point visé par rapport au
                         sujet. La valeur par défaut place le robot au
                         tiers droit de l'image : centré, il touchait le
                         panneau d'état ; au tiers, la gauche reste libre
                         pour le texte, et c'est de toute façon une
                         meilleure composition.

        La caméra est ensuite replacée à chaque image par cadrer().
        """
        self._suivi = (position_sujet, decalage, hauteur_visee, lissage,
                       decalage_cadre)

    def cadrer(self) -> None:
        """Replace la caméra avant le rendu de l'image suivante.

        À appeler avant le pas de simulation qui rend l'image, pas après :
        la caméra doit être en place au moment du rendu. Sans effet si
        aucun suivi n'est réglé (suivre()) ou si la caméra n'existe pas
        encore (start()).
        """
        if self._suivi is None or self._cam is None:
            return
        import numpy as np
        fn, dec, h, a, cadre = self._suivi
        p = np.asarray(fn(), dtype=float)
        # Lissage : la position suivie ne parcourt, à chaque image, qu'une
        # fraction de l'écart avec la position réelle du sujet.
        if self._pos_lissee is None:
            self._pos_lissee = p.copy()
        else:
            self._pos_lissee += a * (p - self._pos_lissee)
        sx, sy = float(self._pos_lissee[0]), float(self._pos_lissee[1])
        # La caméra se place par rapport au sujet, mais vise un point
        # décalé : c'est ce qui déplace le sujet dans l'image.
        self._cam._position = (sx + dec[0], sy + dec[1], h + dec[2])
        self._cam.look_at((sx + cadre[0], sy + cadre[1], h))

    def start(self) -> bool:
        """Crée la caméra spectateur et ouvre le fichier MP4.

        Renvoie False, avec un message, si la caméra ou le fichier ne
        peuvent pas être créés. Une caméra qui ne vise pas la cible
        n'arrête pas l'enregistrement, mais elle est signalée.
        """
        from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
        self._reglages_rendu()
        self._cam = IsaacSimCamera(
            "spectateur", prim_path="/World/spectateur",
            width=self._w, height=self._h, position=self._position)
        if self._cam.initialize().name != "OK":
            print("  [vidéo] caméra spectateur non créée")
            return False
        if not self._cam.look_at(self._cible):
            print("  [vidéo] AVERTISSEMENT : la caméra spectateur ne vise "
                  "pas la cible — la vidéo risque de montrer autre chose")
        self._cam.wait_until_ready(self._world)

        self._chemin.parent.mkdir(parents=True, exist_ok=True)
        # libx264 : encodeur H.264, format vidéo lisible presque partout.
        try:
            import imageio.v2 as imageio
            self._writer = imageio.get_writer(
                str(self._chemin), fps=self.fps, codec="libx264",
                quality=self._qualite, macro_block_size=8)
        except Exception as exc:        # noqa: BLE001
            print(f"  [vidéo] écriture MP4 impossible ({exc}) — "
                  f"`pip install imageio imageio-ffmpeg`")
            return False
        print(f"  [vidéo] enregistrement -> {self._chemin}")
        return True

    @staticmethod
    def _reglages_rendu() -> None:
        """Règle le rendu d'Isaac Sim pour la qualité plutôt que pour la
        vitesse.

        Par défaut, Isaac Sim calcule l'image à résolution réduite puis
        l'agrandit (DLSS, technique d'agrandissement d'image de NVIDIA,
        en mode « performance ») ; constaté dans les journaux : rendu
        interne en 320x240 pour une image demandée en 640x480. Pour une
        vidéo, on privilégie la qualité. Si le réglage n'existe pas dans
        la version utilisée, il reste sans effet ; si le système de
        réglages est absent ou refuse le réglage, un message le signale.
        Dans les deux cas, l'enregistrement continue.
        """
        # carb.settings : système de réglages d'Omniverse Kit, la base
        # logicielle d'Isaac Sim. L'anti-crénelage (« aa ») lisse les bords
        # en escalier des objets.
        try:
            import carb.settings
            st = carb.settings.get_settings()
            st.set("/rtx/post/dlss/execMode", 2)       # 2 = qualité
            st.set("/rtx/post/aa/op", 3)               # anti-crénelage
        except Exception as exc:            # noqa: BLE001
            print(f"  [vidéo] réglages de rendu non appliqués : {exc}")

    def ajouter_son(self, t_fin_s: float, chunk) -> None:
        """Place un son dans la piste audio, de sorte qu'il se termine à
        t_fin_s (en secondes de simulation).

        Pour la parole, t_fin_s est l'instant où le micro a capté la
        phrase : on l'entend donc juste avant la réaction du robot, dans
        l'ordre où les choses se passent (parler, comprendre, agir).
        chunk est converti en mono, en valeurs entre -1 et 1 à 16 kHz,
        par le même outil que la reconnaissance vocale (WhisperRecognizer,
        fondé sur Whisper, le modèle de transcription de la parole
        d'OpenAI).
        """
        import numpy as np
        from axis_os.services.speech.whisper_recognizer import WhisperRecognizer
        x = WhisperRecognizer.to_float_mono(chunk)      # 16 kHz, [-1, 1]
        self._sons.append((float(t_fin_s), np.asarray(x, dtype=np.float32)))

    def _piste_audio(self, duree_s: float, chemin_wav: Path) -> None:
        """Écrit la piste son complète, de la durée de la vidéo, dans un
        fichier WAV (audio non compressé, mono, 16 bits, 16 kHz).

        Chaque son est placé pour se terminer à son instant (un son plus
        long que cet instant commence à 0 s, et la partie d'un son qui
        dépasse la fin de la vidéo est coupée) ; des sons qui se
        chevauchent s'additionnent, et le total est écrêté entre -1 et 1
        avant conversion en entiers 16 bits.
        """
        import wave
        import numpy as np
        sr = 16000
        piste = np.zeros(int(duree_s * sr) + 1, dtype=np.float32)
        for t_fin, x in self._sons:
            debut = max(0, int(t_fin * sr) - x.size)
            fin = min(piste.size, debut + x.size)
            piste[debut:fin] += x[:fin - debut]
        pcm = (np.clip(piste, -1.0, 1.0) * 32767).astype(np.int16)
        with wave.open(str(chemin_wav), "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
            w.writeframes(pcm.tobytes())

    def terminer(self) -> Optional[Path]:
        """Ferme la vidéo et y ajoute la piste son s'il y a des sons.

        Renvoie le chemin du MP4, ou None si l'enregistrement n'a jamais
        démarré. Si l'ajout du son échoue, un message le signale et la
        vidéo est conservée sans son : sous son nom normal, ou sous
        « <nom>_muet.mp4 » si l'échec survient pendant l'assemblage final.
        Attention : dans ce dernier cas, le chemin renvoyé reste celui du
        nom normal, qui peut ne pas contenir la vidéo, et le fichier WAV
        intermédiaire n'est pas effacé.
        """
        if self._writer is None:
            return None
        self._writer.close()
        self._writer = None
        duree = self.images / self.fps
        if self._sons:
            # La vidéo a été écrite sans son ; ffmpeg assemble ensuite
            # image et son en un seul fichier. La vidéo n'est pas
            # réencodée (-c:v copy, copie telle quelle du flux vidéo) :
            # aucune perte de qualité. Le son est encodé en AAC, le format
            # audio habituel des MP4.
            import subprocess
            try:
                import imageio_ffmpeg
                ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
                wav = self._chemin.with_suffix(".wav")
                muet = self._chemin.with_name(self._chemin.stem + "_muet.mp4")
                self._piste_audio(duree, wav)
                self._chemin.rename(muet)
                subprocess.run([ffmpeg, "-y", "-loglevel", "error",
                                "-i", str(muet), "-i", str(wav),
                                "-c:v", "copy", "-c:a", "aac",
                                "-shortest", str(self._chemin)], check=True)
                muet.unlink(missing_ok=True)
                wav.unlink(missing_ok=True)
                print(f"  [vidéo] piste son ajoutée ({len(self._sons)} son(s))")
            except Exception as exc:            # noqa: BLE001
                print(f"  [vidéo] son NON ajouté ({exc}) — vidéo muette")
        print(f"  [vidéo] {self.images} images, {duree:.1f} s -> {self._chemin}")
        return self._chemin

    # ---- contenu --------------------------------------------------------

    def evenement(self, texte: str, t_s: float, duree_s: float = 3.0,
                  couleur: Tuple[int, int, int] = (220, 60, 50)) -> None:
        """Affiche un bandeau d'événement, centré en bas de l'image,
        pendant duree_s secondes de simulation à partir de t_s.

        Un nouvel événement remplace le précédent. Couleur de fond rouge
        par défaut.
        """
        self._bandeau = texte
        self._bandeau_fin_s = t_s + duree_s
        self._bandeau_couleur = couleur

    def image(self, etat: Sequence[str], t_s: float, incrustation=None) -> None:
        """Capture une image, y incruste l'état du système et l'ajoute à
        la vidéo.

        etat : lignes du panneau d'état ; l'instant de simulation t_s est
        ajouté en dernière ligne. incrustation : image PIL (la vue du
        robot) posée en bas à droite, au tiers de la largeur. Sans effet si l'enregistrement n'a pas
        démarré ou si la caméra ne fournit pas encore d'image. L'image
        composée est aussi transmise au suivi en direct, s'il est branché.
        """
        if self._cam is None or self._writer is None:
            return
        f = self._cam.capture()
        if f is None:
            return
        import numpy as np
        from PIL import Image, ImageDraw

        # Les textes sont dessinés sur un calque transparent (RGBA : rouge,
        # vert, bleu et opacité), fusionné ensuite avec l'image : c'est ce
        # qui permet des fonds translucides.
        img = Image.frombytes("RGB", (f.width, f.height), f.data).convert("RGBA")
        calque = Image.new("RGBA", img.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(calque)

        # Panneau d'état en haut, à gauche (ou à droite si demandé), sur
        # fond sombre translucide. Panneau compact, à la largeur du texte :
        # la première version, trop large, masquait le robot quand il
        # passait à gauche du cadre.
        lignes: List[str] = list(etat) + [f"t = {t_s:5.1f} s"]
        largeur = 20 + max(
            [d.textlength(self._titre, font=self._f_titre)]
            + [d.textlength(l, font=self._f_etat) for l in lignes])
        k = self._k
        haut = (48 + 26 * len(lignes)) * k
        x0 = 16 * k
        if self._panneau == "droite":
            x0 = self._w - 16 * k - (largeur + 20 * k)
        d.rectangle([x0, 16 * k, x0 + largeur + 20 * k, 16 * k + haut],
                    fill=(15, 20, 30, 180))
        d.text((x0 + 14 * k, 24 * k), self._titre, font=self._f_titre,
               fill=(120, 200, 255, 255))
        for i, ligne in enumerate(lignes):
            d.text((x0 + 14 * k, (60 + 26 * i) * k), ligne, font=self._f_etat,
                   fill=(235, 235, 235, 255))

        # Bandeau d'événement, centré en bas, tant que sa durée court.
        if self._bandeau and t_s <= self._bandeau_fin_s:
            tw = d.textlength(self._bandeau, font=self._f_bandeau)
            k = self._k
            x0 = (self._w - tw) / 2 - 30 * k
            y0 = self._h - 90 * k
            r, g, b = self._bandeau_couleur
            d.rectangle([x0, y0, x0 + tw + 60 * k, y0 + 60 * k],
                        fill=(r, g, b, 225))
            d.text((x0 + 30 * k, y0 + 12 * k), self._bandeau, font=self._f_bandeau,
                   fill=(255, 255, 255, 255))

        img = Image.alpha_composite(img, calque).convert("RGB")
        if incrustation is not None:
            w_i = self._w // 3
            h_i = round(incrustation.height * w_i / incrustation.width)
            img.paste(incrustation.convert("RGB").resize((w_i, h_i)),
                      (int(self._w - w_i - 16 * self._k), int(self._h - h_i - 16 * self._k)))
        self._writer.append_data(np.asarray(img))
        self.images += 1
        if self.direct is not None:
            self.direct.image(img)
