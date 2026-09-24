"""
AXIS-OS — Enregistrement vidéo des démonstrations en simulation.

Une vidéo où un robot marche puis s'arrête ne montre RIEN : le
spectateur ne sait pas pourquoi. Ce qui rend une démonstration
compréhensible, c'est l'état du système affiché EN MÊME TEMPS que le
robot — l'app en cours, la consigne reçue, l'état du service, les
refus au moment où ils arrivent.

Ce module fournit donc deux choses :
  - une caméra SPECTATEUR, distincte de celle du robot, qui filme la
    scène de trois quarts ;
  - une INCRUSTATION : panneau d'état permanent, et bandeau
    d'événement quand quelque chose d'important se produit.

Le MP4 est écrit AU FIL DE L'EAU (imageio + ffmpeg) plutôt qu'en
milliers d'images sur disque : une minute en 720p, c'est 1500 images et
plusieurs gigaoctets en PNG.

Le temps de la vidéo est celui de la SIMULATION : la simulation est plus
lente que le temps réel, mais la vidéo se regarde à vitesse normale.
"""
from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence, Tuple


_TEST_ACCENTS = "éèàêôçÉ"


def _rendu(police, c: str) -> bytes:
    """Rendu d'un caractère dans une petite image — comparable d'une
    version de Pillow à l'autre, contrairement à getmask(), dont l'objet
    retourné n'a pas de tobytes() partout."""
    from PIL import Image, ImageDraw
    img = Image.new("L", (64, 64), 0)
    ImageDraw.Draw(img).text((4, 4), c, font=police, fill=255)
    return img.tobytes()


def _couvre_accents(police) -> bool:
    """La police dessine-t-elle VRAIMENT les accents ?

    Une police sans le glyphe dessine un rectangle vide — sans erreur.
    Constaté : « ex□cution », « demand□e » dans la première vidéo. On
    compare chaque caractère accentué au rendu d'un caractère que
    personne ne possède : s'ils sont identiques, le glyphe manque.

    Première version fausse, et silencieusement : elle comparait
    getmask(...).tobytes(), méthode absente dans cette version de
    Pillow. L'exception était avalée et TOUTES les polices étaient
    déclarées sans accents — y compris DejaVu, qui les a.
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
    connus = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
    ]
    for c in connus:
        yield Path(c)
    # matplotlib embarque DejaVu Sans, qui couvre les accents : c'est la
    # source la plus sûre dans un environnement scientifique comme celui
    # d'Isaac Sim, où aucune police système n'est garantie.
    try:
        import matplotlib
        yield (Path(matplotlib.get_data_path()) / "fonts" / "ttf"
               / "DejaVuSans.ttf")
    except Exception:                   # noqa: BLE001
        pass
    # Polices embarquées par Isaac Sim ou le système, recherche bornée :
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


_POLICE_CHOISIE = {}


def _police(taille: int):
    """Police qui dessine réellement les accents, ou la police par défaut
    en le SIGNALANT — jamais une vidéo aux accents cassés en silence."""
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
    try:
        return ImageFont.load_default(size=taille)
    except TypeError:
        return ImageFont.load_default()


class SceneRecorder:
    """Filme une scène et incruste l'état du système.

        rec = SceneRecorder(world, "/isaac-sim/videos/demo.mp4",
                            position=(2.0, -3.5, 1.8), cible=(1.2, 0, 0.6))
        rec.start()
        ...
        rec.evenement("L'app a cessé de répondre", duree_s=3.0)
        rec.image(etat=["Service : RUNNING", "Consigne : 0.5 m/s"], t_s=t)
        ...
        rec.terminer()
    """

    def __init__(self, world, chemin_mp4: str,
                 position: Tuple[float, float, float] = (2.0, -3.5, 1.8),
                 cible: Tuple[float, float, float] = (1.2, 0.0, 0.6),
                 largeur: int = 1920, hauteur: int = 1080, fps: int = 25,
                 titre: str = "AXIS-OS", qualite: int = 9):
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
        # Tailles proportionnelles à la hauteur : un texte calibré pour du
        # 720p devient illisible sur un téléphone en 1080p.
        self._k = hauteur / 720.0
        self._f_titre = _police(int(24 * self._k))
        self._f_etat = _police(int(18 * self._k))
        self._f_bandeau = _police(int(30 * self._k))
        # Suivi : None = caméra fixe.
        self._suivi = None
        self._pos_lissee = None

    # ---- cycle de vie -------------------------------------------------

    def suivre(self, position_sujet, decalage=(2.4, -2.8, 0.5),
               hauteur_visee: float = 0.75, lissage: float = 0.12,
               decalage_cadre=(-0.45, -0.40)) -> None:
        """Caméra qui ACCOMPAGNE le sujet, comme un travelling.

        Avec une caméra fixe, se rapprocher fait sortir le robot du champ
        pendant sa marche ; la première vidéo, cadrée large pour l'éviter,
        montrait un robot minuscule au milieu d'un sol vide.

        position_sujet : fonction rendant la position (x, y, z) du sujet.
        decalage       : position de la caméra RELATIVE au sujet — par
                         défaut devant et sur le côté, trois quarts avant.
        lissage        : fraction du chemin parcourue à chaque image. Sans
                         lissage, la caméra reproduit chaque oscillation
                         du bassin pendant la marche, et l'image tremble.
        decalage_cadre : décalage du point VISÉ par rapport au sujet. Le
                         défaut place le robot au TIERS DROIT de l'image :
                         centré, il touchait le panneau d'état ; au tiers,
                         la gauche reste libre pour le texte — et c'est de
                         toute façon une meilleure composition.
        """
        self._suivi = (position_sujet, decalage, hauteur_visee, lissage,
                       decalage_cadre)

    def cadrer(self) -> None:
        """Replace la caméra avant le RENDU de l'image suivante.

        À appeler AVANT le pas de simulation qui rend l'image, pas
        après : la caméra doit être en place quand le rendu a lieu.
        """
        if self._suivi is None or self._cam is None:
            return
        import numpy as np
        fn, dec, h, a, cadre = self._suivi
        p = np.asarray(fn(), dtype=float)
        if self._pos_lissee is None:
            self._pos_lissee = p.copy()
        else:
            self._pos_lissee += a * (p - self._pos_lissee)
        sx, sy = float(self._pos_lissee[0]), float(self._pos_lissee[1])
        # La caméra se place par rapport au SUJET, mais vise un point
        # décalé : c'est ce qui déplace le sujet dans l'image.
        self._cam._position = (sx + dec[0], sy + dec[1], h + dec[2])
        self._cam.look_at((sx + cadre[0], sy + cadre[1], h))

    def start(self) -> bool:
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
        """Rendu en qualité plutôt qu'en vitesse.

        Par défaut, Isaac Sim rend à résolution réduite puis agrandit
        (DLSS en mode performance) — constaté dans les journaux : rendu
        interne en 320x240 pour une image demandée en 640x480. Pour une
        vidéo, on privilégie la qualité. Sans effet si le réglage
        n'existe pas dans cette version : l'enregistrement continue.
        """
        try:
            import carb.settings
            st = carb.settings.get_settings()
            st.set("/rtx/post/dlss/execMode", 2)       # 2 = qualité
            st.set("/rtx/post/aa/op", 3)               # anti-crénelage
        except Exception as exc:            # noqa: BLE001
            print(f"  [vidéo] réglages de rendu non appliqués : {exc}")

    def terminer(self) -> Optional[Path]:
        if self._writer is not None:
            self._writer.close()
            self._writer = None
            print(f"  [vidéo] {self.images} images, "
                  f"{self.images / self.fps:.1f} s -> {self._chemin}")
            return self._chemin
        return None

    # ---- contenu --------------------------------------------------------

    def evenement(self, texte: str, t_s: float, duree_s: float = 3.0,
                  couleur: Tuple[int, int, int] = (220, 60, 50)) -> None:
        """Bandeau affiché pendant `duree_s` secondes de simulation."""
        self._bandeau = texte
        self._bandeau_fin_s = t_s + duree_s
        self._bandeau_couleur = couleur

    def image(self, etat: Sequence[str], t_s: float) -> None:
        """Capture une image, incruste l'état, l'ajoute à la vidéo."""
        if self._cam is None or self._writer is None:
            return
        f = self._cam.capture()
        if f is None:
            return
        import numpy as np
        from PIL import Image, ImageDraw

        img = Image.frombytes("RGB", (f.width, f.height), f.data).convert("RGBA")
        calque = Image.new("RGBA", img.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(calque)

        # Panneau d'état, en haut à gauche, sur fond sombre translucide.
        # Panneau COMPACT : la première version, trop large, masquait le
        # robot quand il passait à gauche du cadre.
        lignes: List[str] = list(etat) + [f"t = {t_s:5.1f} s"]
        largeur = 20 + max(
            [d.textlength(self._titre, font=self._f_titre)]
            + [d.textlength(l, font=self._f_etat) for l in lignes])
        k = self._k
        haut = (48 + 26 * len(lignes)) * k
        d.rectangle([16 * k, 16 * k, 16 * k + largeur + 20 * k, 16 * k + haut],
                    fill=(15, 20, 30, 180))
        d.text((30 * k, 24 * k), self._titre, font=self._f_titre,
               fill=(120, 200, 255, 255))
        for i, ligne in enumerate(lignes):
            d.text((30 * k, (60 + 26 * i) * k), ligne, font=self._f_etat,
                   fill=(235, 235, 235, 255))

        # Bandeau d'événement, centré en bas.
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
        self._writer.append_data(np.asarray(img))
        self.images += 1
