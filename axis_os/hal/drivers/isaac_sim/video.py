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


def _police(taille: int):
    """Police lisible, ou la police par défaut si aucune n'est trouvée."""
    from PIL import ImageFont
    for chemin in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    ):
        if Path(chemin).exists():
            return ImageFont.truetype(chemin, taille)
    try:
        return ImageFont.load_default(size=taille)
    except TypeError:                   # Pillow ancien
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
                 largeur: int = 1280, hauteur: int = 720, fps: int = 25,
                 titre: str = "AXIS-OS"):
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
        self._f_titre = _police(30)
        self._f_etat = _police(22)
        self._f_bandeau = _police(34)

    # ---- cycle de vie -------------------------------------------------

    def start(self) -> bool:
        from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
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
                quality=8, macro_block_size=8)
        except Exception as exc:        # noqa: BLE001
            print(f"  [vidéo] écriture MP4 impossible ({exc}) — "
                  f"`pip install imageio imageio-ffmpeg`")
            return False
        print(f"  [vidéo] enregistrement -> {self._chemin}")
        return True

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
        lignes: List[str] = list(etat) + [f"t = {t_s:5.1f} s"]
        haut = 60 + 34 * len(lignes)
        d.rectangle([20, 20, 560, 20 + haut], fill=(15, 20, 30, 190))
        d.text((38, 30), self._titre, font=self._f_titre, fill=(120, 200, 255, 255))
        for i, ligne in enumerate(lignes):
            d.text((38, 76 + 34 * i), ligne, font=self._f_etat,
                   fill=(235, 235, 235, 255))

        # Bandeau d'événement, centré en bas.
        if self._bandeau and t_s <= self._bandeau_fin_s:
            tw = d.textlength(self._bandeau, font=self._f_bandeau)
            x0 = (self._w - tw) / 2 - 30
            y0 = self._h - 110
            r, g, b = self._bandeau_couleur
            d.rectangle([x0, y0, x0 + tw + 60, y0 + 70], fill=(r, g, b, 225))
            d.text((x0 + 30, y0 + 14), self._bandeau, font=self._f_bandeau,
                   fill=(255, 255, 255, 255))

        img = Image.alpha_composite(img, calque).convert("RGB")
        self._writer.append_data(np.asarray(img))
        self.images += 1
