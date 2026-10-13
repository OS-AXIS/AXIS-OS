"""
Enregistreur d'épisodes pour entraîner un VLA (GR00T) par imitation.

Un VLA (Vision-Language-Action) est un modèle appris qui reçoit une image,
une consigne en langage naturel et l'état du robot, et qui propose des
mouvements ; GR00T est celui que publie NVIDIA pour les humanoïdes. Un
épisode est une tentative de prise, enregistrée du début à la fin.

Le robot actuel, avec ses réflexes codés (le comportement programmé à la
main), sert de professeur : chaque tentative de prise est enregistrée
(image de la tête, état des bras, consignes envoyées), puis seules les
prises réussies servent à l'entraînement (voir vers_lerobot.py).

Format sur disque, volontairement simple et sans dépendance lourde, car le
conteneur Isaac Sim (le simulateur de NVIDIA) n'a ni pandas ni pyarrow
(bibliothèques Python de tableaux de données) :

    <dossier>/episode_000012.npz   etat (T, n_etat), action (T, n_action),
                                   temps (T,), et images (T, H, W, 3) si
                                   la vidéo n'a pas pu être écrite
    <dossier>/episode_000012.mp4   les images, si imageio sait écrire du MP4
    <dossier>/episodes.jsonl       une ligne JSON par épisode : consigne,
                                   succès, nombre d'images, méta-données

Un .npz est une archive compressée de tableaux numpy ; T est le nombre
d'images gardées, H et W la hauteur et la largeur de l'image. imageio est
une bibliothèque Python de lecture et d'écriture d'images et de vidéos.

La conversion au format LeRobot se fait ensuite, hors simulation.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np


class EnregistreurEpisodes:
    """Enregistre un épisode à la fois : debut(), puis pas() à chaque pas
    de la boucle de simulation, puis fin(), qui écrit l'épisode sur
    disque."""

    def __init__(self, dossier: str, fps: float = 10.0,
                 noms_etat: Sequence[str] = (), noms_action: Sequence[str] = (),
                 duree_max_s: float = 60.0):
        """
        dossier      : où écrire les épisodes ; créé au besoin. La
                       numérotation reprend après le plus grand numéro déjà
                       présent : aucun épisode existant n'est écrasé.
        fps          : cadence d'enregistrement, en temps simulé. La boucle
                       appelle pas() à chaque pas ; l'enregistreur ne garde
                       qu'une image toutes les 1/fps secondes.
        noms_etat    : nom de chaque composante de l'état (écrits dans les
                       méta-données, pour la conversion).
        noms_action  : nom de chaque composante de l'action, de même.
        duree_max_s  : un épisode plus long est clos en échec. C'est une
                       sécurité : une tentative bloquée ne remplit pas le
                       disque.
        """
        self.dossier = Path(dossier)
        self.dossier.mkdir(parents=True, exist_ok=True)
        self.periode = 1.0 / fps
        self.fps = fps
        self.noms_etat, self.noms_action = list(noms_etat), list(noms_action)
        self.duree_max_s = duree_max_s
        self._index = self._prochain_index()
        self._ep: Optional[Dict] = None
        self.stats = {"episodes": 0, "reussis": 0, "rates": 0, "images": 0}

    # ---- état ----------------------------------------------------------
    @property
    def actif(self) -> bool:
        return self._ep is not None

    def veut_image(self, t: float) -> bool:
        """Vrai si un appel à pas() à l'instant t garderait une image : la
        boucle de simulation ne rend la caméra que dans ce cas."""
        # 1e-9 : tolérance d'arrondi, pour qu'un pas qui tombe pile sur la
        # période soit gardé.
        return self._ep is not None and (
            self._ep["t_dernier"] is None or t - self._ep["t_dernier"] >= self.periode - 1e-9)

    def _prochain_index(self) -> int:
        """Numéro du prochain épisode : un de plus que le plus grand déjà
        sur disque, 0 si le dossier est vide."""
        idx = [int(p.stem.split("_")[1]) for p in self.dossier.glob("episode_*.npz")]
        return max(idx) + 1 if idx else 0

    # ---- épisode ---------------------------------------------------------
    def debut(self, instruction: str, t: float, meta: Optional[Dict] = None) -> None:
        """Ouvre un épisode pour la consigne `instruction`. Si le précédent
        n'a pas été clos, il l'est d'abord, en échec."""
        if self._ep is not None:
            self.fin(False, t, raison="nouvel épisode commencé avant la fin du précédent")
        self._ep = {"instruction": instruction, "t0": t, "t_dernier": None,
                    "meta": dict(meta or {}), "etat": [], "action": [],
                    "temps": [], "images": []}

    def pas(self, t: float, image: Optional[np.ndarray], etat: Sequence[float],
            action: Sequence[float]) -> bool:
        """Ajoute un échantillon (image, état, action) si la période est
        écoulée ; rend True s'il est gardé. Sans image (caméra pas encore
        rendue), le pas est ignoré : jamais d'échantillon sans image,
        puisque le VLA apprend de ce qu'il voit. Un épisode qui dépasse
        duree_max_s est clos en échec."""
        ep = self._ep
        if ep is None or image is None or not self.veut_image(t):
            return False
        if t - ep["t0"] > self.duree_max_s:
            self.fin(False, t, raison="épisode trop long")
            return False
        ep["t_dernier"] = t
        ep["temps"].append(t - ep["t0"])
        ep["etat"].append(np.asarray(etat, dtype=np.float32))
        ep["action"].append(np.asarray(action, dtype=np.float32))
        ep["images"].append(np.asarray(image, dtype=np.uint8))
        self.stats["images"] += 1
        return True

    def fin(self, succes: bool, t: float, raison: str = "") -> Optional[Path]:
        """Clôt l'épisode et l'écrit sur disque ; rend le chemin du .npz.
        Rend None s'il n'y avait pas d'épisode ouvert, ou si aucune image
        n'a été gardée : un tel épisode est compté dans les statistiques,
        mais rien n'est écrit."""
        ep, self._ep = self._ep, None
        if ep is None:
            return None
        n = len(ep["temps"])
        self.stats["episodes"] += 1
        self.stats["reussis" if succes else "rates"] += 1
        if n == 0:
            return None
        nom = f"episode_{self._index:06d}"
        self._index += 1
        images = np.stack(ep["images"])
        video = self._ecrire_video(self.dossier / f"{nom}.mp4", images)
        donnees = {"etat": np.stack(ep["etat"]), "action": np.stack(ep["action"]),
                   "temps": np.asarray(ep["temps"], dtype=np.float32)}
        if not video:
            donnees["images"] = images        # repli : sans vidéo, images dans le npz
        np.savez_compressed(self.dossier / f"{nom}.npz", **donnees)
        ligne = {"episode": nom, "instruction": ep["instruction"], "succes": bool(succes),
                 "raison": raison, "images": n, "duree_s": round(t - ep["t0"], 3),
                 "fps": self.fps, "video": video, "hauteur": int(images.shape[1]),
                 "largeur": int(images.shape[2]), "noms_etat": self.noms_etat,
                 "noms_action": self.noms_action, "meta": ep["meta"],
                 "ecrit_le": time.strftime("%Y-%m-%d %H:%M:%S")}
        with open(self.dossier / "episodes.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(ligne, ensure_ascii=False) + "\n")
        return self.dossier / f"{nom}.npz"

    def _ecrire_video(self, chemin: Path, images: np.ndarray) -> bool:
        """Écrit les images en MP4 (codec H.264). En cas d'échec (imageio
        absent, encodeur indisponible…), rend False sans laisser de fichier
        partiel : les images iront alors dans le npz."""
        try:
            import imageio.v2 as imageio
            # macro_block_size=1 : garder la taille d'image exacte, sans
            # l'arrondir à un multiple de 16.
            with imageio.get_writer(str(chemin), fps=self.fps, codec="libx264",
                                    quality=8, macro_block_size=1) as w:
                for im in images:
                    w.append_data(im)
            return True
        except Exception as exc:              # noqa: BLE001
            print(f"  [vla] vidéo non écrite ({exc}) — images gardées dans le npz", flush=True)
            if chemin.exists():
                chemin.unlink()
            return False


def lire_episodes(dossier: str) -> List[Dict]:
    """Les lignes de episodes.jsonl, épisodes réussis et ratés ; liste vide
    si le fichier n'existe pas. Aucune vérification ici : vers_lerobot.py
    valide chaque ligne avant de s'en servir."""
    p = Path(dossier) / "episodes.jsonl"
    if not p.exists():
        return []
    return [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines() if l.strip()]
