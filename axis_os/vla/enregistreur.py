"""
Enregistreur d'épisodes pour entraîner un VLA (GR00T) par imitation.

Le robot actuel, avec ses réflexes codés, sert de PROFESSEUR : chaque
tentative de prise est enregistrée (image de la tête, état des bras,
consignes envoyées), puis seules les prises RÉUSSIES servent à
l'entraînement (vers_lerobot.py).

Format sur disque, volontairement simple et sans dépendance lourde (le
conteneur Isaac Sim n'a ni pandas ni pyarrow) :

    <dossier>/episode_000012.npz   etat (T, n_etat), action (T, n_action),
                                   temps (T,), images (T, H, W, 3) si pas de
                                   vidéo
    <dossier>/episode_000012.mp4   les images, si imageio sait écrire du MP4
    <dossier>/episodes.jsonl       une ligne par épisode : consigne, succès,
                                   nombre d'images, méta-données

La conversion au format LeRobot se fait ensuite, hors simulation.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np


class EnregistreurEpisodes:
    def __init__(self, dossier: str, fps: float = 10.0,
                 noms_etat: Sequence[str] = (), noms_action: Sequence[str] = (),
                 duree_max_s: float = 60.0):
        """
        fps          : cadence d'enregistrement (temps SIMULÉ). La boucle
                       appelle pas() à chaque pas ; l'enregistreur ne garde
                       qu'une image toutes les 1/fps secondes.
        noms_etat    : nom de chaque composante de l'état (écrits dans les
                       méta-données, pour la conversion).
        duree_max_s  : un épisode plus long est clos en échec (sécurité :
                       une tentative bloquée ne remplit pas le disque).
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
        """Vrai si le prochain pas() à l'instant t gardera une image : la
        boucle de simulation ne rend la caméra que dans ce cas."""
        return self._ep is not None and (
            self._ep["t_dernier"] is None or t - self._ep["t_dernier"] >= self.periode - 1e-9)

    def _prochain_index(self) -> int:
        idx = [int(p.stem.split("_")[1]) for p in self.dossier.glob("episode_*.npz")]
        return max(idx) + 1 if idx else 0

    # ---- épisode ---------------------------------------------------------
    def debut(self, instruction: str, t: float, meta: Optional[Dict] = None) -> None:
        if self._ep is not None:
            self.fin(False, t, raison="nouvel épisode commencé avant la fin du précédent")
        self._ep = {"instruction": instruction, "t0": t, "t_dernier": None,
                    "meta": dict(meta or {}), "etat": [], "action": [],
                    "temps": [], "images": []}

    def pas(self, t: float, image: Optional[np.ndarray], etat: Sequence[float],
            action: Sequence[float]) -> bool:
        """Ajoute une image si la période est écoulée. Rend True si gardée.
        Une image manquante (caméra pas encore rendue) : le pas est ignoré —
        jamais d'échantillon sans image, le VLA apprend de ce qu'il VOIT."""
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
            donnees["images"] = images        # repli : images dans le npz
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
        try:
            import imageio.v2 as imageio
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
    """Les lignes de episodes.jsonl (épisodes réussis ET ratés)."""
    p = Path(dossier) / "episodes.jsonl"
    if not p.exists():
        return []
    return [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines() if l.strip()]
