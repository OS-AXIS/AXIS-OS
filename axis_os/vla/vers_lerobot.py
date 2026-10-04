"""
Conversion des épisodes enregistrés au format LeRobot v2 — celui que lisent
les scripts d'entraînement de NVIDIA Isaac GR00T.

    python -m axis_os.vla.vers_lerobot <dossier_episodes> <dossier_sortie>

Seuls les épisodes RÉUSSIS sont gardés : le VLA imite le professeur quand
il a bien fait, jamais quand il a raté.

Arborescence produite :

    meta/info.json, meta/episodes.jsonl, meta/tasks.jsonl, meta/modality.json
    data/chunk-000/episode_000000.parquet
    videos/chunk-000/observation.images.ego_view/episode_000000.mp4

meta/modality.json est propre à GR00T : il découpe le vecteur d'état et
d'action en parties nommées (bras gauche, bras droit, base).

LIMITE HONNÊTE : ce format suit la documentation publique de GR00T et de
LeRobot v2 au moment de l'écriture. L'outillage de NVIDIA évolue (N1.5,
N1.6, N1.7…) : vérifier avec leur chargeur de données, sur ce jeu, avant de
lancer un entraînement de plusieurs heures.

Nécessite pandas et pyarrow (présents dans l'environnement d'entraînement
de GR00T, pas dans le conteneur Isaac Sim).
"""
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

from axis_os.vla.enregistreur import lire_episodes

CLE_VIDEO = "observation.images.ego_view"


def _decoupage(noms: List[str]) -> Dict[str, Dict[str, int]]:
    """Parties contiguës du vecteur, d'après les noms : left_*, right_*,
    base_*. Un nom qui ne rentre dans aucune partie forme « autres »."""
    parties: Dict[str, List[int]] = {}
    for i, n in enumerate(noms):
        cle = ("bras_gauche" if n.startswith("left_") else
               "bras_droit" if n.startswith("right_") else
               "base" if n.startswith("base_") else "autres")
        parties.setdefault(cle, []).append(i)
    out = {}
    for cle, idx in parties.items():
        if idx != list(range(idx[0], idx[-1] + 1)):
            raise ValueError(f"partie « {cle} » non contiguë dans le vecteur : {idx}")
        out[cle] = {"start": idx[0], "end": idx[-1] + 1}
    return out


def convertir(dossier: str, sortie: str, robot: str = "unitree_g1_axis_os") -> Dict:
    import pandas as pd

    eps = [e for e in lire_episodes(dossier) if e["succes"]]
    if not eps:
        raise SystemExit("aucun épisode réussi à convertir")
    src, dst = Path(dossier), Path(sortie)
    if dst.exists():
        shutil.rmtree(dst)
    (dst / "meta").mkdir(parents=True)
    (dst / "data" / "chunk-000").mkdir(parents=True)
    (dst / "videos" / "chunk-000" / CLE_VIDEO).mkdir(parents=True)

    taches: Dict[str, int] = {}
    index_global, lignes_ep = 0, []
    fps = eps[0]["fps"]
    noms_etat, noms_action = eps[0]["noms_etat"], eps[0]["noms_action"]
    h, w = eps[0]["hauteur"], eps[0]["largeur"]
    for k, e in enumerate(eps):
        d = np.load(src / f"{e['episode']}.npz")
        etat, action, temps = d["etat"], d["action"], d["temps"]
        n = len(temps)
        t_idx = taches.setdefault(e["instruction"], len(taches))
        df = pd.DataFrame({
            "observation.state": [r.astype(np.float32).tolist() for r in etat],
            "action": [r.astype(np.float32).tolist() for r in action],
            "timestamp": (np.arange(n) / fps).astype(np.float32),
            "frame_index": np.arange(n, dtype=np.int64),
            "episode_index": np.full(n, k, dtype=np.int64),
            "index": np.arange(index_global, index_global + n, dtype=np.int64),
            "task_index": np.full(n, t_idx, dtype=np.int64),
            "annotation.human.action.task_description": np.full(n, t_idx, dtype=np.int64),
            "next.reward": np.r_[np.zeros(n - 1), 1.0].astype(np.float32),
            "next.done": np.r_[np.zeros(n - 1, dtype=bool), True],
        })
        df.to_parquet(dst / "data" / "chunk-000" / f"episode_{k:06d}.parquet")
        video = dst / "videos" / "chunk-000" / CLE_VIDEO / f"episode_{k:06d}.mp4"
        if e.get("video") and (src / f"{e['episode']}.mp4").exists():
            shutil.copyfile(src / f"{e['episode']}.mp4", video)
        else:
            import imageio.v2 as imageio
            with imageio.get_writer(str(video), fps=fps, codec="libx264",
                                    macro_block_size=1) as wr:
                for im in d["images"]:
                    wr.append_data(im)
        lignes_ep.append({"episode_index": k, "tasks": [e["instruction"]], "length": n})
        index_global += n

    with open(dst / "meta" / "episodes.jsonl", "w", encoding="utf-8") as f:
        for l in lignes_ep:
            f.write(json.dumps(l, ensure_ascii=False) + "\n")
    with open(dst / "meta" / "tasks.jsonl", "w", encoding="utf-8") as f:
        for t, i in sorted(taches.items(), key=lambda kv: kv[1]):
            f.write(json.dumps({"task_index": i, "task": t}, ensure_ascii=False) + "\n")
    modalite = {
        "state": _decoupage(noms_etat),
        "action": _decoupage(noms_action),
        "video": {"ego_view": {"original_key": CLE_VIDEO}},
        "annotation": {"human.action.task_description": {}},
    }
    (dst / "meta" / "modality.json").write_text(json.dumps(modalite, indent=2))
    info = {
        "codebase_version": "v2.0", "robot_type": robot,
        "total_episodes": len(eps), "total_frames": index_global,
        "total_tasks": len(taches), "total_videos": len(eps), "total_chunks": 1,
        "chunks_size": 1000, "fps": fps, "splits": {"train": f"0:{len(eps)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            CLE_VIDEO: {"dtype": "video", "shape": [h, w, 3],
                        "names": ["height", "width", "channel"],
                        "video_info": {"video.fps": fps, "video.codec": "h264"}},
            "observation.state": {"dtype": "float32", "shape": [len(noms_etat)],
                                  "names": noms_etat},
            "action": {"dtype": "float32", "shape": [len(noms_action)],
                       "names": noms_action},
            "timestamp": {"dtype": "float32", "shape": [1]},
            "frame_index": {"dtype": "int64", "shape": [1]},
            "episode_index": {"dtype": "int64", "shape": [1]},
            "index": {"dtype": "int64", "shape": [1]},
            "task_index": {"dtype": "int64", "shape": [1]},
            "annotation.human.action.task_description": {"dtype": "int64", "shape": [1]},
            "next.reward": {"dtype": "float32", "shape": [1]},
            "next.done": {"dtype": "bool", "shape": [1]},
        },
    }
    (dst / "meta" / "info.json").write_text(json.dumps(info, indent=2))
    return {"episodes": len(eps), "images": index_global, "taches": len(taches),
            "ignores_rates": len(lire_episodes(dossier)) - len(eps)}


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    print(convertir(sys.argv[1], sys.argv[2]))
