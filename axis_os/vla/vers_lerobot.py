"""
Conversion des épisodes enregistrés au format LeRobot v2 — celui que lisent
les scripts d'entraînement de NVIDIA Isaac GR00T.

    python -m axis_os.vla.vers_lerobot <dossier_episodes> <dossier_sortie> [--prise-seule]

--prise-seule : ne garde de chaque épisode que la PRISE, base immobile —
les images qui suivent le dernier mouvement de la base (serrage, levage).
Constaté au premier essai GPU (VLA complet, approche comprise) : le modèle
marchait jusqu'à la table et tendait les bras, mais se plaçait 20-30 cm à
côté du carton et ne serrait jamais. Partage des rôles retenu : les réflexes
codés s'approchent (navigation, visée de face), le VLA ne fait que le geste.

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

LES ÉPISODES SONT DES DONNÉES NON FIABLES (audit du 2026-10-10) : ils
reviennent d'une machine louée, ou d'une sauvegarde téléchargée
(sauvegarder_vla.sh --restaurer). Donc :
  - np.load toujours avec allow_pickle=False, écrit explicitement (c'est
    la valeur par défaut de numpy, mais une valeur par défaut peut changer
    et un tableau d'objets exécuterait du code au chargement) ;
  - le nom d'épisode, lu dans episodes.jsonl, sert à construire un chemin :
    seul « episode_NNNNNN » est accepté — sinon « ../../ailleurs » ferait
    lire ou copier n'importe quel fichier. Un nom invalide ARRÊTE la
    conversion (fichier d'index corrompu ou falsifié), avant d'effacer quoi
    que ce soit ;
  - formes, types et finitude vérifiés : un épisode avec un NaN, ou des
    dimensions qui ne collent pas, est ÉCARTÉ et signalé — sinon il
    empoisonnerait l'entraînement sans aucune erreur visible.
"""
from __future__ import annotations

import json
import re
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from axis_os.vla.enregistreur import lire_episodes

CLE_VIDEO = "observation.images.ego_view"
# Nom d'épisode tel que l'écrit EnregistreurEpisodes ; rien d'autre (voir
# l'en-tête : il sert à construire des chemins).
_NOM_EPISODE = re.compile(r"^episode_\d{6,}$")


def _verifier_nom(e: Dict) -> str:
    nom = e.get("episode") if isinstance(e, dict) else None
    if not isinstance(nom, str) or not _NOM_EPISODE.match(nom):
        raise ValueError(f"nom d'épisode invalide {str(nom)[:80]!r} dans episodes.jsonl "
                         "(attendu episode_NNNNNN) — index corrompu ou falsifié : "
                         "conversion refusée")
    return nom


def _incoherence(e: Dict, ref: Dict) -> Optional[str]:
    """L'en-tête de l'épisode doit être celui du premier : un seul format par jeu."""
    for cle in ("fps", "noms_etat", "noms_action", "hauteur", "largeur"):
        if e.get(cle) != ref.get(cle):
            return f"{cle} = {str(e.get(cle))[:60]} ≠ {str(ref.get(cle))[:60]} (premier épisode)"
    if not isinstance(e.get("instruction"), str) or not e["instruction"].strip():
        return "consigne absente ou non textuelle"
    return None


def _verifier_tableaux(etat, action, temps, images, n_etat: int, n_action: int,
                       h: int, w: int) -> Optional[str]:
    """Formes, types numériques réels, finitude. None si tout va bien."""
    for nom, a in (("etat", etat), ("action", action), ("temps", temps)):
        if a.dtype.kind not in "iuf":
            return f"{nom} de type {a.dtype} (nombres réels attendus)"
        if not np.all(np.isfinite(a)):
            return f"{nom} contient NaN/inf"
    if etat.ndim != 2 or etat.shape[1] != n_etat:
        return f"etat de forme {etat.shape}, attendu (T, {n_etat})"
    if action.ndim != 2 or action.shape[1] != n_action:
        return f"action de forme {action.shape}, attendu (T, {n_action})"
    if temps.ndim != 1 or not (len(temps) == len(etat) == len(action)):
        return f"longueurs incohérentes : temps {temps.shape}, etat {etat.shape}, action {action.shape}"
    if images is not None and (images.dtype != np.uint8
                               or images.shape != (len(temps), h, w, 3)):
        return f"images {images.dtype} {images.shape}, attendu uint8 ({len(temps)}, {h}, {w}, 3)"
    return None


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


def debut_prise(action: np.ndarray, n_base: int = 3, seuil: float = 0.01) -> int:
    """Indice de la première image après le dernier mouvement de la base
    (les n_base dernières composantes de l'action : vx, vy, wz)."""
    bouge = np.flatnonzero(np.abs(action[:, -n_base:]).max(axis=1) > seuil)
    return int(bouge[-1]) + 1 if len(bouge) else 0


def debut_geste(action: np.ndarray, i0: int, n_base: int = 3, seuil: float = 0.02) -> int:
    """Après l'arrêt de la base, la première image où la consigne des BRAS
    change (début du serrage) — moins une image. Constaté sur GPU (prise
    seule, bras figés 30 s autour du carton) : les démonstrations commençaient
    par 0.6 à 3 s d'immobilité (stabilisation de la base) ; pour la même image
    et le même état, le modèle avait surtout appris « ne rien faire »."""
    bras = action[:, :-n_base]
    if i0 >= len(bras):
        return i0
    bouge = np.flatnonzero(np.abs(bras[i0:] - bras[i0]).max(axis=1) > seuil)
    return i0 + max(0, int(bouge[0]) - 1) if len(bouge) else i0


def convertir(dossier: str, sortie: str, robot: str = "unitree_g1_axis_os",
              prise_seule: bool = False, images_min: int = 10) -> Dict:
    import pandas as pd

    eps = [e for e in lire_episodes(dossier) if e.get("succes") is True]
    if not eps:
        raise SystemExit("aucun épisode réussi à convertir")
    # Noms vérifiés AVANT d'effacer la sortie : un index falsifié n'efface rien.
    for e in eps:
        _verifier_nom(e)
    src, dst = Path(dossier), Path(sortie)
    if dst.exists():
        shutil.rmtree(dst)
    (dst / "meta").mkdir(parents=True)
    (dst / "data" / "chunk-000").mkdir(parents=True)
    (dst / "videos" / "chunk-000" / CLE_VIDEO).mkdir(parents=True)

    taches: Dict[str, int] = {}
    index_global, lignes_ep, longueurs, trop_courts = 0, [], [], 0
    fps = eps[0]["fps"]
    noms_etat, noms_action = eps[0]["noms_etat"], eps[0]["noms_action"]
    h, w = eps[0]["hauteur"], eps[0]["largeur"]
    k = -1
    rejetes: List[str] = []

    def rejeter(nom: str, raison: str) -> None:
        rejetes.append(f"{nom} : {raison}")
        print(f"  [vers_lerobot] épisode ÉCARTÉ — {nom} : {raison}", flush=True)

    for e in eps:
        nom = e["episode"]
        raison = _incoherence(e, eps[0])
        if raison:
            rejeter(nom, raison)
            continue
        try:
            d = np.load(src / f"{nom}.npz", allow_pickle=False)
            if not isinstance(d, np.lib.npyio.NpzFile):
                # Un .npy renommé en .npz se charge en simple tableau.
                raise ValueError(f"pas une archive npz ({type(d).__name__})")
            with d:
                etat, action, temps = d["etat"], d["action"], d["temps"]
                images_npz = d["images"] if "images" in d.files else None
        except Exception as exc:              # noqa: BLE001 — écarté, jamais chargé
            # Fichier absent, clé manquante, tableau d'objets (refusé par
            # allow_pickle=False), archive tronquée ou corrompue (BadZipFile,
            # EOFError, zlib…) : l'épisode est ÉCARTÉ. Toute exception, et pas
            # une liste : trouvé à la revue, un npz coupé par un docker cp
            # interrompu levait BadZipFile, non attrapée, et faisait échouer
            # toute la conversion après l'effacement de la sortie existante.
            rejeter(nom, f"npz illisible ou refusé ({type(exc).__name__}: {str(exc)[:120]})")
            continue
        raison = _verifier_tableaux(etat, action, temps, images_npz, len(noms_etat),
                                    len(noms_action), h, w)
        if raison:
            rejeter(nom, raison)
            continue
        i0 = debut_geste(action, debut_prise(action)) if prise_seule else 0
        if len(temps) - i0 < images_min:
            trop_courts += 1
            continue
        k += 1
        etat, action = etat[i0:], action[i0:].copy()
        if prise_seule:
            action[:, -3:] = 0.0              # base immobile, par construction
        n = len(etat)
        longueurs.append(n)
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
        if i0 == 0 and e.get("video") and (src / f"{nom}.mp4").exists():
            shutil.copyfile(src / f"{nom}.mp4", video)
        else:
            import imageio.v2 as imageio
            if images_npz is not None:
                images = images_npz
            else:                             # découper la vidéo d'origine
                with imageio.get_reader(str(src / f"{nom}.mp4")) as rd:
                    images = [im for im in rd]
            with imageio.get_writer(str(video), fps=fps, codec="libx264",
                                    macro_block_size=1) as wr:
                for im in images[i0:i0 + n]:
                    wr.append_data(im)
        lignes_ep.append({"episode_index": k, "tasks": [e["instruction"]], "length": n})
        index_global += n

    if k < 0:
        raise SystemExit(f"aucun épisode valide à convertir ({len(rejetes)} écartés, "
                         f"{trop_courts} trop courts) : {rejetes[:5]}")
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
        # Clé lue par la configuration de modalités de GR00T :
        # "annotation.human.task_description" -> colonne task_index.
        "annotation": {"human.task_description": {"original_key": "task_index"}},
    }
    (dst / "meta" / "modality.json").write_text(json.dumps(modalite, indent=2))
    info = {
        "codebase_version": "v2.0", "robot_type": robot,
        "total_episodes": k + 1, "total_frames": index_global,
        "total_tasks": len(taches), "total_videos": k + 1, "total_chunks": 1,
        "chunks_size": 1000, "fps": fps, "splits": {"train": f"0:{k + 1}"},
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
    out = {"episodes": k + 1, "images": index_global, "taches": len(taches),
           "ignores_rates": len(lire_episodes(dossier)) - len(eps),
           "ecartes_invalides": len(rejetes)}
    if rejetes:
        out["detail_ecartes"] = rejetes
    if prise_seule:
        out.update({"trop_courts": trop_courts,
                    "images_par_episode_min_med_max": (
                        min(longueurs), int(np.median(longueurs)), max(longueurs))})
    return out


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "--prise-seule"]
    if len(args) != 2:
        raise SystemExit(__doc__)
    print(convertir(args[0], args[1], prise_seule="--prise-seule" in sys.argv))
