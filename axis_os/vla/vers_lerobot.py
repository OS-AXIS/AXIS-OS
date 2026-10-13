"""
Conversion des épisodes enregistrés au format LeRobot v2, celui que lisent
les scripts d'entraînement de NVIDIA Isaac GR00T.

Usage :

    python -m axis_os.vla.vers_lerobot <dossier_episodes> <dossier_sortie> [--prise-seule]

    <dossier_episodes>  dossier écrit par l'enregistreur (enregistreur.py) :
                        episodes.jsonl et les fichiers episode_NNNNNN.npz
                        (et .mp4) ;
    <dossier_sortie>    jeu de données LeRobot à produire. Attention : s'il
                        existe déjà, il est effacé puis recréé ;
    --prise-seule       ne garde de chaque épisode que la prise, base
                        immobile (voir plus bas).

La commande affiche à la fin un bilan : épisodes convertis, images,
consignes, épisodes ratés ignorés, épisodes écartés et pourquoi.

GR00T est le modèle VLA (Vision-Language-Action) de NVIDIA pour les
humanoïdes : il reçoit une image, une consigne en langage naturel et
l'état du robot, et propose des mouvements. LeRobot v2 est le format de
jeu de données de la bibliothèque LeRobot, que GR00T reprend.

Seuls les épisodes réussis sont gardés : le VLA imite le professeur (le
robot piloté par ses réflexes codés) quand il a bien fait, jamais quand il
a raté. Un épisode qui garde moins de 10 images (après le découpage, si
--prise-seule est donné) est ignoré.

--prise-seule : ne garde de chaque épisode que la prise, base immobile,
c'est-à-dire les images qui suivent le dernier mouvement de la base
(serrage, levage), sans l'immobilité qui précède le geste. Pourquoi :
constaté au premier essai sur la machine GPU (la machine louée, équipée
d'une carte de calcul, où tournent la simulation et l'entraînement) avec
le VLA complet (approche comprise), le modèle marchait jusqu'à la table et
tendait les bras, mais se plaçait 20 à 30 cm à côté du carton et ne
serrait jamais. Partage des rôles
retenu : les réflexes codés font l'approche (navigation, visée de face), le
VLA ne fait que le geste.

Arborescence produite :

    meta/info.json, meta/episodes.jsonl, meta/tasks.jsonl, meta/modality.json
    data/chunk-000/episode_000000.parquet
    videos/chunk-000/observation.images.ego_view/episode_000000.mp4

Un fichier .parquet est un tableau de données rangé par colonnes (une
ligne par image) ; « chunk-000 » est le premier paquet d'épisodes.

meta/modality.json est propre à GR00T : il découpe le vecteur d'état et
d'action en parties nommées (bras gauche, bras droit, base), que reprend
config_gr00t_g1.py.

Limite connue : ce format suit la documentation publique de GR00T et de
LeRobot v2 au moment de l'écriture. L'outillage de NVIDIA évolue (N1.5,
N1.6, N1.7…) : vérifier avec leur chargeur de données, sur ce jeu, avant de
lancer un entraînement de plusieurs heures.

Nécessite pandas et pyarrow (présents dans l'environnement d'entraînement
de GR00T, pas dans le conteneur Isaac Sim, le simulateur de NVIDIA).

Les épisodes sont traités comme des données non fiables, à la demande d'un
audit de sécurité (2026-10-10) : ils reviennent d'une machine louée, ou
d'une sauvegarde téléchargée (deploy/machine_gpu/sauvegarder_vla.sh
--restaurer). Donc :
  - np.load est toujours appelé avec allow_pickle=False, écrit
    explicitement. Pickle est le format Python de sérialisation d'objets,
    dans lequel numpy range les tableaux d'objets : charger un tel tableau
    exécuterait du code. C'est déjà la valeur par défaut de numpy, mais une
    valeur par défaut peut changer ;
  - le nom d'épisode, lu dans episodes.jsonl, sert à construire un chemin :
    seul « episode_NNNNNN » est accepté. Sinon « ../../ailleurs » ferait
    lire ou copier n'importe quel fichier. Un nom invalide arrête la
    conversion (fichier d'index corrompu ou falsifié), avant d'effacer quoi
    que ce soit ;
  - formes, types et finitude sont vérifiés : un épisode avec un NaN (« pas
    un nombre », valeur numérique invalide), ou des dimensions qui ne
    collent pas, est écarté et signalé. Sinon il empoisonnerait
    l'entraînement sans aucune erreur visible. Un npz illisible est écarté
    de même. S'il ne reste aucun épisode valide, la conversion s'arrête en
    erreur.
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

# Nom de la vidéo de la caméra de tête dans le jeu LeRobot.
CLE_VIDEO = "observation.images.ego_view"
# Nom d'épisode tel que l'écrit EnregistreurEpisodes, et rien d'autre : il
# sert à construire des chemins (voir l'en-tête).
_NOM_EPISODE = re.compile(r"^episode_\d{6,}$")


def _verifier_nom(e: Dict) -> str:
    """Rend le nom de l'épisode s'il a la forme episode_NNNNNN ; sinon lève
    ValueError, ce qui arrête toute la conversion (voir l'en-tête)."""
    nom = e.get("episode") if isinstance(e, dict) else None
    if not isinstance(nom, str) or not _NOM_EPISODE.match(nom):
        raise ValueError(f"nom d'épisode invalide {str(nom)[:80]!r} dans episodes.jsonl "
                         "(attendu episode_NNNNNN) — index corrompu ou falsifié : "
                         "conversion refusée")
    return nom


def _incoherence(e: Dict, ref: Dict) -> Optional[str]:
    """Raison du rejet, ou None. L'en-tête de l'épisode (cadence, noms des
    composantes, taille d'image) doit être celui du premier : un seul
    format par jeu. La consigne doit être un texte non vide."""
    for cle in ("fps", "noms_etat", "noms_action", "hauteur", "largeur"):
        if e.get(cle) != ref.get(cle):
            return f"{cle} = {str(e.get(cle))[:60]} ≠ {str(ref.get(cle))[:60]} (premier épisode)"
    if not isinstance(e.get("instruction"), str) or not e["instruction"].strip():
        return "consigne absente ou non textuelle"
    return None


def _verifier_tableaux(etat, action, temps, images, n_etat: int, n_action: int,
                       h: int, w: int) -> Optional[str]:
    """Vérifie formes, types (nombres réels uniquement) et finitude (ni
    NaN, ni infini). Rend la raison du rejet, ou None si tout va bien."""
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
    """Parties contiguës du vecteur, d'après les noms : left_* (bras
    gauche), right_* (bras droit), base_*. Un nom qui ne rentre dans aucune
    partie va dans « autres ». Une partie qui n'est pas d'un seul tenant
    lève ValueError : meta/modality.json ne la décrit que par un début et
    une fin."""
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
    (les n_base dernières composantes de l'action : vx, vy, wz). Une
    composante qui dépasse `seuil` en valeur absolue compte comme un
    mouvement. Rend 0 si la base n'a jamais bougé."""
    bouge = np.flatnonzero(np.abs(action[:, -n_base:]).max(axis=1) > seuil)
    return int(bouge[-1]) + 1 if len(bouge) else 0


def debut_geste(action: np.ndarray, i0: int, n_base: int = 3, seuil: float = 0.02) -> int:
    """Après l'arrêt de la base (image i0), la première image où la
    consigne des bras s'écarte de plus de `seuil` (début du serrage), moins
    une image. Pourquoi : constaté sur la machine GPU en mode prise seule
    (bras restés figés 30 s autour du carton), les démonstrations
    commençaient par 0,6 à 3 s d'immobilité (stabilisation de la base).
    Pour la même image et le même état, le modèle avait surtout appris à ne
    rien faire."""
    bras = action[:, :-n_base]
    if i0 >= len(bras):
        return i0
    bouge = np.flatnonzero(np.abs(bras[i0:] - bras[i0]).max(axis=1) > seuil)
    return i0 + max(0, int(bouge[0]) - 1) if len(bouge) else i0


def convertir(dossier: str, sortie: str, robot: str = "unitree_g1_axis_os",
              prise_seule: bool = False, images_min: int = 10) -> Dict:
    """Convertit les épisodes réussis de `dossier` en jeu LeRobot v2 dans
    `sortie`, effacé s'il existe déjà (voir l'en-tête du module). Rend le
    bilan affiché par la commande.

    robot        : nom écrit dans meta/info.json (robot_type).
    prise_seule  : ne garder que la prise (option --prise-seule).
    images_min   : un épisode qui garde moins d'images est ignoré (compté
                   dans « trop_courts », affiché en mode prise seule).

    Lève SystemExit s'il n'y a aucun épisode réussi, ou aucun valide, et
    ValueError si un nom d'épisode est invalide (avant tout effacement).
    """
    import pandas as pd

    eps = [e for e in lire_episodes(dossier) if e.get("succes") is True]
    if not eps:
        raise SystemExit("aucun épisode réussi à convertir")
    # Noms vérifiés avant d'effacer la sortie : un index falsifié n'efface
    # rien.
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
    k = -1                                    # dernier épisode converti (-1 : aucun)
    rejetes: List[str] = []

    def rejeter(nom: str, raison: str) -> None:
        # Un épisode n'est jamais écarté en silence : noté et affiché.
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
                # Un .npy renommé en .npz se charge en simple tableau, pas
                # en archive : refusé.
                raise ValueError(f"pas une archive npz ({type(d).__name__})")
            with d:
                etat, action, temps = d["etat"], d["action"], d["temps"]
                images_npz = d["images"] if "images" in d.files else None
        except Exception as exc:              # noqa: BLE001 — écarté, jamais chargé
            # Fichier absent, clé manquante, tableau d'objets (refusé par
            # allow_pickle=False), archive tronquée ou corrompue (BadZipFile,
            # EOFError, zlib…) : l'épisode est écarté. On attrape toute
            # exception, et non une liste choisie : une revue du code a
            # trouvé qu'un npz coupé par une copie interrompue (docker cp)
            # levait BadZipFile, qui n'était pas attrapée, et faisait échouer
            # toute la conversion après l'effacement de la sortie existante.
            rejeter(nom, f"npz illisible ou refusé ({type(exc).__name__}: {str(exc)[:120]})")
            continue
        raison = _verifier_tableaux(etat, action, temps, images_npz, len(noms_etat),
                                    len(noms_action), h, w)
        if raison:
            rejeter(nom, raison)
            continue
        # Première image gardée : le début du geste en mode prise seule
        # (voir debut_prise et debut_geste), sinon tout l'épisode.
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
        # Une ligne par image, avec les colonnes attendues par LeRobot v2 et
        # GR00T. La récompense vaut 1 à la dernière image seulement : seuls
        # des épisodes réussis arrivent ici.
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
        # Vidéo : copiée telle quelle si l'épisode est gardé entier ; sinon
        # réécrite à partir de l'image i0, depuis les images du npz ou en
        # relisant la vidéo d'origine.
        video = dst / "videos" / "chunk-000" / CLE_VIDEO / f"episode_{k:06d}.mp4"
        if i0 == 0 and e.get("video") and (src / f"{nom}.mp4").exists():
            shutil.copyfile(src / f"{nom}.mp4", video)
        else:
            import imageio.v2 as imageio
            if images_npz is not None:
                images = images_npz
            else:                             # relire la vidéo d'origine, pour la découper
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
        # Clé lue par la configuration de modalités de GR00T
        # (config_gr00t_g1.py) : "annotation.human.task_description"
        # -> colonne task_index.
        "annotation": {"human.task_description": {"original_key": "task_index"}},
    }
    (dst / "meta" / "modality.json").write_text(json.dumps(modalite, indent=2))
    # meta/info.json : description du jeu au format LeRobot v2 (chemins,
    # cadence, type et forme de chaque colonne).
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
    # Mauvais nombre d'arguments : afficher le mode d'emploi (l'en-tête).
    args = [a for a in sys.argv[1:] if a != "--prise-seule"]
    if len(args) != 2:
        raise SystemExit(__doc__)
    print(convertir(args[0], args[1], prise_seule="--prise-seule" in sys.argv))
