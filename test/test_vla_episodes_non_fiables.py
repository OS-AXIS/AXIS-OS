"""
Conversion LeRobot — les épisodes sont des données NON FIABLES (audit du
2026-10-10, M2).

Ils reviennent d'une machine louée ou d'une sauvegarde téléchargée. Vérifié :
  - un nom d'épisode qui sort du dossier (« ../../x ») ARRÊTE la conversion,
    avant d'effacer la sortie existante ;
  - un tableau d'objets (pickle) n'est jamais chargé : épisode écarté ;
  - NaN, formes ou en-têtes incohérents, npz manquant : épisode écarté et
    signalé, les épisodes sains sont convertis.
"""
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.vla.enregistreur import EnregistreurEpisodes, lire_episodes  # noqa: E402

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


try:
    import pandas  # noqa: F401
    import pyarrow  # noqa: F401
    import imageio  # noqa: F401
    # convertir() écrit les vidéos en libx264 : sans le backend ffmpeg
    # d'imageio, il échouerait — ce test se saute alors au lieu de planter.
    import imageio_ffmpeg
    imageio_ffmpeg.get_ffmpeg_exe()
except Exception as exc:                      # noqa: BLE001
    print(f"  (pandas/pyarrow/imageio+ffmpeg absents : conversion non testable ici — {exc})")
    sys.exit(0)

from axis_os.vla.vers_lerobot import convertir  # noqa: E402

NOMS_ETAT = [f"left_j{i}" for i in range(7)] + [f"right_j{i}" for i in range(7)]
NOMS_ACTION = NOMS_ETAT + ["base_vx", "base_vy", "base_wz"]


def jeu(n=6):
    d = Path(tempfile.mkdtemp()) / "brut"
    e = EnregistreurEpisodes(str(d), fps=10, noms_etat=NOMS_ETAT, noms_action=NOMS_ACTION)
    for i in range(n):
        e.debut("pick up the red box", 10.0 * i)
        for k in range(15):
            e.pas(10.0 * i + 0.1 * k, np.zeros((24, 32, 3), np.uint8),
                  np.full(14, 0.01 * k), np.full(17, 0.02 * k))
        e.fin(True, 10.0 * i + 1.5)
    return d


def reecrire(d, nom, **tableaux):
    with np.load(d / f"{nom}.npz", allow_pickle=False) as z:
        donnees = {k: z[k] for k in z.files}
    donnees.update(tableaux)
    np.savez(d / f"{nom}.npz", **donnees)


section("1 — Nom d'épisode qui sort du dossier : conversion REFUSÉE")

d = jeu(2)
lignes = (d / "episodes.jsonl").read_text().splitlines()
l0 = json.loads(lignes[0])
l0["episode"] = "../../../etc/passwd"
(d / "episodes.jsonl").write_text("\n".join([json.dumps(l0)] + lignes[1:]) + "\n")
sortie = d.parent / "lerobot"
sortie.mkdir()
(sortie / "temoin.txt").write_text("à garder")
try:
    convertir(str(d), str(sortie))
    check("Nom « ../../../etc/passwd » : refusé", False)
except ValueError as exc:
    check(f"Nom « ../../../etc/passwd » : refusé ({str(exc)[:70]}…)", "invalide" in str(exc))
check("… AVANT d'effacer la sortie existante", (sortie / "temoin.txt").exists())
for mauvais in ("episode_000001/../../x", "/etc/passwd", "episode_1", 42, None):
    l0["episode"] = mauvais
    (d / "episodes.jsonl").write_text("\n".join([json.dumps(l0)] + lignes[1:]) + "\n")
    try:
        convertir(str(d), str(sortie))
        ok = False
    except ValueError:
        ok = True
    check(f"Nom {mauvais!r} : refusé", ok)

section("2 — Épisodes défectueux écartés, épisodes sains convertis")

d = jeu(6)
eps = [e["episode"] for e in lire_episodes(str(d))]
action_nan = np.full((15, 17), 0.1)
action_nan[3, 2] = np.nan
reecrire(d, eps[1], action=action_nan)                                   # NaN
reecrire(d, eps[2], etat=np.zeros((15, 13)))                             # forme
reecrire(d, eps[3], temps=np.array([object()] * 15, dtype=object))       # pickle
(d / f"{eps[4]}.npz").unlink()                                           # absent
lignes = [json.loads(l) for l in (d / "episodes.jsonl").read_text().splitlines()]
lignes[5]["noms_etat"] = NOMS_ETAT[:13]                                  # en-tête
(d / "episodes.jsonl").write_text("\n".join(json.dumps(l) for l in lignes) + "\n")

r = convertir(str(d), str(d.parent / "lerobot2"))
detail = " | ".join(r.get("detail_ecartes", []))
check(f"1 épisode sain converti, 5 écartés ({r['episodes']} / {r['ecartes_invalides']})",
      r["episodes"] == 1 and r["ecartes_invalides"] == 5)
check("Écarté : NaN dans l'action", f"{eps[1]} : action contient NaN/inf" in detail)
check("Écarté : état de mauvaise forme", f"{eps[2]} : etat de forme (15, 13)" in detail)
check("Écarté : tableau d'objets, JAMAIS chargé (allow_pickle=False)",
      f"{eps[3]} : npz illisible ou refusé" in detail and "pickle" in detail.lower())
check("Écarté : npz absent", f"{eps[4]} : npz illisible" in detail)
check("Écarté : en-tête incohérent avec le premier épisode",
      f"{eps[5]} : noms_etat" in detail)
info = json.loads((d.parent / "lerobot2" / "meta" / "info.json").read_text())
check("Le jeu produit ne compte que l'épisode sain", info["total_episodes"] == 1
      and info["total_frames"] == 15)

section("2b — Archives tronquées ou déguisées : écartées, la conversion continue")

# Trouvé à la revue : BadZipFile, EOFError, TypeError n'étaient pas attrapées.
d = jeu(4)
eps = [e["episode"] for e in lire_episodes(str(d))]
brut = (d / f"{eps[1]}.npz").read_bytes()
(d / f"{eps[1]}.npz").write_bytes(brut[:len(brut) // 2])        # coupé au milieu
(d / f"{eps[2]}.npz").write_bytes(brut[:200])                   # presque vide
np.save(d / "deguise.npy", np.zeros(3))
(d / "deguise.npy").replace(d / f"{eps[3]}.npz")                # .npy renommé
r = convertir(str(d), str(d.parent / "lerobot_casse"))
detail = " | ".join(r.get("detail_ecartes", []))
check(f"1 sain converti, 3 écartés sans interrompre la conversion "
      f"({r['episodes']} / {r['ecartes_invalides']})",
      r["episodes"] == 1 and r["ecartes_invalides"] == 3)
check("npz tronqué au milieu : écarté", f"{eps[1]} : npz illisible" in detail)
check("npz presque vide : écarté", f"{eps[2]} : npz illisible" in detail)
check("npy renommé en npz : écarté", f"{eps[3]} : npz illisible" in detail
      and "pas une archive npz" in detail)

section("3 — Aucun épisode valide : arrêt clair, pas de jeu vide")

d = jeu(1)
a = np.full((15, 17), np.inf)
reecrire(d, lire_episodes(str(d))[0]["episode"], action=a)
try:
    convertir(str(d), str(d.parent / "lerobot3"))
    check("Tous écartés : SystemExit", False)
except SystemExit as exc:
    check(f"Tous écartés : arrêt explicite ({str(exc)[:60]}…)", "aucun épisode valide" in str(exc))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
