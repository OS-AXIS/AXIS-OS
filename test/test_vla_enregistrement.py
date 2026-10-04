"""
Collecte de démonstrations VLA : enregistreur d'épisodes et conversion au
format LeRobot v2 de GR00T — sans GPU.
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


NOMS_ETAT = [f"left_j{i}" for i in range(7)] + [f"right_j{i}" for i in range(7)]
NOMS_ACTION = NOMS_ETAT + ["base_vx", "base_vy", "base_wz"]


def image(k):
    im = np.zeros((120, 160, 3), dtype=np.uint8)
    im[:, :, 0] = (k * 20) % 255
    return im


def episode(e, t0, n_pas, succes, dt=0.02):
    e.debut("pick up the red box", t0, meta={"couleur": "rouge"})
    gardees = 0
    for k in range(n_pas):
        t = t0 + k * dt
        gardees += e.pas(t, image(k), np.full(14, k, float), np.full(17, -k, float))
    e.fin(succes, t0 + n_pas * dt)
    return gardees


d = Path(tempfile.mkdtemp())
section("1 — Enregistrement : cadence, échantillons complets, succès/échec")
e = EnregistreurEpisodes(str(d / "brut"), fps=10, noms_etat=NOMS_ETAT, noms_action=NOMS_ACTION)
n = episode(e, 0.0, 100, True)                 # 2 s à 50 Hz -> 20 images à 10 Hz
check(f"10 images par seconde simulée, pas 50 ({n} gardées sur 100 pas)", n == 20)
check("Un pas sans image est ignoré (jamais d'échantillon sans ce que voit le robot)",
      (e.debut("x", 10.0), e.pas(10.0, None, [0] * 14, [0] * 17), e.fin(False, 10.1))[1] is False)
episode(e, 20.0, 50, False)
eps = lire_episodes(str(d / "brut"))
check("Épisodes réussis ET ratés notés dans episodes.jsonl (le tri se fait à la conversion)",
      [x["succes"] for x in eps] == [True, False])
z = np.load(d / "brut" / f"{eps[0]['episode']}.npz")
check("État (T, 14), action (T, 17), temps (T,) alignés",
      z["etat"].shape == (20, 14) and z["action"].shape == (20, 17) and z["temps"].shape == (20,))
check("Vidéo MP4 écrite (ou images gardées dans le npz)",
      (d / "brut" / f"{eps[0]['episode']}.mp4").exists() or "images" in z.files)
e2 = EnregistreurEpisodes(str(d / "brut"), fps=10)
check("Reprise dans le même dossier : les numéros continuent, rien n'est écrasé",
      e2._index == 2)
e3 = EnregistreurEpisodes(str(d / "long"), fps=10, duree_max_s=1.0)
episode(e3, 0.0, 200, True)
check("Épisode trop long : clos en ÉCHEC (une tentative bloquée ne remplit pas le disque)",
      lire_episodes(str(d / "long"))[0]["succes"] is False)

section("2 — Conversion LeRobot v2 (format d'entraînement de GR00T)")
try:
    import pandas  # noqa: F401
    import pyarrow  # noqa: F401
    ok_dep = True
except ImportError:
    ok_dep = False
if not ok_dep:
    print("  (pandas/pyarrow absents : conversion non testée ici)")
else:
    from axis_os.vla.vers_lerobot import convertir
    r = convertir(str(d / "brut"), str(d / "lerobot"))
    check(f"Seuls les épisodes RÉUSSIS sont gardés ({r})",
          r["episodes"] == 1 and r["ignores_rates"] == 1)
    import pandas as pd
    df = pd.read_parquet(d / "lerobot" / "data" / "chunk-000" / "episode_000000.parquet")
    check("Colonnes LeRobot présentes",
          {"observation.state", "action", "timestamp", "episode_index", "index",
           "task_index", "annotation.human.action.task_description"} <= set(df.columns))
    check("20 lignes, état de 14 et action de 17",
          len(df) == 20 and len(df["observation.state"][0]) == 14 and len(df["action"][0]) == 17)
    mod = json.loads((d / "lerobot" / "meta" / "modality.json").read_text())
    check(f"modality.json : bras gauche 0-7, bras droit 7-14, base 14-17 ({mod['action']})",
          mod["state"] == {"bras_gauche": {"start": 0, "end": 7},
                           "bras_droit": {"start": 7, "end": 14}}
          and mod["action"]["base"] == {"start": 14, "end": 17})
    info = json.loads((d / "lerobot" / "meta" / "info.json").read_text())
    check("info.json : 1 épisode, 20 images, 10 i/s",
          info["total_episodes"] == 1 and info["total_frames"] == 20 and info["fps"] == 10)
    check("Vidéo de l'épisode au bon endroit",
          (d / "lerobot" / "videos" / "chunk-000" / "observation.images.ego_view"
           / "episode_000000.mp4").exists())
    tache = json.loads((d / "lerobot" / "meta" / "tasks.jsonl").read_text().splitlines()[0])
    check("La consigne en langage naturel est la tâche", tache["task"] == "pick up the red box")

section("3 — Collecte : boucle placer / chercher / accoster / prendre / reposer")
from axis_os.vla.collecte import CollecteurPrises, consigne  # noqa: E402


class FacadeFactice:
    """Trajets de 3 s, prise de 2 s, une prise sur trois ratée."""
    def __init__(self, t):
        self.t, self.fin_nav, self.fin_manip, self.manip = t, None, None, "LIBRE"
        self.n_prises, self.tenu, self.appels = 0, False, []

    def chercher(self, couleur):
        from axis_os.apps.tri_colis.logique import Colis
        return [Colis(couleur, (4.0, 2.1, 0.9), 0.5)]

    def accoster(self, colis):
        self.appels.append("accoster"); self.fin_nav = self.t[0] + 3.0
        return True, ""

    def aller_au_poste(self, poste):
        self.appels.append("retour"); self.fin_nav = self.t[0] + 3.0
        return True, ""

    def navigation(self):
        return "ARRIVE" if self.fin_nav is not None and self.t[0] >= self.fin_nav else "EN_ROUTE"

    def prendre(self, colis):
        self.n_prises += 1
        self.tenu = self.n_prises % 3 != 0
        self.fin_manip, self.manip = self.t[0] + 2.0, "EN_COURS"
        return True, ""

    def manipulation(self):
        if self.manip == "EN_COURS" and self.t[0] >= self.fin_manip:
            self.manip = "TENU" if self.tenu else "ECHEC"
        if self.manip == "POSE" and self.t[0] >= self.fin_manip:
            self.manip = "LIBRE"
        return self.manip

    def poser(self):
        self.manip, self.fin_manip = "POSE", self.t[0] + 1.0
        return True, ""

    def reinitialiser_manipulation(self):
        self.manip = "LIBRE"


t = [0.0]
ff = FacadeFactice(t)
places, retraits = [], [0]
enr = EnregistreurEpisodes(str(d / "collecte"), fps=10, noms_etat=NOMS_ETAT,
                           noms_action=NOMS_ACTION)
col = CollecteurPrises(ff, enr, placer=lambda rng: (places.append(1) or "rouge", {"x": 4.0}),
                       retirer=lambda: retraits.__setitem__(0, retraits[0] + 1),
                       horloge=lambda: t[0],
                       etat_action=lambda: (np.zeros(14), np.zeros(17)),
                       image=lambda: image(0), n_episodes=6, journal=lambda m: None)
images_hors_episode = 0
while not col.termine and t[0] < 600:
    actif = enr.actif
    n_av = enr.stats["images"]
    col.pas()
    if not actif and not enr.actif and enr.stats["images"] > n_av:
        images_hors_episode += 1
    t[0] += 0.02
check(f"6 épisodes puis FIN ({col.stats})", col.termine and col.stats["episodes"] == 6)
check("Une prise sur trois ratée : 4 réussis, 2 ratés notés comme tels",
      col.stats["reussis"] == 4 and col.stats["rates"] == 2
      and [e["succes"] for e in lire_episodes(str(d / "collecte"))].count(True) == 4)
check("Rien n'est enregistré hors des épisodes", images_hors_episode == 0)
check("Un carton posé par épisode, retiré à chaque retour", len(places) == 6 and retraits[0] >= 6)
check("Consigne en anglais, couleur comprise", consigne("bleu") == "pick up the blue box"
      and lire_episodes(str(d / "collecte"))[0]["instruction"] == "pick up the red box")
ep0 = lire_episodes(str(d / "collecte"))[0]
check(f"Un épisode couvre l'approche ET la prise (~5 s, ici {ep0['duree_s']} s, "
      f"{ep0['images']} images)", 4.5 <= ep0["duree_s"] <= 5.5 and 45 <= ep0["images"] <= 55)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
