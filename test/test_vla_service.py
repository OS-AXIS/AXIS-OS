"""
VLAService : écrit les bras (pose_bras) et pilote la base d'après une politique
GR00T — sans GPU, avec un connecteur et une locomotion factices.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.policy.vla_service import VLAService, EtatVLA  # noqa: E402
from axis_os.store.sdk.ai.gr00t_connector import GR00TAction  # noqa: E402
from axis_os.store.sdk.manifest import Capability  # noqa: E402

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


ARMS = [f"left_j{i}" for i in range(7)] + [f"right_j{i}" for i in range(7)]
DT = 0.02                       # un tick = 20 ms (50 Hz)


class Horloge:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FauxConnecteur:
    arm_joints = ARMS

    def __init__(self):
        self.connecte, self.resets, self.appels = True, 0, []
        self.cible, self.base, self.erreur = 0.5, (0.2, 0.0, -0.1), None

    def is_connected(self):
        return self.connecte

    def reset(self):
        self.resets += 1

    def predict_action(self, image, instruction, state):
        self.appels.append((image, instruction, dict(state)))
        if self.erreur:
            raise RuntimeError(self.erreur)
        return GR00TAction(target_angles={j: self.cible for j in ARMS}, base_velocity=self.base,
                           latency_s=0.05)


class FauxLoco:
    def __init__(self):
        self.vitesses, self.halts = [], 0

    def set_velocity(self, vx, vy, wz):
        self.vitesses.append((vx, vy, wz))

    def halt(self):
        self.halts += 1


def monter(**kw):
    h, c, loco = Horloge(), FauxConnecteur(), FauxLoco()
    bras = {j: 0.0 for j in ARMS}
    mesure = {j: 0.1 for j in ARMS}
    img = np.zeros((4, 4, 3), np.uint8)
    p = dict(connecteur=c, bras=bras, loco=loco, lire_etat_bras=lambda: mesure,
             lire_image=lambda: img, delta_max_par_joint={j: 0.02 for j in ARMS}, horloge=h,
             journal=lambda m: None)
    p.update(kw)
    return VLAService(**p), h, c, loco, bras


def tourner(s, h, n):
    for _ in range(n):
        s.tick()
        h.t += DT


section("Contrat et construction")
s, h, c, loco, bras = monter()
check("service « vla », capacité AI_POLICY", s.service_id == "vla" and s.capability is Capability.AI_POLICY)
try:
    monter(delta_max_par_joint={j: 0.02 for j in ARMS[:5]})
    manque = False
except KeyError:
    manque = True
check("delta_max manquant pour un bras : KeyError", manque)
try:
    monter(bras={j: 0.0 for j in ARMS[:5]})
    manque = False
except KeyError:
    manque = True
check("bras absents du dictionnaire de l'adaptateur : KeyError", manque)
check("sans mission, tick() n'écrit rien", s.tick().ok is False and all(v == 0.0 for v in bras.values())
      and loco.vitesses == [] and c.appels == [])

section("Suivi d'une action fraîche")
r = s.demarrer_mission("pick up the red box")
check("mission démarrée, serveur remis à zéro", r.ok and c.resets == 1 and s.etat is EtatVLA.ACTIF)
tourner(s, h, 1)
check("1er tick : une inférence, avec l'état MESURÉ et la consigne",
      len(c.appels) == 1 and c.appels[0][1] == "pick up the red box"
      and all(v == 0.1 for v in c.appels[0][2].values()))
check("bras : un pas de delta_max × 0,5 = 0,01 rad, pas plus (cible 0,5 loin)",
      all(abs(v - 0.01) < 1e-12 for v in bras.values()))
check("base : vitesse transmise à la locomotion (qui la borne elle-même)", loco.vitesses[-1] == (0.2, 0.0, -0.1))
tourner(s, h, 4)
check("inférence toutes les 0,1 s (5 ticks), pas à chaque tick", len(c.appels) == 1)
tourner(s, h, 1)
check("…puis la suivante", len(c.appels) == 2)
tourner(s, h, 100)
check("cible atteinte sans dépassement", all(abs(v - 0.5) < 1e-12 for v in bras.values()))
check("latence mesurée remontée", s.stats["latence_max_s"] == 0.05)

section("Échéance de fraîcheur")
s, h, c, loco, bras = monter(lire_image=lambda: None)         # plus jamais d'image
s.demarrer_mission("x")
tourner(s, h, 40)
check("sans image : aucune décision, aucune inférence", c.appels == [] and s.stats["sans_image"] > 0)
check("…donc périmé : base à l'arrêt, bras inchangés",
      s.etat is EtatVLA.PERIME and loco.halts > 0 and loco.vitesses == [] and all(v == 0.0 for v in bras.values()))
s, h, c, loco, bras = monter(defauts_max=99)
s.demarrer_mission("x")
tourner(s, h, 10)
c.erreur = "serveur injoignable"
tourner(s, h, 20)                                              # 0,4 s : encore frais (< 0,5 s)
check("erreur d'inférence : la dernière action reste valable tant qu'elle est fraîche",
      s.etat is EtatVLA.ACTIF and s.stats["erreurs"] >= 1)
c.erreur = None
s2, h2, c3, loco2, bras2 = monter(max_age_s=0.15, defauts_max=99)
s2.demarrer_mission("x")
tourner(s2, h2, 3)
c3.erreur = "lent"
tourner(s2, h2, 20)
fige = dict(bras2)
tourner(s2, h2, 10)
check("action périmée : bras TENUS (on ne progresse pas vers une décision périmée)", bras2 == fige)
check("…et base arrêtée en équilibre (halt, pas de coupure)", s2.etat is EtatVLA.PERIME and loco2.halts > 0)
c3.erreur = None
tourner(s2, h2, 10)
check("une action fraîche relance le suivi", s2.etat is EtatVLA.ACTIF and bras2 != fige)

section("Erreurs d'inférence répétées")
s, h, c, loco, bras = monter(defauts_max=3, intervalle_s=0.02)
s.demarrer_mission("x")
c.erreur = "boum"
tourner(s, h, 10)
check("3 erreurs d'affilée → DÉFAUT, base arrêtée", s.etat is EtatVLA.DEFAUT and loco.halts >= 1)
check("en DÉFAUT : plus aucune écriture ni consigne", s.tick().ok is False)
check("santé : non saine", s.health().healthy is False)

section("Arrêt de sécurité : bras figés, pas relâchés")
arret = {"v": False}
s, h, c, loco, bras = monter(arret_securite=lambda: arret["v"])
s.demarrer_mission("x")
tourner(s, h, 6)
fige = dict(bras)
arret["v"] = True
halts_avant, vitesses_avant = loco.halts, len(loco.vitesses)
tourner(s, h, 10)
check("arrêt actif : état GEL, bras inchangés, aucune nouvelle inférence", s.etat is EtatVLA.GEL
      and bras == fige and len(c.appels) == 2 and s.stats["gels"] == 10)
check("pendant le gel : base à l'arrêt (halt à chaque tick), aucune vitesse proposée",
      loco.halts == halts_avant + 10 and len(loco.vitesses) == vitesses_avant)
arret["v"] = False
tourner(s, h, 3)
check("fin de l'arrêt : le geste reprend", s.etat is EtatVLA.ACTIF and bras != fige)

section("Un seul écrivain sur les bras")
manip = {"actif": True}
s, h, c, loco, bras = monter(autre_ecrivain=lambda: manip["actif"])
check("démarrage refusé si la manipulation écrit", s.demarrer_mission("x").ok is False and c.resets == 0)
manip["actif"] = False
check("autorisé quand elle est libre", s.demarrer_mission("x").ok)
tourner(s, h, 3)
manip["actif"] = True
s.tick()
check("conflit en cours de mission → DÉFAUT, base arrêtée", s.etat is EtatVLA.DEFAUT and loco.halts >= 1)
s, h, c, loco, bras = monter()
c.connecte = False
check("connecteur déconnecté : démarrage refusé", s.demarrer_mission("x").ok is False)

section("Fin de mission et action hors des bras")
s, h, c, loco, bras = monter()
s.demarrer_mission("x")
tourner(s, h, 6)
avant, halts = dict(bras), loco.halts
s.terminer_mission()
tourner(s, h, 5)
check("terminer : base à l'arrêt en équilibre, bras tenus, plus rien n'est écrit",
      s.etat is EtatVLA.INACTIF and loco.halts == halts + 1 and bras == avant)
s, h, c, loco, bras = monter()
s.demarrer_mission("x")
class Etrangere(FauxConnecteur):
    def predict_action(self, image, instruction, state):
        return GR00TAction(target_angles={"left_hip_pitch": 0.3}, base_velocity=(0, 0, 0))
s.__dict__["_c"] = Etrangere()
tourner(s, h, 2)
check("action visant une jambe → DÉFAUT, jamais écrite", s.etat is EtatVLA.DEFAUT
      and all(v == 0.0 for v in bras.values()) and "left_hip_pitch" not in bras)

section("Exécution du CHUNK : le geste vient dans les pas suivants, pas dans le premier")


class ConnecteurChunk(FauxConnecteur):
    """Chunk de 16 pas : le pas k vise 0.1 × k (le pas 0 dit « rester là »),
    base à 0.01 × k."""
    def predict_action(self, image, instruction, state):
        self.appels.append((image, instruction, dict(state)))
        ch = {"bras_gauche": np.tile((0.1 * np.arange(16))[:, None], (1, 7)).astype(np.float32),
              "bras_droit": np.tile((0.1 * np.arange(16))[:, None], (1, 7)).astype(np.float32),
              "base": np.tile((0.01 * np.arange(16))[:, None], (1, 3)).astype(np.float32)}
        return GR00TAction(target_angles={j: 0.0 for j in ARMS}, base_velocity=(0.0, 0.0, 0.0),
                           chunk=ch, latency_s=0.05)


c1 = ConnecteurChunk()
s1, h1, _, l1, b1 = monter(connecteur=c1, delta_max_par_joint={j: 1.0 for j in ARMS})
s1.start(); s1.demarrer_mission("pick up the red box")
tourner(s1, h1, 40)                                    # 0.8 s
check(f"pas_executes=1 (ancien comportement) : bras FIGÉS au pas 0 ({b1[ARMS[0]]:.2f})",
      abs(b1[ARMS[0]]) < 1e-9)
c8 = ConnecteurChunk()
s8, h8, _, l8, b8 = monter(connecteur=c8, delta_max_par_joint={j: 1.0 for j in ARMS},
                            intervalle_s=0.8, max_age_s=0.9, pas_executes=8)
s8.start(); s8.demarrer_mission("pick up the red box")
vus = []
for _ in range(40):                                    # 0.8 s : une seule décision
    s8.tick(); vus.append(round(b8[ARMS[0]], 3)); h8.t += DT
check(f"pas_executes=8 : un pas du chunk toutes les 0.1 s, jusqu'au pas 7 ({sorted(set(vus))})",
      sorted(set(vus)) == [round(0.1 * k, 3) for k in range(8)] and len(c8.appels) == 1)
check("Base : vitesses du pas courant du chunk", l8.vitesses[-1] == (0.07, 0.07, 0.07)
      or all(abs(x - 0.07) < 1e-6 for x in l8.vitesses[-1]))
s8.tick()
check("Nouvelle décision à la fin des 8 pas (0.8 s)", len(c8.appels) == 2)
try:
    monter(intervalle_s=0.8, max_age_s=0.5, pas_executes=8)
    trop_court = False
except ValueError:
    trop_court = True
check("max_age_s plus court que les pas exécutés : refusé à la construction", trop_court)
cg = ConnecteurChunk()
sg, hg, _, lg, bg = monter(connecteur=cg, delta_max_par_joint={j: 1.0 for j in ARMS},
                            intervalle_s=0.8, max_age_s=0.9, pas_executes=8,
                            arret_securite=lambda: 0.3 <= hg.t < 0.6)
sg.start(); sg.demarrer_mission("x")
tourner(sg, hg, 32)                                    # gel de 0.3 à 0.6 s, puis reprise
check("Après un arrêt de sécurité : nouvelle décision, pas la suite du vieux chunk",
      len(cg.appels) == 2)


print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
