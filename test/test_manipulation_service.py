"""
Service de manipulation — prendre et poser, et savoir échouer proprement.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.unitree_g1.robot import build_g1_joint_definitions
from axis_os.services.base import IService
from axis_os.services.manipulation.service import (
    Etat, ManipulationService, Objet, Postures, posture_bras,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class GardienFactice:
    def __init__(self, accepte=True):
        self.accepte, self.demandes, self.levees = accepte, [], 0

    def exempter(self, volume, duree_s, maintenant, motif=""):
        self.demandes.append((volume, duree_s, motif))
        return (True, "") if self.accepte else (False, "trop grand")

    def lever_exemption(self):
        self.levees += 1


def monter(verif=True, gardien=None):
    t = [0.0]
    bras = {}
    mains = lambda: (np.array([0.29, 0.165, 0.90]), np.array([0.29, -0.165, 0.90]))
    m = ManipulationService(bras, mains, lambda p: np.asarray(p, dtype=float),
                            lambda: verif, gardien=gardien, horloge=lambda: t[0])
    m.start()
    return m, bras, t


def avancer(m, t, secondes, dt=0.02):
    for _ in range(int(round(secondes / dt))):
        t[0] += dt
        m.tick()


CARTON = Objet((0.29, 0.0, 0.90), largeur=0.24, profondeur=0.20, hauteur=0.20)

section("0 — Contrat et postures")
m, bras, t = monter()
check("ManipulationService est un IService", isinstance(m, IService))
check("Au démarrage : bras en posture de port, ouverts", bras and m.etat is Etat.LIBRE)
check("Le service n'écrit QUE des articulations de bras — jamais une jambe",
      all(("shoulder" in j or "elbow" in j) for j in bras))
lim = {d.joint_id: d.limits for d in build_g1_joint_definitions("29dof")}
p = Postures()
toutes = [posture_bras(r, tg, p.coude) for r in (p.roulis_ouvert, p.roulis_serre)
          for tg in (p.tangage_port, p.tangage_leve)]
dans = all(lim[j].pos_min <= v <= lim[j].pos_max for po in toutes for j, v in po.items())
check("Toutes les postures restent dans les butées de l'URDF 29 DOF", dans)

section("1 — Règle de Kenzo : on ne serre que si l'objet est entre les mains")
check("Carton centré entre les mains : accepté", m.entre_les_mains(CARTON)[0])
decale = Objet((0.29, 0.08, 0.90), 0.24, 0.20, 0.20)
ok, r = m.entre_les_mains(decale)
check(f"Décalé de 8 cm sur le côté : refusé (« {r[:40]}… »)", not ok)
loin = Objet((0.60, 0.0, 0.90), 0.24, 0.20, 0.20)
check("Trop loin devant : refusé", not m.entre_les_mains(loin)[0])
large = Objet((0.29, 0.0, 0.90), 0.34, 0.20, 0.20)
check("Plus large que l'écartement des mains : refusé",
      not m.entre_les_mains(large)[0])
res = m.prendre(decale)
check("prendre() sur un objet mal placé : REFUS motivé, bras inchangés",
      not res.ok and "refusée" in res.reason and m.etat is Etat.LIBRE)

section("2 — Prendre : serrer, soulever, VÉRIFIER")
g = GardienFactice()
m, bras, t = monter(verif=True, gardien=g)
check("prendre() accepté", m.prendre(CARTON).ok and m.etat is Etat.SERRAGE)
check("Exemption demandée au gardien, autour de l'objet et bornée",
      len(g.demandes) == 1 and g.demandes[0][2] == "manipulation : prise")
avancer(m, t, 0.52)
check("Après le serrage : les bras serrent (roulis vers l'intérieur)",
      m.etat is Etat.LEVAGE and bras["left_shoulder_roll_joint"] < 0)
avancer(m, t, 0.82)
check("Après le levage et la vérification : objet TENU", m.etat is Etat.TENU)
check("Exemption LEVÉE dès l'objet soulevé — plus besoin", g.levees >= 1)
check("Bras en position levée", abs(bras["left_shoulder_pitch_joint"] - Postures().tangage_leve) < 1e-9)
check("Une seconde prise pendant qu'il tient : refusée",
      not m.prendre(CARTON).ok)

section("3 — Prise ratée : ÉCHEC propre, jamais partir les mains vides")
g = GardienFactice()
m, bras, t = monter(verif=False, gardien=g)
m.prendre(CARTON)
avancer(m, t, 1.4)
check("L'objet n'a pas suivi : ÉCHEC, avec la raison",
      m.etat is Etat.ECHEC and "NON confirmée" in m.raison)
check("… bras rouverts, exemption levée", bras["left_shoulder_roll_joint"] > 0 and g.levees >= 1)
m.reinitialiser()
check("reinitialiser() : prêt pour un nouvel essai", m.etat is Etat.LIBRE)

section("4 — Poser")
m, bras, t = monter(verif=True)
check("poser() sans rien en main : refusé", not m.poser().ok)
m.prendre(CARTON)
avancer(m, t, 1.4)
check("poser() depuis TENU : accepté", m.poser().ok and m.etat is Etat.POSE)
avancer(m, t, 0.82)
check("Redescendu à hauteur de port, puis OUVERTURE", m.etat is Etat.OUVERTURE)
avancer(m, t, 0.52)
check("Relâché : bras ouverts, rien en main", m.etat is Etat.LIBRE
      and bras["left_shoulder_roll_joint"] > 0 and m.stats["poses"] == 1)

section("5 — Exemption refusée : pas de prise")
m, bras, t = monter(gardien=GardienFactice(accepte=False))
res = m.prendre(CARTON)
check("Le gardien refuse l'exemption : la prise n'a PAS lieu",
      not res.ok and "exemption" in res.reason and m.etat is Etat.LIBRE)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
