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
check("… exemption levée", g.levees >= 1)
r_av = bras["left_shoulder_roll_joint"]
m.reinitialiser(large=True)
sauts = []
for _ in range(60):
    prec = dict(bras)
    t[0] += 0.02
    m.tick()
    sauts.append(max(abs(bras[j] - prec[j]) for j in bras))
check(f"Bras rouverts PROGRESSIVEMENT (de {r_av:+.2f} rad, plus grand pas "
      f"{max(sauts):.3f} rad) — jamais le saut de 0.72 rad que le garde-fou "
      f"refusait sur GPU", max(sauts) < 0.1
      and abs(bras["left_shoulder_roll_joint"] - Postures().roulis_presaisie) < 1e-9)
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

section("6 — Arrêt de sécurité : les bras se FIGENT, puis reprennent")
arret = [False]
t6 = [0.0]
bras6 = {}
mains6 = lambda: (np.array([0.29, 0.165, 0.90]), np.array([0.29, -0.165, 0.90]))
m6 = ManipulationService(bras6, mains6, lambda p: np.asarray(p, dtype=float),
                         lambda: True, horloge=lambda: t6[0],
                         arret_securite=lambda: arret[0])
m6.start()
m6.prendre(CARTON)
avancer(m6, t6, 0.2)
check("Serrage commencé", m6.etat is Etat.SERRAGE)
fige = dict(bras6)
arret[0] = True
avancer(m6, t6, 5.0)                    # 5 s d'arrêt : bien plus que le geste
check("Pendant l'arrêt : AUCUNE articulation n'a bougé", bras6 == fige)
check("... et l'état n'a pas avancé (toujours en serrage)", m6.etat is Etat.SERRAGE)
check("Gels comptés", m6.stats["gels_securite"] > 0)
arret[0] = False
m6.tick(); t6[0] += 0.02; m6.tick()
saut = max(abs(bras6[j] - fige[j]) for j in fige)
check(f"Reprise sans à-coup (écart au premier pas {saut:.3f} rad)", saut < 0.05)
avancer(m6, t6, 5.0)
check("Le geste se termine normalement après la reprise", m6.etat is Etat.TENU)
arret[0] = True
tenu = dict(bras6)
avancer(m6, t6, 1.0)
check("Objet tenu pendant un arrêt : la prise est gardée (figer n'est pas lâcher)",
      m6.etat is Etat.TENU and bras6 == tenu)

from axis_os.services.locomotion.proximite import GardienProximite
g6 = GardienProximite(lambda: None, lambda: (0.0, 0.0, 0.0), horloge=lambda: 0.0)
g6.evaluer(0.0)                          # lidar muet -> arrêt
m7 = ManipulationService({}, mains6, lambda p: np.asarray(p, dtype=float),
                         lambda: True, gardien=g6, horloge=lambda: t6[0])
check("Branché sur un vrai gardien : l'arrêt est lu sur lui par défaut",
      m7._arret_securite is not None and m7._arret_securite() is True)

section("7 — Dépose SUR UN PLATEAU : descendre, effleurer, ouvrir")


def monter_plateau(arret=lambda: False):
    t = [0.0]
    bras = {}

    def mains():
        # Proprioception simulée : les mains descendent quand le tangage
        # augmente (~0.3 m/rad, cohérent avec « levé -0.2 rad = +6 cm »).
        tg = bras.get("left_shoulder_pitch_joint", -0.30)
        z = 0.90 - 0.30 * (tg + 0.30)
        return np.array([0.29, 0.165, z]), np.array([0.29, -0.165, z])

    m = ManipulationService(bras, mains, lambda p: np.asarray(p, dtype=float),
                            lambda: True, horloge=lambda: t[0],
                            arret_securite=arret)
    m.start()
    m.prendre(CARTON)
    avancer(m, t, 2.0)
    return m, bras, t, mains


m, bras, t, mains = monter_plateau()
check("Carton tenu, levé", m.etat is Etat.TENU)
fond_leve = mains()[0][2] - CARTON.hauteur / 2
res = m.poser(hauteur_surface=0.75)
check("Dépose sur un plateau à 0.75 m acceptée", res.ok and m.etat is Etat.DESCENTE)
fonds = []
while m.etat is Etat.DESCENTE:
    t[0] += 0.02
    m.tick()
    fonds.append(mains()[0][2] - CARTON.hauteur / 2)
fond_ouv = fonds[-1]
check(f"Ouverture quand le fond est à {100 * (fond_ouv - 0.75):.1f} cm du plateau "
      f"(il partait de {100 * (fond_leve - 0.75):.0f} cm)",
      0.0 <= fond_ouv - 0.75 <= 0.015)
check("Les mains n'ont PAS bougé verticalement pendant l'ouverture",
      m.etat is Etat.OUVERTURE)
z_ouv = mains()[0][2]
while m.etat is Etat.OUVERTURE:
    t[0] += 0.02
    m.tick()
    if m.etat is Etat.OUVERTURE:
        assert abs(mains()[0][2] - z_ouv) < 1e-9
check("Posé : bras ouverts, puis retour en posture de port",
      m.etat is Etat.LIBRE and m.stats["deposes_plateau"] == 1
      and bras["left_shoulder_pitch_joint"] == Postures().tangage_port)

m, bras, t, mains = monter_plateau()
res = m.poser(hauteur_surface=1.10)
check(f"Plateau plus haut que le carton : refus motivé (« {res.reason[:30]}… »), "
      "carton toujours tenu", not res.ok and m.etat is Etat.TENU)

m, bras, t, mains = monter_plateau()
m.poser(hauteur_surface=0.40)
avancer(m, t, 5.0)
check("Plateau hors d'atteinte vers le bas : on NE lâche PAS de haut — "
      "remonté et toujours TENU, raison donnée",
      m.etat is Etat.TENU and "impossible" in m.raison
      and m.stats["deposes_impossibles"] == 1)

arret = [False]
m, bras, t, mains = monter_plateau(lambda: arret[0])
m.poser(hauteur_surface=0.75)
avancer(m, t, 0.1)
arret[0] = True
fige = dict(bras)
avancer(m, t, 3.0)
check("Arrêt de sécurité pendant la descente : bras figés",
      bras == fige and m.etat is Etat.DESCENTE)
arret[0] = False
avancer(m, t, 3.0)
check("... puis la dépose se termine", m.etat is Etat.LIBRE
      and m.stats["deposes_plateau"] == 1)

section("8 — Pré-saisie : ouvrir en grand AVANT d'entourer l'objet")
m, bras, t = monter()

lim = {d.joint_id: d.limits for d in build_g1_joint_definitions("29dof")}
p8 = Postures()
po = posture_bras(p8.roulis_presaisie, p8.tangage_port, p8.coude)
check("Posture de pré-saisie dans les butées de l'URDF 29 DOF",
      all(lim[j].pos_min <= v <= lim[j].pos_max for j, v in po.items()))
check("Plus ouverte que la posture de port (les mains s'écartent)",
      p8.roulis_presaisie > p8.roulis_ouvert)
check("presaisir() accepté depuis LIBRE", m.presaisir().ok and m.etat is Etat.PRESAISIE)
avancer(m, t, 0.3)
mi = bras["left_shoulder_roll_joint"]
check("Ouverture PROGRESSIVE, pas un à-coup",
      p8.roulis_ouvert < mi < p8.roulis_presaisie)
avancer(m, t, 0.5)
check("Puis PRÊT, bras grands ouverts", m.etat is Etat.PRET
      and abs(bras["left_shoulder_roll_joint"] - p8.roulis_presaisie) < 1e-9)
check("prendre() accepté depuis PRÊT", m.prendre(CARTON).ok and m.etat is Etat.SERRAGE)
t[0] += 0.02
m.tick()
check("Le serrage part de l'ouverture LARGE (pas de saut vers la posture de port)",
      abs(bras["left_shoulder_roll_joint"] - p8.roulis_presaisie) < 0.05)
avancer(m, t, 2.0)
check("… et la prise se termine normalement", m.etat is Etat.TENU)
m2, bras2, t2 = monter()
m2.presaisir()
avancer(m2, t2, 1.0)
m2.reinitialiser()
avancer(m2, t2, 1.0)
check("Pré-saisie abandonnée (accostage raté) : retour en posture de port",
      m2.etat is Etat.LIBRE and bras2["left_shoulder_roll_joint"] == p8.roulis_ouvert)
m3, bras3, t3 = monter()
m3.reinitialiser(large=True)
avancer(m3, t3, 1.0)
check("Après une prise ratée : ouverture GRANDE pour lâcher un objet coincé",
      bras3["left_shoulder_roll_joint"] == p8.roulis_presaisie and m3.etat is Etat.LIBRE)

section("9 — Franchir : lever l'objet AU-DESSUS du plateau avant d'avancer")
m, bras, t, mains = monter_plateau()
fond = lambda: mains()[0][2] - CARTON.hauteur / 2
check(f"Porté : fond à {fond():.2f} m — trop bas pour passer au-dessus d'un "
      f"plateau à 0.85 m avec 10 cm de marge", not m.degage_au_dessus(0.85))
check("franchir() accepté", m.franchir(0.85).ok and m.etat is Etat.MONTEE)
avancer(m, t, 3.0)
check(f"Monté jusqu'à dégager le plateau (fond à {fond():.2f} m), toujours TENU",
      m.etat is Etat.TENU and fond() >= 0.85 + Postures().marge_franchissement - 0.01)
check("Sans monter plus que nécessaire (fond < plateau + 15 cm)", fond() < 0.85 + 0.15)
m.poser(hauteur_surface=0.85)
avancer(m, t, 5.0)
check("Puis dépose sur ce plateau depuis la hauteur atteinte, jusqu'au bout",
      m.etat is Etat.LIBRE and m.stats["deposes_plateau"] == 1)
m, bras, t, mains = monter_plateau()
r = m.franchir(0.60)
check("Déjà au-dessus : rien à faire, pas de mouvement", r.ok and m.etat is Etat.TENU)
m.franchir(1.40)
avancer(m, t, 5.0)
check("Plateau trop haut : on s'arrête à la limite des épaules, carton TENU, "
      "raison donnée", m.etat is Etat.TENU and "trop haut" in m.raison
      and bras["left_shoulder_pitch_joint"] >= Postures().tangage_haut - 1e-9)

# Constaté sur GPU : bras CHARGÉ en retard sur la consigne. Échec déclaré à
# l'instant où la consigne touchait la limite, mains encore en montée.
t_l = [0.0]
bras_l = {}
z_l = [0.96]


def mains_lentes():
    return np.array([0.29, 0.165, z_l[0]]), np.array([0.29, -0.165, z_l[0]])


def tick_lent(m_):
    # Mains qui suivent la consigne avec une constante de temps de 0.8 s,
    # et plafonnent 7 cm sous le modèle (bras chargé qui s'affaisse).
    tg = bras_l.get("left_shoulder_pitch_joint", -0.50)
    cible = 0.90 - 0.30 * (tg + 0.30) - 0.07
    z_l[0] += (cible - z_l[0]) * 0.02 / 0.8
    t_l[0] += 0.02
    m_.tick()


ml = ManipulationService(bras_l, mains_lentes, lambda p: np.asarray(p, dtype=float),
                         lambda: True, horloge=lambda: t_l[0])
ml.start()
ml.prendre(CARTON)
for _ in range(150):
    tick_lent(ml)
ml.franchir(0.80)
for _ in range(300):
    tick_lent(ml)
    if ml.etat is not Etat.MONTEE:
        break
fond_l = z_l[0] - CARTON.hauteur / 2
check(f"Bras chargé en retard : on attend qu'il rejoigne la limite avant de "
      f"conclure (fond {fond_l:.2f} m), et la marge réduite suffit",
      ml.etat is Etat.TENU and ml.peut_franchir(0.80) and not ml.raison)
check("… sans prétendre avoir la marge pleine", not ml.degage_au_dessus(0.80))
check("Le franchissement ne vaut QUE pour ce plateau", not ml.peut_franchir(0.85))
ml.poser(hauteur_surface=0.80)
check("… et il est oublié dès qu'on pose", not ml.peut_franchir(0.80))
check("Plateau vraiment trop haut : la raison donne la hauteur atteinte",
      "fond du carton à" in m.raison)
print("      raison :", m.raison)

lim = {d.joint_id: d.limits for d in build_g1_joint_definitions("29dof")}
po = posture_bras(Postures().roulis_serre, Postures().tangage_haut, Postures().coude)
check("Posture la plus haute dans les butées de l'URDF 29 DOF",
      all(lim[j].pos_min <= v <= lim[j].pos_max for j, v in po.items()))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
