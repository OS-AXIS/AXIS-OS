"""
AXIS-OS — Campagne de cartons variés pour la prise à deux mains
================================================================
Pourquoi : la prise a été mise au point sur un seul carton, celui de la
scène T1 (24 × 20 × 20 cm, ~1 kg, toujours sur la même table, à la même
place). Rien ne disait qu'elle tenait ailleurs. Ici, des cartons tirés au
hasard, graine fixée, donc rejouables :

  - tailles, masses, frottements des mains sur leurs faces, place sur la
    table (près du bord ou loin dedans : dans et hors de portée), lacet ;
  - exprès, un carton sur dix trop large pour les mains, un sur dix trop
    lourd pour les plafonds de couple : l'OS doit les refuser proprement.

tirer_cartons() ne dépend que de random.Random(graine).random() : la même
graine donne les mêmes cartons ici, sur le robot simulé, et sur le banc
Isaac (sim/validation/test_isaac_t1_tri_usine.py, --campagne-cartons).

essayer_carton() fait, sur le robot simulé léger (horloge simulée, mains du
G1, 7 articulations par bras comme le banc Isaac), ce que fait le banc T1 :
s'approcher de la table face au carton, à la distance de prise de l'OS
(erreur de navigation comprise), le prendre, le porter, le déposer sur la
table. Il relève, vérité du monde simulé, ce qui ne doit JAMAIS arriver :
carton tombé, écrasé (main commandée enfoncée dans sa face au-delà des
bornes de l'OS), plafond de couple dépassé, consigne refusée par le
garde-fou, prise crue tenue alors que les mains sont vides.

rapport() montre les vraies limites du robot. D'abord, les cartons
impossibles par construction (trop lourds, trop larges, hors de toute
portée même pieds contre la table : impossible_par_construction) à part
des faisables, et la réussite sur les faisables ; puis la réussite par
plage (taille, masse, frottement, lacet, place), et toutes les raisons de
refus. Avant et après le planificateur de prise :
python -m axis_os.testing.cartons [graine] [nombre].
"""
from __future__ import annotations

import contextlib
import io
import math
import os
import random
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# Plages des tirages (m, kg, degrés).
LARGEUR_M = (0.15, 0.35)
PROFONDEUR_M = (0.15, 0.30)
HAUTEUR_M = (0.10, 0.30)
MASSE_KG = (0.2, 2.5)
FROTTEMENT = (0.5, 1.2)              # mains caoutchouc sur le carton (μ)
LACET_DEG = (-45.0, 45.0)
# Place sur la table : de la face avant du carton au bord de la table (retrait),
# et le long du bord. Loin dans la table, il est hors de portée des bras.
RETRAIT_M = (0.0, 0.25)
DECALAGE_M = (-0.20, 0.20)
# Exprès hors des moyens du robot : le 4e carton de chaque dizaine est trop
# large pour les mains, le 8e trop lourd pour les plafonds de couple.
TROP_LARGE_M = (0.45, 0.60)
TROP_LOURD_KG = (3.0, 5.0)
# Approche comme le banc T1 : face à la table, aligné sur le carton, à la
# distance de prise de l'OS ; à l'erreur de navigation près.
ERREUR_APPROCHE_M = 0.02
ERREUR_CAP_DEG = 3.0
# Comme le banc T1 (facade.py) : bras ouverts en grand à la pose d'attente,
# RECUL_INSERTION_M avant la pose de prise, puis insertion à VITESSE_INSERTION.
RECUL_INSERTION_M = 0.28
VITESSE_INSERTION_M_S = 0.15
PLATEAU_M = 0.80                     # hauteur de la table de prise de la scène T1
TABLE = (0.0, 1.5, -1.0, 1.0, PLATEAU_M)   # son bord le long de x = 0 (robot simulé, plateaux)


@dataclass(frozen=True)
class CartonTire:
    """Un carton tiré, et l'erreur d'approche du robot qui va le prendre.
    `expres` : "trop large" ou "trop lourd" (hors des moyens du robot,
    exprès), sinon ""."""
    numero: int
    largeur: float           # m, entre les mains
    profondeur: float        # m
    hauteur: float           # m
    masse_kg: float
    frottement: float
    lacet_deg: float         # autour de la verticale (0 : largeur le long du bord)
    retrait_m: float         # du bord de la table à l'avant du carton
    decalage_m: float        # le long du bord de la table
    erreur_x_m: float        # approche : trop loin (+) ou trop près (-)
    erreur_y_m: float
    erreur_cap_deg: float
    expres: str = ""


def tirer_cartons(n: int, graine: int) -> List[CartonTire]:
    """`n` cartons tirés avec `graine` (voir l'en-tête du module) : onze
    tirages par carton, toujours dans le même ordre."""
    rng = random.Random(graine)
    cartons = []
    for k in range(n):
        expres = "trop large" if k % 10 == 3 else "trop lourd" if k % 10 == 7 else ""
        u = [rng.random() for _ in range(11)]

        def entre(plage, x):
            return plage[0] + (plage[1] - plage[0]) * x
        cartons.append(CartonTire(
            numero=k,
            largeur=round(entre(TROP_LARGE_M if expres == "trop large" else LARGEUR_M, u[0]), 4),
            profondeur=round(entre(PROFONDEUR_M, u[1]), 4),
            hauteur=round(entre(HAUTEUR_M, u[2]), 4),
            masse_kg=round(entre(TROP_LOURD_KG if expres == "trop lourd" else MASSE_KG, u[3]), 3),
            frottement=round(entre(FROTTEMENT, u[4]), 3),
            lacet_deg=round(entre(LACET_DEG, u[5]), 2),
            retrait_m=round(entre(RETRAIT_M, u[6]), 4),
            decalage_m=round(entre(DECALAGE_M, u[7]), 4),
            erreur_x_m=round(entre((-ERREUR_APPROCHE_M, ERREUR_APPROCHE_M), u[8]), 4),
            erreur_y_m=round(entre((-ERREUR_APPROCHE_M, ERREUR_APPROCHE_M), u[9]), 4),
            erreur_cap_deg=round(entre((-ERREUR_CAP_DEG, ERREUR_CAP_DEG), u[10]), 2),
            expres=expres))
    return cartons


def place_sur_table(c: CartonTire) -> Tuple[float, float, float]:
    """Centre du carton (m, repère monde) : bord de la table le long de
    x = 0, table vers +x, plateau à PLATEAU_M ; le carton tourné garde son
    retrait du bord."""
    a = math.radians(c.lacet_deg)
    etendue_x = c.profondeur * abs(math.cos(a)) + c.largeur * abs(math.sin(a))
    return c.retrait_m + etendue_x / 2.0, c.decalage_m, PLATEAU_M + c.hauteur / 2.0


# Gestes où l'OS croit tenir le carton.
_TENU = ("RAMENER", "TENU", "MONTEE", "DESCENTE", "POSE")


def pied_au_dela(x: float, y: float, cap: float) -> float:
    """De combien (m) le pied le plus avancé du robot (bassin en x, y, cap ;
    G1 debout) dépasse le bord de la table (x = 0, table vers +x) : négatif
    s'il reste devant."""
    from axis_os.services.locomotion.equilibre import pieds_debout
    c, s = math.cos(cap), math.sin(cap)
    return max(x + px * c - py * s + rayon for px, py, rayon in pieds_debout())


def impossible_par_construction(c: CartonTire, m) -> str:
    """Pourquoi le carton est impossible par construction, quelle que soit
    la façon de le prendre (avec ou sans planificateur) ; "" s'il est
    faisable. `m` : le ManipulationService du robot, debout, qui ne bouge
    pas (ses contrôles de prise, evaluer_prise, sur toutes les positions du
    planificateur, les deux paires de faces) :
      - « trop lourd » : plus lourd que masse_max_kg, la charge pour laquelle
        la prise est vérifiée (l'OS le repose après la pesée) ;
      - « trop large » : aucune paire de faces ne tient entre les mains
        ouvertes en grand, mais l'une serait atteinte (comme ci-dessous) si le
        carton n'était pas plus large qu'elles ne le prennent ;
      - « trop bas pour une prise en pendule » : moins haut que deux fois la
        visée au-dessus du centre et le bord sous l'arête (prise.hauteur_prise) ;
      - « hors de toute portée » : même ainsi, d'aucune position, pieds contre
        la table au plus (erreur d'approche comprise), les mains n'atteignent
        le milieu des faces d'une paire.
    Chaque carton reçoit la première de ces raisons, dans cet ordre."""
    from dataclasses import replace
    from axis_os.services.manipulation import planificateur as P
    from axis_os.services.manipulation.prise import hauteur_prise
    if c.masse_kg > m.r.masse_max_kg:
        return "trop lourd"
    if hauteur_prise(m.r, c.hauteur) is None:
        return "trop bas pour une prise en pendule"
    bord = P.BordTable((0.0, 0.0), 0.0, PLATEAU_M)
    boite = P.boite_depuis_coins(P.coins_boite(place_sur_table(c), math.radians(c.lacet_deg),
                                               c.profondeur, c.largeur, c.hauteur))
    # Le plus large que prennent les mains ouvertes en grand, jeu compris
    # (le contrôle « largeur » d'evaluer_prise).
    plus_large = m._ouverture(math.inf)[1] - 2.0 * m.r.jeu_ouverture_m
    large, contexte = False, {}
    for paire in P.paires(boite, bord):
        contexte.pop("graine", None)
        objet = m._objet_paire(boite, paire, bord)
        essai = replace(objet, largeur=min(objet.largeur, plus_large - 1e-9))   # passe, à l'arrondi près
        if any("portée" in {g.nom for g in m.evaluer_prise(essai, p_[:3], contexte=contexte)
                            if g.relative >= 0.0} for p_ in P.positions(boite, paire)):
            if objet.largeur <= plus_large:
                return ""                  # tient entre les mains et à portée
            large = True
    return "trop large" if large else "hors de toute portée"


def plan_en_bref(plan) -> Dict:
    """Ce que le planificateur de prise a vu et choisi, pour le rapport."""
    ch = plan.choisie
    return {"candidates": len(plan.candidates), "passent": sum(c_.ok for c_ in plan.candidates),
            "paire": None if ch is None else ch.paire.numero,
            "cap_deg": None if ch is None else round(math.degrees(ch.cap), 1),
            "travers_m": None if ch is None else round(ch.s, 3),
            "score": None if ch is None else round(ch.score, 3),
            "explication": (ch or plan.meilleure).explication() if (ch or plan.meilleure) else "",
            "raison": plan.raison}


def essayer_carton(c: CartonTire, reglages=None, port_s: float = 2.0, bruit: bool = True,
                   pencher: bool = True, planificateur: bool = True,
                   postures: bool = False) -> Dict:
    """Le banc T1 sur le robot simulé léger, pour un carton (voir l'en-tête
    du module) ; rend ce qui s'est passé (dictionnaire, voir rapport()).
    `bruit` : couples estimés avec le bruit, le biais et le gain du banc
    (BruitCouples), graine tirée du numéro du carton. `pencher` : trop loin
    pour les bras debout, le robot se penche (plan_penche, consigne de
    posture de la marche), se redresse le carton tenu pour le porter, et se
    penche de nouveau pour le déposer à sa place ; False : jamais (le
    rapport « avant »). `planificateur` : le robot choisit sa prise
    (ManipulationService.planifier_prise : la paire de faces et la position
    de base, contrôles faits sans bouger), y va, face à cette paire, et prend
    comme aujourd'hui (debout : `pencher` ne sert alors pas) ; False : face à
    la table, à la distance de prise (le rapport « avant »). `postures` (avec
    le planificateur) : si aucune prise ne passe debout, il essaie les jambes
    pliées puis le torse penché (planifier_prise(postures=True)) ; le robot
    prend la posture avant d'ouvrir les bras, se redresse le carton tenu
    pour le porter, la reprend pour le poser à sa place."""
    from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples
    from axis_os.services.manipulation.service import Etat, Objet
    from axis_os.testing.robot_simule import SimulatedObject, SimulatedRobot

    t = [0.0]
    cx, cy, cz = place_sur_table(c)
    lacet = math.radians(c.lacet_deg)
    carton = SimulatedObject("carton", (cx, cy, cz), size=(c.largeur, c.profondeur, c.hauteur),
                             lacet=lacet, masse_kg=c.masse_kg, frottement=c.frottement)
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        r = SimulatedRobot(objects=[carton], horloge=lambda: t[0], contact_main_m=0.11,
                           poignets_29dof=True, plateaux=[TABLE])
    with contextlib.redirect_stdout(sortie):
        r.start(fil=False)                      # la marche répond aux essais de posture
    m = r.manipulation
    if reglages is not None:
        m.r = reglages
    if bruit:
        r.bruit_couples = BruitCouples(len(r._BRAS) + len(r._poignets), graine=1000 + c.numero)
    journal: List[str] = []
    m._journal = journal.append
    # L'OS connaît la boîte (perception) et le bord de la table, à x = 0.
    objet = Objet((cx, cy, cz), c.largeur, c.profondeur, c.hauteur, lacet=lacet,
                  bord_m=cx - c.profondeur / 2.0)
    res = {"carton": asdict(c), "issue": "", "raison": "", "etats": [], "tenu": False,
           "depasse_couple_nm": 0.0, "depasse_vrai_nm": 0.0, "depasse_materiel_nm": -1e9,
           "enfoncement_max_m": 0.0, "depasse_borne_m": -1.0, "fausse_prise": 0, "bruit": bruit,
           "refus_garde_fou": 0, "chutes": [], "tombe": False, "sur_table": True,
           "poids_os_n": None, "poids_vrai_n": round(9.81 * c.masse_kg, 2),
           "marge_vraie_min": None, "marge_os_min": None, "rotation_max_deg": 0.0,
           "perte_ignoree_s": 0.0, "distance_m": None, "temps_s": 0.0, "journal": [],
           # pour l'audit des réglages (audit())
           "rotation_os_ramener_deg": 0.0, "retour_arrete": False, "avance_max_m": 0.0,
           "borne_poids": False, "enfoncement_contact_m": None, "recalage_cm": None,
           # le même avec ou sans planificateur : les faisables, au rapport
           "impossible": impossible_par_construction(c, m)}
    plan = None
    if planificateur:
        from axis_os.services.manipulation.planificateur import BordTable, coins_boite
        bord = BordTable((0.0, 0.0), 0.0, PLATEAU_M)
        plan = m.planifier_prise(coins_boite((cx, cy, cz), lacet, c.profondeur, c.largeur,
                                             c.hauteur), bord, postures=postures)
        res["plan"] = plan_en_bref(plan)
        if plan.choisie is None:
            res.update(issue="refusée", raison=f"prise refusée — {plan.raison}",
                       tangage_deg=0.0, abaissement_cm=0.0)
            return res
        ch = plan.choisie
        objet = m._objet_paire(plan.boite, ch.paire, bord)
        d, cap = ch.d, ch.cap
        tangage, abaissement = ch.posture
    elif pencher:
        (d, tangage, pourquoi), abaissement = m.plan_penche(objet), 0.0
    else:
        (d, pourquoi), tangage, abaissement = m.distance_de_prise(objet), 0.0, 0.0
    res["tangage_deg"] = round(math.degrees(tangage), 1)
    res["abaissement_cm"] = round(100 * abaissement, 1)
    if d is None:
        res.update(issue="refusée", raison=f"approche impossible — {pourquoi}",
                   portee=pourquoi_hors_de_portee(m, objet, pourquoi))
        return res
    res["distance_m"] = round(d, 3)
    if plan is None:
        # Pose d'attente, mains encore devant le carton (cap de prise : face à la table).
        x_prise = cx - d - c.erreur_x_m
        r.x, r.y, r.heading = x_prise - RECUL_INSERTION_M, cy + c.erreur_y_m, \
            math.radians(c.erreur_cap_deg)
    else:
        # Pose choisie, l'erreur d'arrivée de la marche dans le repère du robot ;
        # pose d'attente RECUL_INSERTION_M en arrière, le long du cap.
        u = (math.cos(cap), math.sin(cap))
        cible = (ch.x - c.erreur_x_m * u[0] - c.erreur_y_m * u[1],
                 ch.y - c.erreur_x_m * u[1] + c.erreur_y_m * u[0])
        r.x, r.y = cible[0] - RECUL_INSERTION_M * u[0], cible[1] - RECUL_INSERTION_M * u[1]
        r.heading = cap + math.radians(c.erreur_cap_deg)
    limites = {j: m._limite_effort(j) for j in r._BRAS + tuple(r._poignets)}
    materiel = {j: m.mesures.couple_max[j] for j in limites}
    poids = 9.81 * c.masse_kg

    def pas(n=1):
        with contextlib.redirect_stdout(sortie):
            for _ in range(n):
                t[0] += r._dt
                r.pas()
                e = m.etat.name
                if e == "RAMENER" and res["etats"][-1:] != ["RAMENER"] and r.held is None:
                    res["fausse_prise"] += 1          # prise confirmée, mains vides
                if not res["etats"] or res["etats"][-1] != e:
                    res["etats"].append(e)
                for j, lim in limites.items():
                    mesure = r._couples_poignets.get(j)      # ce que lit l'OS (bruité)
                    if mesure is None:
                        mesure = r.raw_hal.get_actuator(j)._torque
                    vrai = abs(r.couples_vrais.get(j, 0.0))
                    res["depasse_couple_nm"] = max(res["depasse_couple_nm"], abs(mesure) - lim)
                    res["depasse_vrai_nm"] = max(res["depasse_vrai_nm"], vrai - lim)
                    res["depasse_materiel_nm"] = max(res["depasse_materiel_nm"],
                                                     vrai - materiel[j])
                res["enfoncement_max_m"] = max(res["enfoncement_max_m"],
                                               *r.enfoncement_commande.values())
                if r.held is not None:
                    res["tenu"] = True
                    res["rotation_max_deg"] = max(res["rotation_max_deg"],
                                                  abs(r.rotation_objet_deg))
                if e == "RAMENER" and m.rotation_carton is not None:
                    res["rotation_os_ramener_deg"] = max(res["rotation_os_ramener_deg"],
                                                         *(abs(v) for v in m.rotation_carton))
                if e == "SERRAGE" and res["enfoncement_contact_m"] is None:
                    res["enfoncement_contact_m"] = max(r.enfoncement_commande.values())
                if e in ("LEVAGE", "RAMENER", "TENU"):
                    res["avance_max_m"] = max(res["avance_max_m"], *m._delta.values())
                    bornes = getattr(m, "_delta_max_poids", None) or {}
                    res["borne_poids"] = res["borne_poids"] or any(
                        m._delta[k] >= bornes[k] - 1e-4 for k in bornes)
                # La consigne de serrage dans ses bornes : au toucher (_delta_max),
                # puis d'après le poids (_delta_max_poids, fixée au levage).
                bornes = (m._delta_max if e == "SERRAGE" else getattr(m, "_delta_max_poids", None)
                          if e in ("LEVAGE", "RAMENER", "TENU", "MONTEE", "DESCENTE") else None)
                if bornes:
                    res["depasse_borne_m"] = max(res["depasse_borne_m"],
                                                 *(m._delta[k] - bornes[k] for k in bornes))
                if e in _TENU and r.held is None and not (m.a_reposer or m.penche):
                    perdu[0] = perdu[0] or t[0]
                    res["perte_ignoree_s"] = max(res["perte_ignoree_s"], t[0] - perdu[0])
                elif r.held is not None:
                    perdu[0] = 0.0
                if e == "TENU" and r.held is not None and not r._sur_plateau:
                    vraie = r._portance(r.held) / poids
                    res["marge_vraie_min"] = vraie if res["marge_vraie_min"] is None \
                        else min(res["marge_vraie_min"], vraie)
                    if m.marge_frottement is not None:
                        res["marge_os_min"] = m.marge_frottement if res["marge_os_min"] is None \
                            else min(res["marge_os_min"], m.marge_frottement)

    perdu = [0.0]                       # instant où le carton a quitté les mains tenues

    def jusqua(fini, duree):
        t0 = t[0]
        while not fini() and t[0] - t0 < duree:
            pas()

    pas(20)
    if tangage or abaissement:                  # penché, jambes pliées, avant d'ouvrir les bras
        penche = r.locomotion.posture(tangage, abaissement)
        if not penche.ok:
            res.update(issue="refusée", raison=str(penche.reason))
            return res
        jusqua(lambda: r.locomotion.posture_atteinte, 10.0)
    pre = m.presaisir(objet)                    # ouverts juste assez pour ce carton
    if not pre.ok:
        res.update(issue="refusée", raison=str(pre.reason))
        if r.locomotion.penche:                 # redressé, mains vides
            r.locomotion.posture(0.0)
            jusqua(lambda: r.locomotion.posture_atteinte, 10.0)
        res["droit_a_la_fin"] = not r.locomotion.penche
        return res
    jusqua(lambda: m.etat is Etat.PRET, 5.0)
    if plan is None:
        while r.x < x_prise - 1e-9:             # insertion, bras ouverts
            r.x = min(x_prise, r.x + VITESSE_INSERTION_M_S * r._dt)
            pas()
    else:
        fait = 0.0
        while fait < RECUL_INSERTION_M - 1e-9:  # insertion, bras ouverts, le long du cap
            dl = min(VITESSE_INSERTION_M_S * r._dt, RECUL_INSERTION_M - fait)
            fait += dl
            r.x, r.y = r.x + dl * u[0], r.y + dl * u[1]
            pas()
    res["pied_au_dela_m"] = round(pied_au_dela(r.x, r.y, r.heading), 4)
    pas(10)
    lance = m.prendre(objet)
    if not lance.ok:
        res.update(issue="refusée", raison=str(lance.reason),
                   portee=pourquoi_hors_de_portee(m, objet, str(lance.reason)))
    else:
        jusqua(lambda: m.etat in (Etat.TENU, Etat.ECHEC), 40.0)
        redresse = False
        if m.etat is Etat.TENU and tangage:     # redressé, le carton tenu, pour le porter
            red = m.changer_tangage(0.0)
            res["redressement"] = red.reason
            if red.ok:
                jusqua(lambda: not m.tangage_en_cours, 10.0)
                redresse = abs(r.tangage_taille) < 1e-3
        elif m.etat is Etat.TENU and abaissement:  # jambes dépliées, le carton tenu
            red = m.changer_abaissement(0.0)
            res["redressement"] = red.reason
            if red.ok:
                jusqua(lambda: r.locomotion.posture_atteinte, 10.0)
                redresse = r.abaissement_bassin < 1e-4
        if m.etat is Etat.TENU:
            pas(int(port_s / r._dt))
            signale = m.raison if (m.a_reposer or m.penche) else ""
            if redresse and tangage:            # penché de nouveau, pour le poser à sa place
                rep = m.changer_tangage(tangage)
                if rep.ok:
                    jusqua(lambda: not m.tangage_en_cours, 10.0)
            elif redresse and abaissement:      # jambes pliées de nouveau
                a_, _ = m.abaissement_pour_deposer(PLATEAU_M)
                if a_:
                    if m.changer_abaissement(a_).ok:
                        jusqua(lambda: r.locomotion.posture_atteinte, 10.0)
            pose = m.poser(PLATEAU_M)
            if pose.ok:
                jusqua(lambda: m.etat is Etat.LIBRE or (m.etat is Etat.TENU and m.raison != ""),
                       20.0)
            if m.etat is Etat.TENU:           # dépose refusée ou impossible : là où il a été pris
                signale = signale or m.raison or str(pose.reason)
                m.poser()
                jusqua(lambda: m.etat is Etat.LIBRE, 20.0)
            if m.etat is not Etat.LIBRE:
                res.update(issue="bloquée", raison=f"dépose inachevée ({m.etat.name}) {m.raison}")
            elif signale:
                res.update(issue="reposée", raison=signale)
            else:
                res.update(issue="déposée")
        else:
            raison = m.raison or f"geste inachevé ({m.etat.name})"
            m.reinitialiser()
            jusqua(lambda: m.etat is Etat.LIBRE, 5.0)
            res.update(issue="reposée" if res["tenu"] else "refusée", raison=raison)
    if r.locomotion.penche:                     # mains vides : redressé
        r.locomotion.posture(0.0)
        jusqua(lambda: r.locomotion.posture_atteinte, 10.0)
    pas(10)                                   # le carton retombe, s'il doit tomber
    res["chutes"] = [(round(a, 2), b, e) for a, b, e in r.chutes]
    res["droit_a_la_fin"] = not r.locomotion.penche
    z = carton.position[2]
    res["tombe"] = z < 0.5 * PLATEAU_M
    res["sur_table"] = carton.position[0] > 0.0 and abs(z - (PLATEAU_M + c.hauteur / 2)) < 0.03
    res["refus_garde_fou"] = sortie.getvalue().count("SAFETY VIOLATION")
    res["poids_os_n"] = None if m.poids_carton is None else round(m.poids_carton, 2)
    res["temps_s"] = round(t[0], 1)
    res["retour_arrete"] = bool(m.stats.get("retours_arretes"))
    for ligne in journal:
        if ligne.startswith("recalage au toucher : carton à "):
            res["recalage_cm"] = float(ligne.split("carton à ")[1].split(" cm")[0])
    res["journal"] = journal[-6:]
    for k in ("depasse_couple_nm", "depasse_vrai_nm", "depasse_materiel_nm", "enfoncement_max_m",
              "rotation_max_deg",
              "rotation_os_ramener_deg", "avance_max_m"):
        res[k] = round(res[k], 4)
    for k in ("marge_vraie_min", "marge_os_min", "enfoncement_contact_m"):
        if res[k] is not None:
            res[k] = round(res[k], 3)
    return res


def campagne(cartons: Sequence[CartonTire], reglages=None, processus: Optional[int] = None,
             port_s: float = 2.0, bruit: bool = True, pencher: bool = True,
             planificateur: bool = True, postures: bool = False) -> List[Dict]:
    """essayer_carton() sur chaque carton, en parallèle (un robot simulé par
    carton, rien de partagé) ; résultats dans l'ordre des cartons."""
    n = processus or max(1, min(len(cartons), os.cpu_count() or 1))
    if n == 1:
        return [essayer_carton(c, reglages, port_s, bruit, pencher, planificateur, postures)
                for c in cartons]
    import multiprocessing
    # fork (Linux) : les processus partent du nôtre, modules déjà chargés ;
    # ailleurs, spawn (le script appelant doit alors garder `if __name__ == "__main__"`).
    contexte = multiprocessing.get_context(
        "fork" if "fork" in multiprocessing.get_all_start_methods() else "spawn")
    with ProcessPoolExecutor(max_workers=n, mp_context=contexte) as ex:
        return list(ex.map(essayer_carton, cartons, [reglages] * len(cartons),
                           [port_s] * len(cartons), [bruit] * len(cartons),
                           [pencher] * len(cartons), [planificateur] * len(cartons),
                           [postures] * len(cartons)))


# ---- rapport ---------------------------------------------------------------

# Catégories des raisons de refus (la première qui correspond).
CATEGORIES = (
    ("trop large", ("trop large",)),
    # Prise en pendule : un carton trop bas pour viser 6 cm au-dessus de son
    # centre, 3 cm sous son arête (avant « hors de portée » : ce n'est pas la
    # distance) ; et les mains plus haut que le carton, jambes au plus pliées.
    ("trop bas pour une prise en pendule", ("trop bas pour une prise en pendule",)),
    ("trop bas pour les mains", ("trop bas pour les mains", "objet trop bas", "objet entre")),
    ("trop lourd ou trop glissant pour le serrage de prise", ("n'a pas suivi",
                                                               "n'a pas quitté la table")),
    ("trop lourd", ("trop lourd",)),
    ("dépose refusée ou impossible", ("dépose impossible", "plateau trop haut")),
    # Avant « hors de portée » : « avant-bras trop près du plateau à toute
    # distance » est un carton trop bas, pas trop loin.
    ("avant-bras trop près du plateau", ("avant-bras",)),
    ("hors de portée", ("hors de portée", "hors d'atteinte", "à toute distance")),
    ("serrage insuffisant", ("serrage insuffisant", "cède")),
    ("glisse au port", ("glisse", "ne pèse plus", "serrage perdu")),
    ("tourne ou penche", ("tourn", "bascul", "penché")),
    ("ligne des mains loin du centre de gravité", ("centre de gravité",)),
    ("équilibre", ("équilibre",)),
    ("chemin des bras", ("chemin des bras",)),
    ("contact ou placement des mains", ("contact", "appui", "mains mal placées", "loin de leur cible",
                                        "recalage", "mains mesurées", "rien entre les mains")),
)


def pourquoi_hors_de_portee(m, objet, raison: str) -> Optional[Dict]:
    """Un refus « hors de portée » : le robot aurait-il pu se rapprocher, ou
    est-ce la longueur des bras, ou le lacet ? None pour un autre refus.

    - « à toute distance » : le bassin ne s'approche jamais du bord de la
      table à moins de ReglagesPrise.bord_min_m (pieds contre la table).
      Sans cette règle, jusqu'à quelle distance les mains atteignent-elles
      le milieu des faces (cinématique de l'URDF, avant-bras au-dessus du
      plateau ; repères des mains entre 25 et 45 cm du carton, comme pour
      distance_de_prise) ? De la plus longue au bassin pieds contre la
      table, l'écart est ce qui manque aux bras (manque_m) ; None si elles
      n'y arrivent à aucune distance. (Mesuré d'abord depuis la plus courte,
      ce manque était surestimé, de 7 cm en médiane.)
    - « prise refusée » après l'approche : la distance de prise est calculée
      de face ; tourné, le carton présente une face plus loin, qu'une main
      n'atteint pas (lacet)."""
    if categorie(raison) != "hors de portée":
        return None
    if raison.startswith("prise refusée"):
        return {"cause": "lacet", "lacet_deg": round(math.degrees(objet.lacet), 1)}
    from dataclasses import replace
    from axis_os.services.manipulation.prise import distance_ideale
    from axis_os.services.manipulation.service import posture_bras
    r, p = m.r, m.p
    z = float(objet.centre[2]) - float(m.mesures.bassin_z())
    port = {**m._articulations_fixes(), **posture_bras(p.roulis_ouvert, p.tangage_port, p.coude)}
    d, _ = distance_ideale(m.mesures.modele, objet.largeur, objet.profondeur, objet.hauteur, z,
                           port, bord_m=objet.bord_m,
                           reglages=replace(r, contact_main_m=m._bout, bord_min_m=-1.0),
                           distances=np.arange(0.25, 0.45, 0.005)[::-1],   # la plus longue
                           seulement_au_vrai_bord=True)
    table = r.bord_min_m + objet.profondeur / 2.0 + objet.bord_m   # bassin au plus près
    return {"cause": "bras", "manque_m": None if d is None else round(table - d, 3),
            "bassin_bord_m": round(r.bord_min_m, 3)}


def categorie(raison: str) -> str:
    for nom, mots in CATEGORIES:
        if any(mot in raison for mot in mots):
            return nom
    return "autre"


def violations(res: Dict, reglages=None) -> List[str]:
    """Ce qui ne doit jamais arriver (voir l'en-tête du module), pour un
    résultat d'essayer_carton() ; [] si rien. Couples : le VRAI couple (pas
    l'estimation) sous le plafond de l'OS, à l'erreur d'estimation du banc
    près quand elle est simulée (son biais au plus, BruitCouples : l'OS ne
    voit que l'estimation), et toujours sous la limite matérielle."""
    from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples
    from axis_os.services.manipulation.prise import ReglagesPrise
    import inspect
    r = reglages or ReglagesPrise()
    biais = inspect.signature(BruitCouples).parameters["biais_max_nm"].default
    tolerance = 0.1 + (biais if res.get("bruit") else 0.0)
    v = []
    if res["tombe"] or res["chutes"]:
        v.append(f"carton tombé ({res['chutes'] or 'au sol'})")
    if not res["sur_table"] and not res["tombe"]:
        v.append("carton ni sur la table ni tenu à la fin")
    if res["depasse_borne_m"] > 1e-6:
        v.append(f"carton écrasé : consigne de serrage {100 * res['depasse_borne_m']:.1f} cm au-delà "
                 f"de sa borne")
    if (res.get("pied_au_dela_m") or -1.0) > 1e-6:
        v.append(f"pied au-delà du bord de la table, de {100 * res['pied_au_dela_m']:.1f} cm")
    if res["enfoncement_contact_m"] is not None and \
            res["enfoncement_contact_m"] > r.tol_face_contact_m:
        v.append(f"carton écrasé : contact vu la main déjà {100 * res['enfoncement_contact_m']:.1f} "
                 f"cm dans la face (au plus {100 * r.tol_face_contact_m:.1f})")
    if res["depasse_vrai_nm"] > tolerance:
        v.append(f"plafond de couple dépassé de {res['depasse_vrai_nm']:.2f} N·m (couple vrai, au "
                 f"plus {tolerance:.1f})")
    if res["depasse_materiel_nm"] > 0.0:
        v.append(f"couple vrai {res['depasse_materiel_nm']:.2f} N·m au-delà de la limite matérielle")
    if res["refus_garde_fou"]:
        v.append(f"{res['refus_garde_fou']} consigne(s) refusée(s) par le garde-fou")
    if res["fausse_prise"]:
        v.append(f"prise confirmée mains vides ({res['fausse_prise']} fois)")
    if res["perte_ignoree_s"] > 0.5:
        v.append(f"carton perdu sans que l'OS le voie pendant {res['perte_ignoree_s']:.1f} s")
    if res["issue"] not in ("déposée", "refusée", "reposée"):
        v.append(f"issue {res['issue']} : {res['raison']}")
    if res["issue"] != "déposée" and not res["raison"].strip():
        v.append("refusé ou reposé sans raison")
    return v


def _plages(resultats, cle, bornes, nom, unite="", facteur=1.0, absolu=False):
    lignes = [f"  {nom} :"]
    for lo, hi in zip(bornes[:-1], bornes[1:]):
        dedans = [x for x in resultats
                  if lo <= (abs(x["carton"][cle]) if absolu else x["carton"][cle]) < hi]
        if not dedans:
            continue
        dep = sum(x["issue"] == "déposée" for x in dedans)
        lignes.append(f"    {facteur * lo:6.2f} à {facteur * hi:6.2f}{unite} : {len(dedans):3d} cartons, "
                      f"{dep:3d} déposés ({100 * dep / len(dedans):3.0f} %), "
                      f"{sum(x['issue'] == 'reposée' for x in dedans):3d} reposés, "
                      f"{sum(x['issue'] == 'refusée' for x in dedans):3d} refusés")
    return lignes


def rapport(resultats: Sequence[Dict], graine: Optional[int] = None) -> str:
    """Réussite par plage et raisons de refus (voir l'en-tête du module) ;
    d'abord, les impossibles par construction (impossible_par_construction)
    à part, et la réussite sur les faisables."""
    n = len(resultats)
    dep = [x for x in resultats if x["issue"] == "déposée"]
    lignes = [f"Campagne de {n} cartons" + (f" (graine {graine})" if graine is not None else "")
              + f" : {len(dep)} tenus et déposés ({100 * len(dep) / max(1, n):.0f} %), "
              f"{sum(x['issue'] == 'reposée' for x in resultats)} reposés, "
              f"{sum(x['issue'] == 'refusée' for x in resultats)} refusés."]
    impossibles = [x for x in resultats if x["impossible"]]
    pourquoi = Counter(x["impossible"] for x in impossibles)
    lignes.append(f"  impossibles par construction : {len(impossibles)}"
                  + (" (" + ", ".join(f"{k} {v}" for k, v in pourquoi.most_common()) + ")"
                     if pourquoi else "")
                  + f", {sum(x['issue'] == 'déposée' for x in impossibles)} déposés")
    faisables = [x for x in resultats if not x["impossible"]]
    dep_f = sum(x["issue"] == "déposée" for x in faisables)
    raisons_f = Counter(categorie(x["raison"]) for x in faisables if x["issue"] != "déposée")
    lignes.append(f"  faisables : {len(faisables)} cartons, {dep_f} tenus et déposés "
                  f"({100 * dep_f / max(1, len(faisables)):.0f} %), "
                  f"{sum(x['issue'] == 'reposée' for x in faisables)} reposés, "
                  f"{sum(x['issue'] == 'refusée' for x in faisables)} refusés"
                  + (" ; raisons : " + ", ".join(f"{k} {v}" for k, v in raisons_f.most_common())
                     if raisons_f else ""))
    lignes += _plages(resultats, "largeur", [0.15, 0.20, 0.25, 0.30, 0.35, 0.45, 0.61],
                      "largeur (entre les mains)", " cm", 100)
    lignes += _plages(resultats, "profondeur", [0.15, 0.20, 0.25, 0.301], "profondeur", " cm", 100)
    lignes += _plages(resultats, "hauteur", [0.10, 0.15, 0.20, 0.25, 0.301], "hauteur", " cm", 100)
    lignes += _plages(resultats, "masse_kg", [0.2, 0.7, 1.2, 1.7, 2.2, 2.51, 3.0, 5.01],
                      "masse", " kg")
    lignes += _plages(resultats, "frottement", [0.5, 0.7, 0.9, 1.1, 1.21], "frottement (μ)")
    lignes += _plages(resultats, "lacet_deg", [0.0, 10.0, 20.0, 30.0, 45.01], "lacet", "°",
                      absolu=True)
    lignes += _plages(resultats, "retrait_m", [0.0, 0.05, 0.10, 0.15, 0.20, 0.251],
                      "place : de l'avant du carton au bord de la table", " cm", 100)
    exp = [x for x in resultats if x["carton"]["expres"]]
    if exp:
        lignes.append(f"  hors des moyens, exprès : {len(exp)} cartons, "
                      f"{sum(x['issue'] == 'déposée' for x in exp)} déposés")
    if dep:
        def plus(cle, f=max):
            return f(x["carton"][cle] for x in dep)
        lignes.append(f"  limites vues (cartons tenus et déposés) : jusqu'à {plus('masse_kg'):.2f} kg, "
                      f"{100 * plus('largeur'):.0f} cm de large, {100 * plus('retrait_m'):.0f} cm du "
                      f"bord, μ dès {plus('frottement', min):.2f}, lacet jusqu'à "
                      f"{max(abs(x['carton']['lacet_deg']) for x in dep):.0f}°")
        mv = [x["marge_vraie_min"] for x in dep if x["marge_vraie_min"] is not None]
        if mv:
            lignes.append(f"  marge de frottement vraie au port (2 μ min N / poids) : au plus bas "
                          f"{min(mv):.2f}, médiane {sorted(mv)[len(mv) // 2]:.2f}")
    raisons: Dict[str, List[str]] = {}
    for x in resultats:
        if x["issue"] != "déposée":
            raisons.setdefault(categorie(x["raison"]), []).append(
                f"n° {x['carton']['numero']} ({x['issue']}) : {x['raison']}")
    lignes.append("  raisons de refus et de repose :")
    for cat, ex in sorted(raisons.items(), key=lambda kv: -len(kv[1])):
        lignes.append(f"    {cat} : {len(ex)}")
        lignes += [f"      {e[:200]}" for e in ex[:3]]
    portee = [x["portee"] for x in resultats if x.get("portee")]
    if portee:
        bras = [p_ for p_ in portee if p_["cause"] == "bras"]
        manques = sorted(p_["manque_m"] for p_ in bras if p_["manque_m"] is not None)
        lignes.append("  hors de portée, pourquoi :")
        if bras:
            lignes.append(
                f"    longueur des bras, robot au plus près permis (bord de la table à "
                f"{100 * bras[0]['bassin_bord_m']:.0f} cm du bassin) : {len(bras)}"
                + (f" ; il manque de {100 * manques[0]:.0f} à {100 * manques[-1]:.0f} cm "
                   f"(médiane {100 * manques[len(manques) // 2]:.0f})" if manques else "")
                + (f" ; {len(bras) - len(manques)} hors d'atteinte à toute distance"
                   if len(manques) < len(bras) else ""))
        lacets = [p_ for p_ in portee if p_["cause"] == "lacet"]
        if lacets:
            lignes.append(
                f"    lacet, carton abordé de face : {len(lacets)} (tournés de "
                f"{min(abs(p_['lacet_deg']) for p_ in lacets):.0f} à "
                f"{max(abs(p_['lacet_deg']) for p_ in lacets):.0f}°) ; une face plus loin "
                f"qu'une main n'atteint pas")
    return "\n".join(lignes)


# ---- balayage autour du carton T1 ---------------------------------------------

# Le carton de la scène T1, au bord de la table, robot bien placé.
CARTON_T1 = CartonTire(numero=0, largeur=0.24, profondeur=0.20, hauteur=0.20, masse_kg=1.0,
                       frottement=1.0, lacet_deg=0.0, retrait_m=0.0, decalage_m=0.0,
                       erreur_x_m=0.0, erreur_y_m=0.0, erreur_cap_deg=0.0)
# Un paramètre à la fois : où chacun cesse de passer, les autres ceux de T1.
BALAYAGE = (("largeur", (0.15, 0.20, 0.25, 0.30, 0.33)),
            ("profondeur", (0.15, 0.25, 0.30)),
            ("hauteur", (0.10, 0.13, 0.16, 0.25, 0.30)),
            ("masse_kg", (0.2, 0.6, 1.2, 1.4, 1.8, 2.5)),
            ("frottement", (0.5, 0.6, 0.7, 0.9, 1.2)),
            ("lacet_deg", (10.0, 20.0, 30.0, 45.0)),
            # 25 et 30 cm : atteints pieds contre la table (distance_ideale, second palier).
            ("retrait_m", (0.05, 0.10, 0.15, 0.20, 0.25, 0.30)))


def balayage() -> List[Tuple[str, float, CartonTire]]:
    """Le carton T1, puis BALAYAGE : (paramètre, valeur, carton)."""
    from dataclasses import replace
    lots = [("T1", 0.0, replace(CARTON_T1, numero=1000))]
    for cle, valeurs in BALAYAGE:
        lots += [(cle, v, replace(CARTON_T1, numero=1000 + len(lots) + k, **{cle: v}))
                 for k, v in enumerate(valeurs)]
    return lots


# ---- audit des réglages choisis sur le carton T1 --------------------------------

def audit(resultats: Sequence[Dict], reglages=None) -> List[str]:
    """Chaque réglage choisi sur le carton de la scène T1, confronté aux
    cartons de la campagne : ce qu'ils en montrent (voir
    docs/validation_port_carton.md, campagne de cartons variés)."""
    from axis_os.services.manipulation.prise import ReglagesPrise
    r = reglages or ReglagesPrise()
    portes = [x for x in resultats if x["marge_vraie_min"] is not None]
    lignes = []
    bas = [x for x in portes if x["carton"]["frottement"] < r.frottement_serrage]
    if bas:
        pire = min(bas, key=lambda x: x["marge_vraie_min"])
        mu = min(x["carton"]["frottement"] for x in bas)
        lignes.append(
            f"frottement_serrage {r.frottement_serrage:g} : {len(bas)} carton(s) porté(s) avec μ en "
            f"dessous ; marge vraie au plus bas {pire['marge_vraie_min']:.2f} (μ "
            f"{pire['carton']['frottement']:.2f}, l'OS en croyait {pire['marge_os_min'] or 0:.2f}) ; "
            f"garantie, au serrage requis : {r.securite_serrage * mu / r.frottement_serrage:.2f} "
            f"(sécurité {r.securite_serrage:g} × μ {mu:.2f} / {r.frottement_serrage:g})")
    ramenes = [x for x in portes if "RAMENER" in x["etats"]]
    if ramenes:
        faux = [x for x in ramenes if x["retour_arrete"] and x["rotation_max_deg"] < 1.0]
        haut = max(ramenes, key=lambda x: x["rotation_os_ramener_deg"])
        lignes.append(
            f"rotation_ramener_max_deg {r.rotation_ramener_max_deg:g} : estimée au retour jusqu'à "
            f"{haut['rotation_os_ramener_deg']:.1f}° ({haut['carton']['masse_kg']:.2f} kg, lacet "
            f"{haut['carton']['lacet_deg']:+.0f}°) ; {sum(x['retour_arrete'] for x in ramenes)} "
            f"retour(s) arrêté(s), dont {len(faux)} sans que le carton tourne (vrai < 1°)")
    if portes:
        lignes.append(
            f"penetration_poids_max_m {100 * r.penetration_poids_max_m:.0f} cm : avance au plus "
            f"{100 * max(x['avance_max_m'] for x in portes):.1f} cm ; borne atteinte "
            f"{sum(x['borne_poids'] for x in resultats)} fois ; « trop lourd » : "
            f"{sum(categorie(x['raison']) == 'trop lourd' for x in resultats)}")
    contacts = [x["enfoncement_contact_m"] for x in resultats if x["enfoncement_contact_m"] is not None]
    if contacts:
        rec = [abs(x["recalage_cm"]) for x in resultats if x["recalage_cm"] is not None]
        lignes.append(
            f"seuils de contact (force_contact_n {r.force_contact_n:g} N, sigmas_contact "
            f"{r.sigmas_contact:g}, hausse_contact_nm {r.hausse_contact_nm:g}) : contact vu la main "
            f"{100 * sorted(contacts)[len(contacts) // 2]:.1f} cm dans la face (médiane), au plus "
            f"{100 * max(contacts):.1f} (tol_face_contact_m {100 * r.tol_face_contact_m:.1f}) ; "
            f"recalage au plus {max(rec, default=0.0):.1f} cm ; refus de contact ou de placement : "
            f"{sum(categorie(x['raison']) == 'contact ou placement des mains' for x in resultats)}")
    if portes:
        lourd = max(portes, key=lambda x: x["carton"]["masse_kg"])
        pesees = [x["carton"]["masse_kg"] for x in resultats if "kg pesés" in x["raison"]]
        lignes.append(
            f"masse_max_kg {r.masse_max_kg:g} (charge la plus lourde portée, reposée au-delà dès "
            f"la pesée ; tolérance de la ligne des mains au centre de gravité) : porté jusqu'à "
            f"{lourd['carton']['masse_kg']:.2f} kg, rotation vraie au plus "
            f"{max(x['rotation_max_deg'] for x in portes):.1f}° ; reposés à la pesée : "
            f"{len(pesees)}" + (f" (de {min(pesees):.2f} à {max(pesees):.2f} kg)" if pesees else ""))
    glisse = [x for x in resultats if categorie(x["raison"]).startswith("trop lourd ou trop glissant")]
    if glisse:
        lignes.append(
            f"force_serrage_n {r.force_serrage_n:g} N (serrage de prise, avant de peser) : "
            f"{len(glisse)} carton(s) qui ne suivent pas le levage (de "
            f"{min(x['carton']['masse_kg'] for x in glisse):.2f} kg, μ dès "
            f"{min(x['carton']['frottement'] for x in glisse):.2f})")
    return lignes


if __name__ == "__main__":
    # Avant et après le planificateur de prise, mêmes cartons.
    import sys
    graine = int(sys.argv[1]) if len(sys.argv) > 1 else 20261118
    cartons = tirer_cartons(int(sys.argv[2]) if len(sys.argv) > 2 else 120, graine)
    for planifie in (False, True):
        print(f"\n=== {'Avec' if planifie else 'Sans'} le planificateur de prise ===")
        print(rapport(campagne(cartons, planificateur=planifie), graine))
