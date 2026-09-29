"""
App de tri de colis — la logique métier, face à des pannes provoquées.

Un monde factice joue le robot : trajets qui durent, prises qui ratent,
postes inaccessibles, humains qui bloquent le robot. Il note toute
VIOLATION (commande pendant un arrêt de sécurité, prise les bras chargés,
dépose en marche…). Le test ne vérifie pas seulement que ça trie : il
vérifie que ça ne fait jamais n'importe quoi.
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis.logique import ConfigTri, TriColis

sys.path.insert(0, str(Path(__file__).resolve().parent))
from monde_tri_factice import (  # noqa: E402
    REGLES, MondeFactice, bien_livres, executer, mal_livres,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


P = [(-0.3, 0.0, 0.8), (-0.1, 0.1, 0.8), (0.1, -0.1, 0.8), (0.3, 0.0, 0.8),
     (0.0, 0.2, 0.8)]

section("1 — Nominal : cinq colis, deux postes")
m = MondeFactice([("rouge", P[0]), ("bleu", P[1]), ("rouge", P[2]),
                  ("bleu", P[3]), ("rouge", P[4])])
tri = executer(m, 600)
check("Les 5 colis sont triés", tri.bilan.colis_tries == 5)
check("Tous au BON poste (rouge->A, bleu->B)", bien_livres(m) == 5 and mal_livres(m) == 0)
check("par couleur : 3 rouges, 2 bleus", tri.bilan.par_couleur == {"rouge": 3, "bleu": 2})
check("Aucune violation", not m.violations)
check("La mission se termine proprement (FIN), bras vides",
      tri.etat == "FIN" and m.en_main is None)

section("2 — Prise ratée, puis réussie au nouvel essai")
m = MondeFactice([("rouge", P[0])], prises={0: 1})
tri = executer(m, 300)
check("Le colis est finalement livré", tri.bilan.colis_tries == 1 and bien_livres(m) == 1)
check("1 prise ratée, 1 réussite après nouvel essai",
      tri.bilan.prises_ratees == 1 and tri.bilan.reussies_apres_nouvel_essai == 1)
check("Aucune violation", not m.violations)

section("3 — Colis impossible à prendre : abandonné après 3 essais, signalé")
m = MondeFactice([("rouge", P[0]), ("bleu", P[1])], prises={0: 99})
tri = executer(m, 250)
check("Le colis impossible est abandonné (1), l'autre livré (1)",
      tri.bilan.colis_abandonnes == 1 and tri.bilan.colis_tries == 1)
check("Exactement 3 prises ratées, pas une de plus", tri.bilan.prises_ratees == 3)
check("Il est SIGNALÉ dans le journal", any("SIGNALÉ" in l for l in m.logs))
check("Et reste sur la table, jamais retenté", any(c["id"] == 0 for c in m.table))
check("Aucune violation", not m.violations)
m = MondeFactice([("rouge", P[0])], prises={0: 99})
tri = executer(m, 400, ConfigTri(regles=dict(REGLES), duree_mission_s=400,
                                 oubli_ignore_s=120))
check("Passé le délai d'oubli, l'emplacement est réexaminé (l'opérateur a pu "
      "le dégager) — puis réabandonné proprement",
      tri.bilan.prises_ratees == 9 and tri.bilan.colis_abandonnes == 3)

section("3c — Prise ratée qui fait TOMBER le colis : on re-regarde, pas de prise dans le vide")
m = MondeFactice([("rouge", P[0]), ("bleu", P[1])], prises={0: 1})
m.tombe_si_rate = {0}
tri = executer(m, 300)
check("Aucune prise refermée sur le vide (le robot re-regarde la table)",
      m.prises_dans_le_vide == 0, )
check("Une seule prise ratée, rien d'abandonné, l'autre colis livré",
      tri.bilan.prises_ratees == 1 and tri.bilan.colis_abandonnes == 0
      and tri.bilan.colis_tries == 1)
check("Aucune violation", not m.violations)

section("3b — Accostage raté : retour au poste, puis nouvel essai")
m = MondeFactice([("rouge", P[0])])
m.echecs_accostage = 2
tri = executer(m, 300)
check("Livré après 2 accostages ratés", tri.bilan.colis_tries == 1)
check("Pas abandonné", tri.bilan.colis_abandonnes == 0)
check("Aucune violation", not m.violations)

section("4 — Un humain bloque le robot 120 s en plein trajet")
m = MondeFactice([("rouge", P[0])], humains=[(5.0, 125.0)])
tri = executer(m, 400)
check("Le colis est livré malgré tout", tri.bilan.colis_tries == 1)
check("L'arrêt n'est PAS compté comme un échec de navigation",
      tri.bilan.echecs_navigation == 0)
check(f"Le temps d'arrêt est compté à part ({tri.bilan.temps_arret_securite_s:.0f} s)",
      119.0 <= tri.bilan.temps_arret_securite_s <= 121.0)
check("Aucune commande pendant l'arrêt", not m.violations)
check("Arrêt prolongé (120 s) : UNE alerte pour l'opérateur, pas une par pas",
      sum(1 for l in m.logs if "arrêt de sécurité depuis" in l) == 1)

section("5 — Poste B inaccessible : bleu suspendu, colis ramené, rouge continue")
m = MondeFactice([("bleu", P[0]), ("rouge", P[1]), ("bleu", P[2]), ("rouge", P[3])],
                 inaccessibles={"B"})
tri = executer(m, 900)
check("Couleur bleu SUSPENDUE", tri.bilan.couleurs_suspendues == ["bleu"])
check("ALERTE au journal", any("ALERTE" in l and "SUSPENDU" in l for l in m.logs))
check("Le colis bleu est reposé au poste de prise", ("bleu", "prise") in m.deposes)
check("Les 2 rouges sont livrés en A", tri.bilan.par_couleur.get("rouge") == 2)
check("Aucun bleu livré ailleurs", mal_livres(m) == 0)
check("L'autre bleu n'est jamais pris", sum(1 for c in m.table if c["couleur"] == "bleu") == 2)
check("Aucune violation", not m.violations)

section("5b — Poste de prise bloqué un moment : pause signalée, puis reprise")
m = MondeFactice([("rouge", P[0])], inaccessibles={"prise"})
cfg = ConfigTri(regles=dict(REGLES), duree_mission_s=300)
tri = TriColis(m, cfg)
while m.t < 60:
    tri.pas(); m.avancer(0.1)
check("Poste de prise inaccessible : PAUSE signalée, pas d'arrêt de mission",
      tri.etat in ("PAUSE", "REVENIR") and not tri.termine
      and any("ALERTE" in l and "pause" in l for l in m.logs))
m.inaccessibles.clear()           # l'obstruction est dégagée
while not tri.termine and m.t < 900:
    tri.pas(); m.avancer(0.1)
check("L'obstruction levée, le robot reprend et livre", tri.bilan.colis_tries == 1)
check("Aucune violation", not m.violations)

section("6 — Couleur inconnue : jamais touchée")
m = MondeFactice([("vert", P[0]), ("rouge", P[1])])
tri = executer(m, 300)
check("Le rouge est livré", tri.bilan.colis_tries == 1)
check("Le vert reste sur la table, jamais pris",
      any(c["couleur"] == "vert" for c in m.table) and
      all(c != "vert" for c, _ in m.deposes))

section("7 — Fin de mission : jamais les bras chargés")
m = MondeFactice([("rouge", p) for p in P], duree_trajet=30.0)
tri = executer(m, 60)       # la mission expire en plein transport
check("La mission se termine", tri.termine and tri.etat == "FIN")
check("Bras vides à la fin", m.en_main is None and m.manip == "LIBRE")
check("Le colis en cours a été livré avant de s'arrêter", tri.bilan.colis_tries >= 1)
check("Aucune violation", not m.violations)

section("8 — Endurance 30 min, pannes aléatoires, humains, réappro")
rng = random.Random(7)
humains, t = [], 60.0
while t < 1800:
    d = rng.uniform(5, 25)
    humains.append((t, t + d))
    t += rng.uniform(90, 240)
m = MondeFactice([("rouge", P[0]), ("bleu", P[1])], humains=humains, graine=11,
                 p_rate=0.25, p_nav=0.05, reappro_s=20.0)
tri = executer(m, 1800)
b = tri.bilan
print(f"    {b.colis_tries} triés, {b.colis_abandonnes} abandonnés, "
      f"{b.prises_ratees} prises ratées, {b.echecs_navigation} échecs nav, "
      f"{b.temps_arret_securite_s:.0f} s d'arrêts sécurité, "
      f"taux {100 * b.taux_reussite():.0f} %")
check("Au moins 20 colis triés en 30 min (critère de la démo)", b.colis_tries >= 20)
check("Tous au bon poste", mal_livres(m) == 0)
check("Taux de réussite ≥ 90 %", b.taux_reussite() >= 0.90)
check("Les verts (sans règle) ne sont jamais pris",
      all(c != "vert" for c, _ in m.deposes))
check("Fin propre, bras vides", tri.etat == "FIN" and m.en_main is None)
check(f"Aucune violation ({len(m.violations)})", not m.violations)
for v in m.violations[:5]:
    print("     ", v)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
