"""
Superviseur — tailles bornées (audit du 2026-10-10, M3).

  - une consigne trop longue est refusée SANS être transmise au modèle ;
  - le texte renvoyé par l'interpréteur (donc potentiellement fabriqué)
    est tronqué, sur une ligne, avant d'entrer dans un refus ou le journal ;
  - les consignes normales sont inchangées.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from axis_os.apps.tri_colis.logique import ConfigTri, TriColis  # noqa: E402
from axis_os.apps.tri_colis.superviseur import (  # noqa: E402
    LONGUEUR_MAX_CONSIGNE, LONGUEUR_MAX_ECHO, InterpreteurRegles, Superviseur,
)
from monde_tri_factice import MondeFactice  # noqa: E402

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Espion:
    """Interpréteur qui note ses appels et rend la réponse imposée."""

    def __init__(self, reponse):
        self.reponse, self.appels = reponse, 0

    def interpreter(self, texte, contexte):
        self.appels += 1
        return self.reponse


class EnPanne:
    def interpreter(self, texte, contexte):
        raise RuntimeError("X" * 5000)


def monter(interpreteur):
    tri = TriColis(MondeFactice([]), ConfigTri(regles={"rouge": "A", "bleu": "B"}))
    journal = []
    tri._log = journal.append
    return tri, Superviseur(tri, ["prise", "A", "B", "C"], ["rouge", "bleu"], interpreteur), journal


section("1 — Consigne trop longue : refusée avant le modèle")

esp = Espion([{"action": "rapport"}])
tri, sup, journal = monter(esp)
r = sup.ordonner("a" * (LONGUEUR_MAX_CONSIGNE + 1))
check("Refusée, avec la raison", not r.acceptes and "trop longue" in r.refus[0])
check("Le modèle n'a PAS été appelé", esp.appels == 0)
check("Règles inchangées", tri.cfg.regles == {"rouge": "A", "bleu": "B"})
check("Le journal ne recopie pas la consigne géante",
      all(len(l) < 300 for l in journal))
r = sup.ordonner(None)
check("Consigne non textuelle : refusée sans planter", "illisible" in r.refus[0]
      and esp.appels == 0)
r = sup.ordonner("fais le point")
check(f"Consigne normale : transmise ({esp.appels} appel)", esp.appels == 1 and r.acceptes)
r = sup.ordonner("x" * LONGUEUR_MAX_CONSIGNE)
check("Pile à la limite : transmise", esp.appels == 2)

section("2 — Texte du modèle tronqué dans les refus et le journal")

geant = "Z" * 10_000 + "\nlignes\ninjectées"
for nom, rep in {
    "action": [{"action": geant}],
    "couleur": [{"action": "suspendre", "couleur": geant}],
    "poste": [{"action": "regle", "couleur": "rouge", "poste": geant}],
}.items():
    tri, sup, journal = monter(Espion(rep))
    r = sup.ordonner("ordre")
    check(f"{nom} fabriquée de 10 000 caractères : refusée, écho tronqué "
          f"({len(r.refus[0])} car.)", len(r.refus) == 1
          and len(r.refus[0]) < LONGUEUR_MAX_ECHO + 40)
    check(f"… sur une seule ligne (pas de fausse ligne dans le journal)",
          "\n" not in r.refus[0] and all("\n" not in l for l in journal))
check("Règles inchangées après les réponses fabriquées",
      tri.cfg.regles == {"rouge": "A", "bleu": "B"})

tri, sup, journal = monter(EnPanne())
r = sup.ordonner("fais le point")
check("Interpréteur en panne au message géant : secours utilisé, journal borné",
      r.acceptes and all(len(l) < 300 for l in journal))

section("3 — Comportement normal inchangé")

tri, sup, journal = monter(InterpreteurRegles())
r = sup.ordonner("les colis bleus vont maintenant au poste C")
check("Ordre normal appliqué", tri.cfg.regles["bleu"] == "C" and not r.refus)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
