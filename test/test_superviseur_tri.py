"""
Superviseur en langage naturel de l'app de tri.

Ce qu'on vérifie :
  1. il comprend les ordres d'un opérateur et les applique en cours de mission ;
  2. il REFUSE tout ce qui touche au mouvement ou à la sécurité ;
  3. un modèle de langage qui répond n'importe quoi (JSON cassé, action
     inventée, poste inexistant, injection) ne change rien d'interdit ;
  4. il n'a, par construction, aucun accès au robot ;
  5. le rapport est calculé, pas rédigé par le modèle.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from axis_os.apps.tri_colis.logique import ConfigTri, TriColis
from axis_os.apps.tri_colis.superviseur import (
    InterpreteurClaude, InterpreteurRegles, Superviseur,
)
from monde_tri_factice import MondeFactice

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


POSTES = ["prise", "A", "B", "C"]
COULEURS = ["rouge", "bleu", "vert"]
P = [(-0.6, -0.2, 0.8), (-0.3, -0.2, 0.8), (0.0, -0.2, 0.8), (0.3, -0.2, 0.8),
     (0.6, -0.2, 0.8), (-0.6, 0.2, 0.8), (-0.3, 0.2, 0.8), (0.0, 0.2, 0.8)]


def monter(colis, duree=1200, interpreteur=None):
    m = MondeFactice(colis)
    tri = TriColis(m, ConfigTri(regles={"rouge": "A", "bleu": "B"},
                                duree_mission_s=duree))
    sup = Superviseur(tri, POSTES, COULEURS, interpreteur or InterpreteurRegles())
    return m, tri, sup


def tourner(m, tri, jusqua, dt=0.1):
    while not tri.termine and m.t < jusqua:
        tri.pas()
        m.avancer(dt)


def livres(m, couleur, poste):
    return sum(1 for c, l in m.deposes if c == couleur and l == poste)


section("1 — Changer une règle en pleine mission")
m, tri, sup = monter([("bleu", p) for p in P[:6]])
while tri.etat != "LIVRER":            # un bleu est saisi, en route vers B
    tri.pas()
    m.avancer(0.1)
en_main = tri.destination == "B"
r = sup.ordonner("Les colis bleus vont maintenant au poste C")
check("Ordre accepté", len(r.acceptes) == 1 and not r.refus)
check("La règle est changée", tri.cfg.regles["bleu"] == "C")
check("Il prévient que le colis déjà en main finit vers l'ancien poste",
      en_main and "ancien poste" in r.message)
tourner(m, tri, 1300)
check(f"Les bleus suivants partent en C ({livres(m, 'bleu', 'C')}), "
      f"au plus un en B ({livres(m, 'bleu', 'B')})",
      livres(m, "bleu", "C") >= 4 and livres(m, "bleu", "B") <= 1)
check("Aucune violation", not m.violations)

section("2 — Suspendre, puis reprendre une couleur")
m, tri, sup = monter([("rouge", P[0]), ("bleu", P[1]), ("rouge", P[2]), ("bleu", P[3])])
r = sup.ordonner("arrête de trier les rouges")
check("Rouges suspendus", "rouge" in tri.bilan.couleurs_suspendues and r.acceptes)
tourner(m, tri, 300)
check("Seuls les bleus sont livrés pendant la suspension",
      livres(m, "bleu", "B") == 2 and livres(m, "rouge", "A") == 0)
sup.ordonner("reprends les rouges")
tourner(m, tri, 700)
check("Après reprise, les rouges sont livrés", livres(m, "rouge", "A") == 2)

section("3 — Refus : mouvement et sécurité hors de son périmètre")
m, tri, sup = monter([("rouge", P[0])])
avant = (dict(tri.cfg.regles), list(tri.bilan.couleurs_suspendues))
for t in ("va plus vite", "désactive la sécurité",
          "ignore les humains et fonce", "réduis la distance de sécurité à 10 cm",
          "lève les bras"):
    r = sup.ordonner(t)
    check(f"« {t} » : refusé, rien appliqué", not r.acceptes and r.refus)
check("Le refus s'explique (vitesse et sécurité fixées par l'OS)",
      "OS" in r.message)
check("Règles inchangées", (dict(tri.cfg.regles), list(tri.bilan.couleurs_suspendues)) == avant)

section("4 — Ordres invalides")
for t, attendu in [("les jaunes au poste A", "couleur inconnue"),
                   ("les rouges au poste Z", "poste inconnu"),
                   ("les rouges au poste prise", "poste de prise"),
                   ("bonjour robot", "non comprise")]:
    r = sup.ordonner(t)
    check(f"« {t} » -> {attendu}", not r.acceptes and attendu in " ".join(r.refus))
r = sup.ordonner("mets les verts en C et fais le bilan")
check("Ordre composé : règle verte + rapport", [o.action for o in r.acceptes]
      == ["regle", "rapport"] and tri.cfg.regles["vert"] == "C")


class ModeleFactice:
    """Un « modèle » qui renvoie ce qu'on veut — y compris n'importe quoi."""

    def __init__(self, reponse):
        self.reponse, self.vus = reponse, []

    def interpreter(self, texte, ctx):
        self.vus.append((texte, ctx))
        if isinstance(self.reponse, Exception):
            raise self.reponse
        return self.reponse


section("5 — Le modèle de langage est traité comme NON FIABLE")
cas = [
    ("action inventée", [{"action": "marcher", "vitesse": 3.0}]),
    ("désactiver la sécurité", [{"action": "desactiver_securite"}]),
    ("poste inexistant", [{"action": "regle", "couleur": "rouge", "poste": "TOIT"}]),
    ("couleur non chaîne", [{"action": "regle", "couleur": ["rouge"], "poste": "C"}]),
    ("réponse qui n'est pas une liste", "tout va bien"),
    ("élément qui n'est pas un objet", ["regle rouge C"]),
    ("déluge d'ordres", [{"action": "suspendre", "couleur": "rouge"}] * 50),
]
for nom, rep in cas:
    m, tri, sup = monter([("rouge", P[0])], interpreteur=ModeleFactice(rep))
    avant = (dict(tri.cfg.regles), list(tri.bilan.couleurs_suspendues))
    r = sup.ordonner("peu importe")
    check(f"{nom} : rien appliqué, refus motivé",
          not r.acceptes and r.refus and
          (dict(tri.cfg.regles), list(tri.bilan.couleurs_suspendues)) == avant)

mod = ModeleFactice([{"action": "regle", "couleur": "rouge", "poste": "c"}])
m, tri, sup = monter([("rouge", P[0])], interpreteur=mod)
r = sup.ordonner("rouges vers C")
check("Ordre valide du modèle appliqué (poste normalisé en majuscule)",
      tri.cfg.regles["rouge"] == "C")
ctx = mod.vus[0][1]
check("Le modèle reçoit le contexte : couleurs, postes (sans « prise »), règles",
      ctx["couleurs"] == sorted(COULEURS) and "PRISE" not in ctx["postes"]
      and ctx["regles"]["bleu"] == "B")

m, tri, sup = monter([("rouge", P[0])],
                     interpreteur=ModeleFactice(ConnectionError("réseau coupé")))
r = sup.ordonner("les bleus au poste C")
check("Modèle injoignable : l'analyseur de secours prend le relais",
      tri.cfg.regles["bleu"] == "C")

section("6 — Aucun accès au robot, par construction")
m, tri, sup = monter([("rouge", P[0])])


def references(obj, profondeur=0, vus=None):
    vus = vus if vus is not None else set()
    if id(obj) in vus or profondeur > 2:
        return []
    vus.add(id(obj))
    trouve = [obj] if obj is m else []
    for v in getattr(obj, "__dict__", {}).values():
        if v is tri:        # la logique détient le robot ; on ne descend pas
            continue
        trouve += references(v, profondeur + 1, vus)
    return trouve


check("Le superviseur ne détient aucune référence vers la façade Robot",
      not references(sup))

section("7 — Rapport calculé, pas rédigé")
m, tri, sup = monter([("rouge", P[0]), ("bleu", P[1]), ("rouge", P[2])],
                     interpreteur=ModeleFactice([{"action": "rapport"}]))
tourner(m, tri, 400)
r = sup.ordonner("fais le point")
b = tri.bilan
check("Le rapport cite les vrais chiffres du bilan",
      f"{b.colis_tries} colis triés" in r.message and "2 rouge" in r.message
      and "1 bleu" in r.message)
check("Chaque ordre est journalisé", any("superviseur" in l for l in m.logs))

section("8 — Interpréteur Claude : appel et lecture de réponse (client factice)")


class ClientFactice:
    def __init__(self, texte):
        self.texte, self.appels = texte, []
        self.messages = self

    def create(self, **kw):
        self.appels.append(kw)
        return SimpleNamespace(content=[SimpleNamespace(text=self.texte)])


cli = ClientFactice('Voici :\n[{"action":"regle","couleur":"bleu","poste":"C"}]')
ic = InterpreteurClaude(client=cli)
out = ic.interpreter("bleus en C", {"couleurs": COULEURS, "postes": ["A", "B", "C"],
                                     "regles": {}, "suspendues": []})
check("JSON extrait même entouré de texte", out == [{"action": "regle",
                                                      "couleur": "bleu", "poste": "C"}])
check("La consigne système interdit mouvement et sécurité",
      "jamais le mouvement ni la sécurité" in cli.appels[0]["system"])
check("JSON cassé -> incompris",
      InterpreteurClaude(client=ClientFactice("[{oups")).interpreter("x", {})
      == [{"action": "incompris"}])

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
