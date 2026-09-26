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

from axis_os.apps.tri_colis.logique import Colis, ConfigTri, TriColis

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


REGLES = {"rouge": "A", "bleu": "B"}
EMPLACEMENTS = [(x, y, 0.8) for x in (-0.6, -0.3, 0.0, 0.3, 0.6) for y in (-0.2, 0.2)]


class MondeFactice:
    """Implémente la façade Robot, avec des pannes réglables."""

    def __init__(self, colis, prises=None, inaccessibles=(), humains=(),
                 duree_trajet=8.0, graine=None, p_rate=0.0, p_nav=0.0,
                 reappro_s=None):
        self.t = 0.0
        self.table = [dict(id=i, couleur=c, pos=p) for i, (c, p) in enumerate(colis)]
        self.prochain_id = len(self.table)
        self.prises = dict(prises or {})       # id -> nb d'échecs avant succès
        self.inaccessibles = set(inaccessibles)
        self.humains = list(humains)           # (début, fin) d'arrêt sécurité
        self.duree_trajet = duree_trajet
        self.rng = random.Random(graine)
        self.p_rate, self.p_nav, self.reappro_s = p_rate, p_nav, reappro_s
        self._prochain_reappro = reappro_s or 0.0
        self.lieu = None
        self.nav = "ARRIVE"
        self._reste_nav = 0.0
        self._nav_echoue = False
        self.manip = "LIBRE"
        self._reste_manip = 0.0
        self._manip_resultat = None
        self.en_main = None
        self.deposes = []          # (couleur, lieu)
        self.violations = []
        self.logs = []
        self.echecs_accostage = 0

    # --- horloge du monde ------------------------------------------------
    def avancer(self, dt):
        self.t += dt
        if self.reappro_s and self.t >= self._prochain_reappro:
            self._prochain_reappro += self.reappro_s
            self._reapprovisionner()
        if self.bloque_par_securite():
            return                  # l'OS a figé le robot : rien n'avance
        if self.nav == "EN_ROUTE":
            self._reste_nav -= dt
            if self._reste_nav <= 0:
                self.nav = "ECHEC" if self._nav_echoue else "ARRIVE"
                if self.nav == "ARRIVE":
                    self.lieu = self._cible
        if self.manip in ("PRISE", "POSE"):
            self._reste_manip -= dt
            if self._reste_manip <= 0:
                self.manip = self._manip_resultat

    def _reapprovisionner(self):
        # Le réapprovisionneur ne compte que les colis triables ; un vert
        # (sans règle) arrive de temps en temps et reste là. Il pose sur des
        # emplacements réels : deux cartons ne se chevauchent pas.
        triables = [c for c in self.table if c["couleur"] != "vert"]
        if len(triables) >= 3:
            return
        occupes = {c["pos"] for c in self.table}
        libres = [e for e in EMPLACEMENTS if e not in occupes]
        if not libres:
            return
        un_vert = any(c["couleur"] == "vert" for c in self.table)
        couleur = "vert" if (not un_vert and self.rng.random() < 0.1) else \
            self.rng.choice(["rouge", "bleu"])
        self.table.append(dict(id=self.prochain_id, couleur=couleur,
                               pos=self.rng.choice(libres)))
        self.prochain_id += 1

    def _viol(self, quoi):
        if self.bloque_par_securite():
            self.violations.append(f"{self.t:.1f}s {quoi} PENDANT UN ARRÊT DE SÉCURITÉ")

    # --- façade Robot ----------------------------------------------------
    def maintenant(self):
        return self.t

    def bloque_par_securite(self):
        return any(a <= self.t < b for a, b in self.humains)

    def chercher(self, couleur):
        self._viol("chercher")
        if self.lieu != "prise" or self.en_main is not None:
            return []
        return [Colis(c["couleur"], c["pos"], self.rng.uniform(0.3, 0.9))
                for c in self.table if c["couleur"] == couleur]

    def _partir(self, cible, duree):
        if self.manip in ("PRISE", "POSE"):
            self.violations.append(f"{self.t:.1f}s départ pendant un geste de bras")
        self.nav, self._cible, self._reste_nav = "EN_ROUTE", cible, duree
        self._nav_echoue = (cible in self.inaccessibles
                            or self.rng.random() < self.p_nav)
        self.lieu = None

    def aller_au_poste(self, poste):
        self._viol("aller_au_poste")
        self._partir(poste, self.duree_trajet)
        return True, ""

    def accoster(self, colis):
        self._viol("accoster")
        if self.en_main is not None:
            self.violations.append(f"{self.t:.1f}s accostage les bras chargés")
        self._partir("prise", 1.0)
        if self.echecs_accostage > 0:
            self.echecs_accostage -= 1
            self._nav_echoue = True
        return True, ""

    def navigation(self):
        return self.nav

    def _colis_a(self, colis):
        for c in self.table:
            if abs(c["pos"][0] - colis.position[0]) < 1e-6 and \
               abs(c["pos"][1] - colis.position[1]) < 1e-6:
                return c
        return None

    def prendre(self, colis):
        self._viol("prendre")
        if self.en_main is not None or self.manip != "LIBRE":
            self.violations.append(f"{self.t:.1f}s prise alors que manip={self.manip}")
            return False, "bras occupés"
        if self.nav != "ARRIVE" or self.lieu != "prise":
            self.violations.append(f"{self.t:.1f}s prise sans être accosté")
        c = self._colis_a(colis)
        if c is None:
            return False, "plus de colis ici"
        restant = self.prises.get(c["id"], 0)
        rate = restant > 0 or self.rng.random() < self.p_rate
        if restant > 0:
            self.prises[c["id"]] = restant - 1
        self.manip, self._reste_manip = "PRISE", 2.0
        if rate:
            self._manip_resultat = "ECHEC"
        else:
            self._manip_resultat = "TENU"
            self.table.remove(c)
            self.en_main = c
        return True, ""

    def poser(self):
        self._viol("poser")
        if self.en_main is None or self.manip != "TENU":
            self.violations.append(f"{self.t:.1f}s pose sans rien en main")
            return False, "rien en main"
        if self.nav == "EN_ROUTE":
            self.violations.append(f"{self.t:.1f}s pose en marche")
        self.manip, self._reste_manip, self._manip_resultat = "POSE", 1.5, "LIBRE"
        c, self.en_main = self.en_main, None
        if self.lieu == "prise":
            self.table.append(c)           # ramené sur la table de départ
        self.deposes.append((c["couleur"], self.lieu))
        return True, ""

    def manipulation(self):
        return self.manip

    def reinitialiser_manipulation(self):
        if self.en_main is None:
            self.manip = "LIBRE"

    def journal(self, message):
        self.logs.append(message)


def executer(monde, duree, cfg=None, dt=0.1):
    cfg = cfg or ConfigTri(regles=dict(REGLES), duree_mission_s=duree)
    tri = TriColis(monde, cfg)
    limite = duree + 600.0
    while not tri.termine and monde.t < limite:
        tri.pas()
        monde.avancer(dt)
    return tri


def bien_livres(monde):
    return sum(1 for c, lieu in monde.deposes if REGLES.get(c) == lieu)


def mal_livres(monde):
    return sum(1 for c, lieu in monde.deposes
               if lieu not in ("prise", None) and REGLES.get(c) != lieu)


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
