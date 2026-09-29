"""
Monde factice pour tester l'app de tri sans simulateur : implémente la
façade Robot, injecte des pannes et relève toute VIOLATION.
"""
import random

from axis_os.apps.tri_colis.logique import Colis, ConfigTri, TriColis

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
        self.tombe_si_rate = set()              # ids qui tombent quand la prise rate
        self.prises_dans_le_vide = 0
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
            self.prises_dans_le_vide += 1
            return False, "plus de colis ici"
        restant = self.prises.get(c["id"], 0)
        rate = restant > 0 or self.rng.random() < self.p_rate
        if restant > 0:
            self.prises[c["id"]] = restant - 1
        self.manip, self._reste_manip = "PRISE", 2.0
        if rate:
            self._manip_resultat = "ECHEC"
            if c["id"] in self.tombe_si_rate:
                # Constaté sur GPU : bousculé par la prise ratée, le carton
                # tombe de la table.
                self.table.remove(c)
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
