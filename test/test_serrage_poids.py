"""
Serrer d'après le poids, et garder ce serrage pendant le port.

Constat GPU (essai T1 du 5 novembre 2026, 21 h 57 ; test_rejeu_port_t2157.py) :
prise parfaite, puis le carton glisse au port. Serré à 6,5 N par main pour
10,9 N de poids : aucune marge de frottement ; une traction vers un torse hors
d'atteinte ; la consigne d'une main desserrée par un plafond, jamais rendue.

Désormais (prise.py, service.py) :
  - en haut du levage, le serrage requis d'après le poids mesuré par l'OS,
    securite_serrage × poids / (2 frottement_serrage), est atteint avant de
    ramener (visé avec marge_serrage_n de plus), plafonds de couple
    inchangés ; impossible dans les plafonds : le carton est reposé, « trop
    lourd pour ces mains » ;
  - torse hors d'atteinte : rapproché en position, sans effort vers le robot ;
  - au port, la consigne garde le serrage ; mains qui s'écartent, ou marge de
    frottement sous le coefficient de sécurité : base arrêtée, on resserre,
    sinon à reposer.

Robot simulé léger, horloge simulée, mains du G1 (contact 11 cm le long de
la main) ; masse du carton changée sur le robot simulé (vérité du monde).
"""
import contextlib
import io
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Banc:
    def __init__(self, masse=1.0, objets=True, x=0.41, contact=0.11):
        self.t = [0.0]
        self.x = x
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (x, 0.0, 0.90))]
                                if objets else [], horloge=lambda: self.t[0],
                                contact_main_m=contact)
        self.r.MASSE_OBJET_KG = masse
        self.r.start(fil=False)
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append
        self.etats, self.sortie = [], io.StringIO()
        self.depasse = 0.0                  # pire couple au-delà de son plafond (N·m)
        self.pas(20)

    def pas(self, n=1):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()
                if not self.etats or self.etats[-1] is not self.m.etat:
                    self.etats.append(self.m.etat)
                tau, _ = self.m._efforts()
                if tau:
                    self.depasse = max([self.depasse] + [abs(v) - self.m._limite_effort(j)
                                                         for j, v in tau.items()])

    def prendre(self, jusqua=(Etat.TENU, Etat.ECHEC), duree=40.0):
        self.m.prendre(Objet((self.x, 0.0, 0.90), 0.24, 0.20, 0.20))
        t0 = self.t[0]
        while self.m.etat not in jusqua and self.t[0] - t0 < duree:
            self.pas()
        return self

    def ligne(self, debut):
        return next((l for l in self.journal if l.startswith(debut)), "")

    def refus_garde_fou(self):
        return self.sortie.getvalue().count("SAFETY VIOLATION")


section("1. Carton de 1 kg : serrage d'après le poids, atteint avant de ramener")
b = Banc().prendre()
m, r = b.m, b.r
poids = m._poids_ref or 0.0
check("Carton pris et tenu", m.etat is Etat.TENU, m.raison)
check(f"Serrage requis {m.serrage_requis:.1f} N par main = {R.securite_serrage:.0f} × "
      f"{poids:.1f} N / (2 × {R.frottement_serrage}), visé {m.serrage_vise:.1f}",
      abs(poids - 9.81) <= 1.5 and abs(m.serrage_requis - R.securite_serrage * poids
                                       / (2 * R.frottement_serrage)) < 0.3)
check(f"… atteint en haut du levage, au journal : « {b.ligne('serrage d')} »",
      b.ligne("serrage d'après le poids : G") != ""
      and min(m.force_serrage.values()) >= m.serrage_requis, str(m.force_serrage))
check(f"… vraie force à chaque main (monde simulé) : {min(r.forces_contact.values()):.1f} N, au "
      f"moins le requis", min(r.forces_contact.values()) >= m.serrage_requis,
      str(r.forces_contact))
check(f"Plafonds de couple inchangés et tenus : au plus {max(0.0, b.depasse):.2f} N·m au-delà, "
      f"aucune consigne refusée par le garde-fou",
      b.depasse <= 0.1 and b.refus_garde_fou() == 0, f"{b.depasse} {b.refus_garde_fou()}")
check(f"Consigne au plus {100 * R.penetration_poids_max_m:.0f} cm dans la face touchée",
      all(m._delta[c] <= m._delta_max_poids[c] + 1e-9 for c in m._delta))
check("Torse hors d'atteinte (mains du G1) : rapproché en position, sans effort vers le robot "
      "(au journal)", "sans effort vers le robot, tenu par les mains seules (torse hors d'atteinte)"
      in b.ligne("carton ramené sans toucher le torse"), b.journal[-3:])
b.pas(250)
check(f"Cinq secondes de port : tenu, marge de frottement {m.marge_frottement or 0.0:.2f} (au "
      f"moins {R.securite_serrage:.0f}), rien de signalé", m.etat is Etat.TENU
      and not m.a_reposer and not m.penche and m.marge_frottement is not None
      and m.marge_frottement >= R.securite_serrage and r.held is not None,
      f"{m.marge_frottement} {m.raison}")
avant = dict(m._delta)
r.object_tilt_deg = 25.0
b.pas(30)
check("Carton penché : la correction d'inclinaison (inchangée) ne desserre pas le serrage "
      "d'après le poids (sa borne le suit)", m.stats.get("corrections_inclinaison", 0) >= 1
      and all(m._delta[c] >= avant[c] - 1e-9 for c in avant), f"{avant} {m._delta}")

section("2. Trop lourd pour ces mains : reposé doucement, signalé")
h = Banc(masse=1.6).prendre(jusqua=(Etat.TENU, Etat.ECHEC))
o = h.r.objects[0]
check("Carton de 1,6 kg : le serrage requis dépasse ce que les poignets donnent dans leurs "
      "plafonds : « trop lourd pour ces mains », jamais tenu pour marcher",
      h.m.etat is Etat.ECHEC and "trop lourd pour ces mains" in h.m.raison
      and Etat.TENU not in h.etats and Etat.RAMENER not in h.etats, f"{h.m.raison} {h.etats}")
check("… reposé doucement : sur la table, à sa place, mains ouvertes, gains rendus",
      Etat.REPOSE in h.etats and h.r.held is None and abs(o.position[2] - 0.90) < 0.03
      and abs(o.position[0] - h.x) < 0.03 and not h.r.raideur_active,
      f"{o.position} {h.r.raideur_active}")
check(f"… plafonds tenus pendant l'essai (au plus {max(0.0, h.depasse):.2f} N·m au-delà)",
      h.depasse <= 0.1 and h.refus_garde_fou() == 0)

section("3. Torse hors d'atteinte : pas d'arrêt sur une traction vers le robot")
t = Banc()
charge = t.m._charge


def tirer(tau):
    """Les mains tirent de 5 N vers le robot pendant qu'on ramène (traction
    de l'essai GPU : 3 N)."""
    v = charge(tau)
    if v is not None and t.m.etat is Etat.RAMENER:
        t.m.charge = (v[0], 5.0)
    return t.m.charge if v is not None else None


t.m._charge = tirer
t.prendre()
check("5 N de traction pendant le retour, torse hors d'atteinte : jamais « contre le torse », "
      "le carton rapproché jusqu'à sa cible, tenu",
      t.m.etat is Etat.TENU and not t.ligne("carton contre le torse")
      and t.ligne("carton ramené sans toucher le torse"), f"{t.m.raison} {t.journal[-3:]}")

section("4. Au port, la consigne garde le serrage")
c = Banc(masse=0.6).prendre()
c.pas(30)
ref = dict(c.m._delta)
check("Carton de 0,6 kg tenu", c.m.etat is Etat.TENU, c.m.raison)
LACET = "left_wrist_yaw_joint"
c.m.mesures.couple_max[LACET] = 1.6              # plafond 0,8 N·m : pointe de couple
c.pas(4)
desserre = ref["left"] - c.m._delta["left"]
c.m.mesures.couple_max[LACET] = 5.0              # plafond rendu
c.pas(100)
check(f"Pointe au-delà d'un plafond : main G desserrée de {1000 * desserre:.1f} mm "
      f"(_garder_sous_la_limite, inchangé), puis la consigne est rendue "
      f"({1000 * (ref['left'] - c.m._delta['left']):+.1f} mm) ; tenu, rien de signalé",
      desserre >= 0.002 and abs(ref["left"] - c.m._delta["left"]) <= 0.0006
      and c.m.etat is Etat.TENU and not c.m.a_reposer and not c.m.penche,
      f"{desserre} {c.m._delta} {ref} {c.m.raison}")
e = Banc(x=0.30, contact=0.0).prendre()        # mains idéales : du jeu aux poignets
e.pas(30)
marge0 = e.m.marge_frottement or 0.0
e.m.r = replace(e.m.r, securite_serrage=marge0 + 0.6)
penche = False
for _ in range(80):
    e.pas()
    penche = penche or e.m.penche
check(f"Marge de frottement {marge0:.2f}, coefficient de sécurité porté à {marge0 + 0.6:.2f} en "
      f"cours de port : base arrêtée, on resserre, marge retrouvée "
      f"({e.m.marge_frottement or 0.0:.2f}) : la marche reprend (au journal), plafonds tenus",
      penche and e.ligne("port : marge de frottement") and e.ligne("port : serrage retrouvé")
      and not e.m.penche and not e.m.a_reposer and e.m.etat is Etat.TENU
      and (e.m.marge_frottement or 0.0) >= marge0 + 0.6 and e.depasse <= 0.1,
      f"{e.m.marge_frottement} {e.journal[-3:]}")
d = Banc().prendre()
d.pas(30)
d.r.MASSE_OBJET_KG = 1.4                          # au-delà de ce que les poignets donnent
penche = False
for _ in range(150):
    d.pas()
    penche = penche or d.m.penche
check("Carton alourdi à 1,4 kg : base arrêtée, le resserrage n'y suffit pas dans les plafonds : "
      "« serrage perdu au port », à reposer (la façade arrête la base et le signale)",
      penche and d.m.a_reposer and d.m.raison.startswith("serrage perdu au port")
      and d.r.held is not None, f"{d.m.raison} {d.journal[-3:]}")
d.pas(50)
check("… l'alerte tient (pas de reprise de la marche), plafonds tenus",
      d.m.a_reposer and d.m.penche and d.depasse <= 0.1, f"{d.depasse}")

section("5. Rien entre les mains : toujours refusé")
v = Banc(objets=False).prendre()
check("Aucun carton : « rien entre les mains », jamais tenu", v.m.etat is Etat.ECHEC
      and "rien entre les mains" in v.m.raison and Etat.TENU not in v.etats, v.m.raison)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
