"""
Suivi d'une personne (« suis-moi ») : le comportement dans l'OS
(axis_os/services/people/follow.py) et les API du SDK (people,
person_in_front, follow, follow_status, stop_follow), en direct sur le robot
simulé léger. Le même parcours depuis le processus de l'app est dans
test/test_suivi_personne_bac_a_sable.py.

Chaque règle a son essai : distance, vitesse, jamais en arrière, personne
qui s'arrête, perdue, cachée un instant, autre personne qui passe (habillée
autrement, puis pareil), obstacle de la carte, arrêt d'urgence, homme mort,
demande d'arrêt, fin du hook ; permissions, arguments invalides, capacité
absente, écrivains concurrents ; et ce qui reste d'une personne après le
suivi : rien.
"""
import builtins
import contextlib
import io
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.navigation.service import Carte, Rectangle, Suiveur  # noqa: E402
from axis_os.services.people.follow import FollowService  # noqa: E402
from axis_os.store.sdk.app_base import EstopActiveError, PermissionError, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.results import InvalidArgumentError, PersonInfo  # noqa: E402
from axis_os.store.sdk.stream import BusyError  # noqa: E402
from axis_os.testing import SimulatedRobot  # noqa: E402

FAIL = 0
ECHELLE = 2.0                 # le temps du robot passe deux fois plus vite


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def silencieux(f, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return f(*a, **kw)


ROUGE, BLEU, VERT, NOIR = (200, 30, 30), (30, 50, 200), (30, 160, 40), (30, 30, 30)
TOUT = [Permission.PEOPLE_DETECT, Permission.PEOPLE_FOLLOW, Permission.LOCOMOTION_WALK,
        Permission.ACTUATORS_WRITE_LEGS, Permission.NAVIGATION_GOTO, Permission.TELEOP_STREAM,
        Permission.MANIPULATION, Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS]


def contexte(robot, perms=None, app_id="com.test.suivi", services=True):
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, AppManifest(
        app_id=app_id, name="Suivi", version="1.0.0", author="T", author_email="t@t.com",
        description="d", category=AppCategory.UTILITIES,
        permissions=list(TOUT if perms is None else perms)))
    ctx._set_clock(robot.clock)
    if services:
        ctx._attach_services(locomotion=robot.locomotion, navigation=robot.navigation,
                             manipulation=robot.manipulation, perception=robot.perception,
                             people=robot.people_service, follow=robot.follow_service)
    return ctx


class Consignes:
    """Enregistre chaque consigne de marche envoyée à la locomotion."""

    def __init__(self, robot):
        self.robot, self.liste, self._origine = robot, [], robot.locomotion.set_velocity
        robot.locomotion.set_velocity = self._envoyer

    def _envoyer(self, vx=0.0, vy=0.0, wz=0.0):
        self.liste.append((self.robot.clock(), vx, vy, wz))
        return self._origine(vx, vy, wz)

    def apres(self, t):
        return [c for c in self.liste if c[0] > t]


def distance(robot, p):
    return math.hypot(p.x - robot.x, p.y - robot.y)


def suivre(ctx, robot, duree, p=None, battement=True, jusqua_fin=True):
    """Fait battre l'homme mort (follow_status) pendant `duree` s du robot ;
    rend (états vus, distance vraie minimale à `p`, dernier résultat)."""
    fin, phases, d_min, r = robot.clock() + duree, [], float("inf"), None
    while robot.clock() < fin:
        if battement:
            r = ctx.follow_status()
            phases.append(r.data.get("phase") or r.status)
            if jusqua_fin and r.status != "FOLLOWING":
                break
        if p is not None:
            d_min = min(d_min, distance(robot, p))
        ctx.sleep(0.1)
    return phases, d_min, r


def arrete_ensuite(cons, robot, ctx):
    """Vrai si aucune consigne non nulle n'est envoyée dans la demi-seconde
    qui suit."""
    t = robot.clock()
    ctx.sleep(0.5)
    return all(c[1] == 0.0 and c[3] == 0.0 for c in cons.apres(t))


def immobile_depuis(cons, robot, secondes):
    t = robot.clock() - secondes
    return all(c[1] == 0.0 and c[3] == 0.0 for c in cons.apres(t))


section("1. La personne marche puis s'arrête : le robot suit, puis s'arrête à la distance cible")
with SimulatedRobot(time_scale=ECHELLE) as robot:
    p = robot.add_person(2.5, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    cons = Consignes(robot)
    devant = ctx.person_in_front()
    check("person_in_front : la personne devant, à 2.5 m", isinstance(devant, PersonInfo)
          and abs(devant.distance_m - 2.5) < 0.2 and abs(devant.bearing_rad) < 0.1, str(devant))
    check("people() : un numéro, une distance, une direction, une durée — rien d'autre",
          set(vars(ctx.people()[0])) == {"person_id", "distance_m", "bearing_rad",
                                         "seen_for_s", "confidence"})
    r = ctx.follow(devant.person_id)
    check("follow : FOLLOWING", r.ok and r.status == "FOLLOWING", str(r))
    p.walk_to(4.5, 0.0, speed_mps=0.25)
    phases, d_min, r = suivre(ctx, robot, 22.0, p)
    d_fin = distance(robot, p)
    check("Toujours FOLLOWING tant que la personne est vue", r.status == "FOLLOWING", str(r))
    check(f"Arrêté à la distance cible : {d_fin:.2f} m (cible 1.5 m)", 1.45 <= d_fin <= 1.85)
    check(f"Jamais plus près que la cible moins 10 cm (minimum {d_min:.2f} m)", d_min >= 1.4)
    check("… et le robot ne bouge plus (personne arrêtée)", immobile_depuis(cons, robot, 2.0))
    vmax = max(c[1] for c in cons.liste)
    check(f"Vitesse maximale {vmax:.2f} m/s ≤ {FollowService.MAX_SPEED} m/s, "
          f"sous celle de la navigation ({Suiveur().v_max} m/s)",
          vmax <= FollowService.MAX_SPEED < Suiveur().v_max)
    check("Jamais en arrière, jamais de côté (vx ≥ 0, vy = 0 à chaque consigne)",
          all(c[1] >= 0.0 and c[2] == 0.0 for c in cons.liste))
    check("Rotation bornée", all(abs(c[3]) <= FollowService.MAX_TURN for c in cons.liste))
    r = ctx.stop_follow()
    check("stop_follow : STOPPED_BY_APP, le robot a marché (moved)",
          r.ok and r.status == "STOPPED_BY_APP" and r.data.get("moved") is True, str(r))
    check("Après le suivi : apparence de référence effacée, plus aucune piste",
          robot.follow_service._reference is None and len(robot.people_service._tracker) == 0)
    check("stop_follow une seconde fois : la même issue", ctx.stop_follow().status == "STOPPED_BY_APP")

section("2. La personne vient vers le robot : il ne recule pas, le gardien l'arrête")
with SimulatedRobot(time_scale=ECHELLE) as robot:
    p = robot.add_person(2.2, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    cons = Consignes(robot)
    r = ctx.follow(ctx.person_in_front(5.0).person_id)
    t0 = robot.clock()
    p.walk_to(0.9, 0.0, speed_mps=0.4)
    suivre(ctx, robot, 6.0, p)
    check("Aucune consigne en arrière", all(c[1] >= 0.0 for c in cons.liste))
    check("Le robot n'a pas avancé vers elle quand elle était sous la distance cible",
          robot.x < 0.75, f"x = {robot.x:.2f}")
    v = robot.locomotion.dernier_verdict
    check("Le gardien de proximité tient le robot à l'arrêt (priorité au gardien)",
          v is not None and v.facteur == 0.0, str(v))
    silencieux(ctx.stop_follow)

section("3. Distance trop faible, arguments invalides : refusés")
with SimulatedRobot(time_scale=ECHELLE) as robot:
    robot.add_person(2.5, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    pid = ctx.person_in_front(5.0).person_id
    dmin = robot.follow_service.distance_min
    arret = robot.guard.distance_arret()
    check(f"Distance minimale {dmin} m : au-delà de la zone où le gardien arrête le robot "
          f"({arret:.2f} m), plus une marge", dmin >= arret + FollowService.MARGE_GARDIEN_M - 1e-9)
    check("… et la distance par défaut (1.5 m) l'est aussi", FollowService.DISTANCE_DEFAULT >= dmin)

    def invalide(**kw):
        args = dict(person_id=pid)
        args.update(kw)
        pid_ = args.pop("person_id")
        try:
            ctx.follow(pid_, **args)
            return False
        except InvalidArgumentError:
            return True
    try:
        ctx.follow(pid, distance_m=1.0)
        msg = ""
    except InvalidArgumentError as exc:
        msg = str(exc)
    check("distance_m = 1.0 : INVALID_ARGUMENT, qui explique le gardien", "gardien" in msg, msg)
    check(f"distance_m = {dmin - 0.01:.2f} (juste sous le minimum) : refusée",
          invalide(distance_m=dmin - 0.01))
    for nom, kw in [("distance NaN", dict(distance_m=float("nan"))),
                    ("distance infinie", dict(distance_m=float("inf"))),
                    ("distance 3.5 m (> 3)", dict(distance_m=3.5)),
                    ("distance texte", dict(distance_m="1.5")),
                    ("distance booléen", dict(distance_m=True)),
                    ("numéro booléen", dict(person_id=True)),
                    ("numéro 0", dict(person_id=0)),
                    ("numéro texte", dict(person_id="1")),
                    ("numéro flottant", dict(person_id=1.0)),
                    ("délai 0", dict(timeout_s=0)),
                    ("délai NaN", dict(timeout_s=float("nan"))),
                    ("homme mort 0.1 s", dict(deadman_s=0.1)),
                    ("homme mort 5 s", dict(deadman_s=5.0))]:
        check(f"{nom} : INVALID_ARGUMENT", invalide(**kw))
    for v in (0.0, -1.0, float("nan"), 9.0, "3"):
        try:
            ctx.person_in_front(v)
            ok = False
        except InvalidArgumentError:
            ok = True
        check(f"person_in_front({v!r}) : INVALID_ARGUMENT", ok)
    r = ctx.follow(9999)
    check("Numéro inconnu : REFUSED / TARGET_LOST", not r.ok and r.code == "TARGET_LOST", str(r))
    check("… rien n'a bougé", not robot.follow_service.active)

section("4. Personne perdue derrière un mur : arrêt, recherche bornée, puis LOST")
carte = Carte(-5.0, -5.0, 8.0, 8.0, obstacles=[Rectangle(2.2, 0.7, 2.6, 4.0)])
with SimulatedRobot(time_scale=ECHELLE, carte=carte) as robot:
    p = robot.add_person(3.2, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    cons = Consignes(robot)
    r = ctx.follow(ctx.person_in_front(5.0).person_id)
    p.walk_to(3.2, 3.0, speed_mps=0.5)
    phases, _, r = suivre(ctx, robot, 25.0)
    vu = robot.people_service.metrics
    check("Le suivi passe par « waiting » puis « searching »",
          "waiting" in phases and "searching" in phases, str(sorted(set(phases))))
    tours = [c for c in cons.liste if c[1] == 0.0 and c[3] != 0.0]
    check("La recherche tourne sur place (vx = 0, rotation)", len(tours) > 0)
    check("Puis LOST (TARGET_LOST)", r.status == "LOST" and r.code == "TARGET_LOST", str(r))
    check("Le robot est arrêté après LOST", arrete_ensuite(cons, robot, ctx))
    check("Après LOST : plus aucune piste ni apparence en mémoire",
          len(robot.people_service._tracker) == 0 and robot.follow_service._reference is None)
    check(f"Recherche bornée : LOST au plus {FollowService.SEARCH_S:g} s après la dernière vue",
          sum(1 for ph in phases if ph in ("waiting", "searching")) * 0.1
          <= FollowService.SEARCH_S + 1.5)

section("5. Cachée un instant par un poteau : reprise, même personne")
carte = Carte(-5.0, -5.0, 8.0, 8.0, obstacles=[Rectangle(2.0, -0.3, 2.3, 0.3)])
with SimulatedRobot(time_scale=ECHELLE, carte=carte) as robot:
    p = robot.add_person(3.5, -1.2, torso=ROUGE)
    ctx = contexte(robot)
    cons = Consignes(robot)
    vise = ctx.people()[0]
    r = ctx.follow(vise.person_id, distance_m=2.0)
    p.walk_to(3.5, 1.4, speed_mps=0.5)
    phases, _, r = suivre(ctx, robot, 9.0)
    check("Toujours FOLLOWING, même numéro", r.status == "FOLLOWING"
          and r.data.get("person_id") == vise.person_id, str(r))
    check("… après une reprise signalée à l'app (reacquisitions ≥ 1)",
          r.data.get("reacquisitions", 0) >= 1, str(r))
    check("Le robot n'est jamais entré dans le poteau élargi de la carte",
          robot.navigation.planif.libre(robot.x, robot.y) or robot.x < 1.6, f"x = {robot.x:.2f}")
    silencieux(ctx.stop_follow)

section("6. Une autre personne passe tout près : immobile, et jamais la mauvaise")
for tenue, attendu in ((BLEU, "FOLLOWING"), (ROUGE, "LOST")):
    with SimulatedRobot(time_scale=ECHELLE) as robot:
        p = robot.add_person(3.0, 0.0, torso=ROUGE)
        ctx = contexte(robot)
        cons = Consignes(robot)
        vise = ctx.person_in_front(5.0)
        r = ctx.follow(vise.person_id, distance_m=2.0)
        suivre(ctx, robot, 2.0)
        autre = robot.add_person(3.4, -2.0, torso=tenue)
        autre.walk_to(3.4, 2.0, speed_mps=0.8)
        phases, _, r = suivre(ctx, robot, 9.0)
        nom = "bleue" if tenue == BLEU else "habillée pareil"
        check(f"Personne {nom} qui passe : le robot s'immobilise (« holding »)",
              "holding" in phases, str(sorted(set(phases))))
        if attendu == "FOLLOWING":
            check("… puis reprend le suivi de la MÊME personne", r.status == "FOLLOWING"
                  and r.data.get("person_id") == vise.person_id, str(r))
        else:
            check("… puis rend LOST (identité incertaine) au lieu de choisir",
                  r.status == "LOST" and "identité" in r.message, str(r))
            check("… et le robot est arrêté", arrete_ensuite(cons, robot, ctx))
        silencieux(ctx.stop_follow)

section("7. Obstacle de la carte entre le robot et la personne : il n'y entre pas")
carte = Carte(-5.0, -5.0, 8.0, 8.0, obstacles=[Rectangle(1.2, -0.5, 1.5, 0.5, zmax=0.8)])
with SimulatedRobot(time_scale=ECHELLE, carte=carte) as robot:
    p = robot.add_person(3.6, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    r = ctx.follow(ctx.person_in_front(5.0).person_id)
    suivre(ctx, robot, 8.0)
    check("Le robot s'arrête avant la table (que le gardien ignore, car connue)",
          robot.x < 1.2 - 0.30 + 0.05 and robot.follow_service.stats["map_stops"] > 0,
          f"x = {robot.x:.2f}, {robot.follow_service.stats}")
    silencieux(ctx.stop_follow)

section("8. Arrêt d'urgence, homme mort, demande d'arrêt, fin du hook : arrêt immédiat")
with SimulatedRobot(time_scale=ECHELLE) as robot:
    p = robot.add_person(3.0, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    cons = Consignes(robot)
    r = ctx.follow(ctx.person_in_front(5.0).person_id)
    p.walk_to(5.0, 0.0, speed_mps=0.3)
    suivre(ctx, robot, 2.0)
    t_arret = robot.clock()
    silencieux(robot.hal.emergency_stop_all)
    ctx.sleep(0.2)
    r = ctx.follow_status()
    check("Arrêt d'urgence : ESTOP (ESTOP_ACTIVE)", r.status == "ESTOP" and r.code == "ESTOP_ACTIVE",
          str(r))
    check("… plus aucune consigne de marche non nulle après l'arrêt d'urgence",
          all(c[1] == 0.0 and c[3] == 0.0 for c in cons.apres(t_arret + 0.05)))
    try:
        ctx.follow(1)
        leve = False
    except EstopActiveError:
        leve = True
    check("… follow() pendant l'arrêt d'urgence : EstopActiveError", leve)
    check("… stop_follow reste servi", ctx.stop_follow().status == "ESTOP")

for nom, provoquer, statut, code in (
        ("Homme mort (l'app ne donne plus de nouvelles)", None, "DEADMAN", "DEADMAN"),
        ("request_stop (un humain demande l'arrêt)", lambda c: c._demander_arret("human"),
         "CANCELLED", "CANCELLED"),
        ("Fin du hook de l'app (ou sa mort)", lambda c: c._fermer(), "CANCELLED", "CANCELLED")):
    with SimulatedRobot(time_scale=ECHELLE) as robot:
        p = robot.add_person(3.0, 0.0, torso=ROUGE)
        ctx = contexte(robot)
        cons = Consignes(robot)
        ctx.follow(ctx.person_in_front(5.0).person_id, deadman_s=1.0)
        p.walk_to(5.0, 0.0, speed_mps=0.3)
        suivre(ctx, robot, 2.0)
        t_arret = robot.clock()
        if provoquer is None:
            ctx.sleep(1.5)                     # plus de follow_status
        else:
            provoquer(ctx)
            ctx.sleep(0.2)
        r = robot.follow_service.status(ctx)
        check(f"{nom} : {statut} ({code})", r.get("status") == statut and r.get("code") == code,
              str(r))
        limite = t_arret + (1.0 + 0.1 if provoquer is None else 0.1)
        check("… le robot est arrêté aussitôt", all(c[1] == 0.0 and c[3] == 0.0
                                                   for c in cons.apres(limite)))

section("9. Permissions : chacune exigée, à chaque appel")
with SimulatedRobot(time_scale=ECHELLE) as robot:
    robot.add_person(2.5, 0.0, torso=ROUGE)
    pid = contexte(robot).person_in_front().person_id
    for manque in (Permission.PEOPLE_FOLLOW, Permission.PEOPLE_DETECT, Permission.LOCOMOTION_WALK,
                   Permission.ACTUATORS_WRITE_LEGS):
        ctx = contexte(robot, [p for p in TOUT if p is not manque])
        try:
            ctx.follow(pid)
            ok = False
        except PermissionError:
            ok = True
        check(f"follow sans {manque.name} : PermissionError", ok)
    ctx = contexte(robot, [p for p in TOUT if p is not Permission.PEOPLE_DETECT])
    for f in (ctx.people, ctx.person_in_front):
        try:
            f()
            ok = False
        except PermissionError:
            ok = True
        check(f"{f.__name__} sans PEOPLE_DETECT : PermissionError", ok)
    ctx = contexte(robot, [p for p in TOUT if p is not Permission.PEOPLE_FOLLOW])
    for f in (ctx.follow_status, ctx.stop_follow):
        try:
            f()
            ok = False
        except PermissionError:
            ok = True
        check(f"{f.__name__} sans PEOPLE_FOLLOW : PermissionError", ok)
    check("Rien n'a été suivi", robot.follow_service.stats["follows"] == 0)

section("10. Capacité absente : CAPABILITY_MISSING, jamais un suivi sans gardien")
with SimulatedRobot(time_scale=ECHELLE) as robot:
    ctx = contexte(robot, services=False)
    ctx._attach_services(locomotion=robot.locomotion)
    check("Sans service de personnes : people() = [], person_in_front() = None",
          silencieux(ctx.people) == [] and silencieux(ctx.person_in_front) is None)
    r = ctx.follow(1)
    check("… follow : CAPABILITY_MISSING", r.code == "CAPABILITY_MISSING", str(r))
    check("… follow_status, stop_follow : CAPABILITY_MISSING",
          ctx.follow_status().code == "CAPABILITY_MISSING"
          and ctx.stop_follow().code == "CAPABILITY_MISSING")
with SimulatedRobot(time_scale=ECHELLE, proximity_guard=False) as robot:
    robot.add_person(2.5, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    r = ctx.follow(ctx.person_in_front(5.0).person_id)
    check("Robot sans gardien de proximité : suivi refusé (CAPABILITY_MISSING)",
          r.code == "CAPABILITY_MISSING" and "gardien" in r.message, str(r))

section("11. Un seul écrivain de la marche")
with SimulatedRobot(time_scale=ECHELLE) as robot:
    robot.add_person(3.0, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    pid = ctx.person_in_front(5.0).person_id
    ctx.follow(pid)
    check("walk() pendant le suivi : refusé (False)", silencieux(ctx.walk, 0.3) is False)
    r = ctx.go_to(1.0, 0.0, timeout_s=5.0)
    check("go_to pendant le suivi : BUSY", r.code == "BUSY", str(r))
    try:
        ctx.stream_targets("walk")
        ok = False
    except BusyError:
        ok = True
    check("stream_targets(\"walk\") pendant le suivi : BusyError", ok)
    from axis_os.store.sdk.results import ObjectSpec
    r = ctx.pick(ObjectSpec((3.0, 0.0, 0.9), (0.2, 0.2, 0.2)))
    check("pick pendant le suivi : BUSY (aucune exemption du gardien)", r.code == "BUSY", str(r))
    autre = contexte(robot, app_id="com.test.autre")
    r = autre.follow(pid)
    check("Une autre app : BUSY (le robot est déjà conduit)", r.code == "BUSY", str(r))
    check("… elle ne peut pas arrêter ce suivi par stop_follow (IDLE)",
          autre.stop_follow().status == "IDLE" and robot.follow_service.active)
    ctx.follow_status()
    autre_marche = contexte(robot, [Permission.LOCOMOTION_WALK], app_id="com.test.frein")
    silencieux(autre_marche.stop_walking)
    check("stop_walking() (de n'importe quelle app) arrête le suivi : STOPPED_BY_APP",
          not robot.follow_service.active
          and robot.follow_service.status(ctx).get("status") == "STOPPED_BY_APP")

section("12. Pendant une prise : pas de suivi")
from axis_os.services.manipulation.service import Etat  # noqa: E402
with SimulatedRobot(time_scale=ECHELLE) as robot:
    robot.add_person(3.0, 0.0, torso=ROUGE)
    ctx = contexte(robot)
    pid = ctx.person_in_front(5.0).person_id
    robot.manipulation.etat = Etat.TENU
    r = ctx.follow(pid)
    check("Objet tenu (exemption possible du gardien) : follow BUSY", r.code == "BUSY", str(r))
    robot.manipulation.etat = Etat.LIBRE

section("13. L'humain sait, à l'installation, que l'app détecte les personnes")
from axis_os.store.runtime.play_store import confirmer_au_terminal  # noqa: E402
from axis_os.store.sdk.manifest import PERMISSION_NOTICES, ProtectionLevel, get_protection_level  # noqa: E402
check("PEOPLE_DETECT est NORMAL, PEOPLE_FOLLOW DANGEROUS",
      get_protection_level(Permission.PEOPLE_DETECT) is ProtectionLevel.NORMAL
      and get_protection_level(Permission.PEOPLE_FOLLOW) is ProtectionLevel.DANGEROUS)
m = AppManifest(app_id="com.test.voir", name="Voir", version="1.0.0", author="T",
                author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                permissions=[Permission.PEOPLE_DETECT, Permission.PEOPLE_FOLLOW,
                             Permission.LOCOMOTION_WALK, Permission.ACTUATORS_WRITE_LEGS])
sortie = io.StringIO()
entree = builtins.input
builtins.input = lambda q="": "n"
try:
    with contextlib.redirect_stdout(sortie):
        confirmer_au_terminal(m, [Permission.PEOPLE_FOLLOW])
finally:
    builtins.input = entree
check("La question de confirmation montre « cette app détecte la présence de personnes »",
      "détecte la présence de personnes" in sortie.getvalue())
check("… et que le robot peut marcher derrière une personne",
      PERMISSION_NOTICES[Permission.PEOPLE_FOLLOW] in sortie.getvalue())

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
