"""
Intentions vocales cloisonnées par app (SECURITY.md, ancienne limite n° 8).

Vérifié ici, avec deux apps qui partagent le même service de parole :
  1. une app ne reçoit que ses propres intentions : la phrase qui
     correspond à l'intention d'une autre app ne lui rend rien, et jamais
     les paramètres choisis par l'autre ;
  2. une app ne peut pas réserver le nom d'intention d'une autre : les
     deux déclarent le même nom, chacune reçoit le sien ;
  3. chaque app a son propre plafond : une app qui a rempli le sien
     n'empêche pas une autre de déclarer ;
  4. depuis le processus de l'app (pont JSON), même cloisonnement ;
  5. le vocabulaire d'une app arrêtée est retiré, et redéclaré au
     redémarrage.
"""
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.interfaces import IMicrophone  # noqa: E402
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.core.types import AudioChunk, HALStatus, SensorType  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.services.speech.service import (  # noqa: E402
    IntentRegistry, ISpeechRecognizer, SpeechService,
)
from axis_os.store.runtime.play_store import (  # noqa: E402
    PlayStoreRuntime, accepter_tout_SIMULATION,
)
from axis_os.store.runtime.profil import Profil  # noqa: E402
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402

FAIL = 0


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


class Micro(IMicrophone):
    def __init__(self):
        super().__init__("micro_test", SensorType.MICROPHONE)
        self._initialized = True

    def initialize(self): return HALStatus.OK
    def shutdown(self): return HALStatus.OK
    def is_healthy(self): return True

    def capture(self, duration_s):
        return AudioChunk(sample_rate=16000, channels=1, sample_width=2, data=b"\x00\x01" * 160)


class Transcription(ISpeechRecognizer):
    """Tient le rôle du modèle : « entend » toujours la même phrase."""
    def transcribe(self, audio): return "va vers le carton", 1.0

    @property
    def language(self): return "fr"


def parole():
    svc = SpeechService(Micro(), Transcription(), IntentRegistry())
    svc.start()
    return svc


with contextlib.redirect_stdout(io.StringIO()):
    ROBOT = NormalizedRobotHAL(UnitreeG1HAL("g1_intentions"), build_g1_normalizer())
    ROBOT.initialize_all()
WD = WatchdogSystem(ROBOT)


def contexte(app_id, svc):
    man = AppManifest(app_id=app_id, name=app_id, version="1.0.0", author="T",
                      author_email="t@t.com", description="d",
                      category=AppCategory.UTILITIES,
                      permissions=[Permission.SENSORS_MICROPHONE])
    ctx = RobotContext.for_testing(ROBOT, WD, man)
    ctx._attach_services(speech=svc)
    return ctx


MOTS = [["va", "aller"], ["carton", "colis"]]

section("1. Une app ne reçoit que ses propres intentions")
svc = parole()
a, b = contexte("com.test.a", svc), contexte("com.test.b", svc)
check("A déclare « aller_vers_colis »",
      silencieux(a.declare_intent, "aller_vers_colis", MOTS, {"cible": "choisie par A"}))
check("A reçoit son intention", getattr(silencieux(a.listen), "name", None) == "aller_vers_colis")
recu = silencieux(b.listen)
check("B, qui n'a rien déclaré, ne reçoit pas l'intention de A", recu is None,
      f"reçu : {recu}")

section("2. Une app ne peut pas réserver le nom d'une autre")
check("B déclare le même nom, avec ses propres paramètres : accepté",
      silencieux(b.declare_intent, "aller_vers_colis", MOTS, {"cible": "choisie par B"}))
recu_b, recu_a = silencieux(b.listen), silencieux(a.listen)
check("B reçoit SES paramètres, pas ceux choisis par A",
      recu_b is not None and recu_b.params.get("cible") == "choisie par B", str(recu_b))
check("A garde les siens", recu_a is not None and recu_a.params.get("cible") == "choisie par A",
      str(recu_a))
check("Un nom déjà déclaré par la MÊME app reste refusé",
      silencieux(a.declare_intent, "aller_vers_colis", MOTS) is False)

section("3. Un plafond par app")
svc = parole()
a, b = contexte("com.test.a", svc), contexte("com.test.b", svc)
remplies = sum(bool(silencieux(a.declare_intent, f"i{i}", [["mot"]]))
               for i in range(IntentRegistry.MAX_INTENTIONS))
check(f"A déclare {IntentRegistry.MAX_INTENTIONS} intentions", remplies == IntentRegistry.MAX_INTENTIONS)
check(f"A a rempli son plafond ({IntentRegistry.MAX_INTENTIONS}) : la suivante est refusée",
      silencieux(a.declare_intent, "une_de_trop", MOTS) is False)
check("B peut encore déclarer : A n'a pas rempli le registre pour les autres",
      silencieux(b.declare_intent, "aller_vers_colis", MOTS))


class AppA(RobotApp):
    """Déclare une intention dont les paramètres ne regardent qu'elle."""

    def manifest(self):
        return AppManifest(app_id="com.test.voix_a", name="A", version="1.0.0", author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.SENSORS_MICROPHONE])

    def on_start(self, ctx):
        return ctx.declare_intent("aller_vers_colis", [["va"], ["carton"]],
                                  {"cible": "secret de A"})

    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        intention = ctx.listen(0.1)
        return MissionResult(success=intention is not None, mission_id="", app_id="",
                             duration_s=0.0,
                             metrics={"intention": getattr(intention, "name", None)})


class AppB(RobotApp):
    """Écoute sans rien déclarer, puis tente de prendre le nom de A."""

    def manifest(self):
        return AppManifest(app_id="com.test.voix_b", name="B", version="1.0.0", author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.SENSORS_MICROPHONE])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        if params.get("declarer"):
            declare = ctx.declare_intent("aller_vers_colis", [["va"], ["carton"]],
                                         {"cible": "à B"})
        else:
            declare = None
        intention = ctx.listen(0.1)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics={"declare": declare,
                                      "intention": getattr(intention, "name", None),
                                      "params": dict(getattr(intention, "params", {}) or {})})


section("4. Depuis le processus de l'app (pont JSON)")
svc = parole()
rt = silencieux(PlayStoreRuntime, ROBOT, WD, speech_service=svc, profil=Profil.SIMULATION,
                confirmer_permissions=accepter_tout_SIMULATION)
for app in (AppA(), AppB()):
    silencieux(rt.install, app)
check("A démarre et déclare son intention", silencieux(rt.start_app, "com.test.voix_a"))
silencieux(rt.start_app, "com.test.voix_b")
res = silencieux(rt.run_mission, "com.test.voix_b", {})
check("B, sandboxée, n'entend pas l'intention de A",
      res is not None and res.metrics.get("intention") is None, str(getattr(res, "metrics", None)))
res = silencieux(rt.run_mission, "com.test.voix_b", {"declarer": True})
check("B déclare le même nom : accepté, et B reçoit ses paramètres à elle",
      res is not None and res.metrics.get("declare") is True
      and res.metrics.get("params") == {"cible": "à B"}, str(getattr(res, "metrics", None)))
res = silencieux(rt.run_mission, "com.test.voix_a", {})
check("A reçoit toujours son intention", res is not None and res.success,
      str(getattr(res, "metrics", None)))

section("5. Le vocabulaire d'une app arrêtée est retiré")
silencieux(rt.stop_app, "com.test.voix_a")
check("A arrêtée : plus aucune de ses intentions dans le registre",
      not svc.registry.match("va vers le carton", "com.test.voix_a"))
check("A redémarre : elle redéclare son intention sans conflit",
      silencieux(rt.start_app, "com.test.voix_a"))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
