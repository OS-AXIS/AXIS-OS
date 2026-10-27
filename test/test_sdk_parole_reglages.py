"""
SDK, partie 6 : parole (ctx.speak) et réglages par app (ctx.get_setting).

Vérifié ici, en direct puis depuis le processus de l'app :
  1. speak : passe par ISpeechSynthesizer (une synthèse factice qui note
     ce qui est dit, aucun modèle téléchargé) ; texte borné et neutralisé ;
     langue vérifiée ; permission AUDIO_OUTPUT (NORMAL) ; refus sous arrêt
     d'urgence ; pas de synthèse vocale : CAPABILITY_MISSING ;
  2. réglages : déclarés dans le manifeste (nom, type, bornes, défaut),
     validés à la déclaration ; la valeur choisie par l'utilisateur est
     vérifiée contre la déclaration ; get_setting la rend, ou le défaut ;
     permission APP_SETTINGS (NORMAL) ; le texte JSON d'un manifeste sans
     réglage ne change pas (signatures existantes intactes).
"""
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.speech.synthesis import FakeSpeechSynthesizer, ISpeechSynthesizer  # noqa: E402
from axis_os.store.runtime.manifeste_json import (  # noqa: E402
    manifeste_depuis_json, manifeste_texte, manifeste_vers_json,
)
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    PERMISSION_LEVELS, AppCategory, AppManifest, Permission, ProtectionLevel, SettingSpec,
)
from axis_os.store.sdk.results import AxisRefusal, RefusalCode  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedRobot  # noqa: E402

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


def leve(code, f, *a, **kw):
    try:
        silencieux(f, *a, **kw)
        return False
    except AxisRefusal as exc:
        return exc.code is code


REGLAGES = [SettingSpec("volume", "int", 5, min=0, max=10),
            SettingSpec("vitesse", "float", 0.5, min=0.1, max=1.0),
            SettingSpec("saluer", "bool", True),
            SettingSpec("prenom", "str", "Kenzo", max_length=20),
            SettingSpec("langue", "choice", "fr", choices=("fr", "en"))]


def manifeste(perms, settings=None):
    return AppManifest(app_id="com.test.parole", name="Parole", version="1.0.0", author="T",
                       author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                       permissions=list(perms),
                       settings=list(REGLAGES if settings is None else settings))


def contexte(robot, perms=(Permission.AUDIO_OUTPUT, Permission.APP_SETTINGS,
                           Permission.SYSTEM_INFO), voix=True):
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste(perms))
    ctx._set_clock(robot.clock)
    if voix:
        ctx._set_voice(robot.voice)
    return ctx


class VoixEnPanne(ISpeechSynthesizer):
    languages = ("en",)

    def say(self, text, language):
        raise OSError("haut-parleur débranché")


section("0. Permissions déclarées")
check("AUDIO_OUTPUT et APP_SETTINGS sont NORMAL (rien ne bouge)",
      PERMISSION_LEVELS[Permission.AUDIO_OUTPUT] is ProtectionLevel.NORMAL
      and PERMISSION_LEVELS[Permission.APP_SETTINGS] is ProtectionLevel.NORMAL)

section("1. speak en direct")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    r = ctx.speak("Bonjour", language="fr")
    check("speak : SPOKEN, et la synthèse factice l'a noté", r.ok and r.status == "SPOKEN"
          and robot.voice.spoken == [("Bonjour", "fr")], f"{r} {robot.voice.spoken}")
    ctx.speak("x" * 5000)
    check("Texte coupé à 500 caractères", len(robot.voice.spoken[-1][0]) == 500)
    ctx.speak("ligne\n\x1b[2Jeffacée", language="fr-FR")
    check("Caractères de contrôle neutralisés ; « fr-FR » accepté (synthèse en « fr »)",
          "\x1b" not in robot.voice.spoken[-1][0] and "\n" not in robot.voice.spoken[-1][0]
          and robot.voice.spoken[-1][1] == "fr-FR", repr(robot.voice.spoken[-1]))
    r = ctx.speak("Hallo", language="de")
    check("Langue non prise en charge : REFUSED/CAPABILITY_MISSING", r.code == "CAPABILITY_MISSING",
          str(r))
    for label, args, kw in [("texte vide", ("",), {}), ("texte nombre", (42,), {}),
                            ("langue « french »", ("hi",), {"language": "french"}),
                            ("langue vide", ("hi",), {"language": ""}),
                            ("langue nombre", ("hi",), {"language": 1})]:
        check(f"{label} : INVALID_ARGUMENT", leve(RefusalCode.INVALID_ARGUMENT, ctx.speak, *args, **kw))
    check("robot_info : service speech_out présent",
          silencieux(ctx.robot_info).services.get("speech_out") == "available")
    n = len(robot.voice.spoken)
    check("Sans AUDIO_OUTPUT : PERMISSION_DENIED, rien n'est dit",
          leve(RefusalCode.PERMISSION_DENIED, contexte(robot, [Permission.APP_SETTINGS]).speak, "hi")
          and len(robot.voice.spoken) == n)
    r = silencieux(contexte(robot, voix=False).speak, "hi")
    check("Pas de synthèse vocale : REFUSED/CAPABILITY_MISSING", r.code == "CAPABILITY_MISSING", str(r))
    panne = contexte(robot)
    panne._set_voice(VoixEnPanne())
    r = silencieux(panne.speak, "hi")
    check("Synthèse en panne : FAILED, sans plantage", r.status == "FAILED", str(r))
    silencieux(robot.hal.emergency_stop_all)
    check("Sous arrêt d'urgence : ESTOP_ACTIVE, rien n'est dit",
          leve(RefusalCode.ESTOP_ACTIVE, ctx.speak, "hi") and len(robot.voice.spoken) == n)

section("2. Réglages : déclaration")
check("Les cinq types de réglage déclarés : manifeste valide", manifeste([]).validate() == [],
      str(manifeste([]).validate()))
for label, spec in [("nom avec espace", SettingSpec("mon reglage", "int", 1)),
                    ("type inconnu", SettingSpec("x", "date", "2026")),
                    ("défaut hors bornes", SettingSpec("x", "int", 50, min=0, max=10)),
                    ("défaut d'un autre type", SettingSpec("x", "int", "5")),
                    ("min plus grand que max", SettingSpec("x", "float", 1.0, min=2.0, max=1.0)),
                    ("borne NaN", SettingSpec("x", "float", 1.0, max=float("nan"))),
                    ("choix vide", SettingSpec("x", "choice", "a")),
                    ("défaut hors des choix", SettingSpec("x", "choice", "c", choices=("a", "b"))),
                    ("défaut texte trop long", SettingSpec("x", "str", "abcdef", max_length=3)),
                    ("défaut avec caractère de contrôle", SettingSpec("x", "str", "a\nb"))]:
    check(f"Réglage refusé : {label}", manifeste([], [spec]).validate() != [])
check("Deux réglages du même nom : refusé",
      manifeste([], [SettingSpec("x", "bool", True), SettingSpec("x", "bool", False)]).validate() != [])
m = manifeste([Permission.APP_SETTINGS])
retour = manifeste_depuis_json(manifeste_vers_json(m))
check("Aller-retour JSON du manifeste : réglages identiques", retour.settings == m.settings,
      f"{retour.settings}")
sans = manifeste([], [])
check("Manifeste sans réglage : pas de clé « settings » dans son texte (signatures intactes)",
      '"settings"' not in manifeste_texte(sans))

section("3. Réglages : valeur choisie, lecture")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    check("Rien de choisi : la valeur par défaut", ctx.get_setting("volume") == 5
          and ctx.get_setting("langue") == "fr")
    ctx._set_settings({"volume": 8, "prenom": "Ada"})
    check("Valeur choisie : rendue", ctx.get_setting("volume") == 8 and ctx.get_setting("prenom") == "Ada")
    ctx._set_settings({"volume": 99})
    check("Valeur devenue invalide (déclaration changée) : la valeur par défaut",
          ctx.get_setting("volume") == 5)
    for label, arg in [("réglage non déclaré", "couleur"), ("nom nombre", 3), ("nom vide", "")]:
        check(f"{label} : INVALID_ARGUMENT", leve(RefusalCode.INVALID_ARGUMENT, ctx.get_setting, arg))
    check("Sans APP_SETTINGS : PERMISSION_DENIED",
          leve(RefusalCode.PERMISSION_DENIED, contexte(robot, [Permission.AUDIO_OUTPUT]).get_setting,
               "volume"))
    silencieux(robot.hal.emergency_stop_all)
    check("Sous arrêt d'urgence : la lecture d'un réglage reste possible (rien ne bouge)",
          ctx.get_setting("saluer") is True)


class AppBavarde(RobotApp):
    """Dit bonjour au prénom réglé par l'utilisateur, au volume réglé."""
    def manifest(self):
        return AppManifest(app_id="com.test.bavarde", name="Bavarde", version="1.0.0", author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.AUDIO_OUTPUT, Permission.APP_SETTINGS],
                           settings=[SettingSpec("prenom", "str", "toi", max_length=20),
                                     SettingSpec("volume", "int", 5, min=0, max=10)])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        issues = {"volume": ctx.get_setting("volume")}
        r = ctx.speak(f"Bonjour {ctx.get_setting('prenom')}", language="fr")
        issues["speak"] = r.status
        for nom, appel in [("inconnu", lambda: ctx.get_setting("couleur")),
                           ("langue", lambda: ctx.speak("hi", language="klingon"))]:
            try:
                appel()
                issues[nom] = "accepté"
            except ValueError as exc:
                issues[nom] = type(exc).__name__
        return MissionResult(success=r.ok, mission_id="", app_id="", duration_s=0.0,
                             metrics=issues)


class AppMuette(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.muette", name="Muette", version="1.0.0", author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES, permissions=[Permission.APP_SETTINGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        ctx.speak("je ne devrais pas parler")
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


section("4. Depuis le processus de l'app")
with AppTestHarness(AppBavarde()) as h:
    check("set_setting : valeur valide acceptée", silencieux(h.runtime.set_setting, h.app_id,
                                                             "prenom", "Ada") is True)
    for label, nom, valeur in [("hors bornes", "volume", 11), ("booléen pour un entier", "volume", True),
                               ("texte trop long", "prenom", "x" * 50),
                               ("réglage non déclaré", "couleur", "rouge")]:
        check(f"set_setting {label} : refusé", silencieux(h.runtime.set_setting, h.app_id, nom,
                                                          valeur) is False)
    check("set_setting d'une app inconnue : refusé",
          silencieux(h.runtime.set_setting, "com.x.y", "volume", 3) is False)
    res = h.run()
    m = res.metrics if res is not None else {}
    check("L'app lit le réglage choisi et le défaut, et parle",
          res is not None and res.success and m.get("volume") == 5 and m.get("speak") == "SPOKEN"
          and h.robot.voice.spoken == [("Bonjour Ada", "fr")], f"{m} {h.robot.voice.spoken}")
    check("Réglage non déclaré, langue invalide : InvalidArgumentError dans le processus",
          m.get("inconnu") == "InvalidArgumentError" and m.get("langue") == "InvalidArgumentError",
          str(m))

with AppTestHarness(AppMuette()) as h:
    res = h.run()
    check("speak sans AUDIO_OUTPUT, depuis le processus : mission refusée, rien n'est dit",
          res is not None and not res.success and "AUDIO_OUTPUT" in (res.error_message or "")
          and h.robot.voice.spoken == [], getattr(res, "error_message", None))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
