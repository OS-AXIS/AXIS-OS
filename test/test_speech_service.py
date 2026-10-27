"""
SpeechService — de la voix à une intention, jamais à un mouvement.

Ce que ce test protège avant tout : qu'une phrase inattendue, mal
comprise ou hostile ne déclenche RIEN. Le pire cas d'une reconnaissance
ratée doit être l'inaction.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import AudioChunk, HALStatus, SensorType
from axis_os.hal.core.interfaces import IMicrophone
from axis_os.services.base import IService
from axis_os.services.speech.service import (
    Intent, IntentDeclaration, IntentRegistry, ISpeechRecognizer,
    NullSpeechRecognizer, SpeechService, normalize,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class StubMic(IMicrophone):
    def __init__(self, ok=True):
        super().__init__("stub_mic", SensorType.MICROPHONE)
        self._ok = ok
        self._initialized = True

    def initialize(self): return HALStatus.OK
    def shutdown(self): return HALStatus.OK
    def is_healthy(self): return True

    def capture(self, duration_s):
        if not self._ok:
            return None
        return AudioChunk(sample_rate=16000, channels=1, sample_width=2,
                          data=b"\x00\x01" * 1600)


class StubRec(ISpeechRecognizer):
    def __init__(self, texte="", conf=1.0):
        self.texte, self.conf = texte, conf

    def transcribe(self, audio): return self.texte, self.conf

    @property
    def language(self): return "fr"


def registre():
    r = IntentRegistry()
    r.register(IntentDeclaration(
        "aller_vers_colis",
        [["va", "aller", "avance", "rejoins"],
         ["carton", "colis", "boite", "paquet"]],
        params={"cible": "a cardboard box"}))
    r.register(IntentDeclaration(
        "arreter", [["stop", "arrete", "halte"]]))
    return r


section("1 — Normalisation : accents et ponctuation n'ont pas à compter")

check("Accents, majuscules et ponctuation neutralisés",
      normalize("Va vers le CARTON, s'il te plaît !")
      == "va vers le carton s il te plait")

section("2 — Le registre exige TOUS les groupes de mots-clés")

r = registre()
check("« va vers le carton » -> intention",
      r.match("va vers le carton").name == "aller_vers_colis")
check("« rejoins le colis là-bas » -> même intention (synonymes)",
      r.match("rejoins le colis là-bas").name == "aller_vers_colis")
check("« avance » seul -> AUCUNE intention (il manque l'objet)",
      r.match("avance") is None)
check("« le carton est rouge » -> AUCUNE intention (il manque le verbe)",
      r.match("le carton est rouge") is None)
check("Phrase hors sujet -> AUCUNE intention",
      r.match("quelle heure est-il") is None)
check("Phrase vide -> None", r.match("") is None)
check("Les paramètres déclarés accompagnent l'intention",
      r.match("va vers le carton").params.get("cible") == "a cardboard box")

try:
    r.register(IntentDeclaration("arreter", [["x"]]))
    ok = False
except ValueError:
    ok = True
check("Un même nom ne peut pas être déclaré deux fois dans le même espace "
      "(celui d'une app, ou celui de l'OS)", ok)

section("3 — Une phrase mal comprise ne doit RIEN déclencher")

svc = SpeechService(StubMic(), StubRec("va vers le carton"), registre())
check("Écoute avant start() : refusée", not svc.listen().ok)
svc.start()

r1 = svc.listen()
check("Phrase claire -> intention", isinstance(r1.value, Intent))

svc2 = SpeechService(StubMic(), StubRec("ouvre la porte du garage"), registre())
svc2.start()
r2 = svc2.listen()
check("Phrase hors vocabulaire -> aucune action, ET la raison est dite",
      r2.ok and r2.value is None and "aucune intention" in r2.reason)

svc3 = SpeechService(StubMic(), StubRec(""), registre())
svc3.start()
r3 = svc3.listen()
check("Silence -> aucune action, raison explicite",
      r3.ok and r3.value is None and "rien de compris" in r3.reason)

svc4 = SpeechService(StubMic(), StubRec("va vers le carton", conf=0.05),
                     registre(), min_confidence=0.3)
svc4.start()
r4 = svc4.listen()
check("Transcription PEU SÛRE -> ignorée plutôt qu'exécutée au hasard",
      r4.value is None and "peu sûre" in r4.reason)

svc5 = SpeechService(StubMic(ok=False), StubRec("va vers le carton"), registre())
svc5.start()
check("Micro muet -> échec explicite, pas d'exception",
      svc5.listen().ok is False)

section("4 — Le service ne peut PAS commander d'actionneur")

api = {m for m in dir(svc) if not m.startswith("_")}
interdits = {m for m in api
             if any(k in m for k in ("move", "walk", "send", "command", "actuat"))}
check(f"Aucune méthode d'action exposée (trouvé: {interdits or 'aucune'})",
      not interdits)
check("Une intention ne contient qu'un NOM et des paramètres — "
      "jamais d'angles",
      set(Intent("x").__dict__ if hasattr(Intent("x"), "__dict__")
          else {"name", "params", "confidence", "utterance"})
      <= {"name", "params", "confidence", "utterance"})

section("5 — Sans modèle, la chaîne reste testable et le DIT")

svc6 = SpeechService(StubMic(), NullSpeechRecognizer(), registre())
svc6.start()
check("Service sain mais signalé comme dépourvu de modèle",
      svc6.health().healthy and "aucun modèle" in svc6.health().detail)
check("Sans modèle, aucune intention n'est inventée",
      svc6.listen().value is None)

section("6 — Conversion audio pour Whisper")

try:
    from axis_os.services.speech.whisper_recognizer import WhisperRecognizer
    import numpy as np

    # Stéréo 8 kHz, 16 bits : doit devenir mono 16 kHz flottant.
    data = np.array([1000, -1000] * 800, dtype=np.int16).tobytes()
    a = AudioChunk(sample_rate=8000, channels=2, sample_width=2, data=data)
    x = WhisperRecognizer.to_float_mono(a)
    check("Stéréo -> mono, 8 kHz -> 16 kHz, entiers -> flottants",
          x.ndim == 1 and abs(x.size - 1600) <= 2 and x.dtype.kind == "f")
    check("Amplitude ramenée dans [-1, 1]", float(np.abs(x).max()) <= 1.0)

    try:
        WhisperRecognizer.to_float_mono(
            AudioChunk(16000, 1, 3, b"\x00" * 30))
        ok = False
    except ValueError:
        ok = True
    check("Format inconnu : REFUSÉ plutôt qu'une transcription fausse", ok)

    w = WhisperRecognizer()
    check("Sans modèle chargé, transcribe() rend du vide",
          w.transcribe(a) == ("", 0.0))
except ImportError as exc:
    print(f"  [—] module Whisper non testable ici : {exc}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
