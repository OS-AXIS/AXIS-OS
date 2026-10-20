"""
Exemple du guide docs/CREER_UNE_APP.md : l'app Assistant IA, testée de
bout en bout SANS ROBOT ET SANS GPU.

  robot virtuel (UnitreeG1HAL backend="sim") -> PlayStoreRuntime (vrai
  sandbox) -> SpeechService (vrai registre d'intentions, phrases
  scriptées) -> app -> « IA d'entreprise » (petit serveur HTTP local).
"""
import contextlib
import io
import json
import sys
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.assistant_ia.app import AssistantIAApp, ModeleEssai, ModeleHTTP
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.drivers.sim_common.sim_perception import QueuedMicrophone
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.services.memory.service import MemoryService
from axis_os.services.speech.service import ISpeechRecognizer, SpeechService
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.runtime.profil import Profil

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class PhrasesScriptees(ISpeechRecognizer):
    """Remplace Whisper : rend les phrases dans l'ordre, puis le silence."""

    def __init__(self, phrases):
        self.phrases = list(phrases)

    def transcribe(self, audio):
        return (self.phrases.pop(0), 0.95) if self.phrases else ("", 0.0)

    @property
    def language(self):
        return "fr"


class IAEntreprise(BaseHTTPRequestHandler):
    """L'IA d'une entreprise, derrière une adresse web."""
    recues = []

    def log_message(self, *a):
        pass

    def do_POST(self):
        d = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        IAEntreprise.recues.append(d)
        corps = json.dumps({"reponse": f"Réponse n°{len(d['historique']) + 1} "
                                       f"à : {d['question']}"}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(corps)))
        self.end_headers()
        self.wfile.write(corps)


def monter(phrases, dossier_memoire):
    with contextlib.redirect_stdout(io.StringIO()):
        hal = UnitreeG1HAL(backend="sim")
        hal.initialize_all()
        robot = NormalizedRobotHAL(hal, build_g1_normalizer())
        robot.initialize_all()
        wd = WatchdogSystem(robot, WatchdogConfig.simulation())
        wd.start()
        micro = QueuedMicrophone("micro", sleeper=lambda s: None)
        micro.initialize()
        parole = SpeechService(micro, PhrasesScriptees(phrases))
        parole.start()
        memoire = MemoryService(storage_dir=Path(dossier_memoire))
        memoire.start()
        rt = PlayStoreRuntime(robot, wd, speech_service=parole, memory_service=memoire, confirmer_permissions=accepter_tout_SIMULATION,
                              profil=Profil.SIMULATION)
    return rt, wd, hal


serveur = HTTPServer(("127.0.0.1", 0), IAEntreprise)
threading.Thread(target=serveur.serve_forever, daemon=True).start()
URL = f"http://127.0.0.1:{serveur.server_port}/"

section("1 — Les deux IA, seules")
check("IA d'essai : répond hors ligne", "Vous avez dit" in ModeleEssai().repondre("bonjour", []))
r = ModeleHTTP(URL).repondre("quelle heure ?", [("a", "b")])
check(f"IA d'entreprise par HTTP : « {r} »", r == "Réponse n°2 à : quelle heure ?")

section("2 — L'app, de bout en bout, sans robot ni GPU")
memo = tempfile.mkdtemp()
PHRASES = [
    "Robot, quels colis sont arrivés ce matin ?",
    "on a bien mangé ce midi",                    # pas le mot de réveil
    "Robot, combien de palettes au quai 3 ?",
    "au revoir",
    "Robot, cette phrase arrive après la fin",
]
rt, wd, hal = monter(PHRASES, memo)
app = AssistantIAApp()
with contextlib.redirect_stdout(io.StringIO()):
    installee = rt.install(app)
    demarree = rt.start_app(app.manifest().app_id)
check("Installée et démarrée par le vrai runtime", installee and demarree)
res = rt.run_mission(app.manifest().app_id, params={"ia_url": URL, "ecoute_s": 0.1})
m = res.metrics if res else {}
print(f"    {res}\n    réponses : {m.get('reponses')}")
check("Mission réussie, 2 questions transmises à l'IA",
      res is not None and res.success and res.items_processed == 2)
check("La conversation voisine (sans « robot ») n'est JAMAIS partie vers l'IA",
      all("mangé" not in q["question"] for q in IAEntreprise.recues))
check("L'IA reçoit l'historique (réponse n°2 à la 2e question)",
      m.get("reponses", [""])[-1].startswith("Réponse n°2"))
check("« au revoir » termine : la phrase suivante n'est pas traitée",
      not any("après la fin" in q["question"] for q in IAEntreprise.recues))
check("Mémoire persistante : 2 échanges retenus", m.get("echanges_total") == 2)

section("3 — IA injoignable : l'app le dit, sans planter ni inventer")
rt2, wd2, hal2 = monter(["Robot, tu m'entends ?", "au revoir"], memo)
with contextlib.redirect_stdout(io.StringIO()):
    rt2.install(AssistantIAApp())
    rt2.start_app("com.exemple.assistant_ia")
res2 = rt2.run_mission("com.exemple.assistant_ia",
                       params={"ia_url": "http://127.0.0.1:9/", "ecoute_s": 0.1})
check("Mission marquée en échec, avec la raison, aucune réponse inventée",
      res2 is not None and not res2.success and res2.items_processed == 0
      and "sans réponse" in (res2.error_message or ""))
check("La mémoire d'une mission à l'autre : toujours 2 (rien d'inventé)",
      res2.metrics.get("echanges_total") == 2)

section("4 — Le robot doit avoir un micro")
man = app.manifest()
check("Manifeste valide", man.validate() == [])
check("Aucune permission de mouvement : l'app ne PEUT pas faire bouger le robot",
      not any(p.name.startswith(("ACTUATORS_WRITE", "LOCOMOTION")) for p in man.permissions))

from axis_os.hal.drivers.isaac_sim.hal import IsaacSimHAL
with contextlib.redirect_stdout(io.StringIO()) as sortie:
    h3 = IsaacSimHAL(robot_id="sans_micro")
    h3.initialize_all()
    r3 = NormalizedRobotHAL(h3, build_g1_normalizer())
    wd3 = WatchdogSystem(h3, WatchdogConfig.simulation())
    refus = not PlayStoreRuntime(r3, wd3, confirmer_permissions=accepter_tout_SIMULATION,
                                 profil=Profil.SIMULATION).install(AssistantIAApp())
check("Robot SANS micro : installation REFUSÉE, avec la raison (SPEECH_IN)",
      refus and "SPEECH_IN" in sortie.getvalue())

for w in (wd, wd2):
    w.stop()
serveur.shutdown()
print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
