"""
Bac à sable : une violation de quota relevée par le moniteur, et le processus
de l'app encore vivant à l'instant où run_app décide du résultat.

Constaté en CI (PR #43, job `tests` du 25 novembre 2026) et en local sous
charge (2 fois sur 9) : test_sandbox §5, quota RAM, rendait TIMEOUT au lieu
de VIOLATED. Le moniteur relevait MEMORY_QUOTA et terminait le processus ;
le pont se fermait, mais le processus vivait encore quand run_app décidait ;
la branche du délai dépassé passait en premier, et le classait TIMEOUT (avec
la violation MEMORY_QUOTA).

Ici, la course rendue sûre : le processus de l'app (processus_app.
ProcessusHote) et le moniteur (SandboxRunner._monitor_loop) sont des
doublures ; le reste de run_app est le vrai. Le pont ne se ferme qu'une fois
le moniteur passé ; le processus ne meurt que quand run_app l'attend.
  1. violation mémoire relevée, processus encore vivant : VIOLATED, MEMORY_QUOTA ;
  2. de même pour le CPU : VIOLATED, CPU_QUOTA ;
  3. l'erreur opposée n'existe pas : un vrai délai dépassé, relevé par le
     moniteur, ou sans rien de relevé (boucle à sa limite de durée), le
     processus encore vivant : TIMEOUT, jamais « quota » ;
  4. violation mémoire relevée, processus déjà mort : VIOLATED, comme avant ;
  5. cas mixte : violation mémoire relevée, l'app ignore SIGTERM et vit
     jusqu'à la limite de durée, puis est tuée : VIOLATED, MEMORY_QUOTA, la
     première cause, celle qui l'a fait arrêter (avant : TIMEOUT).
Dans tous les cas, le processus est terminé, puis fermé (récolté).
"""
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.store.runtime import processus_app  # noqa: E402
from axis_os.store.runtime.canal_ipc import CanalFerme  # noqa: E402
from axis_os.store.runtime.sandbox import (  # noqa: E402
    SandboxErrorKind, SandboxRunner, SandboxState, SandboxViolation,
)
from axis_os.store.sdk.manifest import ResourceQuota  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def lancer(regle, vivant_a_la_decision=True, ignore_term=False):
    """runner.run() avec les doublures : le moniteur relève `regle` (None :
    rien) et termine le processus, comme _monitor_loop ; le pont se ferme
    ensuite (regle None, ou l'app qui ignore SIGTERM : jamais, la boucle va
    jusqu'à sa limite de durée). Le processus vit encore quand run_app décide
    si `vivant_a_la_decision` ; `ignore_term` : seul SIGKILL le tue.
    Rend (résultat, processus)."""
    t = [0.0]
    runner = SandboxRunner("com.test.course", ResourceQuota(max_runtime_s=5.0, max_memory_mb=100),
                           clock=lambda: t[0])
    moniteur_passe = threading.Event()
    vu = {}

    class Canal:
        def envoyer(self, msg):
            pass

        def recevoir(self, timeout=None):
            if regle is not None and not moniteur_passe.wait(5.0):
                raise AssertionError("le moniteur n'est jamais passé")
            if regle is None or ignore_term:   # rien ne ferme le pont : le temps passe
                t[0] += 0.5
                raise TimeoutError
            raise CanalFerme("l'app a fermé le pont")

        def fermer(self):
            pass

    class Processus:
        """Comme ProcessusHote : vivant tant que rien ne l'a récolté ; un
        signal reçu ne le tue qu'au join qui suit (la course)."""

        def __init__(self, app_id):
            self.pid, self.canal, self.signaux, self.recolte = -1, Canal(), [], False
            self._vivant = True
            vu["proc"] = self

        def is_alive(self):
            return self._vivant

        def terminate(self):
            self.signaux.append("TERM")

        def kill(self):
            self.signaux.append("KILL")
            self._vivant = False

        def join(self, timeout=None):
            if "KILL" in self.signaux or (self.signaux and not ignore_term):
                self._vivant = False

        def fermer(self):
            self.recolte, self._vivant = True, False

    def moniteur(proc, t_start, cpu_actif=None):
        if regle is not None:
            runner._add_violation(SandboxViolation("com.test.course", regle, 300.0, 100.0))
            proc.terminate()
            if not vivant_a_la_decision:
                proc._vivant = False
        moniteur_passe.set()

    runner._monitor_loop = moniteur
    avant = processus_app.ProcessusHote
    processus_app.ProcessusHote = Processus
    try:
        res = runner.run(time.sleep, args=(0.0,))
    finally:
        processus_app.ProcessusHote = avant
    return res, vu["proc"]


def resume(res):
    return (f"{res.state.name}, {res.error_kind.name if res.error_kind else None}, "
            f"violation {res.violation.rule if res.violation else None}")


section("1. Violation mémoire relevée, processus encore vivant à la décision")
res, proc = lancer("MEMORY_QUOTA")
check(f"VIOLATED, MEMORY_QUOTA ({resume(res)})",
      res.state is SandboxState.VIOLATED and res.error_kind is SandboxErrorKind.MEMORY_QUOTA
      and res.violation is not None and res.violation.rule == "MEMORY_QUOTA" and not res.success)
check(f"Le processus terminé (signaux : {proc.signaux}), puis fermé", "TERM" in proc.signaux
      and proc.recolte and not proc.is_alive())

section("2. Violation CPU relevée, processus encore vivant à la décision")
res, proc = lancer("CPU_QUOTA")
check(f"VIOLATED, CPU_QUOTA ({resume(res)})",
      res.state is SandboxState.VIOLATED and res.error_kind is SandboxErrorKind.CPU_QUOTA
      and res.violation.rule == "CPU_QUOTA")

section("3. L'erreur opposée : un vrai délai dépassé n'est jamais classé « quota »")
res, proc = lancer("TIMEOUT")
check(f"Délai dépassé relevé par le moniteur, processus encore vivant : TIMEOUT ({resume(res)})",
      res.state is SandboxState.TIMEOUT and res.error_kind is SandboxErrorKind.TIMEOUT
      and res.violation.rule == "TIMEOUT")
res, proc = lancer(None)
check(f"Rien de relevé (sans psutil, par exemple), la boucle à sa limite de durée, processus "
      f"encore vivant : TIMEOUT ({resume(res)})",
      res.state is SandboxState.TIMEOUT and res.error_kind is SandboxErrorKind.TIMEOUT
      and res.violation is not None and res.violation.rule == "TIMEOUT")
check(f"… le processus terminé (signaux : {proc.signaux}), puis fermé",
      "TERM" in proc.signaux and proc.recolte)

section("4. Violation mémoire relevée, processus déjà mort : comme avant")
res, proc = lancer("MEMORY_QUOTA", vivant_a_la_decision=False)
check(f"VIOLATED, MEMORY_QUOTA ({resume(res)})",
      res.state is SandboxState.VIOLATED and res.error_kind is SandboxErrorKind.MEMORY_QUOTA
      and proc.recolte)

section("5. Cas mixte : violation mémoire, puis l'app ignore SIGTERM jusqu'à la limite de durée")
res, proc = lancer("MEMORY_QUOTA", ignore_term=True)
check(f"VIOLATED, MEMORY_QUOTA : la première cause, celle qui l'a fait arrêter ({resume(res)}) ; "
      f"tuée (signaux : {proc.signaux}), puis fermée",
      res.state is SandboxState.VIOLATED and res.error_kind is SandboxErrorKind.MEMORY_QUOTA
      and "KILL" in proc.signaux and proc.recolte and not proc.is_alive())

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
