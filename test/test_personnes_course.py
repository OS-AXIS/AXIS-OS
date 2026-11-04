"""
Fin d'un suivi pendant un cycle de détection : aucune piste ne reste.

Constat (CI de la PR 22, test_exemples_sdk.py, exemple follow_me, échec
intermittent) : `len(people_service._tracker) == 0` faux juste après la fin
du suivi. PeopleService.tick vérifiait `watching`, détectait, puis mettait
à jour les pistes sous le verrou ; si release() passait entre-temps, il
vidait les pistes, puis le cycle en cours y remettait ce qu'il venait de
voir. Elles restaient jusqu'au cycle suivant, alors qu'aucune app ne
regardait plus.

Désormais (axis_os/services/people/service.py) : sous le verrou, un cycle
dont la demande a pris fin ignore ses observations.

Déterministe : la détection du robot simulé est retenue par un événement,
le temps que le test appelle release().
"""
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.testing import SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def service():
    """Service « personnes » du robot simulé, sans sa boucle (le test fait
    les cycles), une personne devant le robot, une demande en cours."""
    r = SimulatedRobot()
    r.add_person(2.0, 0.0)
    ps = r.people_service
    ps.start()
    ps.demander()
    return ps


print("\n  Cycle normal : la personne vue devient une piste")
ps = service()
ps.tick()
check("Une piste après un cycle, demande en cours", len(ps._tracker) == 1, str(len(ps._tracker)))

print("\n  release() pendant un cycle de détection")
ps = service()
en_detection, relache = threading.Event(), threading.Event()
detecter = ps._detector.detect


def detection_lente(frame):
    vues = detecter(frame)
    en_detection.set()
    relache.wait(5.0)            # le suivi s'arrête pendant ce temps
    return vues


ps._detector.detect = detection_lente
cycle = threading.Thread(target=ps.tick)
cycle.start()
en_detection.wait(5.0)
ps.release()
pistes_apres_release = len(ps._tracker)
relache.set()
cycle.join(5.0)
check("Juste après release() : aucune piste", pistes_apres_release == 0, str(pistes_apres_release))
check("… ni après la fin du cycle commencé avant (ses observations sont ignorées)",
      not cycle.is_alive() and len(ps._tracker) == 0 and not ps.watching,
      f"{len(ps._tracker)} piste(s), regarde : {ps.watching}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
