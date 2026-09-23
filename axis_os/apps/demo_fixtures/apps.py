"""
Apps FIGURANTES pour la démonstration D1 — pas des applications du
catalogue.

Elles existent pour montrer des REFUS : une app qui exige un organe
absent, une qui tente ce qu'elle n'a pas déclaré, une qui occupe le
robot. Aucune n'a d'usage réel.

POURQUOI UN VRAI MODULE, et non des classes définies dans le script de
démonstration : le sandbox exécute une app dans un SOUS-PROCESSUS, qui
doit pouvoir importer sa classe. Une classe définie dans une fonction ne
peut pas lui être transmise, et une classe définie dans le script
principal oblige l'enfant à réimporter ce script — avec tout ce qu'il
contient. Les deux ont échoué, en 0.3 s, pour des raisons sans rapport
avec les scènes jouées.

C'est une contrainte réelle du SDK, qui vaudra pour les développeurs
tiers : **une app doit être un module importable**.
"""
from axis_os.store.sdk.app_base import (
    MissionResult, RobotApp,
    PermissionError as SandboxPermissionError,
)
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Capability, Permission, ResourceQuota,
)


class AppDeuxMains(RobotApp):
    """Exige une préhension à DEUX mains — le G1 23 DOF ne l'a pas."""

    def manifest(self):
        return AppManifest(
            app_id="com.demo.deux_mains", name="Manipulation bimanuelle",
            version="1.0.0", author="Demo", author_email="d@d.d",
            description="Saisit un objet à deux mains.",
            category=AppCategory.LOGISTICS,
            permissions=[Permission.ACTUATORS_WRITE_ARMS],
            capabilities_required=[Capability.GRASP_DUAL],
            quota=ResourceQuota())

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="m",
                             app_id="com.demo.deux_mains", duration_s=0.0)


class AppCurieuse(RobotApp):
    """Ne déclare que la caméra, mais essaie de faire marcher le robot."""

    def manifest(self):
        return AppManifest(
            app_id="com.demo.curieuse", name="App curieuse",
            version="1.0.0", author="Demo", author_email="d@d.d",
            description="Regarde… et tente de marcher.",
            category=AppCategory.RESEARCH,
            permissions=[Permission.SENSORS_CAMERA],
            quota=ResourceQuota(max_runtime_s=20.0))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        try:
            ctx.walk(0.5)
            marche = True
        except SandboxPermissionError as exc:
            ctx.log(f"refus attendu : {exc}")
            marche = False
        return MissionResult(
            success=not marche, mission_id="m",
            app_id="com.demo.curieuse", duration_s=0.0,
            metrics={"a_pu_marcher": marche})


class AppLente(RobotApp):
    """Occupe le robot pendant plusieurs secondes."""

    def manifest(self):
        return AppManifest(
            app_id="com.demo.lente", name="App lente", version="1.0.0",
            author="Demo", author_email="d@d.d", description="Occupe.",
            category=AppCategory.RESEARCH, permissions=[],
            quota=ResourceQuota(max_runtime_s=30.0))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import time as _t
        _t.sleep(6.0)
        return MissionResult(success=True, mission_id="m",
                             app_id="com.demo.lente", duration_s=6.0)
