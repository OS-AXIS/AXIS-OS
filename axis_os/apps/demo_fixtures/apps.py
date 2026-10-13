"""
Apps figurantes pour les démonstrations, et non des applications du
catalogue.

Elles servent à la démonstration « plusieurs apps sur un même robot »
(sim/validation/test_isaac_d1_multiapp.py) et à la vidéo de l'app qui se
fige (sim/video/video_v1_app_figee.py). Elles existent pour montrer des
refus : une app qui exige un organe absent, une qui tente ce qu'elle n'a
pas déclaré, une qui occupe le robot, une qui cesse de répondre. Aucune
n'a d'usage réel.

Pourquoi un vrai module, et non des classes définies dans le script de
démonstration : le sandbox (le processus isolé dans lequel l'OS exécute
chaque app) fait tourner l'app dans un sous-processus, qui doit pouvoir
importer sa classe. Une classe définie dans une fonction ne peut pas lui
être transmise, et une classe définie dans le script principal oblige le
sous-processus à réimporter ce script, avec tout ce qu'il contient. Les
deux approches ont échoué, en 0.3 s, pour des raisons sans rapport avec
les scènes jouées.

C'est une contrainte réelle du SDK (le kit de développement des apps),
qui vaudra aussi pour les développeurs tiers : **une app doit être un
module importable** (voir docs/CREER_UNE_APP.md).
"""
from axis_os.store.sdk.app_base import (
    MissionResult, RobotApp,
    PermissionError as SandboxPermissionError,
)
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Capability, Permission, ResourceQuota,
)


class AppDeuxMains(RobotApp):
    """Exige une préhension à deux mains (capacité GRASP_DUAL), que le
    G1 23 DOF n'a pas : l'installation doit être refusée faute de
    matériel.

    Le G1 23 DOF est l'humanoïde G1 d'Unitree dans sa version à 23 degrés
    de liberté (23 articulations motorisées).
    """

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
    """Ne déclare que la caméra, mais essaie de faire marcher le robot.

    Le SDK doit refuser la marche (PermissionError du SDK), faute de
    permission déclarée dans le manifeste (la fiche que l'app présente à
    l'OS). La mission réussit justement si la marche a été refusée ; la
    métrique « a_pu_marcher » le confirme.
    """

    def manifest(self):
        return AppManifest(
            app_id="com.demo.curieuse", name="App curieuse",
            version="1.0.0", author="Demo", author_email="d@d.d",
            description="Regarde… et tente de marcher.",
            category=AppCategory.RESEARCH,
            permissions=[Permission.SENSORS_CAMERA],
            quota=ResourceQuota(max_memory_mb=8192, max_runtime_s=20.0))

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
    """Occupe le robot pendant six secondes, pour montrer qu'une deuxième
    app ne peut pas lancer de mission tant que la première le tient."""

    def manifest(self):
        return AppManifest(
            app_id="com.demo.lente", name="App lente", version="1.0.0",
            author="Demo", author_email="d@d.d", description="Occupe.",
            category=AppCategory.RESEARCH, permissions=[],
            # 8192 Mo, comme toutes les vraies apps du projet. Avec le
            # défaut de 256 Mo, le sous-processus de l'app dépasse la
            # limite dans l'environnement Python d'Isaac Sim (le
            # simulateur robotique de NVIDIA), et la mission est tuée en
            # 0.3 s. Constaté sur cette app, alors qu'elle réussit en
            # 6.01 s hors du conteneur Isaac Sim.
            quota=ResourceQuota(max_memory_mb=8192, max_runtime_s=30.0))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import time as _t
        _t.sleep(6.0)
        return MissionResult(success=True, mission_id="m",
                             app_id="com.demo.lente", duration_s=6.0)


class AppQuiSeFige(RobotApp):
    """Fait marcher le robot, puis se fige, comme une app qui plante.

    Elle ne lève pas d'exception et ne se termine pas : elle cesse
    simplement de répondre, bloquée dans une attente sans fin. C'est le
    cas le plus difficile pour un système, parce que rien ne signale la
    panne : l'app est toujours là, elle ne dit juste plus rien.

    Deux mécanismes d'AXIS-OS doivent alors agir, dans cet ordre :
      1. l'échéance de fraîcheur du service de marche (une consigne qui
         n'est plus renouvelée retombe à zéro) arrête le robot en
         équilibre, en une demi-seconde ;
      2. le quota de durée d'exécution du sandbox finit par tuer l'app.
    """

    MARCHE_S = 10.0              # durée de marche avant la panne, s

    def manifest(self):
        return AppManifest(
            app_id="com.demo.se_fige", name="App qui se fige",
            version="1.0.0", author="Demo", author_email="d@d.d",
            description="Marche, puis cesse de répondre.",
            category=AppCategory.RESEARCH,
            permissions=[Permission.ACTUATORS_WRITE_LEGS,
                         Permission.LOCOMOTION_WALK],
            capabilities_required=[Capability.WALK],
            quota=ResourceQuota(max_memory_mb=8192, max_runtime_s=28.0))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import time as _t
        t0 = _t.monotonic()
        while _t.monotonic() - t0 < self.MARCHE_S:
            ctx.walk(0.5, 0.0, 0.0)
            _t.sleep(0.1)
        # La panne : plus aucune consigne, plus aucune réponse, pendant
        # une heure, bien au-delà du quota de 28 s.
        _t.sleep(3600.0)
        return MissionResult(success=True, mission_id="m",
                             app_id="com.demo.se_fige", duration_s=0.0)
