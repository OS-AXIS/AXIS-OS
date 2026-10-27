"""
Fetch — exemple du SDK AXIS-OS
===============================
Va chercher un objet et le rapporte : look_for (dans le repère du monde),
go_to devant l'objet, pick, go_to au point de dépose, place, rest_pose.

Paramètres de mission : "object" (ce qu'on cherche, ex. "red box"),
"size_m" (largeur, profondeur, hauteur de l'objet, en mètres : une image
ne donne pas la taille d'un objet), "drop" ([x, y] du point de dépose).

À chaque étape, l'app vérifie should_stop() et rapporte sa progression ;
chaque échec est rendu tel quel, avec son code (OUT_OF_REACH,
PROXIMITY_GUARD, TIMEOUT...).
"""
from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission

# Distance entre le robot et le centre de l'objet pour le prendre : les
# mains du G1 sont à 30 cm devant lui.
REACH_M = 0.30


class FetchApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.axis_os.examples.fetch",
            name="Fetch",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@example.com",
            description="Va chercher un objet et le rapporte.",
            category=AppCategory.LOGISTICS,
            permissions=[
                Permission.SYSTEM_INFO,
                Permission.MISSION_LIFECYCLE,
                Permission.SENSORS_CAMERA,       # look_for
                Permission.NAVIGATION_GOTO,      # go_to (DANGEROUS)
                Permission.LOCOMOTION_WALK,      # ... et les droits de walk()
                Permission.ACTUATORS_WRITE_LEGS,
                Permission.MANIPULATION,         # pick, place (DANGEROUS)
                Permission.ACTUATORS_READ,       # ... et les droits des bras
                Permission.ACTUATORS_WRITE_ARMS,  # (aussi ceux de rest_pose)
            ],
            min_sdk_version="1.0.0",
            max_velocity_ratio=0.5,
        )

    def on_start(self, ctx) -> bool:
        return True

    def _fail(self, step: str, result, steps: list) -> MissionResult:
        steps.append(f"{step}:{result.status}")
        return MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                             error_message=f"{step} : {result.status} ({result.code}) "
                                           f"— {result.message}",
                             metrics={"steps": steps})

    def run_mission(self, ctx, params: dict) -> MissionResult:
        steps = []
        for capability in ("navigation", "manipulation"):
            if not ctx.has_capability(capability):
                return MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                                     error_message=f"ce robot n'a pas de {capability}")
        seen = ctx.look_for(params.get("object", "box"), "world")
        if not seen:
            return MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                                 error_message="objet introuvable")
        target = seen[0]
        p = target.position_3d
        ctx.report_progress(0.1, f"vu : {target.label}")

        plan = [
            ("go_to", lambda: ctx.go_to(p.x - REACH_M, p.y, 0.0, timeout_s=60.0)),
            ("pick", lambda: ctx.pick(target, size_m=tuple(params.get("size_m", (0.24, 0.20, 0.20))),
                                      timeout_s=15.0)),
            ("go_to_drop", lambda: ctx.go_to(*params.get("drop", (0.0, 0.0)), timeout_s=60.0)),
            ("place", lambda: ctx.place(None, timeout_s=15.0)),
            ("rest_pose", lambda: ctx.rest_pose(duration_s=1.5)),
        ]
        for i, (step, action) in enumerate(plan):
            if ctx.should_stop():
                return MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                                     error_message=f"arrêt demandé avant {step}",
                                     metrics={"steps": steps})
            result = action()
            if not result:
                return self._fail(step, result, steps)
            steps.append(f"{step}:{result.status}")
            ctx.report_progress(0.2 + 0.8 * (i + 1) / len(plan), step)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             items_processed=1, metrics={"steps": steps})

    def on_stop(self, ctx) -> None:
        pass
