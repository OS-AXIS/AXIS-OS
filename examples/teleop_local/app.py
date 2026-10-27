"""
Teleop local — exemple du SDK AXIS-OS
======================================
Téléopération LOCALE de la marche par un flux de consignes
(ctx.stream_targets) sous homme mort : l'opérateur tient un bouton ; dès
qu'il le relâche, ou si les consignes cessent d'arriver, l'OS ralentit
puis arrête le robot, et n'accepte plus rien avant resume().

La manette est ici une manette factice, déroulant le scénario passé en
paramètre : [[durée_s, vx, vy, wz, homme_mort], ...]. Une vraie manette
branchée au robot demanderait un service d'entrée côté OS : une app ne
lit jamais elle-même un périphérique ni un fichier de l'OS.
"""
import time

from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission


class ScriptedGamepad:
    """Manette factice : rend (vitesse, homme mort) selon un scénario ; None
    à la fin."""

    def __init__(self, script):
        self.script = [tuple(step) for step in script]
        self.t0 = time.monotonic()

    def read(self):
        t = time.monotonic() - self.t0
        for duration, vx, vy, wz, deadman in self.script:
            if t < duration:
                return (vx, vy, wz), bool(deadman)
            t -= duration
        return None


class TeleopLocalApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.axis_os.examples.teleop_local",
            name="Teleop local",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@example.com",
            description="Pilote la marche à la manette, sous homme mort.",
            category=AppCategory.UTILITIES,
            permissions=[
                Permission.MISSION_LIFECYCLE,
                Permission.TELEOP_STREAM,         # stream_targets (DANGEROUS)
                Permission.LOCOMOTION_WALK,       # ... et les droits de walk()
                Permission.ACTUATORS_WRITE_LEGS,
            ],
            min_sdk_version="1.0.0",
            max_velocity_ratio=0.5,
        )

    def on_start(self, ctx) -> bool:
        return True

    def run_mission(self, ctx, params: dict) -> MissionResult:
        pad = ScriptedGamepad(params.get("script", []))
        last = None
        with ctx.stream_targets("walk", deadman_ms=200, max_rate_hz=50,
                                max_speed=0.5) as stream:
            while not ctx.should_stop():
                frame = pad.read()
                if frame is None:
                    break
                velocity, deadman = frame
                last = stream.push(velocity, deadman=deadman)
                if last.state == "STOPPED" and deadman and params.get("auto_resume"):
                    # L'opérateur a rappuyé : reprise explicite.
                    stream.resume()
                time.sleep(0.02)
            m = stream.metrics()
        return MissionResult(
            success=True, mission_id="", app_id="", duration_s=0.0,
            metrics={"accepted": m.accepted, "rejected": m.rejected,
                     "latency_ms": round(m.latency_ms_mean, 2), "jitter_ms": round(m.jitter_ms, 2),
                     "safe_stops": m.safe_stops, "state": last.state if last else None})

    def on_stop(self, ctx) -> None:
        pass
