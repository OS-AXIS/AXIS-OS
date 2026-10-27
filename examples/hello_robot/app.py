"""
Hello Robot — exemple du SDK AXIS-OS
=====================================
Le robot salue de la main (animation « wave », fournie avec le G1) et dit
bonjour, dans la langue choisie par l'utilisateur (réglage « language »).

API montrées : has_capability, play_animation, speak, get_setting,
should_stop, report_progress.
"""
from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Permission, SettingSpec,
)

GREETINGS = {"en": "Hello! I am {model}, nice to meet you.",
             "fr": "Bonjour ! Je suis {model}, enchanté."}


class HelloRobotApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.axis_os.examples.hello_robot",
            name="Hello Robot",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@example.com",
            description="Salue de la main et dit bonjour.",
            category=AppCategory.UTILITIES,
            permissions=[
                Permission.SYSTEM_INFO,          # has_capability, robot_info
                Permission.MISSION_LIFECYCLE,    # should_stop, report_progress
                Permission.ANIMATION_PLAY,       # play_animation (DANGEROUS)
                Permission.ACTUATORS_READ,       # ... et les droits de move_arms
                Permission.ACTUATORS_WRITE_ARMS,
                Permission.AUDIO_OUTPUT,         # speak
                Permission.APP_SETTINGS,         # get_setting
            ],
            settings=[SettingSpec("language", "choice", "en", choices=("en", "fr"),
                                  description="Langue du bonjour")],
            min_sdk_version="1.0.0",
            max_velocity_ratio=0.5,
        )

    def on_start(self, ctx) -> bool:
        return True

    def run_mission(self, ctx, params: dict) -> MissionResult:
        done = []
        if ctx.has_capability("arms"):
            result = ctx.play_animation("wave")
            done.append(f"wave:{result.status}")
        ctx.report_progress(0.5, "salut fait")
        if ctx.should_stop():
            return MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                                 error_message="arrêt demandé", metrics={"done": done})
        language = ctx.get_setting("language")
        model = ctx.robot_info().model or "a robot"
        said = ctx.speak(GREETINGS[language].format(model=model), language=language)
        done.append(f"speak:{said.status}")
        ctx.report_progress(1.0, "bonjour dit")
        return MissionResult(success=said.ok, mission_id="", app_id="", duration_s=0.0,
                             metrics={"done": done})

    def on_stop(self, ctx) -> None:
        pass
