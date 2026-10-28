"""
Follow me — exemple du SDK AXIS-OS
==================================
« Suis-moi » : le robot suit la personne qui est devant lui, et le dit.
« Arrête-toi » ou « stop » : il s'arrête, et le dit. Il le dit aussi
quand il perd la personne. Une mission = une séance de suivi.

C'est l'OS qui suit (distance, vitesse, gardien de proximité, recherche,
jamais la mauvaise personne) ; l'app décide seulement quand commencer et
quand s'arrêter, et donne de ses nouvelles (follow_status) pour l'homme
mort. Aucune reconnaissance faciale : l'app ne connaît que « la personne
n° 3, à 2 m devant ».

API montrées : declare_intent, listen, person_in_front, follow,
follow_status, stop_follow, speak, should_stop.
"""
from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Permission, ResourceQuota,
)

SAID = {"start": "D'accord, je vous suis.",
        "nobody": "Je ne vois personne devant moi.",
        "refused": "Je ne peux pas vous suivre.",
        "stop": "Je m'arrête.",
        "lost": "Je vous ai perdu.",
        "end": "Fin du suivi."}


class FollowMeApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.axis_os.examples.follow_me",
            name="Follow me",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@example.com",
            description="Suit la personne devant le robot, à la voix.",
            category=AppCategory.UTILITIES,
            permissions=[
                Permission.SENSORS_MICROPHONE,   # declare_intent, listen
                Permission.PEOPLE_DETECT,        # person_in_front
                Permission.PEOPLE_FOLLOW,        # follow, follow_status (DANGEROUS)
                Permission.LOCOMOTION_WALK,      # ... et les droits de walk
                Permission.ACTUATORS_WRITE_LEGS,
                Permission.AUDIO_OUTPUT,         # speak
                Permission.MISSION_LIFECYCLE,    # should_stop
            ],
            quota=ResourceQuota(max_runtime_s=600.0),
            min_sdk_version="1.1.0",
        )

    def on_start(self, ctx) -> bool:
        ctx.declare_intent("follow_me", [["suis moi", "suivez moi", "follow me"]])
        ctx.declare_intent("stop", [["arrete toi", "arretez vous", "stop"]])
        return True

    def run_mission(self, ctx, params: dict) -> MissionResult:
        events = []
        deadline = ctx.now() + float(params.get("max_s", 300.0))
        following = False

        def say(key):
            ctx.speak(SAID[key], language="fr")
            events.append(key)

        while ctx.now() < deadline and not ctx.should_stop():
            if following:
                # Donner de ses nouvelles : c'est l'homme mort du suivi.
                status = ctx.follow_status()
                if status.status != "FOLLOWING":
                    say("lost" if status.status == "LOST" else "end")
                    return self._done(events, status.status)
            intent = ctx.listen(0.5 if following else 1.0)
            if intent is None:
                continue
            if intent.name == "follow_me" and not following:
                person = ctx.person_in_front(3.0)
                if person is None:
                    say("nobody")
                    continue
                result = ctx.follow(person.person_id, distance_m=1.5,
                                    timeout_s=max(1.0, deadline - ctx.now()), deadman_s=2.0)
                if not result.ok:
                    say("refused")
                    continue
                following = True
                say("start")
            elif intent.name == "stop" and following:
                ctx.stop_follow()
                say("stop")
                return self._done(events, "STOPPED_BY_APP")
        if following:
            ctx.stop_follow()
        return self._done(events, "END")

    @staticmethod
    def _done(events, outcome) -> MissionResult:
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics={"events": events, "outcome": outcome})

    def on_stop(self, ctx) -> None:
        pass
