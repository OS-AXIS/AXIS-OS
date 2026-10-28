"""
Night patrol — exemple du SDK AXIS-OS
=====================================
Une ronde de nuit, lancée par l'OS chaque soir à 22:00 (tâche planifiée) :
le robot passe par les lieux nommés « ronde 1 », « ronde 2 », « ronde 3 »
de la carte du site. À chaque lieu, il regarde : une personne dans une zone
interdite déclenche une notification URGENT. Batterie sous `return_percent`
(30 % par défaut) : il le dit et rentre à la station de charge. Un moteur
qui chauffe (« throttled » ou pire) : il fait une pause jusqu'à ce qu'il
refroidisse.

L'OS garde la main sur ce qui compte : il ne laisse jamais le robot entrer
dans une zone interdite, ralentit dans les zones lentes, bride ou arrête un
moteur trop chaud, et rentre lui-même à la station sous son propre seuil
(20 %), même si l'app ne le fait pas.

API montrées : places, zones, where_am_i, go_to(nom de lieu), people,
notify, battery, thermal_status, should_stop ; triggers et required_places
dans le manifeste.
"""
import math

from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Permission, ResourceQuota, TriggerSpec,
)

ROUTE = ("ronde 1", "ronde 2", "ronde 3")
HOT = ("throttled", "critical")


class NightPatrolApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.axis_os.examples.night_patrol",
            name="Night patrol",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@example.com",
            description="Ronde de nuit planifiée entre des lieux nommés.",
            category=AppCategory.SECURITY,
            permissions=[
                Permission.LOCATION,            # places, zones, where_am_i
                Permission.NAVIGATION_GOTO,     # go_to
                Permission.LOCOMOTION_WALK,     # ... et les droits de walk
                Permission.ACTUATORS_WRITE_LEGS,
                Permission.PEOPLE_DETECT,       # people
                Permission.NOTIFICATIONS,       # notify
                Permission.HARDWARE_HEALTH,     # battery, thermal_status
                Permission.MISSION_LIFECYCLE,   # should_stop
                Permission.SCHEDULED_MISSIONS,  # triggers (DANGEROUS)
            ],
            required_places=list(ROUTE),
            triggers=[TriggerSpec("evening_round", "daily", at="22:00")],
            quota=ResourceQuota(max_runtime_s=1800.0),
            min_sdk_version="1.2.0",
        )

    def on_start(self, ctx) -> bool:
        return True

    def run_mission(self, ctx, params: dict) -> MissionResult:
        events = []
        return_percent = float(params.get("return_percent", 30.0))
        station = next((p.name for p in ctx.places() if p.kind == "charging_station"), None)
        forbidden = [z for z in ctx.zones() if z.kind == "forbidden"]
        reported = set()

        def look():
            here = ctx.where_am_i()
            if not here.known:
                return
            for person in ctx.people():
                a = here.heading + person.bearing_rad
                x = here.x + person.distance_m * math.cos(a)
                y = here.y + person.distance_m * math.sin(a)
                for zone in forbidden:
                    if zone.name not in reported and zone.contains(x, y):
                        reported.add(zone.name)
                        ctx.notify("Intrus", f"une personne dans la zone interdite "
                                             f"« {zone.name} »", priority="urgent")
                        events.append(f"intruder:{zone.name}")

        for place in ROUTE:
            if ctx.should_stop():
                return self._done(events, "STOPPED")
            battery = ctx.battery()
            if battery.known and battery.percent < return_percent:
                ctx.notify("Ronde interrompue", f"batterie à {battery.percent:.0f} % : "
                                                f"retour à la station", priority="info")
                events.append("low_battery")
                if station is not None:
                    events.append(f"{station}:{ctx.go_to(station, timeout_s=300).status}")
                return self._done(events, "LOW_BATTERY")
            if ctx.thermal_status() in HOT:
                ctx.notify("Ronde en pause", "un moteur chauffe", priority="warning")
                events.append("pause")
                while ctx.thermal_status() in HOT and not ctx.should_stop():
                    ctx.sleep(1.0)
                events.append("resume")
            events.append(f"{place}:{ctx.go_to(place, timeout_s=300).status}")
            look()
        return self._done(events, "DONE")

    @staticmethod
    def _done(events, outcome) -> MissionResult:
        return MissionResult(success=outcome != "STOPPED", mission_id="", app_id="",
                             duration_s=0.0, metrics={"events": events, "outcome": outcome})

    def on_stop(self, ctx) -> None:
        pass
