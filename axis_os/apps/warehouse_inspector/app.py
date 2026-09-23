"""
Warehouse Inspector — première app AXIS-OS qui voit ET se déplace.

Mission : chercher un colis, marcher jusqu'à lui, s'arrêter à bonne
distance, et rapporter ce qui a été trouvé.

CE QU'ELLE MONTRE : une app métier tient en quelques dizaines de lignes.
Elle ne connaît ni la marche bipède, ni la politique apprise, ni les
conventions de signe, ni la calibration de la caméra. Elle déclare une
intention — « avance », « cherche un colis » — et l'OS fait le reste,
en vérifiant chaque commande.

TROIS RÈGLES QU'UNE APP N'A PAS À FAIRE RESPECTER ELLE-MÊME, et c'est
tout l'intérêt :
  - si elle plante, le robot s'arrête et reste debout (échéance de
    fraîcheur du service de locomotion) ;
  - si elle demande une vitesse hors domaine, elle est bornée ;
  - si elle commande une articulation hors limites, c'est refusé.

CE QU'ELLE FAIT, EN REVANCHE, ET QU'AUCUNE IA NE PEUT DEVINER : à quelle
distance s'arrêter d'un colis, combien de temps chercher avant
d'abandonner, quoi faire si la distance est inconnue, que rapporter à la
fin. C'est la connaissance métier — elle appartient à l'app.
"""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Capability, Permission, ResourceQuota,
)

CIBLE = "a cardboard box"
DISTANCE_ARRET_M = 0.8       # devant le colis, pas dessus
VITESSE_M_S = 0.4
GAIN_CAP = 1.2               # correction de direction, rad/s par rad d'écart
TIMEOUT_RECHERCHE_S = 20.0
MEMOIRE_S = 2.0              # durée de confiance à une détection perdue


class WarehouseInspectorApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.axisos.warehouse_inspector",
            name="Warehouse Inspector",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@axis-os.local",
            description="Repère un colis, s'en approche et le signale.",
            category=AppCategory.LOGISTICS,
            permissions=[
                Permission.SENSORS_CAMERA,
                # Les DEUX sont nécessaires pour se déplacer : bouger les
                # jambes, et changer de place dans le local.
                Permission.ACTUATORS_WRITE_LEGS,
                Permission.LOCOMOTION_WALK,
            ],
            capabilities_required=[Capability.WALK, Capability.CAMERA_RGB],
            # Le quota doit dépasser le délai de recherche, sinon le
            # sandbox tue la mission avant qu'elle ne rende ses
            # résultats — constaté : métriques vides après un
            # TIMEOUT à la seconde près.
            quota=ResourceQuota(max_memory_mb=8192, max_runtime_s=300.0),
        )

    def on_start(self, ctx: RobotContext) -> bool:
        ctx.log(f"Inspection : recherche de « {CIBLE} »")
        return True

    def on_stop(self, ctx: RobotContext) -> None:
        # Toujours demander l'arrêt en partant. Même si l'OS le ferait de
        # lui-même après expiration, une app propre ne laisse pas une
        # consigne courir derrière elle.
        ctx.stop_walking()

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        t0 = time.monotonic()
        cible = params.get("cible", CIBLE)
        stop_m = float(params.get("distance_arret_m", DISTANCE_ARRET_M))
        # Réglable : en simulation, le temps réel avance plus vite que le
        # temps simulé, et un délai calibré pour le robot réel expire
        # avant que quoi que ce soit ait eu lieu.
        timeout_s = float(params.get("timeout_s", TIMEOUT_RECHERCHE_S))

        vue: Optional[Any] = None
        approches = 0
        sans_distance = 0
        perdu_depuis: Optional[float] = None
        pertes = 0

        while time.monotonic() - t0 < timeout_s:
            detections: List[Any] = ctx.look_for(cible)

            if not detections:
                # MÉMOIRE DE COURTE DURÉE. Une détection est
                # intermittente : un colis lointain passe sous le seuil de
                # confiance une image sur deux. Sans mémoire, l'app
                # pivotait à chaque perte et perdait son cap — 20
                # commandes d'approche seulement sur toute une mission,
                # le reste passé à tourner sur place.
                #
                # Tant que la perte est récente, on continue sur le
                # dernier cap connu. C'est de la connaissance métier :
                # combien de temps faire confiance à une observation
                # périmée dépend de la tâche, pas du robot.
                maintenant = time.monotonic()
                if perdu_depuis is None:
                    perdu_depuis = maintenant
                    pertes += 1
                if vue is not None and maintenant - perdu_depuis < MEMOIRE_S:
                    ctx.walk(VITESSE_M_S, 0.0, GAIN_CAP * self._cap_vers(vue))
                    approches += 1
                else:
                    # Perdu depuis trop longtemps : balayer le local.
                    ctx.walk(0.0, 0.0, 0.5)
                time.sleep(0.05)
                continue

            perdu_depuis = None
            d = detections[0]          # la plus confiante
            vue = d

            if d.position_3d is None:
                # Direction connue, distance inconnue : on s'oriente sans
                # avancer. Avancer vers un objet dont on ignore la
                # distance, c'est accepter de le heurter.
                sans_distance += 1
                cap = self._cap_vers(d)
                ctx.walk(0.0, 0.0, GAIN_CAP * cap)
                time.sleep(0.05)
                continue

            distance = (d.position_3d.x ** 2 + d.position_3d.y ** 2) ** 0.5
            if distance <= stop_m:
                ctx.stop_walking()
                ctx.log(f"Colis atteint à {distance:.2f} m "
                        f"(consigne d'arrêt {stop_m:.2f} m)")
                return self._resultat(ctx, t0, True, d, distance,
                                      approches, sans_distance, pertes)

            # Avancer en corrigeant le cap : le colis n'est pas forcément
            # droit devant.
            cap = self._cap_vers(d)
            ctx.walk(VITESSE_M_S, 0.0, GAIN_CAP * cap)
            approches += 1
            time.sleep(0.05)

        ctx.stop_walking()
        ctx.log(f"Aucun colis atteint en {timeout_s:.0f} s.")
        return self._resultat(ctx, t0, False, vue, None, approches,
                              sans_distance, pertes)

    @staticmethod
    def _cap_vers(d) -> float:
        """Écart angulaire horizontal vers l'objet, en radians.

        La direction est déjà dans le repère du robot : x devant, y à
        gauche. L'angle est donc atan2(y, x) — positif si l'objet est à
        gauche, donc il faut tourner à gauche.
        """
        import math
        v = d.direction
        return math.atan2(v.y, v.x) if v is not None else 0.0

    def _resultat(self, ctx, t0, ok, d, distance, approches, sans_distance,
                  pertes=0):
        metrics = {
            "approches": approches,
            "images_sans_distance": sans_distance,
            "pertes_de_vue": pertes,
            "colis_vu": d is not None,
        }
        if d is not None:
            metrics["confiance"] = round(float(d.confidence), 3)
            if d.position_3d is not None:
                metrics["position"] = [round(d.position_3d.x, 2),
                                       round(d.position_3d.y, 2),
                                       round(d.position_3d.z, 2)]
        if distance is not None:
            metrics["distance_finale_m"] = round(distance, 2)

        return MissionResult(
            success=ok,
            mission_id="inspection",
            app_id="com.axisos.warehouse_inspector",
            duration_s=time.monotonic() - t0,
            items_processed=1 if ok else 0,
            metrics=metrics,
            error_message=None if ok else (
                "colis vu mais jamais atteint" if d is not None
                else "aucun colis trouvé"),
        )
