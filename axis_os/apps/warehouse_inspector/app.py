"""
Warehouse Inspector : la première app AXIS-OS qui voit et se déplace.

Mission : chercher un colis, marcher jusqu'à lui, s'arrêter à bonne
distance et rapporter ce qui a été trouvé.

Ce qu'elle montre : une app métier tient en quelques dizaines de lignes.
Elle ignore tout de la marche bipède, de la politique apprise qui la
produit (le modèle d'IA entraîné qui calcule les mouvements des jambes),
des conventions de signe des axes et de la calibration de la caméra.
Elle déclare une intention (« avance », « cherche un colis ») et l'OS
fait le reste, en vérifiant chaque commande.

Trois règles qu'une app n'a pas à faire respecter elle-même, et c'est
tout l'intérêt :
  - si elle plante, le robot s'arrête et reste debout : le service de
    locomotion applique une échéance de fraîcheur, c'est-à-dire qu'une
    consigne de marche que l'app cesse de renouveler retombe à zéro et le
    robot s'immobilise en équilibre (voir
    axis_os/services/locomotion/service.py) ;
  - si elle demande une vitesse hors du domaine autorisé, cette vitesse
    est bornée ;
  - si elle commande une articulation au-delà de ses limites, la
    commande est refusée.

Ce qu'elle fait, en revanche, et qu'aucune IA ne peut deviner : à quelle
distance s'arrêter d'un colis, combien de temps chercher avant
d'abandonner, quoi faire si la distance est inconnue, que rapporter à la
fin. C'est la connaissance métier, et elle appartient à l'app.

Les positions utilisées ici sont exprimées dans le repère du robot :
x vers l'avant, y vers la gauche, z vers le haut, origine au bassin.
"""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Capability, Permission, ResourceQuota,
)

CIBLE = "a cardboard box"    # description de l'objet cherché, en anglais
DISTANCE_ARRET_M = 1.2
"""Distance d'arrêt devant le colis, en mètres.

Cette valeur tient à une raison physique, pas au hasard : la caméra,
montée sur la tête et inclinée vers l'avant, a une zone aveugle près des
pieds. En dessous d'environ 1 m, le colis sort du champ et la mesure de
distance se dégrade. Constaté : un robot au contact du colis le mesurait
encore à 0.94 m, parce qu'il ne le voyait plus.

C'est aussi ce qu'on attend d'un robot d'inspection : se placer devant un
colis pour le lire, pas le heurter. Un robot qui devrait le saisir aurait
besoin d'une caméra au poignet ou d'un capteur de proximité, matériel que
n'a pas le G1 23 DOF (l'humanoïde G1 d'Unitree dans sa version à 23
degrés de liberté, c'est-à-dire 23 articulations motorisées)."""

HAUTEUR_MIN_M = -0.60
"""Hauteur minimale d'une détection valable, en mètres, dans le repère
du robot (origine au bassin).

Le bassin étant à environ 0.79 m du sol, une détection sous -0.60 m est
au niveau du sol : c'est le plancher pris pour un carton, pas un colis.
Pour comparaison, un colis de 60 cm posé au sol a sa face avant vers
-0.5 m.

Le seuil a été resserré après un faux positif (une détection qui ne
correspond à aucun objet réel) mesuré à -0.69 m : le sol devant le
robot, détecté comme carton avec une confiance de 0.129, avait fait
conclure à l'app qu'elle était arrivée sans avoir bougé."""

CONFIRMATIONS_ARRIVEE = 3
"""Nombre d'images consécutives sous la distance d'arrêt avant de
conclure à l'arrivée.

Une seule image ne prouve rien : une détection isolée peut être un
reflet, le sol, ou une boîte mal placée par le modèle de détection.
Exiger plusieurs images d'affilée coûte quelques dixièmes de seconde et
évite de conclure sur un coup de chance. C'est de la logique métier : la
perception ne peut pas savoir combien de preuves une tâche exige."""
VITESSE_M_S = 0.4            # vitesse d'avance, m/s
GAIN_CAP = 1.2               # correction de direction, rad/s par rad d'écart
TIMEOUT_RECHERCHE_S = 20.0   # durée maximale de la recherche, s
MEMOIRE_S = 2.0              # durée de confiance à une détection perdue, s


class WarehouseInspectorApp(RobotApp):

    def manifest(self) -> AppManifest:
        # Le manifeste est la fiche que l'app présente à l'OS : identité,
        # permissions demandées, matériel requis, quotas de ressources.
        # L'OS refuse à l'app tout ce qui n'y est pas déclaré.
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
                # Les deux permissions suivantes sont nécessaires pour se
                # déplacer : l'une autorise à bouger les jambes, l'autre
                # à changer de place dans le local.
                Permission.ACTUATORS_WRITE_LEGS,
                Permission.LOCOMOTION_WALK,
            ],
            capabilities_required=[Capability.WALK, Capability.CAMERA_RGB],
            # La durée autorisée doit dépasser le délai de recherche,
            # sinon le sandbox (le processus isolé dans lequel l'OS
            # exécute chaque app) tue la mission avant qu'elle ne rende
            # ses résultats. Constaté : avec une durée autorisée trop
            # juste, la mission a été arrêtée pour dépassement de durée
            # (TIMEOUT) à la seconde près, et ses métriques sont restées
            # vides.
            quota=ResourceQuota(max_memory_mb=8192, max_runtime_s=300.0),
        )

    def on_start(self, ctx: RobotContext) -> bool:
        ctx.log(f"Inspection : recherche de « {CIBLE} »")
        return True

    def on_stop(self, ctx: RobotContext) -> None:
        # Toujours demander l'arrêt en partant. L'OS arrêterait le robot
        # de lui-même une fois la consigne expirée, mais une app propre ne
        # laisse pas une consigne courir derrière elle.
        ctx.stop_walking()

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        t0 = ctx.now()
        cible = params.get("cible", CIBLE)
        stop_m = float(params.get("distance_arret_m", DISTANCE_ARRET_M))
        # Délai réglable par paramètre : en simulation, le temps réel
        # avance plus vite que le temps simulé, si bien qu'un délai
        # calibré pour le robot réel expire avant que quoi que ce soit ait
        # eu lieu.
        timeout_s = float(params.get("timeout_s", TIMEOUT_RECHERCHE_S))

        vue: Optional[Any] = None
        approches = 0
        sans_distance = 0
        perdu_depuis: Optional[float] = None
        pertes = 0
        confirmations = 0

        while ctx.now() - t0 < timeout_s:
            detections: List[Any] = ctx.look_for(cible)

            if not detections:
                # Mémoire de courte durée. La détection est
                # intermittente : un colis lointain passe sous le seuil de
                # confiance une image sur deux. Sans mémoire, l'app
                # pivotait à chaque perte et perdait son cap : 20
                # commandes d'approche seulement sur toute une mission,
                # le reste du temps passé à tourner sur place.
                #
                # Tant que la perte est récente (moins de MEMOIRE_S), on
                # continue sur le dernier cap connu. C'est de la
                # connaissance métier : combien de temps se fier à une
                # observation périmée dépend de la tâche, pas du robot.
                maintenant = ctx.now()
                if perdu_depuis is None:
                    perdu_depuis = maintenant
                    pertes += 1
                if vue is not None and maintenant - perdu_depuis < MEMOIRE_S:
                    ctx.walk(VITESSE_M_S, 0.0, GAIN_CAP * self._cap_vers(vue))
                    approches += 1
                else:
                    # Perdu depuis trop longtemps : on tourne sur place
                    # pour balayer le local.
                    ctx.walk(0.0, 0.0, 0.5)
                ctx.sleep(0.05)
                continue

            # Écarter les détections au niveau du sol : à faible
            # confiance, OWL-ViT (le modèle qui détecte des objets décrits
            # par une phrase libre) désigne parfois le plancher. Avancer
            # vers un « colis » qui est en réalité le sol fait tourner le
            # robot en rond.
            plausibles = [x for x in detections
                          if x.position_3d is None
                          or x.position_3d.z > HAUTEUR_MIN_M]
            if not plausibles:
                ctx.walk(0.0, 0.0, 0.4)
                ctx.sleep(0.05)
                continue

            perdu_depuis = None
            d = plausibles[0]          # la plus confiante des plausibles
            vue = d

            if d.position_3d is None:
                # Direction connue, distance inconnue : on s'oriente sans
                # avancer, car avancer vers un objet dont on ignore la
                # distance, c'est accepter de le heurter.
                sans_distance += 1
                cap = self._cap_vers(d)
                ctx.walk(0.0, 0.0, GAIN_CAP * cap)
                ctx.sleep(0.05)
                continue

            distance = (d.position_3d.x ** 2 + d.position_3d.y ** 2) ** 0.5
            if distance <= stop_m:
                confirmations += 1
                if confirmations >= CONFIRMATIONS_ARRIVEE:
                    ctx.stop_walking()
                    ctx.log(f"Colis atteint à {distance:.2f} m, confirmé sur "
                            f"{confirmations} images")
                    return self._resultat(ctx, t0, True, d, distance,
                                          approches, sans_distance, pertes)
                # Arrivée pas encore confirmée : on ralentit (30 % de la
                # vitesse) sans s'arrêter.
                ctx.walk(VITESSE_M_S * 0.3, 0.0, GAIN_CAP * self._cap_vers(d))
                ctx.sleep(0.05)
                continue
            confirmations = 0

            # Avancer en corrigeant le cap, car le colis n'est pas
            # forcément droit devant.
            cap = self._cap_vers(d)
            ctx.walk(VITESSE_M_S, 0.0, GAIN_CAP * cap)
            approches += 1
            ctx.sleep(0.05)

        ctx.stop_walking()
        ctx.log(f"Aucun colis atteint en {timeout_s:.0f} s.")
        return self._resultat(ctx, t0, False, vue, None, approches,
                              sans_distance, pertes)

    @staticmethod
    def _cap_vers(d) -> float:
        """Écart angulaire horizontal vers l'objet, en radians.

        La direction est déjà exprimée dans le repère du robot (x devant,
        y à gauche). L'angle vaut donc atan2(y, x) : positif quand l'objet
        est à gauche, ce qui commande une rotation vers la gauche. Rend 0
        si la direction est inconnue.
        """
        import math
        v = d.direction
        return math.atan2(v.y, v.x) if v is not None else 0.0

    def _resultat(self, ctx, t0, ok, d, distance, approches, sans_distance,
                  pertes=0):
        """Construit le résultat de mission. Les métriques qui dépendent
        d'une détection (confiance, position, distance finale) ne sont
        présentes que si elle existe."""
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
            duration_s=ctx.now() - t0,
            items_processed=1 if ok else 0,
            metrics=metrics,
            error_message=None if ok else (
                "colis vu mais jamais atteint" if d is not None
                else "aucun colis trouvé"),
        )
