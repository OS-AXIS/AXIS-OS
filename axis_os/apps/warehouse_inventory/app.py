"""
Warehouse Inventory — le robot recense ce qu'il voit, et s'en souvient.

Le robot balaie la zone, repère les colis, note leur position, et rend
un inventaire. À la mission suivante, il SAIT déjà ce qu'il avait vu :
il distingue ce qui est nouveau, ce qui est toujours là, et ce qui a
disparu.

CE QUE CETTE APP DÉMONTRE, et que les précédentes ne montraient pas :
une app a une VIE au-delà d'une mission. Sa mémoire survit au
redémarrage du robot, elle lui appartient — une autre app ne peut ni la
lire ni l'écraser —, et c'est elle qui transforme une suite
d'observations en information utile.

DEUX DÉCISIONS MÉTIER, que ni la perception ni l'IA ne peuvent prendre :

  À QUELLE DISTANCE deux observations désignent-elles le même colis ?
  Trop serré, un colis est compté deux fois parce que la mesure a bougé
  de 20 cm ; trop large, deux colis voisins n'en font qu'un. La bonne
  valeur dépend de la taille des objets manipulés — un entrepôt de
  palettes et un atelier de petites pièces n'ont pas la même réponse.

  COMBIEN DE FOIS faut-il voir un colis avant de l'inscrire ? Une
  détection isolée peut être un reflet ou une erreur du modèle.
  Inscrire trop vite pollue l'inventaire ; trop tard en oublie.

Ces deux nombres sont le cœur de l'app. Tout le reste est du cadre.
"""
from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional

from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Capability, Permission, ResourceQuota,
)

CIBLE = "a cardboard box"
CLE_INVENTAIRE = "inventaire"

RAYON_FUSION_M = 0.8
"""Deux observations à moins de cette distance désignent le MÊME colis.

0.8 m : plus large qu'un colis de manutention (0.4 à 0.6 m), assez pour
absorber l'erreur de mesure constatée (12 cm à 1 m, davantage de loin),
assez serré pour distinguer deux colis posés côte à côte."""

VUES_MINIMUM = 2
"""Nombre d'observations avant d'inscrire un colis à l'inventaire.

Une détection isolée peut être un reflet, le sol, ou une erreur du
modèle — on a mesuré le plancher détecté comme carton avec une confiance
de 0.129. Deux observations coûtent une seconde et éliminent l'essentiel
de ces faux positifs."""

VITESSE_ROTATION = 0.5
HAUTEUR_MIN_M = -0.60


class WarehouseInventoryApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.axisos.warehouse_inventory",
            name="Warehouse Inventory",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@axis-os.local",
            description="Recense les colis d'une zone et mémorise leur position.",
            category=AppCategory.LOGISTICS,
            permissions=[
                Permission.SENSORS_CAMERA,
                Permission.STORAGE_PERSISTENT,
                # Tourner sur place est un déplacement : mêmes
                # permissions que pour marcher.
                Permission.ACTUATORS_WRITE_LEGS,
                Permission.LOCOMOTION_WALK,
            ],
            capabilities_required=[Capability.CAMERA_RGB, Capability.WALK,
                                   Capability.MEMORY_PERSISTENT],
            quota=ResourceQuota(max_memory_mb=8192, max_runtime_s=300.0),
        )

    def on_start(self, ctx: RobotContext) -> bool:
        connus = ctx.recall(CLE_INVENTAIRE, default=[])
        ctx.log(f"Inventaire précédent : {len(connus)} colis connu(s)")
        return True

    def on_stop(self, ctx: RobotContext) -> None:
        ctx.stop_walking()

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        t0 = time.monotonic()
        duree = float(params.get("duree_balayage_s", 30.0))
        cible = params.get("cible", CIBLE)

        # L'inventaire précédent : une liste de dicts, pas des objets —
        # ce qui est stocké doit survivre à une mise à jour de l'app.
        anciens: List[dict] = list(ctx.recall(CLE_INVENTAIRE, default=[]) or [])
        observes: List[dict] = []          # colis vus PENDANT cette mission

        while time.monotonic() - t0 < duree:
            # Balayage : tourner sur place montre toute la zone sans
            # quitter le point de départ. Une app d'inventaire ne doit
            # pas dériver loin de sa position de référence, sinon les
            # coordonnées mémorisées perdent leur sens.
            ctx.walk(0.0, 0.0, VITESSE_ROTATION)

            for d in ctx.look_for(cible):
                if d.position_3d is None:
                    continue           # sans distance, impossible de situer
                if d.position_3d.z <= HAUTEUR_MIN_M:
                    continue           # le sol, pas un colis
                self._noter(observes, d)

            time.sleep(0.05)

        ctx.stop_walking()

        retenus = [o for o in observes if o["vues"] >= VUES_MINIMUM]
        nouveaux, revus = self._comparer(anciens, retenus)
        disparus = self._disparus(anciens, retenus)

        # On mémorise l'inventaire COURANT, pas l'accumulation : un colis
        # emporté doit disparaître de la mémoire, sinon l'inventaire ne
        # décrit plus l'entrepôt mais son histoire.
        ctx.remember(CLE_INVENTAIRE, retenus)

        for o in retenus:
            ctx.log(f"  colis à ({o['x']:+.2f}, {o['y']:+.2f}) "
                    f"vu {o['vues']}x, confiance {o['confiance']:.2f}")
        ctx.log(f"Inventaire : {len(retenus)} colis — {len(nouveaux)} nouveau(x), "
                f"{len(revus)} déjà connu(s), {len(disparus)} disparu(s)")

        return MissionResult(
            success=len(retenus) > 0,
            mission_id="inventaire",
            app_id="com.axisos.warehouse_inventory",
            duration_s=time.monotonic() - t0,
            items_processed=len(retenus),
            metrics={
                "colis_inventories": len(retenus),
                "observations_rejetees": len(observes) - len(retenus),
                "nouveaux": len(nouveaux),
                "deja_connus": len(revus),
                "disparus": len(disparus),
                "connus_avant": len(anciens),
            },
            error_message=None if retenus else "aucun colis recensé",
        )

    # ---- logique métier -------------------------------------------------

    def _noter(self, observes: List[dict], d) -> None:
        """Ajoute une observation, ou renforce un colis déjà vu."""
        x, y = float(d.position_3d.x), float(d.position_3d.y)
        for o in observes:
            if math.hypot(o["x"] - x, o["y"] - y) <= RAYON_FUSION_M:
                # Moyenne courante : chaque observation affine la
                # position plutôt que de la remplacer. Une seule mesure
                # aberrante ne déplace pas un colis vu dix fois.
                n = o["vues"]
                o["x"] = (o["x"] * n + x) / (n + 1)
                o["y"] = (o["y"] * n + y) / (n + 1)
                o["vues"] = n + 1
                o["confiance"] = max(o["confiance"], float(d.confidence))
                return
        observes.append({"x": x, "y": y, "vues": 1,
                         "confiance": float(d.confidence),
                         "label": d.label})

    @staticmethod
    def _comparer(anciens: List[dict], retenus: List[dict]):
        nouveaux, revus = [], []
        for r in retenus:
            if any(math.hypot(a["x"] - r["x"], a["y"] - r["y"]) <= RAYON_FUSION_M
                   for a in anciens):
                revus.append(r)
            else:
                nouveaux.append(r)
        return nouveaux, revus

    @staticmethod
    def _disparus(anciens: List[dict], retenus: List[dict]):
        return [a for a in anciens
                if not any(math.hypot(a["x"] - r["x"], a["y"] - r["y"])
                           <= RAYON_FUSION_M for r in retenus)]
