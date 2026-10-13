"""
Voice Inspector : le robot obéit à la voix.

« Va vers le carton. » Le robot écoute, comprend et se déplace.

Ce que cette app démontre, et qui n'est pas visible dans les autres :
entre une phrase prononcée et un moteur qui tourne, il y a quatre
étages, et chacun peut dire non.

    voix
      -> Whisper (modèle)          : rend du TEXTE, jamais une action
      -> registre d'intentions     : le texte doit correspondre à une
                                     intention DÉCLARÉE par cette app,
                                     sinon il ne se passe rien
      -> cette app (code écrit)    : décide quoi faire de l'intention
      -> JointSafetyGuard          : vérifie chaque commande

Whisper est le modèle de reconnaissance vocale (d'OpenAI) qui transcrit
la parole en texte. Une intention est une demande qu'une app déclare
savoir traiter, reconnue à des groupes de mots-clés ; le registre
d'intentions de l'OS confronte le texte transcrit aux intentions
déclarées. JointSafetyGuard est le garde-fou de
l'OS : il vérifie chaque commande d'articulation contre les limites de
celle-ci et la refuse au besoin (axis_os/hal/safety/limits.py).

Une phrase que personne n'a prévue ne produit aucun mouvement. Ce n'est
pas un garde-fou ajouté après coup : c'est la seule chose que
l'architecture permet. Un modèle de parole n'a aucun chemin vers un
moteur.

L'app déclare son vocabulaire au démarrage. Installer une app ajoute
donc des mots que le robot comprend, sans réentraîner aucun modèle.
C'est le principe des intentions d'Android (une app y déclare les
actions qu'elle sait traiter), appliqué à un robot.
"""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Capability, Permission, ResourceQuota,
)

# Réglages de l'approche : mêmes valeurs, et mêmes raisons, que dans
# warehouse_inspector/app.py, où chacune est expliquée.
VITESSE_M_S = 0.4            # vitesse d'avance, m/s
GAIN_CAP = 1.2               # correction de direction, rad/s par rad d'écart
DISTANCE_ARRET_M = 1.2       # distance d'arrêt devant le colis, m
MEMOIRE_S = 2.0              # durée de confiance à une détection perdue, s
HAUTEUR_MIN_M = -0.60        # sous cette hauteur, la détection est le sol
CONFIRMATIONS_ARRIVEE = 3    # images d'affilée avant de conclure

# Vocabulaire de cette app. Chaque intention est décrite par : son nom,
# ses groupes de mots, les paramètres transmis à l'app, une description.
# Un mot de chaque groupe doit être présent : il faut un verbe ET un
# objet. « carton » seul ne fait rien, « va » seul non plus.
INTENTIONS = [
    ("aller_vers_colis",
     [["va", "aller", "avance", "rejoins", "approche"],
      ["carton", "colis", "boite", "paquet"]],
     {"cible": "a cardboard box"},
     "Se rendre devant un colis"),
    ("arreter",
     [["stop", "arrete", "arrête", "halte", "stoppe"]],
     {},
     "Arrêter le déplacement"),
    ("decrire",
     [["decris", "décris", "vois", "voir", "regarde"]],
     {},
     "Dire ce que le robot voit"),
]


class VoiceInspectorApp(RobotApp):

    def manifest(self) -> AppManifest:
        # Le manifeste est la fiche que l'app présente à l'OS : identité,
        # permissions demandées, matériel requis, quotas de ressources.
        return AppManifest(
            app_id="com.axisos.voice_inspector",
            name="Voice Inspector",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@axis-os.local",
            description="Se déplace vers un colis sur commande vocale.",
            category=AppCategory.LOGISTICS,
            permissions=[
                Permission.SENSORS_MICROPHONE,
                Permission.SENSORS_CAMERA,
                Permission.ACTUATORS_WRITE_LEGS,
                Permission.LOCOMOTION_WALK,
            ],
            capabilities_required=[Capability.SPEECH_IN, Capability.WALK,
                                   Capability.CAMERA_RGB],
            quota=ResourceQuota(max_memory_mb=8192, max_runtime_s=300.0),
        )

    def on_start(self, ctx: RobotContext) -> bool:
        declarees = 0
        for nom, mots, params, desc in INTENTIONS:
            if ctx.declare_intent(nom, mots, params, desc):
                declarees += 1
        ctx.log(f"Vocabulaire déclaré : {declarees}/{len(INTENTIONS)} intentions")
        # Une intention refusée signifie qu'une autre app l'a déjà
        # déclarée. Ce n'est pas une raison de ne pas démarrer, mais
        # l'app doit savoir qu'elle ne recevra pas ces phrases.
        return True

    def on_stop(self, ctx: RobotContext) -> None:
        ctx.stop_walking()

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        t0 = ctx.now()
        timeout_s = float(params.get("timeout_s", 120.0))
        stop_m = float(params.get("distance_arret_m", DISTANCE_ARRET_M))

        entendues = 0
        ignorees = 0
        cible: Optional[str] = None
        atteint = False
        distance_finale = None

        # Boucle principale : écouter 2 s, puis, si une cible est en
        # cours, poursuivre l'approche pendant une courte salve (voir
        # _approcher), et recommencer.
        while ctx.now() - t0 < timeout_s and not atteint:
            intent = ctx.listen(duration_s=2.0)

            if intent is None:
                # Cas normal : silence, bruit, ou phrase hors
                # vocabulaire. Le robot ne réagit pas et continue
                # d'écouter : il ne doit pas s'agiter parce qu'il a
                # entendu parler autour de lui. Une approche déjà en
                # cours, elle, se poursuit.
                ignorees += 1
                if cible is None:
                    continue
            else:
                entendues += 1
                ctx.log(f"Compris : « {intent.utterance} » -> {intent.name}")

                if intent.name == "arreter":
                    ctx.stop_walking()
                    cible = None
                    continue
                if intent.name == "decrire":
                    ctx.log(f"Je vois : {ctx.describe_scene()}")
                    continue
                if intent.name == "aller_vers_colis":
                    cible = intent.params.get("cible", "a cardboard box")

            if cible is None:
                continue

            atteint, distance_finale = self._approcher(ctx, cible, stop_m)

        ctx.stop_walking()
        return MissionResult(
            success=atteint,
            mission_id="voix",
            app_id="com.axisos.voice_inspector",
            duration_s=ctx.now() - t0,
            items_processed=1 if atteint else 0,
            metrics={
                "commandes_comprises": entendues,
                "phrases_ignorees": ignorees,
                "cible": cible,
                "distance_finale_m": (round(distance_finale, 2)
                                      if distance_finale else None),
            },
            error_message=None if atteint else (
                "aucune commande comprise" if entendues == 0
                else "colis jamais atteint"),
        )

    def _approcher(self, ctx, cible, stop_m):
        """Une salve d'approche entre deux écoutes, d'au plus 3 s.

        Pourquoi la borner dans le temps : le robot doit rester à
        l'écoute. Un robot qui n'écoute plus pendant qu'il agit ne peut
        pas être arrêté à la voix, et c'est précisément quand il bouge
        qu'on veut pouvoir lui dire stop.

        Même logique d'approche que warehouse_inspector/app.py (filtre du
        sol, mémoire courte d'une détection perdue, confirmation de
        l'arrivée sur plusieurs images). Rend (True, distance) si le colis
        est atteint, sinon (False, None).
        """
        t_salve = ctx.now()
        perdu_depuis = None
        confirmations = 0
        vue = None

        while ctx.now() - t_salve < 3.0:
            # Détections plausibles seulement : celles au niveau du sol
            # sont écartées (voir HAUTEUR_MIN_M).
            dets: List[Any] = [d for d in ctx.look_for(cible)
                               if d.position_3d is None
                               or d.position_3d.z > HAUTEUR_MIN_M]
            if not dets:
                # Colis perdu de vue : on garde le dernier cap pendant
                # MEMOIRE_S, puis on tourne sur place pour le chercher.
                maintenant = ctx.now()
                if perdu_depuis is None:
                    perdu_depuis = maintenant
                if vue is not None and maintenant - perdu_depuis < MEMOIRE_S:
                    ctx.walk(VITESSE_M_S, 0.0, GAIN_CAP * self._cap(vue))
                else:
                    ctx.walk(0.0, 0.0, 0.5)
                ctx.sleep(0.05)
                continue

            perdu_depuis = None
            d = dets[0]
            vue = d
            if d.position_3d is None:
                # Distance inconnue : on s'oriente sans avancer, pour ne
                # pas risquer de heurter le colis.
                ctx.walk(0.0, 0.0, GAIN_CAP * self._cap(d))
                ctx.sleep(0.05)
                continue

            dist = (d.position_3d.x ** 2 + d.position_3d.y ** 2) ** 0.5
            if dist <= stop_m:
                confirmations += 1
                if confirmations >= CONFIRMATIONS_ARRIVEE:
                    ctx.stop_walking()
                    ctx.log(f"Arrivé à {dist:.2f} m du colis.")
                    return True, dist
                # Arrivée pas encore confirmée : on ralentit sans
                # s'arrêter.
                ctx.walk(VITESSE_M_S * 0.3, 0.0, GAIN_CAP * self._cap(d))
            else:
                confirmations = 0
                ctx.walk(VITESSE_M_S, 0.0, GAIN_CAP * self._cap(d))
            ctx.sleep(0.05)

        return False, None

    @staticmethod
    def _cap(d) -> float:
        """Écart angulaire horizontal vers l'objet, en radians, dans le
        repère du robot (x devant, y à gauche) : positif si l'objet est à
        gauche, 0 si la direction est inconnue. Voir _cap_vers dans
        warehouse_inspector/app.py."""
        import math
        v = d.direction
        return math.atan2(v.y, v.x) if v is not None else 0.0
