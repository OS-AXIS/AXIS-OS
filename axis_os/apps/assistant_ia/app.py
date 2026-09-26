"""
Assistant IA — exemple d'app AXIS-OS qui ne fait PAS bouger le robot.

On dit « Robot, ... » : la phrase est transmise à une IA de conversation
— celle de votre entreprise, un modèle hébergé, n'importe quel service
qui répond en JSON — et la réponse est journalisée et rendue dans le
résultat de mission. Le robot ne se déplace pas, ne bouge pas les bras :
l'app n'en a d'ailleurs pas la permission.

C'est l'exemple du guide docs/CREER_UNE_APP.md. Il montre :
  - un manifeste minimal, avec seulement les permissions utiles ;
  - la déclaration d'une intention (« robot » = mot de réveil) ;
  - le branchement d'une IA EXTERNE, par une simple adresse HTTP ;
  - une IA d'essai intégrée, pour développer sans réseau ni robot ;
  - la mémoire persistante propre à l'app.

Ce que l'exemple ne fait PAS, et pourquoi : le robot ne PRONONCE pas la
réponse. Le SDK n'expose pas encore de synthèse vocale (la capacité
SPEECH_OUT existe, pas de ctx.speak()). La réponse est dans le journal et
dans MissionResult.metrics["reponses"].

Contrat HTTP de l'IA externe (volontairement minimal) :
    POST <ia_url>   {"question": "...", "historique": [["q", "r"], ...]}
    réponse         {"reponse": "..."}
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any, Dict, List, Tuple

from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Capability, Permission, ResourceQuota,
)

APP_ID = "com.exemple.assistant_ia"


# ── L'IA : une interface, deux implémentations ─────────────────────────────

class ModeleConversation:
    """Ce que l'app attend d'une IA : une réponse à une question."""

    def repondre(self, question: str, historique: List[Tuple[str, str]]) -> str:
        raise NotImplementedError


class ModeleEssai(ModeleConversation):
    """IA d'essai, hors ligne : pour développer sans réseau ni clé."""

    def repondre(self, question, historique):
        return f"(IA d'essai) Vous avez dit : « {question} ». " \
               f"C'est notre échange n° {len(historique) + 1}."


class ModeleHTTP(ModeleConversation):
    """Votre IA, derrière une adresse web. Aucune dépendance à installer."""

    def __init__(self, url: str, delai_s: float = 10.0):
        self.url, self.delai_s = url, delai_s

    def repondre(self, question, historique):
        corps = json.dumps({"question": question,
                            "historique": [list(h) for h in historique]}).encode()
        req = urllib.request.Request(self.url, data=corps, method="POST",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.delai_s) as r:
            return str(json.loads(r.read().decode())["reponse"])


# ── L'app ─────────────────────────────────────────────────────────────────

class AssistantIAApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id=APP_ID,
            name="Assistant IA",
            version="1.0.0",
            author="Exemple",
            author_email="dev@exemple.com",
            description="Transmet ce qu'on dit au robot à une IA de conversation.",
            category=AppCategory.UTILITIES,
            permissions=[
                Permission.SENSORS_MICROPHONE,   # écouter : listen, declare_intent
                Permission.STORAGE_PERSISTENT,   # retenir le nombre d'échanges
                # Déclarée par HONNÊTETÉ envers l'utilisateur : l'app parle à
                # un service en ligne. Pas encore appliquée par le sandbox
                # (voir Permission dans manifest.py).
                Permission.NETWORK_CLOUD,
            ],
            capabilities_required=[Capability.SPEECH_IN],   # un robot sans micro : refusée
            quota=ResourceQuota(max_runtime_s=120.0),
            max_velocity_ratio=0.1,              # elle ne bouge rien, de toute façon
            requires_human_supervision=False,
        )

    def on_start(self, ctx: RobotContext) -> bool:
        # « robot » ou « axis » : le mot de réveil. Sans lui, rien ne part
        # vers l'IA — une conversation voisine n'est pas envoyée en ligne.
        ok = ctx.declare_intent("parler_a_l_ia", [["robot", "axis"]],
                                description="Poser une question à l'IA")
        ok &= ctx.declare_intent("fin_conversation", [["au revoir", "termine"]],
                                 description="Finir la conversation")
        return ok

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        t0 = ctx.now()
        url = params.get("ia_url")
        ia = ModeleHTTP(url) if url else ModeleEssai()
        ctx.log(f"IA : {'externe, ' + url if url else 'essai (hors ligne)'}")
        historique: List[Tuple[str, str]] = []
        reponses, erreurs, silences = [], 0, 0
        for _ in range(int(params.get("ecoutes_max", 20))):
            intention = ctx.listen(duration_s=float(params.get("ecoute_s", 4.0)))
            if intention is None:
                silences += 1                       # silence, bruit : NORMAL
                continue
            if intention.name == "fin_conversation":
                ctx.log("Fin de conversation demandée.")
                break
            question = intention.utterance
            try:
                reponse = ia.repondre(question, historique)
            except (urllib.error.URLError, TimeoutError, KeyError, ValueError) as exc:
                # L'IA est injoignable ou répond mal : on le DIT, on ne
                # plante pas, on n'invente pas de réponse.
                erreurs += 1
                ctx.log(f"IA indisponible ({exc.__class__.__name__}) — question ignorée")
                continue
            historique.append((question, reponse))
            reponses.append(reponse)
            ctx.log(f"Humain : {question}")
            ctx.log(f"IA     : {reponse}")
        total = int(ctx.recall("echanges_total", 0)) + len(reponses)
        ctx.remember("echanges_total", total)
        return MissionResult(
            success=erreurs == 0, mission_id="", app_id=APP_ID,
            duration_s=ctx.now() - t0, items_processed=len(reponses),
            error_message=f"{erreurs} question(s) sans réponse de l'IA" if erreurs else None,
            metrics={"reponses": reponses, "silences": silences,
                     "erreurs_ia": erreurs, "echanges_total": total})

    def on_stop(self, ctx: RobotContext) -> None:
        ctx.log("Assistant IA arrêté.")
