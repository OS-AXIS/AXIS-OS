"""
AXIS-OS — Manifeste d'app <-> JSON
===================================
En production, le code d'une app n'est jamais exécuté dans le processus
de l'OS, pas même sa méthode manifest() : le processus de l'app l'appelle
et renvoie le manifeste en JSON par le pont (voir hote_app.py). Ce module
fait la conversion dans les deux sens. Côté OS, le JSON reçu est vérifié
champ par champ (SCHEMA_MANIFESTE), puis le manifeste reconstruit passe
par AppManifest.validate() comme tout autre manifeste.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations

from typing import Any, Dict

from axis_os.hal.core.capabilities import RobotCapability
from axis_os.store.runtime.canal_ipc import (
    Booleen, Chaine, Entier, Facultatif, Liste, MessageRefuse, Nombre, Objet, UnParmi,
)
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Capability, HardwareRequirements, IntentDecl, Permission,
    PricingModel, ResourceQuota,
)


def _noms(enum) -> UnParmi:
    return UnParmi(*(m.name for m in enum))


SCHEMA_MANIFESTE = Objet({
    "app_id": Chaine(128), "name": Chaine(128), "version": Chaine(64),
    "author": Chaine(128), "author_email": Chaine(256), "description": Chaine(4096),
    "category": _noms(AppCategory),
    "permissions": Liste(_noms(Permission), 64),
    "hardware": Objet({
        "required": Liste(_noms(RobotCapability), 32),
        "optional": Liste(_noms(RobotCapability), 32),
        "min_dof": Entier(0, 10_000),
        "compatible_models": Liste(Chaine(128), 64),
    }),
    "quota": Objet({
        "max_cpu_percent": Nombre(0, 100_000),
        "max_memory_mb": Entier(0, 1_000_000),
        "max_runtime_s": Nombre(0, 1_000_000),
    }),
    "capabilities_required": Liste(_noms(Capability), 64),
    "intents": Liste(Objet({"intent_id": Chaine(128), "description": Chaine(1024),
                            "examples": Liste(Chaine(1024), 64)}), 64),
    "pricing_model": _noms(PricingModel),
    "base_price_eur": Nombre(0, 1_000_000),
    "max_velocity_ratio": Nombre(0, 1_000),
    "requires_human_supervision": Booleen(),
}, facultatifs={
    "icon_url": Chaine(2048), "website_url": Chaine(2048), "support_email": Chaine(256),
    "sdk_version_min": Chaine(64),
    # Renvoyés pour information seulement : l'OS ne les reprend jamais
    # (voir manifeste_depuis_json).
    "code_signature": Chaine(512), "developer_public_key": Chaine(512),
})


def manifeste_vers_json(m: AppManifest) -> Dict[str, Any]:
    """AppManifest -> dict JSON (enums par leur nom)."""
    hw, q = m.hardware, m.quota
    return {
        "app_id": m.app_id, "name": m.name, "version": m.version, "author": m.author,
        "author_email": m.author_email, "description": m.description,
        "category": m.category.name,
        "permissions": [p.name for p in m.permissions],
        "hardware": {"required": [c.name for c in hw.required],
                     "optional": [c.name for c in hw.optional],
                     "min_dof": int(hw.min_dof),
                     "compatible_models": list(hw.compatible_models)},
        "quota": {"max_cpu_percent": float(q.max_cpu_percent),
                  "max_memory_mb": int(q.max_memory_mb),
                  "max_runtime_s": float(q.max_runtime_s)},
        "capabilities_required": [c.name for c in m.capabilities_required],
        "intents": [{"intent_id": i.intent_id, "description": i.description,
                     "examples": list(i.examples)} for i in m.intents],
        "pricing_model": m.pricing_model.name,
        "base_price_eur": float(m.base_price_eur),
        "max_velocity_ratio": float(m.max_velocity_ratio),
        "requires_human_supervision": bool(m.requires_human_supervision),
        "icon_url": m.icon_url, "website_url": m.website_url,
        "support_email": m.support_email, "sdk_version_min": m.sdk_version_min,
        "code_signature": m.code_signature, "developer_public_key": m.developer_public_key,
    }


def manifeste_depuis_json(d: Any) -> AppManifest:
    """dict JSON reçu d'un processus d'app -> AppManifest. Lève
    MessageRefuse si le dict ne suit pas SCHEMA_MANIFESTE.

    code_signature et developer_public_key ne sont jamais repris du JSON :
    l'appelant y met les valeurs qu'il a lui-même vérifiées, pour qu'une
    app ne puisse pas annoncer une autre signature que celle contrôlée."""
    v = SCHEMA_MANIFESTE.verifier(d, "manifeste")
    hw, q = v["hardware"], v["quota"]
    return AppManifest(
        app_id=v["app_id"], name=v["name"], version=v["version"], author=v["author"],
        author_email=v["author_email"], description=v["description"],
        category=AppCategory[v["category"]],
        permissions=[Permission[p] for p in v["permissions"]],
        hardware=HardwareRequirements(
            required=[RobotCapability[c] for c in hw["required"]],
            optional=[RobotCapability[c] for c in hw["optional"]],
            min_dof=hw["min_dof"], compatible_models=list(hw["compatible_models"])),
        quota=ResourceQuota(max_cpu_percent=float(q["max_cpu_percent"]),
                            max_memory_mb=int(q["max_memory_mb"]),
                            max_runtime_s=float(q["max_runtime_s"])),
        capabilities_required=[Capability[c] for c in v["capabilities_required"]],
        intents=[IntentDecl(intent_id=i["intent_id"], description=i["description"],
                            examples=list(i["examples"])) for i in v["intents"]],
        pricing_model=PricingModel[v["pricing_model"]],
        base_price_eur=float(v["base_price_eur"]),
        max_velocity_ratio=float(v["max_velocity_ratio"]),
        requires_human_supervision=v["requires_human_supervision"],
        icon_url=v["icon_url"], website_url=v["website_url"],
        support_email=v["support_email"], sdk_version_min=v["sdk_version_min"],
    )


__all__ = ["SCHEMA_MANIFESTE", "manifeste_vers_json", "manifeste_depuis_json", "MessageRefuse"]
