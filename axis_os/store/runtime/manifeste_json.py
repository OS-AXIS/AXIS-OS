"""
AXIS-OS — Manifeste d'app <-> JSON
===================================
Une app installée depuis sa source (PlayStoreRuntime.install_source, seul
chemin en production) déclare son manifeste à côté de son code, en JSON,
et les deux sont signés ensemble (signing.sign_app_package) : l'OS lit le
manifeste sans exécuter une ligne de l'app. Ce module fait la conversion
dans les deux sens. Côté OS, le JSON est vérifié champ par champ
(SCHEMA_MANIFESTE), puis le manifeste reconstruit passe par
AppManifest.validate() comme tout autre manifeste.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations

import json
from typing import Any, Dict

from axis_os.hal.core.capabilities import RobotCapability
from axis_os.store.runtime.canal_ipc import (
    TAILLE_MAX_TRAME, Booleen, Chaine, Entier, Facultatif, JsonLibre, Liste, MessageRefuse, Nombre,
    Objet, UnParmi, decoder,
)
from axis_os.store.sdk.manifest import (
    SETTING_TYPES, AppCategory, AppManifest, Capability, HardwareRequirements, IntentDecl,
    Permission, PricingModel, ResourceQuota, SettingSpec,
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
    "min_sdk_version": Chaine(64),
    # Absents du texte quand l'app n'en déclare pas : le texte d'un
    # manifeste signé avant leur ajout reste identique, donc sa signature
    # valide.
    "settings": Liste(Objet({"name": Chaine(32), "type": UnParmi(*SETTING_TYPES),
                             "default": JsonLibre()},
                            facultatifs={"min": Nombre(), "max": Nombre(),
                                         "choices": Liste(Chaine(64), 64),
                                         "max_length": Entier(1, 1024),
                                         "description": Chaine(500)}), 32),
    # Acceptés dans le texte, jamais repris : l'OS y met la signature et la
    # clé qu'il a lui-même vérifiées (voir manifeste_depuis_json).
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
        **({"settings": [_reglage_vers_json(s) for s in m.settings]} if m.settings else {}),
        **({"min_sdk_version": m.min_sdk_version} if m.min_sdk_version is not None else {}),
    }


def _reglage_vers_json(s: SettingSpec) -> Dict[str, Any]:
    d = {"name": s.name, "type": s.type, "default": s.default}
    if s.min is not None:
        d["min"] = s.min
    if s.max is not None:
        d["max"] = s.max
    if s.choices:
        d["choices"] = list(s.choices)
    if s.max_length != 64:
        d["max_length"] = s.max_length
    if s.description:
        d["description"] = s.description
    return d


def manifeste_depuis_json(d: Any) -> AppManifest:
    """dict JSON du manifeste déclaré (voir manifeste_depuis_texte) ->
    AppManifest. Lève MessageRefuse si le dict ne suit pas SCHEMA_MANIFESTE.

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
        settings=[SettingSpec(name=s["name"], type=s["type"], default=s["default"],
                              min=s["min"], max=s["max"], choices=tuple(s["choices"] or ()),
                              max_length=64 if s["max_length"] is None else s["max_length"],
                              description=s["description"] or "")
                  for s in (v["settings"] or [])],
        min_sdk_version=v["min_sdk_version"],
    )


def manifeste_texte(m: AppManifest) -> str:
    """Le manifeste déclaré d'une app, tel qu'on le signe avec son code
    (signing.sign_app_package) : JSON, sans signature ni clé. À produire sur
    la machine du développeur, par exemple manifeste_texte(MonApp().manifest())."""
    d = manifeste_vers_json(m)
    d.pop("code_signature", None)
    d.pop("developer_public_key", None)
    return json.dumps(d, ensure_ascii=False, sort_keys=True, indent=1)


def manifeste_depuis_texte(texte: Any) -> AppManifest:
    """Manifeste déclaré (texte JSON) -> AppManifest, sans rien exécuter.
    Même JSON strict que le pont (canal_ipc.decoder : ni NaN, ni clé en
    double, profondeur bornée), au plus une trame ; lève MessageRefuse."""
    if not isinstance(texte, str):
        raise MessageRefuse("manifeste déclaré attendu sous forme de texte JSON")
    octets = texte.encode("utf-8")
    if len(octets) > TAILLE_MAX_TRAME:
        raise MessageRefuse(f"manifeste de {len(octets)} octets, au-delà de {TAILLE_MAX_TRAME}")
    return manifeste_depuis_json(decoder(octets))


__all__ = ["SCHEMA_MANIFESTE", "manifeste_vers_json", "manifeste_depuis_json",
           "manifeste_texte", "manifeste_depuis_texte", "MessageRefuse"]
