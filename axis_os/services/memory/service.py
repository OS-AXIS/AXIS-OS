"""
AXIS-OS — MemoryService, premier service système réel
============================================================
Écrit pour valider le contrat IService/ServiceResult (axis_os/
services/base.py) par l'usage — pas une coquille vide. Choisi en
premier délibérément : c'est le seul des cinq services envisagés
(voir docs/architecture_services*.md) sans dépendance à un modèle
IA — du stockage clé-valeur persisté sur disque, entièrement
testable ce soir, sans caméra, sans ASR/TTS, sans politique de
marche.

Corrige au passage un vrai bug relevé par un audit externe : l'état
de facturation (InstalledApp.one_time_billed,
last_subscription_bill_time_s dans axis_os/store/runtime/play_store.py)
est tenu EN MÉMOIRE uniquement et perdu à tout redémarrage du
process — documenté explicitement à l'époque comme limite assumée,
faute d'une vraie couche de persistance. MemoryService.app_store()/
app_load() est exactement cette couche, maintenant disponible.
Brancher PlayStoreRuntime dessus reste un chantier séparé (pas fait
ce soir, pour rester dans l'ordre demandé), mais le service lui-même
est prêt à l'usage.

Licence : Apache 2.0
"""

from __future__ import annotations
import json
import os
import threading
from pathlib import Path
from typing import Any, Optional

from axis_os.hal.core.types import HALStatus
from axis_os.store.sdk.manifest import Capability
from axis_os.services.base import IService, ServiceResult, ServiceHealth


def _default_storage_dir() -> Path:
    """
    Répertoire de stockage par défaut : ./memory_store/ relatif au
    répertoire de travail courant — même pattern que
    structured_log.py::_default_log_dir(), pour rester cohérent avec
    le seul autre composant de persistance disque déjà existant dans
    le projet. Override possible via AXIS_OS_MEMORY_DIR, utile pour
    les tests (répertoire temporaire) sans changer le code appelant.
    """
    override = os.environ.get("AXIS_OS_MEMORY_DIR")
    return Path(override) if override else Path("memory_store")


class MemoryService(IService):
    """
    Persisté sur disque (JSON, un fichier par catégorie), survit au
    redémarrage du process — c'est précisément ce qui manquait à
    PlayStoreRuntime pour son état de facturation (voir docstring de
    module).

    Cloisonnement obligatoire pour app_store()/app_load() : chaque
    clé est préfixée par app_id, une app ne peut JAMAIS lire l'espace
    d'une autre — sinon une app gratuite pourrait siphonner les
    données d'une app payante. Implémenté dès cette première version,
    pas ajouté après coup (voir docs/architecture_services*.md §3.2).

    Thread-safety : un verrou protège chaque écriture disque — ce
    service est un singleton partagé, potentiellement appelé
    concurremment par plusieurs apps (services système appelables
    PENDANT une mission, contrairement aux apps sandboxées qui sont
    mutuellement exclues par le verrou de mission — voir
    docs/architecture_services*.md §6.1).
    """

    def __init__(self, storage_dir: Optional[Path] = None):
        self._storage_dir = storage_dir or _default_storage_dir()
        self._lock = threading.Lock()
        self._started = False

    @property
    def service_id(self) -> str:
        return "memory"

    @property
    def capability(self) -> Capability:
        return Capability.MEMORY_PERSISTENT

    def start(self) -> HALStatus:
        if self._started:
            return HALStatus.OK  # idempotent, voir IService.start()
        try:
            self._storage_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            return HALStatus.ERROR_FAULT
        self._started = True
        return HALStatus.OK

    def stop(self) -> None:
        # Rien à libérer — chaque appel ouvre/ferme son propre
        # fichier (voir _read_category()/_write_category()), pas de
        # ressource persistante à relâcher ici. Idempotent par
        # construction : appeler stop() plusieurs fois, ou avant
        # start(), ne fait jamais rien de dangereux.
        self._started = False

    def health(self) -> ServiceHealth:
        if not self._started:
            return ServiceHealth(running=False, healthy=False,
                                 detail="Service non démarré (start() jamais appelé).")
        if not self._storage_dir.exists():
            return ServiceHealth(running=True, healthy=False,
                                 detail=f"Répertoire de stockage absent : {self._storage_dir}")
        return ServiceHealth(running=True, healthy=True)

    # ── Mémoire sémantique : les faits stables ──────────────────────

    def remember_place(self, name: str, pose: dict) -> ServiceResult:
        """`pose` : dict simple (ex: {"x": 1.2, "y": 0.5, "theta": 0.0})
        — pas de dépendance à un type Pose du HAL, pour que ce service
        reste utilisable sans navigation réelle (non implémentée ce
        soir)."""
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        if not name or not name.strip():
            return ServiceResult(ok=False, reason="Nom de lieu vide")
        places = self._read_category("places")
        places[name] = pose
        self._write_category("places", places)
        return ServiceResult(ok=True, value=pose)

    def recall_place(self, name: str) -> ServiceResult:
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        places = self._read_category("places")
        if name not in places:
            return ServiceResult(ok=False, reason=f"Lieu '{name}' inconnu")
        return ServiceResult(ok=True, value=places[name])

    def remember_object(self, label: str, place: str) -> ServiceResult:
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        objects = self._read_category("objects")
        objects.setdefault(label, [])
        if place not in objects[label]:
            objects[label].append(place)
        self._write_category("objects", objects)
        return ServiceResult(ok=True, value=objects[label])

    # ── Mémoire épisodique : ce qui s'est passé ─────────────────────

    def log_episode(self, episode: dict) -> ServiceResult:
        """`episode` : dict simple, structure libre à l'appelant
        (pas de dataclass Episode imposée ce soir — voir
        docs/architecture_services*.md pour la conception cible
        complète, non implémentée ici)."""
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        episodes = self._read_category("episodes")
        if not isinstance(episodes, list):
            episodes = []
        episodes.append(episode)
        self._write_category("episodes", episodes)
        return ServiceResult(ok=True, value=episode)

    def recall_episodes(self, query: str, limit: int = 10) -> ServiceResult:
        """Recherche NAÏVE (sous-chaîne, insensible à la casse) —
        volontairement, pas d'embedding ni de similarité sémantique :
        cohérent avec la même remarque faite pour
        IntentRegistry.match() dans docs/architecture_services*.md,
        « n'a pas besoin d'être intelligent au départ »."""
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        episodes = self._read_category("episodes")
        if not isinstance(episodes, list):
            episodes = []
        query_lower = query.lower()
        matched = [
            ep for ep in episodes
            if query_lower in json.dumps(ep, ensure_ascii=False).lower()
        ]
        return ServiceResult(ok=True, value=matched[-limit:])

    # ── Espace privé par app — cloisonné, comme le stockage Android ─

    def app_store(self, app_id: str, key: str, value: Any) -> ServiceResult:
        """
        Cloisonnement : la clé physique sur disque est TOUJOURS
        préfixée par app_id (voir _app_key() ci-dessous) — une app ne
        peut jamais, même en fournissant une clé identique, écraser
        ou lire les données d'une autre app_id. Vérifié explicitement
        par test_memory_service.py, pas seulement supposé par
        construction.
        """
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        if not app_id or not app_id.strip():
            return ServiceResult(ok=False, reason="app_id vide")
        store = self._read_category("app_store")
        store[self._app_key(app_id, key)] = value
        self._write_category("app_store", store)
        return ServiceResult(ok=True, value=value)

    def app_load(self, app_id: str, key: str) -> ServiceResult:
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        store = self._read_category("app_store")
        full_key = self._app_key(app_id, key)
        if full_key not in store:
            return ServiceResult(ok=False, reason=f"Clé '{key}' inconnue pour '{app_id}'")
        return ServiceResult(ok=True, value=store[full_key])

    @staticmethod
    def _app_key(app_id: str, key: str) -> str:
        """
        Encode app_id et key en JSON avant de les joindre — trouvé
        nécessaire par un test dédié (test_memory_service.py section
        4) : un simple séparateur textuel ("::") reste vulnérable à
        collision quand ce même séparateur apparaît DANS app_id ou
        key (app_id="a::b", key="c" produit la même clé finale que
        app_id="a", key="b::c" avec une simple concaténation).
        json.dumps() échappe explicitement tout caractère spécial
        dans chaque partie AVANT la concaténation, éliminant cette
        classe de collision plutôt que de se reposer sur un
        séparateur qui espère ne jamais apparaître dans les données
        réelles.
        """
        return json.dumps([app_id, key], ensure_ascii=False)

    # ── Persistance disque ───────────────────────────────────────────

    def _category_path(self, category: str) -> Path:
        return self._storage_dir / f"{category}.json"

    def _read_category(self, category: str):
        path = self._category_path(category)
        with self._lock:
            if not path.exists():
                return {} if category != "episodes" else []
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except (OSError, json.JSONDecodeError):
                # Fichier corrompu ou illisible — ne jamais planter la
                # lecture, repartir d'un état vide plutôt que de
                # propager une exception jusqu'à l'appelant d'un
                # service système censé être toujours disponible.
                return {} if category != "episodes" else []

    def _write_category(self, category: str, data) -> None:
        path = self._category_path(category)
        with self._lock:
            tmp_path = path.with_suffix(".json.tmp")
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            # Renommage atomique — évite un fichier à moitié écrit si
            # le process est interrompu en plein milieu de
            # l'écriture (coupure d'alimentation, kill -9).
            tmp_path.replace(path)
