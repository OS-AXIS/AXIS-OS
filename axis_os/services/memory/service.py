"""
AXIS-OS — MemoryService, premier service système réel
============================================================
Ce service a été écrit pour valider par l'usage le contrat
IService/ServiceResult (axis_os/services/base.py) : une vraie
implémentation, pas une coquille vide. Il a été choisi en premier
délibérément : des cinq services envisagés (voir
docs/architecture_services.md §3.2), c'est le seul qui ne dépend
d'aucun modèle d'IA. Il s'agit d'un stockage clé-valeur persisté sur
disque, entièrement testable sans caméra, sans ASR ni TTS
(reconnaissance et synthèse vocales) et sans politique de marche (le
contrôleur qui fait marcher le robot).

Il corrige au passage un vrai bug relevé par un audit externe : l'état
de facturation (InstalledApp.one_time_billed et
last_subscription_bill_time_s, dans axis_os/store/runtime/play_store.py)
n'était tenu qu'en mémoire, et donc perdu à chaque redémarrage du
processus. C'était alors une limite documentée et assumée, faute d'une
vraie couche de persistance. MemoryService.app_store()/app_load() est
exactement cette couche. Le branchement de PlayStoreRuntime sur ce
service a d'abord été laissé à un chantier séparé ; il a été fait
depuis, en option, par le paramètre memory_service de PlayStoreRuntime
(voir CHANGELOG.md, entrée du 2026-09-16, et
test/test_billing_persistence.py).

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
    Répertoire de stockage par défaut : ./memory_store/, relatif au
    répertoire de travail courant.

    C'est le même choix que _default_log_dir() dans
    axis_os/hal/core/structured_log.py, pour rester cohérent avec le
    seul autre composant du projet qui écrivait déjà sur disque à
    l'époque. La variable d'environnement AXIS_OS_MEMORY_DIR permet de
    choisir un autre répertoire sans modifier le code appelant, ce qui
    sert notamment aux tests (répertoire temporaire).
    """
    override = os.environ.get("AXIS_OS_MEMORY_DIR")
    return Path(override) if override else Path("memory_store")


class MemoryService(IService):
    """
    Mémoire persistante du robot : lieux, objets, épisodes passés, et
    un espace privé par app.

    Les données sont écrites sur disque en JSON, un fichier par
    catégorie, et survivent donc au redémarrage du processus. C'est
    précisément ce qui manquait à PlayStoreRuntime pour son état de
    facturation (voir la docstring du module).

    Cloisonnement obligatoire de app_store()/app_load() : chaque clé
    est préfixée par l'app_id (l'identifiant unique de l'app), et une
    app ne peut JAMAIS lire l'espace d'une autre. Sans cela, une app
    gratuite pourrait siphonner les données d'une app payante. Ce
    cloisonnement existe dès cette première version, il n'a pas été
    ajouté après coup (voir docs/architecture_services.md §3.2).

    Accès concurrents : un verrou protège chaque lecture et chaque
    écriture sur disque. Ce service est un singleton (une seule instance
    pour tout le robot) partagé, potentiellement appelé en même temps
    par plusieurs apps : les services système restent appelables
    pendant une mission, alors que
    les apps, chacune dans son bac à sable (processus isolé), sont
    mutuellement exclues par le verrou de mission (voir
    docs/architecture_services.md §6.1). Limite : le verrou couvre la
    lecture et l'écriture séparément, pas la séquence complète « lire,
    modifier, réécrire » d'une méthode publique ; deux modifications
    simultanées de la même catégorie peuvent donc s'écraser.
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
        # Rien à libérer : chaque appel ouvre et ferme son propre
        # fichier (voir _read_category() et _write_category()), aucune
        # ressource ne reste ouverte entre deux appels. stop() est
        # donc idempotent par construction : l'appeler plusieurs fois,
        # ou avant start(), ne fait jamais rien de dangereux.
        self._started = False

    def health(self) -> ServiceHealth:
        if not self._started:
            return ServiceHealth(running=False, healthy=False,
                                 detail="Service non démarré (start() jamais appelé).")
        if not self._storage_dir.exists():
            return ServiceHealth(running=True, healthy=False,
                                 detail=f"Répertoire de stockage absent : {self._storage_dir}")
        return ServiceHealth(running=True, healthy=True)

    # ── Mémoire sémantique : les faits stables (lieux, objets) ──────

    def remember_place(self, name: str, pose: dict) -> ServiceResult:
        """Mémorise la pose (position et orientation) d'un lieu nommé.

        `pose` est un dict simple, par exemple
        {"x": 1.2, "y": 0.5, "theta": 0.0}, plutôt qu'un type Pose du
        HAL (la couche d'abstraction matérielle, axis_os/hal/) : ce
        service reste ainsi utilisable sans navigation réelle, qui
        n'existait pas encore quand il a été écrit."""
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        if not name or not name.strip():
            return ServiceResult(ok=False, reason="Nom de lieu vide")
        places = self._read_category("places")
        places[name] = pose
        self._write_category("places", places)
        return ServiceResult(ok=True, value=pose)

    def recall_place(self, name: str) -> ServiceResult:
        """Rend la pose mémorisée du lieu `name`, ou un échec explicite
        si ce lieu est inconnu."""
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        places = self._read_category("places")
        if name not in places:
            return ServiceResult(ok=False, reason=f"Lieu '{name}' inconnu")
        return ServiceResult(ok=True, value=places[name])

    def remember_object(self, label: str, place: str) -> ServiceResult:
        """Note qu'un objet `label` a été vu au lieu `place`. Un même
        objet peut être associé à plusieurs lieux, sans doublon ; la
        valeur rendue est la liste complète de ses lieux."""
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
        """Ajoute un épisode (un événement passé) à la fin du journal.

        `episode` est un dict simple, de structure libre pour
        l'appelant : aucune dataclass Episode n'est imposée pour
        l'instant. La conception cible complète, non implémentée ici,
        est décrite dans docs/architecture_services.md (§3.2,
        Mémoire)."""
        if not self._started:
            return ServiceResult(ok=False, reason="Service non démarré")
        episodes = self._read_category("episodes")
        if not isinstance(episodes, list):
            episodes = []
        episodes.append(episode)
        self._write_category("episodes", episodes)
        return ServiceResult(ok=True, value=episode)

    def recall_episodes(self, query: str, limit: int = 10) -> ServiceResult:
        """Rend les épisodes dont le contenu contient `query`, au plus
        `limit`, en gardant les plus récents.

        La recherche est volontairement naïve : une sous-chaîne,
        insensible à la casse, cherchée dans le JSON de chaque épisode,
        sans embedding (représentation d'un texte par un vecteur
        numérique) ni similarité sémantique. C'est cohérent avec ce que
        docs/architecture_services.md (§4.1) dit de
        IntentRegistry.match() : il « n'a pas besoin d'être intelligent
        au départ »."""
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
        Écrit `value` sous `key` dans l'espace privé de l'app `app_id`.

        Cloisonnement : la clé physique sur disque est TOUJOURS formée
        à partir de app_id (voir _app_key() ci-dessous). Une app ne
        peut donc jamais, même en fournissant une clé identique, écraser
        ou lire les données d'un autre app_id. C'est vérifié
        explicitement par test/test_memory_service.py, et pas seulement
        supposé par construction.
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
        """Relit une valeur de l'espace privé de l'app `app_id`, ou rend
        un échec explicite si cette clé n'existe pas pour cette app."""
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
        Encode app_id et key en JSON avant de les assembler.

        Un test dédié (section 4 de test/test_memory_service.py) a
        montré que c'était nécessaire : un simple séparateur textuel
        ("::") laisse des collisions possibles dès que ce séparateur
        apparaît dans app_id ou dans key. Avec une concaténation simple,
        app_id="a::b", key="c" donne la même clé finale que app_id="a",
        key="b::c". json.dumps() échappe les caractères spéciaux de
        chaque partie avant l'assemblage, ce qui élimine cette classe de
        collision, au lieu de compter sur un séparateur dont on espère
        qu'il n'apparaîtra jamais dans les données réelles.
        """
        return json.dumps([app_id, key], ensure_ascii=False)

    # ── Persistance disque ───────────────────────────────────────────

    def _category_path(self, category: str) -> Path:
        return self._storage_dir / f"{category}.json"

    def _read_category(self, category: str):
        """Lit une catégorie depuis son fichier JSON. « episodes » est
        une liste (journal dans l'ordre d'arrivée), les autres
        catégories sont des dictionnaires ; une catégorie absente rend
        un conteneur vide du bon type."""
        path = self._category_path(category)
        with self._lock:
            if not path.exists():
                return {} if category != "episodes" else []
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return json.load(f)
            except (OSError, json.JSONDecodeError):
                # Fichier corrompu ou illisible : un service système
                # doit rester disponible, donc on ne fait jamais planter
                # la lecture. On repart d'un état vide plutôt que de
                # propager une exception jusqu'à l'appelant (cas vérifié
                # par la section 5 de test/test_memory_service.py).
                return {} if category != "episodes" else []

    def _write_category(self, category: str, data) -> None:
        path = self._category_path(category)
        with self._lock:
            tmp_path = path.with_suffix(".json.tmp")
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            # Écriture dans un fichier temporaire puis renommage
            # atomique : si le processus est interrompu en pleine
            # écriture (coupure d'alimentation, kill -9, c'est-à-dire
            # un arrêt forcé), le fichier final n'est jamais laissé à
            # moitié écrit.
            tmp_path.replace(path)
