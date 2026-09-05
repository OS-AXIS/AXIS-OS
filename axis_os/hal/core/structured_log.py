"""
AXIS-OS — Structured Persistent Logging
==========================================
Comble un trou identifié explicitement pendant la session : tout
événement de sécurité (violation SafetyGuard, alerte Watchdog, crash
Sandbox) s'affichait uniquement via print(), sans jamais être conservé.
Utile pour déboguer en direct, inutilisable pour comprendre "pourquoi
le robot a fait ça" des jours après l'incident.

Principe : un fichier JSON Lines (une entrée JSON par ligne) par
catégorie d'événement, écrit de façon append-only. Format choisi pour
sa simplicité — lisible par un humain avec `cat`, parsable par n'importe
quel outil sans dépendance externe (pas de vraie base de données, pas
de service de log distant — cohérent avec la portée actuelle du projet,
un seul robot en simulation, pas une flotte).

Portée volontairement limitée à cette version (documenté honnêtement) :
  - Pas de rotation automatique de fichiers par taille (seulement par
    nombre de lignes, simple troncature) — suffisant pour du dev/test,
    à revoir avant un vrai déploiement long terme.
  - Pas d'envoi vers un serveur distant — écriture locale uniquement.
    L'export vers un serveur central est le sujet du futur "Fleet
    Learning" évoqué mais non construit.
  - Un fichier par catégorie, pas de base de données interrogeable —
    suffisant pour "grep/lire après coup", pas pour des requêtes
    complexes ("toutes les violations TORQUE_LIMIT du mois dernier
    triées par robot"). Ce sera le rôle d'une vraie infra de logs
    si/quand la flotte grandit.

Licence : Apache 2.0
"""

from __future__ import annotations
import json
import os
import threading
import time
from dataclasses import dataclass, asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Optional


class LogCategory(Enum):
    SAFETY_VIOLATION = "safety_violation"
    WATCHDOG_ALERT    = "watchdog_alert"
    ESTOP              = "estop"
    SANDBOX_EVENT        = "sandbox_event"
    CONVENTION_ERROR      = "convention_error"


def _default_log_dir() -> Path:
    """
    Répertoire de logs par défaut : ./logs/ relatif au répertoire de
    travail courant. Override possible via la variable d'environnement
    AXIS_OS_LOG_DIR — utile pour les tests (répertoire temporaire) sans
    changer le code appelant.
    """
    override = os.environ.get("AXIS_OS_LOG_DIR")
    return Path(override) if override else Path("logs")


def _json_safe(value: Any) -> Any:
    """
    Convertit une valeur en quelque chose de sérialisable en JSON.
    Gère les dataclasses, les Enum, et les types simples — sans lever
    d'exception sur un type inattendu (mieux vaut un log incomplet
    qu'un crash du logger lui-même, qui ne doit jamais interrompre le
    composant qu'il journalise).
    """
    if is_dataclass(value) and not isinstance(value, type):
        return {k: _json_safe(v) for k, v in asdict(value).items()}
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)  # filet de sécurité : jamais lever d'exception ici


class StructuredLogger:
    """
    Logger persistant, un fichier JSON Lines par catégorie d'événement.
    Thread-safe : plusieurs composants (Watchdog, SafetyGuard, Sandbox)
    peuvent écrire en même temps sans corrompre le fichier.

    Usage typique :
        logger = StructuredLogger()
        logger.log(LogCategory.ESTOP, {
            "reason": "TEMP_CRITICAL", "joint_id": "right_knee",
            "measured": 85.0, "threshold": 80.0,
        })

    Ne lève JAMAIS d'exception vers l'appelant — une panne d'écriture
    disque (disque plein, permissions) ne doit jamais faire planter le
    Watchdog ou le SafetyGuard qui l'utilisent. Les échecs d'écriture
    sont eux-mêmes journalisés en mémoire, consultables via
    get_write_failures(), pour ne pas échouer silencieusement sans
    aucune trace.
    """

    MAX_LINES_BEFORE_ROTATE = 50_000

    def __init__(self, log_dir: Optional[Path] = None):
        self._log_dir = log_dir or _default_log_dir()
        self._lock = threading.Lock()
        self._write_failures: list[str] = []
        try:
            self._log_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._write_failures.append(f"Impossible de créer {self._log_dir}: {exc}")

    def _path_for(self, category: LogCategory) -> Path:
        return self._log_dir / f"{category.value}.jsonl"

    def log(self, category: LogCategory, data: Dict[str, Any],
            robot_id: Optional[str] = None) -> bool:
        """
        Ajoute une entrée au fichier de log de la catégorie donnée.
        Retourne True si l'écriture a réussi, False sinon — l'appelant
        peut choisir d'ignorer la valeur de retour (cas normal) ou de
        la vérifier s'il a besoin de garantir la persistance.
        """
        entry = {
            "timestamp_ns": time.time_ns(),
            "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()),
            "category": category.value,
            "robot_id": robot_id,
            "data": _json_safe(data),
        }

        line = json.dumps(entry, ensure_ascii=False)
        path = self._path_for(category)

        with self._lock:
            try:
                self._rotate_if_needed(path)
                with open(path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
                return True
            except OSError as exc:
                self._write_failures.append(
                    f"[{time.strftime('%H:%M:%S')}] Échec écriture "
                    f"{path.name}: {exc}"
                )
                return False

    def _rotate_if_needed(self, path: Path) -> None:
        """
        Rotation simple : si le fichier dépasse MAX_LINES_BEFORE_ROTATE
        lignes, il est renommé en .jsonl.old (écrasant l'ancien .old
        s'il existe) et un nouveau fichier vide démarre. Pas de rotation
        par taille en octets ni d'archivage horodaté multiple — suffisant
        pour la portée actuelle (voir note de portée en tête de fichier).
        """
        if not path.exists():
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                line_count = sum(1 for _ in f)
        except OSError:
            return
        if line_count >= self.MAX_LINES_BEFORE_ROTATE:
            old_path = path.with_suffix(".jsonl.old")
            path.replace(old_path)

    def read_recent(self, category: LogCategory, n: int = 50) -> list[Dict[str, Any]]:
        """
        Relit les n dernières entrées d'une catégorie. Lecture simple
        (pas optimisée pour de très gros fichiers) — suffisant pour du
        diagnostic ponctuel, pas pour du monitoring temps réel à grande
        échelle. Retourne une liste vide si le fichier n'existe pas ou
        est illisible, plutôt que de lever une exception.
        """
        path = self._path_for(category)
        if not path.exists():
            return []
        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = f.readlines()
        except OSError:
            return []

        results = []
        for line in lines[-n:]:
            line = line.strip()
            if not line:
                continue
            try:
                results.append(json.loads(line))
            except json.JSONDecodeError:
                continue  # ligne corrompue, ignorée plutôt que de tout faire échouer
        return results

    def get_write_failures(self) -> list[str]:
        """Historique des échecs d'écriture — consulté si on suspecte
        une perte de logs (disque plein, permissions), jamais en usage
        normal."""
        with self._lock:
            return list(self._write_failures)


# Instance partagée par défaut — évite que chaque composant (Watchdog,
# SafetyGuard, Sandbox) doive gérer sa propre instanciation et son
# propre répertoire de logs. Un composant qui a besoin d'un répertoire
# différent (tests, isolation) peut toujours créer son propre
# StructuredLogger() explicitement plutôt que d'utiliser celui-ci.
_default_logger: Optional[StructuredLogger] = None
_default_logger_lock = threading.Lock()


def get_default_logger() -> StructuredLogger:
    global _default_logger
    with _default_logger_lock:
        if _default_logger is None:
            _default_logger = StructuredLogger()
        return _default_logger
