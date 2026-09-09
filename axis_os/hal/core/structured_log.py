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

Intégrité par chaînage de hash (ajouté 2026-09-08) — chaque entrée
inclut le hash SHA-256 de l'entrée précédente (prev_hash), formant une
chaîne : modifier ou supprimer une ligne passée invalide tout hash
calculé après elle, détectable par verify_chain(). Motivation :
- L'EU AI Act impose des obligations de transparence pour les systèmes
  d'IA à risque dès août 2026, et des exigences plus complètes
  (tamper-evident logging, articles 9-12 et 14-15) à partir de
  décembre 2027 pour les systèmes "high-risk" — un robot humanoïde en
  interaction physique avec des humains en relève vraisemblablement.
- C'est la même logique que "l'enregistreur de vol" (boîte noire)
  utilisé en aviation puis en automobile : un journal d'incident n'a
  de valeur en cas d'investigation que si son intégrité peut être
  prouvée, pas seulement supposée.

Volontairement PAS de blockchain, PAS d'ancrage externe (on-chain),
PAS de tiers-horodateur (RFC 3161) : ces approches existent dans
l'état de l'art (voir recherches "Black Block Recorder", "Agent
Flight Recorder" 2026) mais sont disproportionnées pour la portée
actuelle du projet (un robot en simulation, pas une flotte déployée).
Le chaînage de hash local suffit à détecter une falsification
après-coup, sans l'infrastructure lourde que ces approches supposent
— à réévaluer si/quand un vrai déploiement commercial l'exige.

Portée volontairement limitée à cette version (documenté honnêtement) :
  - Pas de rotation automatique de fichiers par taille (seulement par
    nombre de lignes, simple troncature) — suffisant pour du dev/test,
    à revoir avant un vrai déploiement long terme. La rotation RESET
    la chaîne de hash (chaque fichier a sa propre chaîne indépendante)
    — voir _rotate_if_needed().
  - Pas d'envoi vers un serveur distant — écriture locale uniquement.
    L'export vers un serveur central est le sujet du futur "Fleet
    Learning" évoqué mais non construit.
  - Un fichier par catégorie, pas de base de données interrogeable —
    suffisant pour "grep/lire après coup", pas pour des requêtes
    complexes ("toutes les violations TORQUE_LIMIT du mois dernier
    triées par robot"). Ce sera le rôle d'une vraie infra de logs
    si/quand la flotte grandit.
  - Le hash-chaining détecte une falsification, mais ne l'EMPÊCHE pas
    (un accès disque suffisant permet toujours de réécrire le fichier
    entier avec une nouvelle chaîne cohérente en interne — seule une
    ANCRE EXTERNE, hors de portée de qui falsifie, empêcherait ça
    complètement). C'est une différence connue et documentée avec un
    vrai système "tamper-proof" — ici on vise "tamper-evident" :
    la falsification devient détectable par comparaison avec une
    copie de sauvegarde externe, pas rendue impossible.

Licence : Apache 2.0
"""

from __future__ import annotations
import hashlib
import json
import os
import threading
import time
from dataclasses import dataclass, asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional


# Hash de l'entrée "racine" — utilisé comme prev_hash pour la toute
# première entrée de chaque fichier de log, ou après une rotation.
# Valeur arbitraire mais fixe et documentée : permet à verify_chain()
# de distinguer "début légitime de chaîne" de "prev_hash manquant/
# corrompu" (qui, lui, ne matchera jamais cette constante).
GENESIS_HASH = "0" * 64


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


def _canonical_json(entry: Dict[str, Any]) -> str:
    """
    Sérialisation JSON déterministe : clés triées, séparateurs sans
    espace superflu. Nécessaire pour que le hash d'une entrée soit
    reproductible indépendamment de l'ordre d'insertion des clés en
    mémoire — sans ça, la même entrée logique pourrait produire deux
    hashs différents selon l'implémentation Python, cassant toute
    vérification faite depuis un autre outil/langage.
    """
    return json.dumps(entry, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _hash_entry(entry: Dict[str, Any]) -> str:
    """SHA-256 hexadécimal de la représentation canonique d'une entrée
    (hors son propre champ 'hash', qui n'existe pas encore au moment
    du calcul — voir StructuredLogger.log())."""
    return hashlib.sha256(_canonical_json(entry).encode("utf-8")).hexdigest()


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
        # Dernier hash écrit par catégorie — clé de la chaîne pour la
        # PROCHAINE entrée. Initialisé paresseusement (voir
        # _last_hash_for()) plutôt qu'en relisant tous les fichiers au
        # démarrage : évite un coût I/O au constructeur pour des
        # fichiers potentiellement volumineux, au prix d'une relecture
        # de la dernière ligne au premier log() de chaque catégorie.
        self._last_hash_cache: Dict[LogCategory, str] = {}
        try:
            self._log_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._write_failures.append(f"Impossible de créer {self._log_dir}: {exc}")

    def _path_for(self, category: LogCategory) -> Path:
        return self._log_dir / f"{category.value}.jsonl"

    def _last_hash_for(self, category: LogCategory) -> str:
        """
        Hash de la dernière entrée déjà écrite pour cette catégorie —
        GENESIS_HASH si le fichier est vide, absent, ou si sa dernière
        ligne est illisible/corrompue (dans ce dernier cas, la chaîne
        redémarre à partir de là : une corruption préexistante ne doit
        jamais bloquer indéfiniment l'écriture de nouveaux logs, mais
        elle reste détectable par verify_chain() sur la partie
        antérieure du fichier).
        """
        if category in self._last_hash_cache:
            return self._last_hash_cache[category]

        path = self._path_for(category)
        if not path.exists():
            return GENESIS_HASH
        try:
            with open(path, "r", encoding="utf-8") as f:
                last_line = None
                for line in f:
                    stripped = line.strip()
                    if stripped:
                        last_line = stripped
                if last_line is None:
                    return GENESIS_HASH
                last_entry = json.loads(last_line)
                return last_entry.get("hash", GENESIS_HASH)
        except (OSError, json.JSONDecodeError):
            return GENESIS_HASH

    def log(self, category: LogCategory, data: Dict[str, Any],
            robot_id: Optional[str] = None) -> bool:
        """
        Ajoute une entrée au fichier de log de la catégorie donnée.
        Retourne True si l'écriture a réussi, False sinon — l'appelant
        peut choisir d'ignorer la valeur de retour (cas normal) ou de
        la vérifier s'il a besoin de garantir la persistance.

        Chaque entrée inclut prev_hash (hash de l'entrée précédente
        dans cette catégorie) et hash (hash de cette entrée elle-même,
        prev_hash inclus) — voir verify_chain() pour la vérification.
        """
        with self._lock:
            prev_hash = self._last_hash_for(category)

            entry = {
                "timestamp_ns": time.time_ns(),
                "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()),
                "category": category.value,
                "robot_id": robot_id,
                "data": _json_safe(data),
                "prev_hash": prev_hash,
            }
            entry["hash"] = _hash_entry(entry)

            line = json.dumps(entry, ensure_ascii=False)
            path = self._path_for(category)

            try:
                self._rotate_if_needed(path, category)
                with open(path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
                self._last_hash_cache[category] = entry["hash"]
                return True
            except OSError as exc:
                self._write_failures.append(
                    f"[{time.strftime('%H:%M:%S')}] Échec écriture "
                    f"{path.name}: {exc}"
                )
                return False

    def _rotate_if_needed(self, path: Path, category: LogCategory) -> None:
        """
        Rotation simple : si le fichier dépasse MAX_LINES_BEFORE_ROTATE
        lignes, il est renommé en .jsonl.old (écrasant l'ancien .old
        s'il existe) et un nouveau fichier vide démarre. Pas de rotation
        par taille en octets ni d'archivage horodaté multiple — suffisant
        pour la portée actuelle (voir note de portée en tête de fichier).

        Une rotation démarre une NOUVELLE chaîne de hash (repart à
        GENESIS_HASH) : la continuité de la chaîne s'arrête au fichier
        .old, qui reste vérifiable indépendamment via verify_chain()
        pointé dessus.
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
            self._last_hash_cache[category] = GENESIS_HASH

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

    def verify_chain(self, category: LogCategory) -> "ChainVerificationResult":
        """
        Vérifie l'intégrité de la chaîne de hash d'une catégorie :
        relit tout le fichier, recalcule le hash de chaque entrée à
        partir de son contenu (prev_hash inclus) et vérifie qu'il
        correspond au champ 'hash' stocké, ET que prev_hash de chaque
        entrée correspond bien au hash de l'entrée précédente.

        Toute rupture (hash recalculé ≠ hash stocké, ou prev_hash qui
        ne correspond pas à l'entrée précédente) signale soit une
        modification, soit une suppression/insertion de ligne(s) —
        les deux cassent la continuité de la chaîne à cet endroit.

        Ne lève jamais d'exception : un fichier absent, vide, ou
        totalement illisible donne un résultat "valide" trivial (rien
        à vérifier) ou "invalide" avec le détail de la première rupture
        trouvée, jamais un crash de l'outil de vérification lui-même.
        """
        path = self._path_for(category)
        if not path.exists():
            return ChainVerificationResult(valid=True, entries_checked=0, break_at_line=None, reason=None)

        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = [l.strip() for l in f if l.strip()]
        except OSError as exc:
            return ChainVerificationResult(
                valid=False, entries_checked=0, break_at_line=None,
                reason=f"Fichier illisible : {exc}",
            )

        expected_prev = GENESIS_HASH
        for i, line in enumerate(lines, start=1):
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                return ChainVerificationResult(
                    valid=False, entries_checked=i - 1, break_at_line=i,
                    reason="Ligne JSON invalide (corruption ou modification structurelle).",
                )

            stored_hash = entry.get("hash")
            stored_prev = entry.get("prev_hash")
            if stored_hash is None or stored_prev is None:
                return ChainVerificationResult(
                    valid=False, entries_checked=i - 1, break_at_line=i,
                    reason="Champ 'hash' ou 'prev_hash' absent (entrée antérieure au "
                           "chaînage, ou champ supprimé).",
                )

            if stored_prev != expected_prev:
                return ChainVerificationResult(
                    valid=False, entries_checked=i - 1, break_at_line=i,
                    reason=f"prev_hash incohérent — attendu {expected_prev[:12]}…, "
                           f"trouvé {stored_prev[:12]}… (ligne insérée, supprimée, "
                           f"ou réordonnée).",
                )

            entry_without_hash = {k: v for k, v in entry.items() if k != "hash"}
            recomputed = _hash_entry(entry_without_hash)
            if recomputed != stored_hash:
                return ChainVerificationResult(
                    valid=False, entries_checked=i - 1, break_at_line=i,
                    reason=f"Hash recalculé ({recomputed[:12]}…) ne correspond pas au "
                           f"hash stocké ({stored_hash[:12]}…) — contenu de l'entrée "
                           f"modifié après écriture.",
                )

            expected_prev = stored_hash

        return ChainVerificationResult(
            valid=True, entries_checked=len(lines), break_at_line=None, reason=None,
        )


@dataclass(frozen=True)
class ChainVerificationResult:
    """Résultat de StructuredLogger.verify_chain(). `entries_checked`
    compte les entrées confirmées valides AVANT toute rupture trouvée
    (donc = nombre total de lignes si valid=True)."""
    valid: bool
    entries_checked: int
    break_at_line: Optional[int]
    reason: Optional[str]


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
