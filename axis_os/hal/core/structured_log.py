"""
AXIS-OS — journal structuré persistant
=======================================
Pourquoi ce module existe : pendant le développement, on a constaté
que les événements de sécurité du robot n'étaient qu'affichés par
print(), sans jamais être conservés. Il s'agit des violations relevées
par SafetyGuard (le garde-fou qui contrôle chaque commande avant
qu'elle n'atteigne le moteur d'une articulation), des alertes du
Watchdog (le « chien de garde » qui surveille en continu chaque
articulation : communication, température, couple, vitesse, position)
et des plantages d'une app dans sa sandbox (l'environnement isolé où
tournent les applications). Un simple affichage aide à déboguer en
direct, mais ne permet pas de comprendre « pourquoi le robot a fait
ça » des jours après un incident.

Principe : un fichier JSON Lines (un objet JSON par ligne) par
catégorie d'événement, écrit en ajout seul (« append-only » : on ajoute
des lignes à la fin, on ne réécrit jamais les précédentes). Ce format a
été choisi pour sa simplicité : il se lit à l'œil avec `cat` et se
traite avec n'importe quel outil, sans dépendance externe. Pas de vraie
base de données ni de service de journalisation distant : c'est
cohérent avec la portée actuelle du projet, un seul robot en simulation
et non une flotte.

Intégrité par chaînage d'empreintes (ajouté le 2026-09-08). Une
empreinte, ou « hash », est ici un résumé SHA-256 de 64 caractères
hexadécimaux, qui change complètement si un seul octet du contenu
change. Chaque entrée stocke l'empreinte de l'entrée précédente dans
son champ prev_hash : les entrées forment ainsi une chaîne, et modifier
ou supprimer une ligne passée invalide toutes les empreintes calculées
après elle, ce que verify_chain() détecte. Deux motivations :
- Le règlement européen sur l'intelligence artificielle (EU AI Act)
  impose des obligations de transparence aux systèmes d'IA à risque dès
  août 2026, puis des exigences plus complètes à partir de décembre
  2027 pour les systèmes dits « à haut risque » (high-risk), dont une
  journalisation où toute altération est décelable (« tamper-evident
  logging », articles 9 à 12 et 14 à 15). Un robot humanoïde en
  interaction physique avec des humains en relève vraisemblablement.
- C'est la logique de l'enregistreur de vol (la « boîte noire ») de
  l'aviation, reprise ensuite dans l'automobile : en cas d'enquête, un
  journal d'incident n'a de valeur que si son intégrité peut être
  prouvée, et non seulement supposée.

Ce qui n'est volontairement pas fait : ni blockchain, ni ancrage
externe des empreintes (leur publication « on-chain », sur une
blockchain), ni tiers horodateur (service qui certifie la date d'un
document, norme RFC 3161). Ces approches existent dans l'état de l'art
(voir les travaux « Black Block Recorder » et « Agent Flight Recorder »,
2026) mais sont disproportionnées pour la portée actuelle du projet (un
robot en simulation, pas une flotte déployée). Le chaînage local suffit
à détecter une falsification après coup, sans l'infrastructure lourde
que ces approches supposent. À réévaluer si un vrai déploiement
commercial l'exige.

Limites assumées de cette version :
  - Pas de rotation des fichiers selon leur taille en octets : la
    rotation se déclenche seulement au-delà d'un nombre de lignes, par
    simple renommage du fichier en .old (voir _rotate_if_needed()).
    Suffisant pour le développement et les essais, à revoir avant un
    déploiement de longue durée. La rotation est censée faire repartir
    la chaîne d'empreintes de zéro, chaque fichier ayant sa propre
    chaîne, indépendante ; ce n'est pas encore le cas pour la première
    entrée du nouveau fichier (limite connue, détaillée dans
    _rotate_if_needed()).
  - Pas d'envoi vers un serveur distant : l'écriture est uniquement
    locale. L'export vers un serveur central relève du futur « Fleet
    Learning » (exploitation centralisée des données d'une flotte de
    robots), évoqué mais pas construit.
  - Un fichier par catégorie, pas de base de données interrogeable :
    suffisant pour relire un fichier ou y chercher avec grep après
    coup, pas pour des requêtes complexes (« toutes les violations
    TORQUE_LIMIT, c'est-à-dire de dépassement de couple, du mois
    dernier, triées par robot »). Ce sera le rôle d'une vraie
    infrastructure de journaux si la flotte grandit.
  - Le chaînage détecte une falsification mais ne l'empêche pas :
    quelqu'un qui dispose d'un accès suffisant au disque peut toujours
    remplacer le fichier entier par un autre dont la chaîne est
    cohérente en interne. Seule une ancre externe, hors de portée de
    celui qui falsifie, l'empêcherait complètement. C'est une
    différence connue et assumée avec un système « tamper-proof »
    (falsification impossible) : ici on vise « tamper-evident »
    (falsification décelable), par exemple en comparant le fichier avec
    une copie de sauvegarde conservée ailleurs.

Tests : test/test_structured_log_chain.py (chaînage d'empreintes) et
test/test_structured_logging.py (écriture, relecture, intégration avec
le Watchdog et le garde-fou des articulations).

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


# Empreinte de l'entrée « racine » (genesis) : elle sert de prev_hash à
# la toute première entrée de chaque fichier de journal, et devrait
# servir aussi à la première entrée qui suit une rotation (voir la
# limite connue décrite dans _rotate_if_needed()). La valeur est
# arbitraire mais fixe et documentée, ce qui permet à verify_chain() de
# distinguer un début de chaîne légitime d'un prev_hash manquant ou
# corrompu (qui, lui, ne sera jamais égal à cette constante).
GENESIS_HASH = "0" * 64


# Catégories d'événements. Chacune a son propre fichier <valeur>.jsonl
# dans le répertoire de journaux (par exemple estop.jsonl). ESTOP
# désigne un arrêt d'urgence (e-stop), journalisé par le Watchdog.
# CONVENTION_ERROR désigne une incohérence dans la table de conventions
# d'un robot (sens et décalage de chaque articulation, voir
# axis_os/hal/drivers/unitree_g1/convention_validator.py).
# CONVENTION_ERROR et SANDBOX_EVENT (événement d'une app dans sa
# sandbox) sont prévues, mais aucun composant ne les écrit encore :
# seuls les tests s'en servent.
class LogCategory(Enum):
    SAFETY_VIOLATION = "safety_violation"
    WATCHDOG_ALERT    = "watchdog_alert"
    ESTOP              = "estop"
    SANDBOX_EVENT        = "sandbox_event"
    CONVENTION_ERROR      = "convention_error"


def _default_log_dir() -> Path:
    """
    Répertoire de journaux par défaut : ./logs/, relatif au répertoire
    de travail courant. La variable d'environnement AXIS_OS_LOG_DIR
    permet d'en choisir un autre sans modifier le code appelant, ce qui
    sert surtout aux tests (répertoire temporaire).
    """
    override = os.environ.get("AXIS_OS_LOG_DIR")
    return Path(override) if override else Path("logs")


def _json_safe(value: Any) -> Any:
    """
    Convertit une valeur en une forme sérialisable en JSON.

    Le journal ne doit jamais interrompre le composant qu'il enregistre :
    mieux vaut une entrée incomplète qu'un plantage du journal lui-même.
    La fonction gère donc les dataclasses, les Enum et les types simples,
    et convertit tout type inattendu en texte au lieu de lever une
    exception.
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
    return str(value)  # filet de sécurité : ne jamais lever d'exception ici


def _canonical_json(entry: Dict[str, Any]) -> str:
    """
    Sérialisation JSON déterministe : clés triées, séparateurs sans
    espace superflu.

    Sans cela, une même entrée pourrait produire deux empreintes
    différentes selon l'ordre d'insertion des clés en mémoire ou
    l'implémentation de Python, ce qui casserait toute vérification
    faite depuis un autre outil ou un autre langage. La forme canonique
    rend l'empreinte reproductible.
    """
    return json.dumps(entry, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _hash_entry(entry: Dict[str, Any]) -> str:
    """Empreinte SHA-256, en hexadécimal, de la forme canonique d'une
    entrée. Le champ 'hash' de l'entrée n'en fait pas partie : il
    n'existe pas encore au moment du calcul (voir StructuredLogger.log())."""
    return hashlib.sha256(_canonical_json(entry).encode("utf-8")).hexdigest()


class StructuredLogger:
    """
    Journal persistant, avec un fichier JSON Lines par catégorie
    d'événement. Il est sûr en multithread (thread-safe) : plusieurs
    composants (Watchdog, SafetyGuard, sandbox) peuvent y écrire en même
    temps sans corrompre le fichier, car toutes les écritures d'une
    même instance passent par un seul verrou.

    Usage typique :
        logger = StructuredLogger()
        logger.log(LogCategory.ESTOP, {
            "reason": "TEMP_CRITICAL", "joint_id": "right_knee",
            "measured": 85.0, "threshold": 80.0,
        })

    Ne lève jamais d'exception vers l'appelant : une panne d'écriture
    (disque plein, permissions refusées) ne doit pas faire planter le
    Watchdog ou le SafetyGuard qui s'en servent. Pour que ces échecs ne
    passent pas pour autant inaperçus, ils sont eux-mêmes notés en
    mémoire et consultables via get_write_failures().
    """

    # Nombre de lignes à partir duquel un fichier est mis de côté en
    # .jsonl.old (voir _rotate_if_needed()).
    MAX_LINES_BEFORE_ROTATE = 50_000

    def __init__(self, log_dir: Optional[Path] = None):
        self._log_dir = log_dir or _default_log_dir()
        self._lock = threading.Lock()
        self._write_failures: list[str] = []
        # Dernière empreinte écrite pour chaque catégorie : c'est le
        # maillon auquel la prochaine entrée sera rattachée. Ce cache se
        # remplit au fil des écritures réussies au lieu de relire tous
        # les fichiers au démarrage, pour éviter des lectures disque
        # coûteuses dans le constructeur quand les fichiers sont gros. En
        # contrepartie, tant qu'une catégorie n'a rien écrit depuis le
        # démarrage, _last_hash_for() relit son fichier pour en retrouver
        # la dernière ligne (c'est le cas du premier log()).
        self._last_hash_cache: Dict[LogCategory, str] = {}
        try:
            self._log_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._write_failures.append(f"Impossible de créer {self._log_dir}: {exc}")

    def _path_for(self, category: LogCategory) -> Path:
        return self._log_dir / f"{category.value}.jsonl"

    def _last_hash_for(self, category: LogCategory) -> str:
        """
        Empreinte de la dernière entrée déjà écrite pour cette catégorie.

        Renvoie GENESIS_HASH si le fichier est absent ou vide, ou si sa
        dernière ligne est illisible ou corrompue. Dans ce dernier cas,
        la chaîne repart de là : une corruption existante ne doit jamais
        bloquer indéfiniment l'écriture de nouvelles entrées, et elle
        reste détectable par verify_chain() sur la partie antérieure du
        fichier.
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
        Ajoute une entrée au fichier de la catégorie donnée.

        Renvoie True si l'écriture a réussi, False sinon. L'appelant
        peut ignorer cette valeur (cas normal) ou la vérifier s'il doit
        garantir que l'entrée a bien été enregistrée.

        Chaque entrée contient prev_hash (empreinte de l'entrée
        précédente de cette catégorie) et hash (empreinte de l'entrée
        elle-même, prev_hash compris). Voir verify_chain() pour la
        vérification.
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
        Rotation simple, pour que les fichiers ne grossissent pas sans
        fin : dès que le fichier atteint MAX_LINES_BEFORE_ROTATE lignes,
        il est renommé en .jsonl.old et un nouveau fichier vide démarre.
        Le renommage écrase l'ancien .old s'il existe : les entrées plus
        anciennes sont alors perdues. Pas de rotation selon la taille en
        octets ni d'archives horodatées multiples ; c'est suffisant pour
        la portée actuelle (voir les limites en tête de fichier).

        Une rotation doit démarrer une nouvelle chaîne d'empreintes (qui
        repart de GENESIS_HASH) : la continuité de la chaîne s'arrête au
        fichier .old, qui reste vérifiable séparément. verify_chain() ne
        lit que le fichier courant d'une catégorie : pour vérifier le
        .old, il faut par exemple le copier sous le nom
        <catégorie>.jsonl dans un autre répertoire et appeler
        verify_chain() sur un StructuredLogger ouvert sur ce répertoire.

        Limite connue (constatée le 2026-10-13, non couverte par les
        tests) : log() calcule le prev_hash de la nouvelle entrée avant
        d'appeler cette méthode. La première entrée du nouveau fichier
        porte donc l'empreinte de la dernière entrée du .old, et non
        GENESIS_HASH : verify_chain() appliqué au nouveau fichier signale
        une rupture dès la ligne 1 (et, à partir de la deuxième rotation,
        au fichier .old aussi), alors que rien n'a été falsifié.
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
        Relit les n dernières entrées d'une catégorie.

        La lecture est simple (le fichier entier est chargé, sans
        optimisation pour les très gros fichiers) : c'est suffisant pour
        un diagnostic ponctuel, pas pour une surveillance en temps réel
        à grande échelle. Renvoie une liste vide, plutôt que de lever
        une exception, si le fichier n'existe pas ou est illisible ; une
        ligne corrompue est simplement sautée.
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
                continue  # ligne corrompue : sautée sans tout faire échouer
        return results

    def get_write_failures(self) -> list[str]:
        """Historique des échecs d'écriture, à consulter quand on
        soupçonne des entrées perdues (disque plein, permissions
        refusées) ; inutile en usage normal."""
        with self._lock:
            return list(self._write_failures)

    def verify_chain(self, category: LogCategory) -> "ChainVerificationResult":
        """
        Vérifie l'intégrité de la chaîne d'empreintes d'une catégorie.

        Relit tout le fichier et, pour chaque entrée, vérifie deux
        choses : que l'empreinte recalculée à partir de son contenu
        (prev_hash compris) est égale au champ 'hash' stocké, et que son
        prev_hash est bien l'empreinte de l'entrée précédente.

        Toute rupture (empreinte recalculée différente de celle stockée,
        ou prev_hash qui ne correspond pas à l'entrée précédente) signale
        soit une modification, soit la suppression ou l'insertion d'une
        ou plusieurs lignes : dans les deux cas, la continuité de la
        chaîne est cassée à cet endroit.

        Ne lève jamais d'exception, pour que l'outil de vérification ne
        plante jamais lui-même. Un fichier absent ou vide donne un
        résultat « valide » trivial (rien à vérifier) ; un fichier
        illisible ou une chaîne rompue donne un résultat « invalide »
        qui décrit la première rupture trouvée.
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
    """Résultat de StructuredLogger.verify_chain().

    `entries_checked` compte les entrées confirmées valides avant la
    première rupture trouvée ; il vaut donc le nombre total d'entrées si
    valid=True. `break_at_line` donne le rang de l'entrée fautive (à
    partir de 1, lignes vides non comptées), ou None quand il n'y a pas
    de rupture ou que le fichier est illisible. `reason` explique la
    rupture en clair."""
    valid: bool
    entries_checked: int
    break_at_line: Optional[int]
    reason: Optional[str]


# Instance partagée par défaut, créée au premier appel de
# get_default_logger() : chaque composant (Watchdog, SafetyGuard,
# sandbox) n'a ainsi ni à créer son propre journal ni à choisir son
# répertoire. Un composant qui a besoin d'un autre répertoire (tests,
# isolation) peut toujours créer explicitement son propre
# StructuredLogger() au lieu d'utiliser celui-ci.
_default_logger: Optional[StructuredLogger] = None
_default_logger_lock = threading.Lock()


def get_default_logger() -> StructuredLogger:
    global _default_logger
    with _default_logger_lock:
        if _default_logger is None:
            _default_logger = StructuredLogger()
        return _default_logger
