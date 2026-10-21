"""
AXIS-OS — Profils d'exécution du Play Store
============================================
Un PlayStoreRuntime tourne toujours sous un profil donné EXPLICITEMENT
par celui qui le crée ; le profil n'est jamais deviné (ni d'après le
robot, ni d'après une variable d'environnement), et aucune app ni aucun
manifeste ne peut le changer.

Dans tous les profils, les quotas déclarés par une app (ResourceQuota)
doivent rester sous les plafonds de PLAFONDS_QUOTAS, plus stricts en
PRODUCTION ; un manifeste qui en dépasse un est refusé à l'installation.

  SIMULATION     robot simulé, tests. Tout ce qui est permis aujourd'hui
                 reste permis (apps non signées, accepter_tout_SIMULATION),
                 mais chaque facilité est signalée dans le journal.
                 Refusé si le runtime pilote un robot réel connu.
  DEVELOPPEMENT  poste de développement, éventuellement un vrai robot sur
                 un banc. Mêmes facilités que la simulation, signalées de
                 même, sauf accepter_tout_SIMULATION sur un robot réel.
  PRODUCTION     un vrai robot près de vraies personnes. Signature
                 obligatoire et non désactivable ; la clé du développeur
                 doit figurer dans le fichier local des clés approuvées ;
                 accepter_tout_SIMULATION interdit ; permissions DANGEROUS
                 toujours soumises à confirmation ; quotas CPU/mémoire
                 exigés (psutil) ; le code d'une app n'est jamais exécuté
                 dans le processus de l'OS (installation depuis la source
                 signée uniquement).

Fichier des clés approuvées
---------------------------
Fichier texte UTF-8 local, lu au démarrage du runtime ; jamais pris dans
le paquet d'une app. Une clé par ligne :

    # commentaire, ligne ignorée
    <clé publique Ed25519 en base64>  <nom du développeur, facultatif>

La clé est celle que produit generate_developer_keypair()
(axis_os/store/sdk/signing.py), soit 44 caractères base64 pour 32 octets.
Une ligne illisible fait refuser tout le fichier : une erreur de saisie
ne doit pas faire accepter ou ignorer une clé en silence. Le fichier doit
être un fichier ordinaire (pas un lien symbolique), appartenir à root ou à
l'utilisateur de l'OS, et n'être modifiable ni par le groupe ni par les
autres. Ses dossiers parents ne sont pas vérifiés : à protéger par
l'administrateur (sshd, lui, les vérifie). Tant que les apps tournent sous
le même utilisateur que l'OS (étape 2 de docs/isolation_apps.md), un
fichier qui lui appartient reste modifiable par une app : le placer hors
de sa portée, par exemple propriété de root.

Registre des apps
-----------------
Fichier JSON local, obligatoire en PRODUCTION (paramètre registre_apps de
PlayStoreRuntime), qui survit aux redémarrages :

    {"format": 1, "apps": {"<app_id>": {"cle": "<clé base64>", "version": "X.Y.Z"}}}

La première clé qui installe un app_id le lie à elle : aucune autre clé ne
peut ensuite installer ou mettre à jour cet app_id, et aucune version
antérieure à la plus récente déjà installée n'est acceptée, même après un
redémarrage. Le fichier doit exister avant le premier démarrage (créé une
fois par creer_registre_apps) : un chemin mal saisi ou un fichier supprimé
ne doit pas remettre les liaisons à zéro en silence. Mêmes règles d'accès
que le fichier des clés approuvées ; il est réécrit par l'OS (fichier
temporaire, fsync, puis renommage), donc modifiable par son utilisateur,
et par une app tant qu'elle tourne sous le même utilisateur (étape 2 de
docs/isolation_apps.md). Un seul runtime par registre.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations

import base64
import binascii
import json
import os
import stat
import tempfile
from enum import Enum
from pathlib import Path
from typing import Dict, Union

from axis_os import SemVer
from axis_os.store.sdk.manifest import ResourceQuota


class Profil(Enum):
    SIMULATION = "simulation"
    DEVELOPPEMENT = "developpement"
    PRODUCTION = "production"


# Plafonds de l'OS pour chaque quota du manifeste (mêmes unités que
# ResourceQuota). Hors production, ils laissent passer les apps de
# démonstration (8 192 Mo, héritage du temps où le processus de l'app était
# une copie de celui de l'OS) et une ronde d'une heure (security_patrol).
# En production, un processus d'app neuf pèse une vingtaine de Mo.
PLAFONDS_QUOTAS: Dict[Profil, ResourceQuota] = {
    Profil.SIMULATION:    ResourceQuota(max_cpu_percent=400.0, max_memory_mb=8192,
                                        max_runtime_s=7200.0),
    Profil.DEVELOPPEMENT: ResourceQuota(max_cpu_percent=400.0, max_memory_mb=8192,
                                        max_runtime_s=7200.0),
    Profil.PRODUCTION:    ResourceQuota(max_cpu_percent=100.0, max_memory_mb=1024,
                                        max_runtime_s=3600.0),
}


class ClesApprouveesInvalides(ValueError):
    """Le fichier des clés approuvées est absent, mal formé ou trop
    ouvert en écriture."""


def _lire_fichier_local(p: Path, erreur, quoi: str) -> bytes:
    """Lit un fichier de configuration de l'OS (clés approuvées, registre
    des apps) en refusant ce qu'un tiers pourrait avoir modifié : lève
    `erreur` (une sous-classe de ValueError)."""
    # Ouvert sans suivre de lien symbolique, puis contrôlé et lu par le même
    # descripteur : le fichier vérifié est bien celui qui est lu.
    try:
        # O_NONBLOCK : un tube nommé placé à ce chemin ne bloque pas l'ouverture.
        fd = os.open(p, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)
    except OSError as exc:
        raise erreur(f"{quoi} illisible (ou lien symbolique) : {exc}") from None
    with os.fdopen(fd, "rb") as f:
        infos = os.fstat(f.fileno())
        if not stat.S_ISREG(infos.st_mode):
            raise erreur(f"{p} n'est pas un fichier ordinaire")
        if infos.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
            raise erreur(
                f"{p} est modifiable par le groupe ou par les autres utilisateurs "
                f"(droits {stat.filemode(infos.st_mode)}) : n'importe qui pourrait y "
                f"ajouter sa clé. Corriger par : chmod go-w {p}")
        if infos.st_uid not in (0, os.geteuid()):
            raise erreur(
                f"{p} appartient à un autre utilisateur (uid {infos.st_uid}), qui "
                f"pourrait y ajouter sa clé : il doit appartenir à root ou à "
                f"l'utilisateur de l'OS.")
        if infos.st_size > (1 << 20):
            raise erreur(f"{p} dépasse 1 Mio : refusé plutôt que tronqué")
        return f.read()


def _cle_valide(cle) -> bool:
    try:
        return isinstance(cle, str) and len(base64.b64decode(cle, validate=True)) == 32
    except (binascii.Error, ValueError):
        return False


def lire_cles_approuvees(chemin: Union[str, os.PathLike]) -> Dict[str, str]:
    """Lit le fichier des clés approuvées (format en tête de module) et
    rend {clé publique base64: nom du développeur}."""
    p = Path(chemin)
    brut = _lire_fichier_local(p, ClesApprouveesInvalides, "fichier des clés approuvées")
    cles: Dict[str, str] = {}
    try:
        lignes = brut.decode("utf-8").splitlines()
    except UnicodeDecodeError as exc:
        raise ClesApprouveesInvalides(f"fichier des clés approuvées illisible : {exc}") from None
    for numero, ligne in enumerate(lignes, start=1):
        ligne = ligne.strip()
        if not ligne or ligne.startswith("#"):
            continue
        cle, _, nom = ligne.partition(" ")
        if not _cle_valide(cle):
            raise ClesApprouveesInvalides(
                f"{p}, ligne {numero} : clé publique Ed25519 base64 attendue "
                f"(32 octets), lu {cle[:16]!r}")
        cles[cle] = nom.strip()
    return cles


class RegistreAppsInvalide(ValueError):
    """Le registre des apps est absent, mal formé ou trop ouvert en
    écriture : le runtime refuse de démarrer plutôt que d'oublier ses
    liaisons."""


def lire_registre_apps(chemin: Union[str, os.PathLike]) -> Dict[str, Dict[str, str]]:
    """Lit le registre des apps (format en tête de module) et rend
    {app_id: {"cle": ..., "version": ...}}. Toute anomalie lève
    RegistreAppsInvalide."""
    from axis_os.store.runtime.canal_ipc import MessageRefuse, decoder
    p = Path(chemin)
    brut = _lire_fichier_local(p, RegistreAppsInvalide, "registre des apps")
    try:
        d = decoder(brut)
    except MessageRefuse as exc:
        raise RegistreAppsInvalide(f"{p} : {exc}") from None
    apps = d.get("apps")
    if set(d) != {"format", "apps"} or d["format"] != 1 or not isinstance(apps, dict):
        raise RegistreAppsInvalide(f"{p} : format attendu {{\"format\": 1, \"apps\": {{...}}}}")
    for app_id, lien in apps.items():
        ok = (isinstance(lien, dict) and set(lien) == {"cle", "version"}
              and _cle_valide(lien["cle"]) and isinstance(lien["version"], str))
        if ok:
            try:
                ok = str(SemVer.parse(lien["version"])) == lien["version"]
            except ValueError:
                ok = False
        if not ok or not app_id or any(not c.isprintable() for c in app_id):
            raise RegistreAppsInvalide(f"{p} : entrée illisible pour {app_id[:60]!r}")
    return apps


def ecrire_registre_apps(chemin: Union[str, os.PathLike],
                         apps: Dict[str, Dict[str, str]]) -> None:
    """Réécrit le registre : fichier temporaire du même dossier (créé
    exclusivement, droits 0600), fsync, renommage, puis fsync du dossier.
    Une coupure en cours d'écriture laisse l'ancien registre intact."""
    p = Path(chemin)
    corps = json.dumps({"format": 1, "apps": apps}, ensure_ascii=False, sort_keys=True,
                       indent=1).encode("utf-8")
    # Nom aléatoire (mkstemp : O_EXCL, droits 0600) : un fichier temporaire
    # resté d'une coupure ne bloque pas l'écriture suivante.
    fd, tmp = tempfile.mkstemp(dir=p.parent, prefix=f".{p.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(corps)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, p)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    # Le nouveau registre est en place : un échec ici ne fait que le rendre
    # moins sûr face à une coupure, il ne doit pas faire croire à un refus.
    try:
        dossier = os.open(p.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(dossier)
        finally:
            os.close(dossier)
    except OSError as exc:
        print(f"  [ATTENTION] Registre des apps écrit, mais sa durabilité n'est pas "
              f"garantie (fsync du dossier : {exc!r:.120}).")


def creer_registre_apps(chemin: Union[str, os.PathLike]) -> None:
    """Crée un registre vide (droits 0600). Refuse d'écraser un fichier
    existant : ce sont les liaisons des apps déjà installées."""
    fd = os.open(chemin, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
                 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write('{"format": 1, "apps": {}}\n')
