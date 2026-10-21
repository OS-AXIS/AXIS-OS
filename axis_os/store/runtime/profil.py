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

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations

import base64
import binascii
import os
import stat
from enum import Enum
from pathlib import Path
from typing import Dict, Union

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


def lire_cles_approuvees(chemin: Union[str, os.PathLike]) -> Dict[str, str]:
    """Lit le fichier des clés approuvées (format en tête de module) et
    rend {clé publique base64: nom du développeur}."""
    p = Path(chemin)
    # Ouvert sans suivre de lien symbolique, puis contrôlé et lu par le même
    # descripteur : le fichier vérifié est bien celui qui est lu.
    try:
        # O_NONBLOCK : un tube nommé placé à ce chemin ne bloque pas l'ouverture.
        fd = os.open(p, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)
    except OSError as exc:
        raise ClesApprouveesInvalides(
            f"fichier des clés approuvées illisible (ou lien symbolique) : {exc}") from None
    with os.fdopen(fd, "rb") as f:
        infos = os.fstat(f.fileno())
        if not stat.S_ISREG(infos.st_mode):
            raise ClesApprouveesInvalides(f"{p} n'est pas un fichier ordinaire")
        if infos.st_mode & (stat.S_IWGRP | stat.S_IWOTH):
            raise ClesApprouveesInvalides(
                f"{p} est modifiable par le groupe ou par les autres utilisateurs "
                f"(droits {stat.filemode(infos.st_mode)}) : n'importe qui pourrait y "
                f"ajouter sa clé. Corriger par : chmod go-w {p}")
        if infos.st_uid not in (0, os.geteuid()):
            raise ClesApprouveesInvalides(
                f"{p} appartient à un autre utilisateur (uid {infos.st_uid}), qui "
                f"pourrait y ajouter sa clé : il doit appartenir à root ou à "
                f"l'utilisateur de l'OS.")
        if infos.st_size > (1 << 20):
            raise ClesApprouveesInvalides(f"{p} dépasse 1 Mio : refusé plutôt que tronqué")
        brut = f.read()
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
        try:
            brut = base64.b64decode(cle, validate=True)
        except (binascii.Error, ValueError):
            brut = b""
        if len(brut) != 32:
            raise ClesApprouveesInvalides(
                f"{p}, ligne {numero} : clé publique Ed25519 base64 attendue "
                f"(32 octets), lu {cle[:16]!r}")
        cles[cle] = nom.strip()
    return cles
