"""
AXIS-OS — Version globale et compatibilité SDK
==================================================
Point d'entrée du paquet Python axis_os. Il expose __version__, le numéro
de version d'AXIS-OS, et la classe SemVer, qui lit et compare des
numéros de version au format SemVer (Semantic Versioning : trois nombres
MAJOR.MINOR.PATCH qui ont chacun un sens précis, voir plus bas).

À quoi il sert : avant d'installer une app tierce, PlayStoreRuntime (le
composant qui installe et exécute les apps du Play Store, la boutique
d'apps d'AXIS-OS) vérifie que l'app a été écrite pour une version
compatible d'AXIS-OS et de son SDK (le kit de développement fourni aux
auteurs d'apps).
L'app déclare cette version minimale dans son manifeste, la fiche qui la
décrit (identifiant, version, permissions, matériel requis) : voir
AppManifest.sdk_version_min dans axis_os/store/sdk/manifest.py.

Pourquoi ce fichier existe : avant la version 0.2.0 (2026-09-08), AXIS-OS
n'avait aucun numéro de version global, seulement des versions isolées et
sans lien entre elles : HAL_VERSION="0.1.0" dans chaque pilote matériel
(le HAL, Hardware Abstraction Layer, est la couche d'abstraction
matérielle par laquelle passent toutes les commandes du robot) et une
version propre à chaque app dans son manifeste. Le changement cassant de
la 0.2.0 n'était donc tracé nulle part : HardwareRequirements.required et
.optional ont remplacé les anciens champs booléens requires_arms,
requires_legs, etc., et les identifiants d'articulation (joint_id) ont
été renommés avec le suffixe "_joint". Une app tierce écrite avant ce
changement aurait échoué à l'installation ou en cours d'exécution, sans
aucun message pour expliquer pourquoi. Le détail et la marche à suivre
pour migrer sont dans CHANGELOG.md, section [0.2.0].

Politique de version adoptée (SemVer 2.0.0, MAJOR.MINOR.PATCH) :
  - AXIS-OS est en 0.x : l'API (l'interface de programmation offerte aux
    apps) n'est pas encore stable, et des changements cassants restent
    possibles même entre versions mineures. C'est la pratique normale et
    documentée pour un logiciel en 0.x (voir Zuplo, « Semantic Versioning
    for APIs », 2026 : "Before 1.0.0, the API is considered unstable and
    any release can contain breaking changes"). Le passage à 1.0.0
    marquera un premier engagement de stabilité envers les développeurs
    tiers.
  - Après 1.0.0 : MAJOR augmente pour tout changement cassant, MINOR pour
    des ajouts rétrocompatibles, PATCH pour des corrections sans effet
    sur le contrat.
  - Principe « pay for play » (repris du SDK Strands, 2026), à suivre dès
    que possible : une nouvelle fonctionnalité optionnelle ne doit jamais
    casser le code qui ne l'utilise pas encore. Le changement de la 0.2.0
    n'a pas suivi ce principe : il a remplacé directement les anciens
    champs au lieu de les ajouter progressivement en gardant un temps les
    anciens, marqués comme dépréciés (appelés à disparaître). C'est le
    prix d'une vraie correction de sécurité faite sans délai : les
    limites de couple codées à la main étaient sous-estimées (jusqu'à
    37 % sur certains joints de hanche et de genou, voir CHANGELOG.md).
    Cette correction a été jugée plus importante que la continuité de
    l'API à ce stade encore expérimental (0.x) du projet.
"""

from __future__ import annotations
from dataclasses import dataclass


# Version actuelle d'AXIS-OS. À incrémenter à chaque changement qui
# touche le contrat public du SDK (manifeste, HAL, interfaces du Play
# Store), pas à chaque commit. L'historique des changements par version
# est dans CHANGELOG.md, à la racine du dépôt.
__version__ = "0.2.0"


@dataclass(frozen=True, order=True)
class SemVer:
    """
    Numéro de version SemVer, immuable (frozen=True) et comparable. Avec
    `order=True`, la dataclass (classe Python dont les méthodes usuelles
    sont écrites d'office à partir des champs) génère les opérateurs de
    comparaison (<, <=, >, >=) : ils comparent les champs dans l'ordre de
    leur déclaration (major, puis minor, puis patch), ce qui est
    exactement l'ordre voulu par SemVer.
    """
    major: int
    minor: int
    patch: int

    @classmethod
    def parse(cls, version_string: str) -> "SemVer":
        """
        Lit une chaîne "X.Y.Z" au format strict : trois entiers, aucun
        négatif. Pourquoi strict : une version mal formée dans un
        manifeste doit être signalée clairement à l'installation, pas
        ignorée ni devinée en silence. Tout autre format lève donc
        ValueError, avec un message qui dit ce qui ne va pas.
        """
        parts = version_string.strip().split(".")
        if len(parts) != 3:
            raise ValueError(
                f"Version SemVer invalide : '{version_string}' — "
                f"format attendu 'MAJOR.MINOR.PATCH' (ex: '0.2.0')."
            )
        try:
            major, minor, patch = (int(p) for p in parts)
        except ValueError as e:
            raise ValueError(
                f"Version SemVer invalide : '{version_string}' — "
                f"chaque partie doit être un entier."
            ) from e
        if major < 0 or minor < 0 or patch < 0:
            raise ValueError(
                f"Version SemVer invalide : '{version_string}' — "
                f"les composants ne peuvent pas être négatifs."
            )
        return cls(major, minor, patch)

    def __str__(self) -> str:
        return f"{self.major}.{self.minor}.{self.patch}"

    def is_compatible_with(self, current: "SemVer") -> bool:
        """
        Renvoie True si une app qui déclare cette version (self) comme
        sdk_version_min peut tourner sur la version `current` d'AXIS-OS
        installée.

        Pourquoi une règle plus stricte en 0.x : par convention SemVer,
        une version 0.x ne garantit aucune compatibilité, même entre
        versions mineures (à partir de 1.0, au contraire, un MINOR plus
        récent reste rétrocompatible) ; voir la docstring du module. Un
        changement cassant comme celui de la 0.2.0 a donc pu se produire
        sans que le MAJOR change, la norme ne l'imposant pas en 0.x.

        Règle appliquée, cohérente avec SemVer 2.0.0 :
          - dans tous les cas, current doit être >= self : l'OS installé
            n'est jamais plus ancien que ce que l'app demande ;
          - si l'une des deux versions est en 0.x (major == 0), on exige
            en plus le même MAJOR et exactement le même MINOR ; seul le
            PATCH peut être plus récent. Une app qui déclare
            sdk_version_min="0.1.0" est donc jugée incompatible avec
            current="0.2.0" ;
          - si les deux sont en 1.x ou au-delà, seule la règle standard
            s'applique : current.major == self.major et current >= self.
        """
        if self.major == 0 or current.major == 0:
            return (
                current.major == self.major
                and current.minor == self.minor
                and current.patch >= self.patch
            )
        return current.major == self.major and current >= self
