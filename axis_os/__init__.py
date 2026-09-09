"""
AXIS-OS — Version globale et compatibilité SDK
==================================================
Point d'entrée du package : expose __version__ et des utilitaires de
comparaison SemVer, utilisés par PlayStoreRuntime pour vérifier qu'une
app a été codée pour une version d'AXIS-OS compatible avant de
l'installer (voir manifest.py : AppManifest.sdk_version_min).

Pourquoi ce fichier existe : jusqu'à cette session, AXIS-OS n'avait
AUCUN numéro de version global — seulement des versions isolées et non
reliées entre elles (HAL_VERSION="0.1.0" dans chaque driver, version
par app dans son manifest). Un changement cassant comme celui fait
cette session (HardwareRequirements.required/.optional remplaçant les
anciens champs booléens requires_arms/requires_legs/etc., joint_id
renommés avec suffixe "_joint") n'était tracé nulle part : une app
tierce codée avant ce changement aurait échoué à l'installation ou en
plein vol, sans qu'aucun message n'explique pourquoi.

Politique de version adoptée (SemVer 2.0.0, MAJOR.MINOR.PATCH) :
  - AXIS-OS est en 0.x : l'API n'est PAS encore stable, des
    changements cassants restent possibles y compris en version
    mineure (pratique normale et documentée pour du logiciel en
    version 0.x — voir Zuplo, "Semantic Versioning for APIs", 2026 :
    "Before 1.0.0, the API is considered unstable and any release can
    contain breaking changes"). Le passage à 1.0.0 signalera un
    premier engagement de stabilité aux développeurs tiers.
  - Une fois passé 1.0.0 : MAJOR pour tout changement cassant, MINOR
    pour des ajouts rétrocompatibles, PATCH pour des corrections sans
    impact sur le contrat.
  - Principe "pay for play" (SDK Strands, 2026) à privilégier dès que
    possible : une nouvelle fonctionnalité optionnelle ne doit jamais
    casser le code qui ne l'utilise pas encore. Le changement de ce
    soir n'a PAS suivi ce principe (remplacement direct plutôt
    qu'ajout progressif avec dépréciation) — c'est le prix de
    corriger une vraie erreur de sécurité (couples sous-estimés) sans
    délai, jugé plus important que la continuité API à ce stade
    encore expérimental (0.x) du projet.
"""

from __future__ import annotations
from dataclasses import dataclass


# Version actuelle d'AXIS-OS. À incrémenter à CHAQUE changement qui
# touche le contrat public du SDK (manifest, HAL, interfaces store) —
# pas à chaque commit. Voir CHANGELOG.md à la racine du repo pour
# l'historique des changements par version.
__version__ = "0.2.0"


@dataclass(frozen=True, order=True)
class SemVer:
    """
    Version SemVer comparable. `order=True` génère automatiquement
    les opérateurs de comparaison (<, <=, >, >=) en comparant les
    champs dans l'ordre de déclaration (major, puis minor, puis
    patch) — exactement la sémantique SemVer voulue.
    """
    major: int
    minor: int
    patch: int

    @classmethod
    def parse(cls, version_string: str) -> "SemVer":
        """
        Parse une chaîne "X.Y.Z" stricte. Lève ValueError sur tout
        format inattendu plutôt que de deviner silencieusement —
        une version mal formée dans un manifest doit être signalée
        clairement à l'installation, pas ignorée.
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
        True si une app qui déclare CETTE version comme sdk_version_min
        peut tourner sur la version `current` d'AXIS-OS installée.

        Règle appliquée, cohérente avec SemVer 2.0.0 :
          - current doit être >= self (l'OS installé n'est jamais plus
            ancien que ce que l'app requiert)
          - tant qu'on reste en 0.x (major == 0), on exige en plus la
            même version MINOR exacte : en 0.x, par convention SemVer,
            AUCUNE compatibilité ascendante n'est garantie même entre
            versions mineures (contrairement à >=1.0 où le MINOR reste
            rétrocompatible) — voir docstring de module. Donc une app
            qui déclare sdk_version_min="0.1.0" est considérée
            incompatible avec current="0.2.0", puisqu'un changement
            cassant (comme celui de cette session) a pu se produire
            entre les deux sans bump de MAJOR, la norme 0.x ne
            l'imposant pas.
          - une fois major >= 1, seule la règle standard s'applique :
            current.major == self.major ET current >= self.
        """
        if self.major == 0 or current.major == 0:
            return (
                current.major == self.major
                and current.minor == self.minor
                and current.patch >= self.patch
            )
        return current.major == self.major and current >= self
