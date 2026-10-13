"""
AXIS-OS — Validateur de conventions
=================================
Validation, au démarrage, de la table de conventions du G1 (sens et
décalage d'origine de chaque articulation, voir conventions.py).

Pourquoi un fichier séparé : une revue extérieure du code a recommandé
de « ne pas transformer JointConvention en classe qui fait tout ».
conventions.py reste donc la source de vérité, faite de données pures,
et ce fichier se charge de les contrôler :

conventions.py          → sign, offset, anatomie (données pures)
convention_validator.py → vérifie que ces données sont cohérentes
                          avant qu'AXIS-OS passe en mode mouvement

Licence : Apache 2.0
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Dict, List

from .conventions import JointConvention


# Types d'erreur qu'une table de conventions peut contenir.
class ConventionErrorKind(Enum):
    MISSING_JOINT       = auto()
    DUPLICATE_JOINT      = auto()
    INVALID_SIGN         = auto()
    INVALID_OFFSET        = auto()
    MISSING_LABEL         = auto()
    EMPTY_TABLE            = auto()


@dataclass
class ConventionError:
    kind:     ConventionErrorKind
    joint_id: str
    detail:   str

    def __str__(self) -> str:
        return f"[CONVENTION] {self.kind.name} joint={self.joint_id} — {self.detail}"


class ConventionValidationResult:
    """Résultat de la validation : valid (aucune erreur bloquante),
    errors (la liste des erreurs) et non_verifiees (articulations dont le
    sens n'a pas encore été mesuré ; tolérées en simulation, refusées sur
    un vrai robot)."""

    def __init__(self, valid: bool, errors: List[ConventionError],
                 non_verifiees: Optional[List[str]] = None):
        self.valid  = valid
        self.errors = errors
        self.non_verifiees = list(non_verifiees or [])

    def __str__(self) -> str:
        if self.valid:
            if self.non_verifiees:
                return (f"[CONVENTION] Table valide, mais {len(self.non_verifiees)} "
                        f"articulation(s) NON VÉRIFIÉE(S) : "
                        f"{', '.join(self.non_verifiees)} — tolérées en "
                        f"simulation, REFUSÉES sur un vrai robot.")
            return "[CONVENTION] Table valide — AXIS-OS peut passer en mode mouvement."
        lines = [f"[CONVENTION] {len(self.errors)} erreur(s) — mode mouvement REFUSÉ :"]
        lines += [f"  • {e}" for e in self.errors]
        return "\n".join(lines)


def validate_convention_table(
    table: Dict[str, JointConvention],
    expected_joint_ids: List[str],
) -> ConventionValidationResult:
    """
    Vérifie la cohérence d'une table de conventions avant son utilisation.

    À appeler au démarrage d'AXIS-OS, dans initialize_all() du driver du
    robot, avant que le moindre mouvement soit possible. Si le résultat
    n'est pas valid=True, le robot ne doit pas bouger.

    Contrôles : table vide (refusée d'emblée, sans autre contrôle),
    articulation attendue mais absente de la table, doublon dans
    expected_joint_ids, signe différent de ±1.0, décalage non fini
    (NaN, « Not a Number », ou infini), étiquette manquante. Les
    articulations non vérifiées ne rendent pas la table invalide : elles
    sont listées à part, et c'est l'appelant qui les refuse sur un robot
    réel.
    """
    errors: List[ConventionError] = []

    if not table:
        errors.append(ConventionError(
            ConventionErrorKind.EMPTY_TABLE, "-", "table de conventions vide"
        ))
        return ConventionValidationResult(False, errors)

    # Articulations manquantes (attendues par le driver mais absentes de
    # la table) et doublons dans la liste des articulations attendues.
    seen_ids = set()
    for jid in expected_joint_ids:
        if jid not in table:
            errors.append(ConventionError(
                ConventionErrorKind.MISSING_JOINT, jid,
                "attendu par le driver, absent de la table de conventions"
            ))
        if jid in seen_ids:
            errors.append(ConventionError(
                ConventionErrorKind.DUPLICATE_JOINT, jid,
                "apparaît plusieurs fois dans expected_joint_ids"
            ))
        seen_ids.add(jid)

    # Cohérence interne de chaque entrée : signe, décalage, étiquettes.
    for jid, conv in table.items():
        if conv.sign not in (-1.0, 1.0):
            errors.append(ConventionError(
                ConventionErrorKind.INVALID_SIGN, jid,
                f"sign={conv.sign} — doit être exactement -1.0 ou +1.0"
            ))
        if not math.isfinite(conv.offset_rad):
            errors.append(ConventionError(
                ConventionErrorKind.INVALID_OFFSET, jid,
                f"offset_rad={conv.offset_rad} n'est pas une valeur finie"
            ))
        if not conv.positive_label or not conv.negative_label:
            errors.append(ConventionError(
                ConventionErrorKind.MISSING_LABEL, jid,
                "positive_label ou negative_label manquant"
            ))

    # Non vérifiée par défaut (règle introduite par un audit de sécurité
    # du 2026-10-10) : seule une entrée qui déclare explicitement
    # verifiee=True est tenue pour vérifiée, c'est-à-dire mesurée en
    # simulation par la suite de validation S2 (voir
    # docs/campagne_validation_isaac.md). Auparavant, un objet sans ce
    # champ (table construite à la main, autre classe) était tenu pour
    # vérifié, et donc accepté sur un vrai robot, alors que personne
    # n'avait jamais mesuré le sens de son articulation. Le test est
    # « is True », et non une simple valeur vraie : une chaîne « non » ou
    # un 1 ne valent pas une vérification.
    non_verifiees = [jid for jid, c in table.items()
                     if getattr(c, "verifiee", False) is not True]
    return ConventionValidationResult(valid=(len(errors) == 0), errors=errors,
                                      non_verifiees=non_verifiees)
