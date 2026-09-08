"""
AXIS-OS — URDFLoader
=======================
Parseur URDF générique : source de vérité UNIQUE pour les définitions de
joints d'un robot, au lieu de les recoder à la main pour chaque nouveau
modèle (voir _build_g1_joint_definitions() dans unitree_g1/robot.py,
qui a produit — et n'a jamais détecté seul — l'écart 24 vs 37 joints
découvert lors de la session Isaac Sim du 2026-09-06 : le stub interne
déclarait 24 joints avec des noms approximatifs, alors que le vrai G1
en a 37, avec un torse à un seul joint et des coudes en deux axes
séparés. Un fichier URDF officiel, une fois parsé, aurait rendu cet
écart impossible : il n'y aurait eu qu'UNE SEULE source, celle du
fabricant, jamais une retranscription manuelle risquant l'erreur de
copie.

URDF (Unified Robot Description Format) est le standard de facto de
l'industrie robotique depuis ROS 1 (2007+) — la quasi-totalité des
fabricants sérieux (dont Unitree) publient un .urdf ou .xacro officiel
pour chaque robot. Ce module ne réimplémente pas la norme complète
(pas de <transmission>, pas de tags Gazebo, pas de xacro/macros) —
seulement ce dont le HAL a besoin : joints, limites, chaîne
parent/enfant. Structure des classes internes inspirée de
`urdf_parser_py` (le parseur officiel maintenu par ROS/Open Robotics,
license BSD), pour rester familière à quiconque a déjà travaillé avec
l'écosystème ROS — mais réécrite ici sans dépendance externe, avec
uniquement `xml.etree` (stdlib), pour ne pas alourdir AXIS-OS d'un
paquet de plus à installer et maintenir.

Ce module NE remplace PAS get_joint_definitions() dans chaque HAL —
il fournit l'outil pour que chaque driver (UnitreeG1HAL, un futur
driver pour un autre robot) puisse CHARGER ses joints depuis un
fichier plutôt que les coder en dur. L'adoption reste PAR DRIVER,
volontaire et progressive (voir conventions.py/robot.py pour l'état
actuel du G1, où le codage manuel reste en place tant que le fichier
URDF officiel n'a pas été obtenu et vérifié contre un vrai G1 en
Isaac Sim — la correction faite dans backend.py cette session reste
la seule source vérifiée pour l'instant).

Licence : Propriétaire — AXIS-OS
"""

from __future__ import annotations
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Union

from axis_os.hal.core.types import JointDefinition, JointLimits, JointType


class URDFParseError(Exception):
    """
    Levée sur tout URDF malformé ou incomplet pour les besoins du HAL.
    Volontairement stricte plutôt que permissive : un joint mal
    parsé et silencieusement ignoré est exactement le genre d'écart
    (joint manquant, limite absente) qui a causé le bug 24 vs 37 —
    mieux vaut un échec de chargement bruyant qu'une définition de
    robot incomplète passée sous silence jusqu'au premier crash en
    Isaac Sim ou, pire, sur du matériel réel.
    """
    pass


# Correspondance entre le type déclaré dans l'URDF (attribut `type` de
# <joint>) et l'enum JointType d'AXIS-OS. "fixed" et "floating" sont
# volontairement exclus : ce sont des joints qui ne bougent jamais
# (fixed) ou hors du périmètre actuel du contrôle moteur d'AXIS-OS
# (floating, utilisé pour ancrer un robot mobile à sa base) — get_joints()
# les ignore silencieusement (voir plus bas), pas une erreur de mapping.
_URDF_JOINT_TYPE_MAP: Dict[str, JointType] = {
    "revolute":  JointType.REVOLUTE,
    "continuous": JointType.REVOLUTE,  # continuous = revolute sans limite d'angle
    "prismatic": JointType.PRISMATIC,
}

# Joints déclarés dans l'URDF mais non pilotés par le contrôle moteur —
# jamais retournés par get_joints(). "fixed" relie deux liens rigides
# sans articulation ; "floating" ancre le robot à sa base (6 DoF virtuels,
# pas un vrai actionneur physique).
_NON_ACTUATED_URDF_TYPES = {"fixed", "floating"}


@dataclass
class URDFJointInfo:
    """
    Un joint tel que décrit dans l'URDF, avant conversion vers
    JointDefinition. Champs nommés à l'identique d'`urdf_parser_py`
    (name, joint_type, parent, child, limit, axis) pour qu'un lecteur
    familier avec l'écosystème ROS retrouve immédiatement ses repères.
    """
    name:        str
    joint_type:  str            # valeur brute de l'URDF ("revolute", "fixed"...)
    parent:      str
    child:       str
    pos_min:     Optional[float] = None
    pos_max:     Optional[float] = None
    vel_max:     Optional[float] = None
    effort_max:  Optional[float] = None  # "effort" en URDF = couple/force max


@dataclass
class URDFRobotModel:
    """
    Résultat du parsing : le nom du robot et tous ses joints, dans
    l'ordre où ils apparaissent dans le fichier. `joints` inclut aussi
    les joints fixed/floating (utile pour inspecter la chaîne
    cinématique complète) — c'est get_joints() qui filtre pour ne
    garder que les joints réellement actionnés.
    """
    name:   str
    joints: List[URDFJointInfo] = field(default_factory=list)

    def get_joints(
        self,
        default_vel_max: float = 20.0,
        default_torque_max: float = 40.0,
        default_current_max: float = 15.0,
        default_temperature_max: float = 80.0,
    ) -> List[JointDefinition]:
        """
        Convertit les joints actionnés (revolute/continuous/prismatic)
        en JointDefinition, prêts à l'emploi pour IRobotHAL.get_joint_definitions().

        Les `default_*` s'appliquent quand l'URDF ne précise pas une
        limite (cas fréquent : `velocity`/`effort` sont optionnels dans
        la balise <limit>, contrairement à `lower`/`upper` — voir la
        doc urdfdom citée en tête de fichier). Un joint SANS aucune
        balise <limit> lève une URDFParseError plutôt que d'inventer
        silencieusement des bornes de position : pos_min/pos_max sont
        des données de sécurité, jamais approximées par un défaut.

        current_max et temperature_max n'existent pas dans le standard
        URDF (qui ne modélise pas l'électronique de puissance) — ils
        restent aux valeurs `default_*` pour CHAQUE joint. C'est
        volontairement documenté ici plutôt que silencieux : un
        intégrateur doit savoir que ces deux limites-là ne viennent
        jamais du fichier, quel que soit le robot chargé.
        """
        result: List[JointDefinition] = []
        for j in self.joints:
            if j.joint_type in _NON_ACTUATED_URDF_TYPES:
                continue
            mapped_type = _URDF_JOINT_TYPE_MAP.get(j.joint_type)
            if mapped_type is None:
                raise URDFParseError(
                    f"Joint '{j.name}' : type URDF '{j.joint_type}' non supporté "
                    f"par AXIS-OS (supportés : {sorted(_URDF_JOINT_TYPE_MAP)} "
                    f"+ fixed/floating, ignorés). Ajout volontairement refusé "
                    f"plutôt qu'ignoré en silence."
                )
            if j.pos_min is None or j.pos_max is None:
                raise URDFParseError(
                    f"Joint '{j.name}' (type={j.joint_type}) : balise <limit> "
                    f"absente ou incomplète (lower/upper manquants). Un joint "
                    f"actionné sans limites de position n'est pas chargé par "
                    f"prudence — complétez l'URDF ou traitez ce joint comme fixed."
                )
            result.append(JointDefinition(
                joint_id=j.name,
                joint_type=mapped_type,
                limits=JointLimits(
                    pos_min=j.pos_min,
                    pos_max=j.pos_max,
                    vel_max=j.vel_max if j.vel_max is not None else default_vel_max,
                    torque_max=j.effort_max if j.effort_max is not None else default_torque_max,
                    current_max=default_current_max,
                    temperature_max=default_temperature_max,
                ),
                parent_link=j.parent,
                child_link=j.child,
            ))
        return result


def _parse_float_attr(elem: Optional[ET.Element], attr: str) -> Optional[float]:
    """None si l'élément ou l'attribut est absent — jamais 0.0 par
    défaut, pour distinguer explicitement "non spécifié" de "borne à
    zéro" (une limite à 0.0 rad est une valeur légitime pour un joint
    qui ne doit jamais bouger de sa position neutre)."""
    if elem is None:
        return None
    val = elem.get(attr)
    return float(val) if val is not None else None


def load_urdf(source: Union[str, Path]) -> URDFRobotModel:
    """
    Parse un fichier URDF (chemin) ou une chaîne XML déjà en mémoire.

    Ne gère PAS xacro (macros, $(find ...), expressions) — un fichier
    .xacro doit être développé en .urdf brut au préalable (le SDK
    officiel du fabricant fournit généralement les deux, ou un outil
    comme `xacro` en ligne de commande fait la conversion). Détecté et
    signalé explicitement plutôt que de produire un parsing silencieusement
    incomplet.
    """
    text = Path(source).read_text() if _looks_like_path(source) else str(source)

    if "xacro" in text[:2000] and ("$(find" in text or "xacro:" in text):
        raise URDFParseError(
            "Ce fichier semble être un .xacro non développé (macros/$(find) "
            "détectés) — load_urdf() attend un .urdf brut. Développez-le "
            "d'abord (ex: `xacro robot.urdf.xacro > robot.urdf`)."
        )

    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        raise URDFParseError(f"XML invalide : {e}") from e

    if root.tag != "robot":
        raise URDFParseError(
            f"Élément racine '<{root.tag}>' inattendu — un URDF doit "
            f"commencer par '<robot ...>'."
        )

    model = URDFRobotModel(name=root.get("name", "unnamed_robot"))

    for joint_elem in root.findall("joint"):
        name = joint_elem.get("name")
        jtype = joint_elem.get("type")
        if not name or not jtype:
            raise URDFParseError(
                "Un <joint> sans attribut 'name' ou 'type' a été rencontré — "
                "ces deux attributs sont obligatoires dans le standard URDF."
            )

        parent_elem = joint_elem.find("parent")
        child_elem  = joint_elem.find("child")
        if parent_elem is None or child_elem is None:
            raise URDFParseError(
                f"Joint '{name}' : balise <parent> ou <child> manquante — "
                f"la chaîne cinématique ne peut pas être construite sans elles."
            )
        parent_link = parent_elem.get("link")
        child_link  = child_elem.get("link")
        if not parent_link or not child_link:
            raise URDFParseError(
                f"Joint '{name}' : <parent>/<child> présents mais sans "
                f"attribut 'link'."
            )

        limit_elem = joint_elem.find("limit")
        info = URDFJointInfo(
            name=name,
            joint_type=jtype,
            parent=parent_link,
            child=child_link,
            pos_min=_parse_float_attr(limit_elem, "lower"),
            pos_max=_parse_float_attr(limit_elem, "upper"),
            vel_max=_parse_float_attr(limit_elem, "velocity"),
            effort_max=_parse_float_attr(limit_elem, "effort"),
        )
        model.joints.append(info)

    if not model.joints:
        raise URDFParseError(
            "Aucun <joint> trouvé dans ce fichier — URDF vide ou mal formé "
            "pour l'usage HAL (un robot sans joint n'a rien à contrôler)."
        )

    return model


def _looks_like_path(source: Union[str, Path]) -> bool:
    """Heuristique simple : une Path explicite, ou une chaîne courte
    sans '<' est traitée comme un chemin de fichier ; toute chaîne
    contenant '<' est traitée comme du XML déjà en mémoire. Évite
    d'exiger que l'appelant précise lui-même le mode."""
    if isinstance(source, Path):
        return True
    return "<" not in source
