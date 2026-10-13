"""
AXIS-OS — chargeur URDF
========================
Pourquoi ce module existe : les articulations d'un robot (leur nom, leur
type, leurs limites, leur place dans la structure mécanique) étaient
recodées à la main dans le pilote (le code propre à chaque modèle de
robot), avec des erreurs de recopie que rien ne détectait. C'est arrivé
sur le robot humanoïde G1 d'Unitree : la fonction
_build_g1_joint_definitions() de
axis_os/hal/drivers/unitree_g1/robot.py, alors écrite à la main,
déclarait 24 articulations aux noms approximatifs, et elle n'a jamais
signalé d'elle-même le problème. L'écart a été découvert lors d'essais
dans le simulateur Isaac Sim (NVIDIA) le 2026-09-06 : on estimait alors
que le vrai G1 avait 37 articulations, avec un torse à une seule
articulation et des coudes à deux axes séparés. Ce décompte, établi lui
aussi par observation dans le simulateur, s'est à son tour révélé
inexact sur des points précis une fois les fichiers officiels du
fabricant obtenus (le coude du G1, par exemple, est une seule
articulation, pas deux) ; le détail figure dans le fichier
axis_os/hal/drivers/unitree_g1/urdf/README.md. Un fichier URDF officiel,
lu par ce module, rend ce genre d'écart impossible : il n'y a qu'une
seule source, celle du fabricant, et plus aucune retranscription
manuelle exposée aux erreurs de copie.

URDF (Unified Robot Description Format) est un format XML qui décrit un
robot : ses pièces rigides (les « links »), les articulations qui les
relient (les « joints ») et leurs limites. C'est le standard de fait de
la robotique depuis ROS 1 (Robot Operating System, l'ensemble logiciel
open source de référence en robotique, à partir de 2007) : la
quasi-totalité des fabricants sérieux, dont Unitree, publient un
fichier .urdf ou .xacro officiel pour chaque robot (xacro est un langage
de macros XML qui sert à produire un URDF).

Ce module ne réimplémente pas la norme complète : ni les balises
<transmission> (liaison entre moteur et articulation), ni les balises
propres au simulateur Gazebo, ni xacro et ses macros. Il ne lit que ce
dont le HAL (la couche d'abstraction matérielle, seul chemin vers les
moteurs) a besoin : les articulations, leurs limites et la chaîne
parent/enfant. La structure des classes s'inspire de `urdf_parser_py`
(le parseur officiel maintenu par ROS / Open Robotics, sous licence
BSD), pour rester familière à qui connaît l'écosystème ROS. Elle est
toutefois réécrite ici sans dépendance externe, avec le seul module
`xml.etree` de la bibliothèque standard de Python, pour ne pas imposer
à AXIS-OS un paquet de plus à installer et à maintenir.

Ce module ne remplace pas get_joint_definitions() de chaque HAL : il
fournit l'outil qui permet à chaque pilote (UnitreeG1HAL, ou un futur
pilote pour un autre robot) de charger ses articulations depuis un
fichier au lieu de les coder en dur. L'adoption se fait pilote par
pilote, volontairement et progressivement. À l'écriture de ce module,
le G1 restait codé à la main, en attendant d'obtenir le fichier URDF
officiel et de le vérifier contre un vrai G1 dans Isaac Sim ; la
correction faite alors dans backend.py pendant ces essais était la
seule source vérifiée. Depuis, le pilote du G1 charge ses articulations
depuis les fichiers URDF officiels d'Unitree, variantes à 23 et 29
articulations (voir build_g1_joint_definitions() dans robot.py, et
conventions.py pour les conventions de signe et de décalage propres au
G1).

Tests : test/test_urdf_loader.py.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Union

from axis_os.hal.core.types import JointDefinition, JointLimits, JointType


class URDFParseError(Exception):
    """
    Levée pour tout URDF malformé, ou incomplet au regard des besoins du
    HAL.

    Le chargeur est volontairement strict plutôt que permissif : une
    articulation mal lue puis ignorée en silence, c'est exactement le
    genre d'écart (articulation manquante, limite absente) qui a causé
    l'erreur de décompte du G1, 24 articulations contre 37, décrite en
    tête de fichier. Mieux vaut un échec de chargement bruyant qu'une
    définition de robot incomplète, passée inaperçue jusqu'au premier
    plantage dans Isaac Sim ou, pire, sur un vrai robot.
    """
    pass


# Correspondance entre le type déclaré dans l'URDF (attribut `type` de
# <joint>) et l'énumération JointType d'AXIS-OS. "fixed" et "floating"
# en sont volontairement absents, ce n'est pas un oubli : ce sont des
# articulations qui ne bougent jamais (fixed) ou qui sortent du
# périmètre actuel du contrôle moteur d'AXIS-OS (floating, qui sert à
# ancrer un robot mobile à sa base). get_joints() les ignore sans
# erreur (voir _NON_ACTUATED_URDF_TYPES ci-dessous).
#
# Limite connue : une articulation "continuous" (rotation sans butée)
# est traitée comme "revolute" (rotation entre deux butées), mais
# get_joints() exige des bornes lower/upper, que la spécification URDF
# demande justement d'omettre pour ce type. Une articulation continuous n'est donc acceptée que si
# le fichier lui donne quand même ces bornes.
_URDF_JOINT_TYPE_MAP: Dict[str, JointType] = {
    "revolute":  JointType.REVOLUTE,
    "continuous": JointType.REVOLUTE,  # revolute sans butée d'angle
    "prismatic": JointType.PRISMATIC,
}

# Articulations déclarées dans l'URDF mais non pilotées par le contrôle
# moteur : get_joints() ne les renvoie jamais. "fixed" relie deux pièces
# rigides sans aucun mouvement possible ; "floating" ancre le robot à sa
# base par six degrés de liberté (DoF) virtuels, sans aucun moteur réel
# derrière.
_NON_ACTUATED_URDF_TYPES = {"fixed", "floating"}


@dataclass
class URDFJointInfo:
    """
    Une articulation telle que décrite dans l'URDF, avant sa conversion
    en JointDefinition (la description d'articulation utilisée partout
    dans AXIS-OS). Les champs name, joint_type, parent et child portent
    les mêmes noms que dans `urdf_parser_py`, pour qu'un habitué de
    l'écosystème ROS retrouve immédiatement ses repères. Les limites,
    regroupées là-bas dans un objet `limit`, sont ici mises à plat
    (pos_min, pos_max, vel_max, effort_max) ; l'axe de rotation (`axis`)
    n'est pas lu.
    """
    name:        str
    joint_type:  str            # valeur brute ("revolute", "fixed"...)
    parent:      str
    child:       str
    # Limites lues dans la balise <limit> ; None quand elles sont
    # absentes. Unités URDF : radians (ou mètres pour une articulation
    # prismatique, c'est-à-dire en translation) pour la position, rad/s
    # (ou m/s) pour la vitesse.
    pos_min:     Optional[float] = None
    pos_max:     Optional[float] = None
    vel_max:     Optional[float] = None
    effort_max:  Optional[float] = None  # "effort" URDF : couple ou force max


@dataclass
class URDFRobotModel:
    """
    Résultat de la lecture : le nom du robot et toutes ses
    articulations, dans l'ordre où elles apparaissent dans le fichier.
    `joints` contient aussi les articulations fixed et floating, utiles
    pour inspecter la chaîne cinématique complète (l'enchaînement des
    pièces et des articulations) ; c'est get_joints() qui ne garde que
    les articulations réellement motorisées.
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
        Convertit les articulations motorisées (revolute, continuous,
        prismatic) en JointDefinition, prêtes à être renvoyées par
        IRobotHAL.get_joint_definitions() (l'interface commune que doit
        offrir le HAL de chaque robot).

        Les paramètres default_* s'appliquent quand l'URDF ne précise
        pas une limite. Ce chargeur tolère l'absence de la vitesse
        (`velocity`) et de l'effort (`effort`) dans la balise <limit> et
        applique alors default_vel_max et default_torque_max, alors que
        la spécification URDF les déclare obligatoires (voir la
        documentation de la balise <joint> d'urdfdom, la bibliothèque
        de référence de ROS pour lire les URDF). Les bornes de position,
        elles, ne sont jamais inventées : une articulation motorisée
        dont la balise <limit> manque, ou ne donne pas `lower` et
        `upper`, lève une URDFParseError, alors que la spécification
        leur donnerait la valeur 0 par défaut. pos_min et pos_max sont
        des données de sécurité, jamais remplacées par une valeur
        approchée.

        current_max et temperature_max n'existent pas dans le standard
        URDF, qui ne décrit pas l'électronique de puissance : ils
        prennent les valeurs default_* pour chaque articulation. C'est
        écrit ici plutôt que laissé implicite : un intégrateur doit
        savoir que ces deux limites ne viennent jamais du fichier, quel
        que soit le robot chargé.
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
    """Renvoie None si l'élément ou l'attribut est absent, jamais 0.0
    par défaut : il faut distinguer « non précisé » d'« une borne à
    zéro », car une limite à 0.0 rad est légitime pour une articulation
    qui ne doit jamais quitter sa position neutre. Une valeur non
    numérique n'est pas interceptée : float() lève alors ValueError,
    et non URDFParseError."""
    if elem is None:
        return None
    val = elem.get(attr)
    return float(val) if val is not None else None


def load_urdf(source: Union[str, Path]) -> URDFRobotModel:
    """
    Lit un fichier URDF (chemin) ou une chaîne XML déjà en mémoire, et
    renvoie le modèle du robot (voir _looks_like_path() pour la façon
    dont les deux cas sont distingués).

    Les fichiers xacro (macros, $(find ...), expressions) ne sont pas
    pris en charge : un .xacro doit d'abord être développé en .urdf
    brut. Le SDK officiel du fabricant (son kit de développement)
    fournit généralement les deux, sinon l'outil en ligne de commande
    `xacro` fait la conversion. Un xacro est détecté et signalé
    explicitement, plutôt que de donner sans prévenir une lecture
    incomplète.

    Lève URDFParseError si le XML est invalide, si la racine n'est pas
    <robot>, s'il manque à une articulation son nom, son type, son
    parent ou son enfant, ou si le fichier ne contient aucune
    articulation.
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
    """Heuristique simple, pour ne pas obliger l'appelant à préciser
    s'il passe un chemin ou du XML : un objet Path, ou une chaîne sans
    '<', est traité comme un chemin de fichier ; toute chaîne contenant
    '<' est traitée comme du XML déjà en mémoire."""
    if isinstance(source, Path):
        return True
    return "<" not in source
