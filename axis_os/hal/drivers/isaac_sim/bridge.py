"""
Pont réutilisable entre le HAL du Unitree G1 et le simulateur Isaac Sim.

Le HAL (couche d'abstraction matérielle) est l'interface par laquelle
AXIS-OS commande les moteurs du robot ; Isaac Sim est le simulateur
physique de NVIDIA. Ce module branche le premier sur le second, pour que
le code de sécurité d'AXIS-OS soit éprouvé contre une vraie physique, et
pas seulement en test unitaire.

Il remplace l'intercepteur de fonction écrit pour une seule séance
d'essais (voir docs/isaac_lab_unitree_sim_status.md). Trois
exigences, fixées par docs/campagne_validation_isaac.md (§1), sont tenues :

1. Boucle fermée : les états articulaires reviennent d'Isaac Sim vers
   AXIS-OS sous forme de vrais ActuatorFeedback (le retour d'un moteur),
   et pas seulement les commandes dans l'autre sens. Les garde-fous
   d'AXIS-OS ont besoin de ce retour physique pour être testés.
2. Le couple exposé est celui RÉELLEMENT appliqué par le solveur
   physique, pas le couple commandé : indispensable au test qui vérifie
   que le couple ne dépasse jamais la limite inscrite dans l'URDF
   (campagne, §2). L'URDF (Unified Robot Description Format) est le
   fichier XML qui décrit le robot : articulations, masses, limites et
   géométries.
3. Réimport sûr du modèle USD (le format de scène 3D d'Isaac Sim) : le
   .usd existant est toujours supprimé avant l'import, pour ne pas
   reproduire le blocage sur l'erreur `is_homogeneous`.

Contenu : start_simulation_app() et import_g1_urdf() préparent le
simulateur ; IsaacSimBridge relie le HAL au robot simulé ;
MainThreadSimPump et run_mission_in_thread() permettent de piloter ce
pont depuis une app isolée, dont les commandes arrivent par un autre
thread.

À exécuter DANS le conteneur Docker d'Isaac Sim (voir
docs/campagne_validation_isaac.md, §1). Suppose que /isaac-sim/axis_os
existe (copié via `docker cp`) et que /isaac-sim/g1_23dof_rev_1_0.urdf
existe (copié depuis le dépôt).
"""
from __future__ import annotations

import os
import sys
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

# Rend importable le paquet axis_os copié dans le conteneur (voir plus haut).
sys.path.insert(0, "/isaac-sim")

from axis_os.hal.core.types import ActuatorFeedback, ActuatorState, HALStatus
from axis_os.hal.drivers.unitree_g1.robot import (
    UnitreeG1HAL, _build_g1_joint_definitions, build_g1_joint_definitions,
)


# Chemins dans le conteneur : le .usd est généré par import_g1_urdf() à
# partir de l'URDF.
USD_PATH = "/isaac-sim/g1_axisos.usd"
URDF_PATH = "/isaac-sim/g1_23dof_rev_1_0.urdf"
# Un URDF par variante du G1 (23 ou 29 degrés de liberté, « dof » en
# anglais), copié dans le conteneur à côté de ses meshes (les fichiers de
# géométrie 3D).
URDF_PATHS = {
    "23dof": URDF_PATH,
    # Copie DÉRIVÉE de l'URDF officiel. Pourquoi : dans l'original, les
    # mains en caoutchouc n'ont aucune géométrie de collision, et elles
    # TRAVERSAIENT le carton lors de l'essai de prise d'un carton en
    # simulation (docs/validation_port_carton.md, §2). Seule différence de
    # la copie : les mains y reçoivent une collision. Elle est produite par
    # unitree_g1/urdf/deriver_mains_collision.py. Le HAL, lui, lit
    # toujours l'URDF officiel pour les articulations et les limites.
    "29dof": "/isaac-sim/g1_29dof_rev_1_0_mains_collision.urdf",
}

# Application Isaac Sim démarrée, gardée ici pour que le ramasse-miettes
# (garbage collector) ne la détruise pas ; None tant qu'elle n'existe pas.
_simulation_app = None


def start_simulation_app(headless: bool = False):
    """Démarre l'application Isaac Sim. À appeler EN PREMIER, avant tout
    autre usage de ce module et avant tout import isaacsim.* dans le
    processus.

    Pourquoi : c'est une contrainte de Carbonite, le framework sur lequel
    repose Isaac Sim, et non une particularité d'AXIS-OS. Ses extensions
    ne sont chargées en mémoire qu'une fois SimulationApp créée. Sans cet
    appel préalable, `from isaacsim.core.api import World` échoue avec
    `ModuleNotFoundError: No module named 'isaacsim.core'`, alors que le
    module est bien installé.

    headless : True pour tourner sans fenêtre (serveur, conteneur).
    Un second appel renvoie l'application déjà démarrée. Au premier appel,
    le DLSS (lissage des images par un réseau de neurones de NVIDIA) est
    désactivé au passage, car il manque sur certaines machines (voir
    desactiver_dlss()).

    Retourne l'objet SimulationApp : garde-le en vie (qu'il ne soit pas
    détruit par le ramasse-miettes) et appelle sa méthode .close() en fin
    de script.
    """
    global _simulation_app
    if _simulation_app is not None:
        return _simulation_app
    from isaacsim import SimulationApp
    _simulation_app = SimulationApp({"headless": headless})
    desactiver_dlss()
    return _simulation_app


def desactiver_dlss() -> None:
    """Remplace l'anticrénelage DLSS par le TAA, disponible partout.

    L'anticrénelage lisse les contours en escalier des images rendues.
    Le DLSS le fait avec un réseau de neurones de NVIDIA (bibliothèque
    NGX) ; le TAA (anticrénelage temporel) n'a besoin d'aucune
    bibliothèque particulière.

    Pourquoi : constaté sur une machine louée chez Vast.ai (service de
    location de cartes graphiques ; carte RTX 5090 GeForce, en
    conteneur), le rendu échouait à chaque image avec « NGX CreateFeature
    failed: 0xbad00002 » et « RenderOp 'DLSS RenderOp' failed » : AUCUNE
    image rendue, caméras
    noires, test arrêté à t = 0. Le DLSS n'est pas disponible partout, et
    nos caméras n'en ont pas besoin. Ce réglage vaut pour toutes les
    cartes : les tests ne dépendent plus de la présence du DLSS.

    Un échec du réglage est affiché, mais n'interrompt rien."""
    try:
        import carb
        s = carb.settings.get_settings()
        s.set("/rtx/post/aa/op", 1)                 # 1 = TAA (3 = DLSS, défaut)
        s.set("/rtx/post/dlss/execMode", 0)
        s.set("/rtx-transient/dlssg/enabled", False)
        print("  [rendu] DLSS désactivé (anticrénelage TAA)", flush=True)
    except Exception as exc:                        # noqa: BLE001
        print(f"  [rendu] impossible de désactiver le DLSS : {exc}", flush=True)



def find_missing_meshes(urdf_path: str = URDF_PATH) -> List[str]:
    """Liste les meshes référencés par l'URDF mais absents du disque.

    Pourquoi cette vérification est NÉCESSAIRE (découvert tardivement) :
    les meshes du G1 (fichiers de géométrie 3D au format .STL) ne sont
    pas versionnés dans le dépôt (106 Mo, voir
    axis_os/hal/drivers/unitree_g1/urdf/meshes/README.md). Sans eux,
    Isaac Sim importe le robot SANS ERREUR, avec tout ce qui fait la
    physique (articulations, masses, limites), mais sans AUCUNE
    géométrie visuelle (dans l'URDF du G1, elle est à 100 %
    faite de meshes) et avec une collision PARTIELLE (20 éléments de
    collision sur 32 sont des meshes).

    Symptôme trompeur, réellement constaté : tous les tests articulaires
    passaient, et les caméras rendaient un sol et un ciel vides. On a
    soupçonné l'orientation de la caméra, puis un modèle de perception
    incapable de reconnaître un G1 non texturé ; il n'y avait simplement
    pas de robot dans l'image. Les avertissements de fichiers
    introuvables existaient, mais étaient masqués par « USD Diagnostics
    are currently muted ».

    Retourne les noms de fichiers tels qu'écrits dans l'URDF, triés. Rend
    une liste vide si l'URDF lui-même est illisible : c'est une autre
    erreur, que cette fonction ne traite pas.
    """
    import xml.etree.ElementTree as ET
    base = os.path.dirname(os.path.abspath(urdf_path))
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:           # noqa: BLE001 — URDF absent : autre erreur
        return []
    missing = set()
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        if not fn:
            continue
        # Un chemin de la forme « package://… » (convention de ROS, Robot
        # Operating System, un environnement logiciel libre pour la
        # robotique) perd son préfixe ; un chemin relatif se lit depuis le
        # dossier de l'URDF.
        rel = fn.split("://", 1)[-1] if "://" in fn else fn
        if rel.startswith("/"):
            path = rel
        else:
            path = os.path.join(base, rel)
        if not os.path.exists(path):
            missing.add(fn)
    return sorted(missing)

def import_g1_urdf(force_reimport: bool = True, fix_base: bool = False,
                   require_meshes: bool = False, urdf_path: str = URDF_PATH):
    """Importe le G1 dans Isaac Sim et retourne le chemin du .usd généré.

    L'import part du vrai URDF officiel utilisé par AXIS-OS, et non de la
    variante générique fournie par NVIDIA : celle-ci nomme 5 joints
    autrement, divergence de nommage déjà rencontrée.

    fix_base : True épingle la base du robot au monde (pas de dynamique
    du corps entier). Nécessaire pour tester la CONVENTION DE SIGNE d'un
    joint, c'est-à-dire le sens dans lequel il tourne
    (docs/campagne_validation_isaac.md, §3). Sans base fixe, le G1, qui
    n'a encore ni gains PD (asservissement proportionnel-dérivé, voir
    IsaacSimBridge.set_holding_gains()) ni couple de maintien, s'effondre
    sous la gravité entre l'import et la première commande. Constaté en
    exécution réelle : positions de départ très dispersées, incohérentes
    d'un joint à l'autre. Mettre False pour les tests de dynamique du
    corps entier (arrêt d'urgence, puis pile complète et endurance, §5
    et §6 de la campagne). Même principe que le protocole réel de
    première mise sous tension (§9 de la campagne) : robot suspendu, pas
    debout, pour isoler ce qu'on teste vraiment.

    require_meshes : True lève RuntimeError si des meshes manquent ;
    False se contente d'un avertissement (voir find_missing_meshes()).

    urdf_path : URDF à importer (par défaut celui de la variante 23dof).

    force_reimport=True supprime le .usd existant avant d'importer. C'est
    le correctif du blocage `AttributeError: 'NoneType' object has no
    attribute 'is_homogeneous'`, rencontré en réimportant par-dessus un
    .usd déjà présent sur disque.
    """
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    import shutil

    # Vérifier les meshes AVANT d'importer. Pourquoi : l'importateur ne
    # signale pas leur absence de façon visible, et produit un robot
    # physiquement correct mais INVISIBLE, avec une collision partielle.
    missing = find_missing_meshes(urdf_path)
    if missing:
        msg = (f"{len(missing)} mesh(es) introuvable(s) à côté de "
               f"{urdf_path} (ex. {missing[0]}). Le robot sera INVISIBLE "
               f"aux caméras et sa COLLISION sera PARTIELLE. Récupération : "
               f"voir axis_os/hal/drivers/unitree_g1/urdf/meshes/README.md")
        if require_meshes:
            raise RuntimeError(msg)
        print(f"[bridge] AVERTISSEMENT : {msg}")
        print("[bridge]   -> sans conséquence pour les suites articulaires "
              "en base fixe ; INVALIDE pour toute perception et toute "
              "locomotion.")

    if force_reimport and os.path.exists(USD_PATH):
        # L'importateur URDF peut laisser au chemin usd_path un dossier
        # (textures, données annexes dites « payload ») plutôt qu'un simple
        # fichier : constaté en exécution réelle (IsADirectoryError). On
        # gère les deux cas plutôt que de supposer l'un ou l'autre.
        if os.path.isdir(USD_PATH):
            shutil.rmtree(USD_PATH)
        else:
            os.remove(USD_PATH)

    config = URDFImporterConfig()
    config.urdf_path = urdf_path
    config.usd_path = USD_PATH
    config.fix_base = fix_base
    config.merge_fixed_joints = False
    config.self_collision = False

    importer = URDFImporter(config)
    return importer.import_urdf()


class IsaacSimBridge:
    """Relie le HAL du G1 (UnitreeG1HAL avec backend="sim") à un G1
    simulé physiquement dans Isaac Sim.

    En mode simulation, le HAL s'appuie sur G1SimulatedActuator, un
    simulateur interne simplifié. Le pont le remplace par des appels réels
    à robot.apply_action() d'Isaac Sim et par la lecture de l'état
    articulaire calculé par le moteur physique.

    Usage :
        start_simulation_app()         # obligatoirement en premier
        bridge = IsaacSimBridge()
        bridge.setup()                 # charge le G1, le plan de sol
        hal = bridge.build_hal()       # UnitreeG1HAL branché sur Isaac
        hal.initialize_all()
        hal.send_joint_commands({...}) # passe réellement par apply_action()
        bridge.step(60)                # avance la simulation physique

    Si les commandes viennent d'un autre thread (app lancée dans un
    sandbox, c'est-à-dire un sous-processus isolé ; voir attach_pump()),
    il faut en plus une MainThreadSimPump.
    """

    def __init__(self, variante: str = "23dof"):
        """variante : "23dof" (23 degrés de liberté ; valeur par défaut,
        comportement historique inchangé) ou "29dof", le robot de référence
        pour la loco-manipulation, c'est-à-dire marcher en manipulant un
        objet (docs/integration_gr00t_wbc.md). Toute autre valeur lève
        ValueError."""
        if variante not in URDF_PATHS:
            raise ValueError(f"variante inconnue : {variante!r}")
        self._variante = variante
        self._robot = None
        self._world = None
        self._joint_defs = build_g1_joint_definitions(variante)
        # Rempli par setup() : nom de joint AXIS-OS -> index du DOF (degré
        # de liberté, une articulation motorisée) dans Isaac Sim.
        self._joint_index: Dict[str, int] = {}
        # Pompe du thread principal, attachée seulement en mode
        # multi-thread (voir attach_pump()).
        self._pump: Optional["MainThreadSimPump"] = None
    def setup(self, fix_base: bool = False, require_meshes: bool = False,
              ground_clearance: float = 0.0,
              physics_dt: Optional[float] = None):
        """Construit la scène : importe le G1, le place, crée le monde
        physique, le sol et une lumière, puis associe chaque joint
        AXIS-OS à son index dans Isaac Sim.

        fix_base : épingle la base du robot au monde (voir
        import_g1_urdf()).
        require_meshes : refuse l'import si des meshes manquent (voir
        find_missing_meshes()).
        ground_clearance : hauteur visée, en mètres, du point le plus bas
        du robot au-dessus du sol ; > 0 suspend le robot, pieds dans le
        vide.
        physics_dt : pas de temps physique, en secondes ; None garde le
        réglage par défaut d'Isaac Sim (60 Hz).

        Lève RuntimeError si start_simulation_app() n'a pas été appelée
        avant, si l'articulation du robot reste introuvable, ou si un
        joint AXIS-OS n'a pas d'équivalent dans Isaac Sim.
        """
        if _simulation_app is None:
            raise RuntimeError(
                "IsaacSimBridge.setup() appelé sans avoir d'abord appelé "
                "start_simulation_app() — obligatoire, voir sa docstring."
            )
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.stage import add_reference_to_stage
        import omni.usd

        # L'ordre compte : le USD importé doit être référencé dans la
        # scène AVANT la création de World() (voir la sélection de
        # variante et le placement ci-dessous).
        usd_path = import_g1_urdf(urdf_path=URDF_PATHS[self._variante],
                                  fix_base=fix_base,
                                  require_meshes=require_meshes)
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/g1")

        # Sélection de la variante physique : l'étape qui manquait vraiment,
        # trouvée par un test direct et non par déduction. L'importateur
        # URDF d'Isaac Sim 6.1.0 crée, sur le prim référencé (un « prim »
        # est un nœud de la scène USD), un VariantSet nommé "Physics",
        # c'est-à-dire un jeu de variantes au choix. Tant qu'aucune variante
        # n'y est sélectionnée, AUCUN joint physique n'est composé dans la
        # scène (le « stage »). Ce n'était ni un problème de timing (des
        # appels répétés à simulation_app.update() n'y changeaient rien),
        # ni un problème lié à l'interface de programmation (API)
        # dépréciée isaacsim.core.api : c'est cette sélection qui manquait,
        # avant même la création de World().
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/World/g1")
        variant_set = prim.GetVariantSet("Physics")
        available = variant_set.GetVariantNames()
        if "physx" in available:
            variant_set.SetVariantSelection("physx")
        elif available:
            # Le nom de la variante pourrait changer selon la version de
            # l'importateur URDF : ne pas planter dans ce cas, mais le
            # signaler clairement plutôt que d'échouer en silence.
            print(f"[bridge] AVERTISSEMENT : variante 'physx' absente, "
                  f"disponibles : {available} — pas de sélection automatique, "
                  f"'{available[0]}' pourrait convenir selon le contexte.")
        else:
            print("[bridge] AVERTISSEMENT : pas de VariantSet 'Physics' sur "
                  "/World/g1 — comportement différent de la version "
                  "d'Isaac Sim testée, l'articulation risque d'être "
                  "introuvable au world.reset() suivant.")

        # Placement dans la scène USD, AVANT que la physique ne démarre.
        #
        # Pourquoi une hauteur d'apparition calculée, et non supposée :
        # constaté en diagnostic, le robot apparaissait avec sa boîte
        # englobante de z=-0.69 à +0.64, pieds 70 cm SOUS le sol. Sans
        # conséquence tant que les meshes de collision manquaient ; dès
        # qu'ils ont été présents, les pieds pris dans le sol ont fait
        # diverger la physique (« PhysX error: non-finite bounds », en
        # boucle ; PhysX est le moteur physique d'Isaac Sim). On mesure
        # donc la géométrie réelle à l'origine, et on relève le robot de ce
        # qu'il faut pour que son point le plus bas touche le sol, plus une
        # marge de 5 mm qui évite toute interpénétration initiale.
        #
        # Pourquoi déplacer le prim racine dans la scène : deux tentatives
        # précédentes ont échoué, et la vérification l'a montré à chaque
        # fois (boîte englobante inchangée, de -0.73 à +0.59) :
        #   1. passer position= au Robot après la création du World ;
        #   2. relever une ancre de joint attachée au monde : il n'en
        #      existe aucune, la base fixe n'est pas réalisée ainsi.
        # Plutôt que de deviner le mécanisme de fixation, on déplace le prim
        # racine dans la scène AVANT la création du World : quel que soit ce
        # mécanisme, la physique s'initialise depuis la pose « authorée »
        # (inscrite explicitement dans la scène, en vocabulaire USD) à ce
        # moment-là.
        #
        # ground_clearance > 0 : robot SUSPENDU, pieds dans le vide.
        # Indispensable aux tests de convention de signe des joints
        # (docs/campagne_validation_isaac.md, §3). Constaté : une fois le
        # robot posé avec de vraies collisions, le sol bloquait la rotation
        # des chevilles (-0.063 -> -0.059 pour +0.15 commandé, identique à
        # gauche et à droite). Même principe que le protocole réel de mise
        # sous tension : robot suspendu, pour isoler ce qu'on teste.
        self._ground_clearance = ground_clearance
        spawn_z = self._compute_spawn_height(margin=0.005 + ground_clearance)
        self._spawn_z = spawn_z
        self._author_root_translation(spawn_z)
        # En base fixe, relever aussi toute ancre attachée au monde (aucune
        # n'a été trouvée lors des essais, voir ci-dessus).
        if fix_base:
            self._shift_world_anchors(spawn_z)

        # physics_dt : pas de temps de la physique. Pourquoi le régler : par
        # défaut, Isaac Sim tourne à 60 Hz ; une politique de marche (le
        # contrôleur appris qui fait marcher le robot) entraînée à 500 Hz
        # (dt = 0.002 s) tomberait à coup sûr sur un pas huit fois plus
        # grossier, et l'on accuserait à tort la politique. Passer le dt
        # utilisé à l'ENTRAÎNEMENT de la politique. Le rendu des images,
        # lui, reste au plus à 60 Hz.
        world_kw = {"stage_units_in_meters": 1.0}
        if physics_dt is not None:
            world_kw["physics_dt"] = physics_dt
            world_kw["rendering_dt"] = max(physics_dt, 1.0 / 60.0)
        self._world = World(**world_kw)
        # Un seul plan de sol physique : add_default_ground_plane() suffit,
        # et évite le doublon de prim rencontré en ajoutant en plus un
        # GroundPlane() au même endroit. La grille affichée par défaut dans
        # Isaac Sim n'a AUCUNE collision : sans ce plan, le G1 tombe à
        # travers.
        self._world.scene.add_default_ground_plane()

        # Lumière. Pourquoi : sans source lumineuse, TOUTE caméra de la
        # scène rend un noir uniforme, indéfiniment, sans lever d'erreur :
        # le tableau d'image a la bonne taille, seuls ses octets trahissent
        # le problème. Sans conséquence tant qu'on ne faisait que de la
        # physique (les cinq premières suites de la campagne de
        # validation, §2 à §6 de docs/campagne_validation_isaac.md, n'ont
        # jamais rendu d'image), mais bloquant dès qu'une caméra existe.
        try:
            from isaacsim.core.api.objects.ground_plane import GroundPlane  # noqa: F401
            import omni.usd
            from pxr import UsdLux, Sdf
            stage = omni.usd.get_context().get_stage()
            if not stage.GetPrimAtPath("/World/DefaultLight"):
                light = UsdLux.DomeLight.Define(
                    stage, Sdf.Path("/World/DefaultLight"))
                light.CreateIntensityAttr(1000.0)
        except Exception as exc:        # noqa: BLE001
            print(f"[bridge] AVERTISSEMENT : lumière non créée ({exc}) — "
                  f"une caméra de cette scène rendra du noir.")
        # Le placement est déjà fait AVANT la création du World (voir plus
        # haut) : on ne repasse PAS de position au Robot, ce qui écraserait
        # la pose authorée dans la scène.
        self._robot = self._world.scene.add(Robot(
            prim_path="/World/g1", name="g1_axisos"))
        self._world.reset()
        # Remesure de la hauteur réelle : un robot mal placé est signalé,
        # sans interrompre setup().
        self._verify_grounded(expected=ground_clearance)
        self._robot.initialize()

        if not self._robot.num_dof:
            raise RuntimeError(
                "IsaacSimBridge.setup() : articulation toujours introuvable "
                "après sélection de variante — voir les avertissements "
                "ci-dessus sur le VariantSet."
            )

        # Table de correspondance nom de joint AXIS-OS -> index du DOF dans
        # Isaac Sim. Avec l'URDF officiel importé directement, les noms
        # coïncident exactement (confirmé le 18/09 : 23 joints sur 23). On
        # vérifie quand même, plutôt que de le supposer, pour détecter
        # toute régression.
        dof_names = list(self._robot.dof_names)
        missing = []
        for jdef in self._joint_defs:
            if jdef.joint_id not in dof_names:
                missing.append(jdef.joint_id)
            else:
                self._joint_index[jdef.joint_id] = dof_names.index(jdef.joint_id)
        if missing:
            raise RuntimeError(
                f"IsaacSimBridge: {len(missing)} joint(s) AXIS-OS absents "
                f"des DOF Isaac Sim : {missing}. Le mapping a divergé depuis "
                f"la validation du 18/09 — ne pas continuer sans corriger."
            )
        print(f"[bridge] {len(self._joint_index)}/{len(self._joint_defs)} joints "
              f"mappés avec Isaac Sim ({self._variante}).")

    def _author_root_translation(self, z: float) -> None:
        """Écrit la hauteur `z` du prim racine /World/g1 dans la scène USD,
        c'est-à-dire la pose authorée dont la physique partira.

        N'interrompt jamais setup() : un échec est affiché comme
        avertissement."""
        try:
            import omni.usd
            from pxr import Gf, UsdGeom
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath("/World/g1")
            api = UsdGeom.XformCommonAPI(prim)
            if api.SetTranslate(Gf.Vec3d(0.0, 0.0, z)):
                print(f"[bridge] translation authorée sur /World/g1 : z={z:+.3f}")
                return
            # Repli : les transformations déjà posées sur le prim forment
            # une pile d'opérations incompatible avec XformCommonAPI
            # (l'interface USD simplifiée de translation, rotation et
            # échelle). On modifie alors directement l'opération de
            # translation, ou on en ajoute une s'il n'y en a pas.
            xf = UsdGeom.Xformable(prim)
            op = next((o for o in xf.GetOrderedXformOps()
                       if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
            if op is None:
                op = xf.AddTranslateOp()
            op.Set(Gf.Vec3d(0.0, 0.0, z))
            print(f"[bridge] translation authorée (op directe) : z={z:+.3f}")
        except Exception as exc:        # noqa: BLE001
            print(f"[bridge] AVERTISSEMENT : translation non authorée ({exc})")

    def _shift_world_anchors(self, dz: float) -> None:
        """Relève de `dz` l'ancre de tout joint attachant le robot au monde.

        En USD Physics, un joint dont l'un des deux corps n'est pas
        renseigné est attaché au monde, et la position locale
        correspondante (localPos0 ou localPos1) est alors exprimée dans
        le repère du monde. C'est cette valeur qu'il faut relever quand on
        déplace le robot.

        Lors des essais sur le G1 importé, aucune ancre de ce type n'a été
        trouvée (voir setup()) ; la méthode le signale alors. Un échec est
        affiché, jamais levé.
        """
        try:
            import omni.usd
            from pxr import Gf, Usd, UsdPhysics
            stage = omni.usd.get_context().get_stage()
            root = stage.GetPrimAtPath("/World/g1")
            moved = 0
            for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies(
                    Usd.PrimAllPrimsPredicate)):
                if not p.IsA(UsdPhysics.Joint):
                    continue
                j = UsdPhysics.Joint(p)
                b0 = j.GetBody0Rel().GetTargets()
                b1 = j.GetBody1Rel().GetTargets()
                # « Monde » = corps absent OU corps situé hors du robot
                # (certains importateurs ciblent /World explicitement).
                def _hors_robot(t):
                    return (not t) or not str(t[0]).startswith("/World/g1")
                attr = None
                if _hors_robot(b0) and not _hors_robot(b1):
                    attr = j.GetLocalPos0Attr()
                elif _hors_robot(b1) and not _hors_robot(b0):
                    attr = j.GetLocalPos1Attr()
                if attr is None:
                    continue
                v = attr.Get() or Gf.Vec3f(0, 0, 0)
                attr.Set(Gf.Vec3f(v[0], v[1], v[2] + dz))
                moved += 1
                print(f"[bridge] ancre monde de {p.GetName()} relevée : "
                      f"z {v[2]:+.3f} -> {v[2] + dz:+.3f}")
            if moved == 0:
                print("[bridge] aucune ancre monde trouvée — si le robot "
                      "reste enterré, l'attache au monde est ailleurs.")
        except Exception as exc:        # noqa: BLE001
            print(f"[bridge] AVERTISSEMENT : ancres non déplacées ({exc})")

    def _verify_grounded(self, tolerance: float = 0.05, expected: float = 0.0) -> bool:
        """Remesure la géométrie APRÈS placement, et signale si le robot
        n'est pas à la hauteur attendue.

        Même principe que look_at() dans sim_camera.py : on ne croit pas ce
        qu'on a demandé, on mesure ce qui est. `expected` est la hauteur
        visée du point le plus bas (0 = pieds au sol, > 0 = robot
        suspendu), `tolerance` l'écart admis, en mètres. Retourne True si
        le placement est correct ; ne lève jamais d'exception."""
        try:
            import omni.usd
            from pxr import Usd, UsdGeom
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath("/World/g1")
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                 UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy])
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            min_z = float(rng.GetMin()[2])
            ok = abs(min_z - expected) <= tolerance
            etat = ("pieds au sol" if expected == 0.0
                    else f"suspendu à {expected:.2f} m, pieds dans le vide")
            print(f"[bridge] VÉRIFICATION : point le plus bas à "
                  f"z={min_z:+.3f} m -> "
                  f"{etat if ok else 'ROBOT MAL PLACÉ'}")
            if not ok and min_z < 0:
                print(f"[bridge]   le robot est ENCORE enterré de "
                      f"{-min_z:.2f} m : avec des meshes de collision, la "
                      f"physique risque de diverger (non-finite bounds).")
            return ok
        except Exception as exc:        # noqa: BLE001
            print(f"[bridge] vérification de hauteur impossible ({exc})")
            return False

    def _compute_spawn_height(self, margin: float = 0.005) -> float:
        """Hauteur à laquelle placer l'origine du robot pour que son point
        le plus bas repose sur le sol (z = 0), plus `margin` mètres.

        Mesure la boîte englobante de TOUTE la géométrie, collision
        comprise, puisque c'est elle qui touche le sol. En USD, chaque
        géométrie porte un usage (« purpose ») ; celle de collision est
        rangée sous l'usage guide, d'où les quatre usages passés au calcul.

        Rend 0.0 si la mesure échoue, avec un avertissement : mieux vaut
        un robot laissé à l'ancienne position, et signalé, qu'une hauteur
        inventée.
        """
        try:
            import omni.usd
            from pxr import Usd, UsdGeom
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath("/World/g1")
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                 UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy])
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                print("[bridge] AVERTISSEMENT : géométrie introuvable, "
                      "hauteur d'apparition laissée à 0 — le robot risque "
                      "d'être partiellement sous le sol.")
                return 0.0
            min_z = float(rng.GetMin()[2])
            spawn = max(0.0, -min_z) + margin
            print(f"[bridge] point le plus bas à z={min_z:+.3f} m -> robot "
                  f"placé à z={spawn:+.3f} m (pieds au sol)")
            return spawn
        except Exception as exc:        # noqa: BLE001
            print(f"[bridge] AVERTISSEMENT : hauteur d'apparition non "
                  f"calculée ({exc}), laissée à 0.")
            return 0.0

    def set_holding_gains(self, kp: float = 80.0, kd: float = 8.0) -> None:
        """Règle des gains PD de maintien sur tous les joints du robot.

        Un asservissement PD (proportionnel-dérivé) tire chaque joint vers
        sa position cible avec une raideur kp, et freine son mouvement avec
        un amortissement kd.

        NÉCESSAIRE : l'URDF officiel ne fournit aucune donnée de raideur
        ni d'amortissement (stiffness/damping). Confirmé en exécution
        réelle : l'avertissement d'Isaac Sim « actuator will be created
        without gain parameters » apparaît sur les 23 joints à chaque
        import. Sans gains, une commande en POSITION ne tient rien du
        tout : le joint devient un pendule libre sous la gravité dès la
        commande suivante. Cela a produit des résultats de test
        entièrement dominés par le bruit gravitationnel, plutôt que par
        la direction réellement commandée.

        kp=80.0 et kd=8.0 sont des valeurs GÉNÉRIQUES DE TEST, choisies
        pour tenir une position sans osciller violemment. Ce ne sont PAS
        les gains réels du G1 : AXIS-OS ne les connaît pas, ce sont ceux
        du firmware Unitree, propriétaires. Ne jamais réutiliser ces
        valeurs comme référence de contrôle d'un robot réel.
        """
        n = self._robot.num_dof
        controller = self._robot.get_articulation_controller()
        controller.set_gains(kps=[kp] * n, kds=[kd] * n)

    def set_pure_torque_mode(self) -> None:
        """Met à zéro le gain de position de tous les joints, en gardant un
        faible amortissement résiduel (kd = 1.0) pour éviter l'instabilité
        numérique.

        Nécessaire avant de tester le mode TORQUE (commande directe en
        couple) : on isole ainsi le couple commandé, sans qu'un gain de
        position résiduel ne s'y oppose ou ne s'y ajoute."""
        n = self._robot.num_dof
        controller = self._robot.get_articulation_controller()
        controller.set_gains(kps=[0.0] * n, kds=[1.0] * n)

    def hold_all_except(self, free_joint_id: str, kp_hold: float = 80.0,
                         kd_hold: float = 8.0, kd_free: float = 1.0) -> None:
        """Maintient tous les joints en position SAUF free_joint_id, laissé
        en couple pur (gain de position nul, amortissement kd_free).

        NÉCESSAIRE pour isoler la réponse d'UN SEUL joint à un couple
        commandé. set_pure_torque_mode() met tous les joints à zéro d'un
        coup : pendant qu'on teste le joint N dans la boucle, les joints 1
        à N-1, déjà « remis à zéro », retombent sous la gravité faute de
        tenue. Et comme le squelette est une chaîne cinématique, un parent
        déplacé change l'orientation effective de la gravité vue par ses
        enfants. Constaté en exécution réelle : right_hip_roll_joint
        lisait avant=+0.524 au lieu de 0.0 juste après son propre reset,
        parce que right_hip_pitch_joint (son parent dans la chaîne, testé
        juste avant) avait dérivé à 1.49 rad entre-temps.
        """
        n = self._robot.num_dof
        free_idx = self._joint_index[free_joint_id]
        kps = [kp_hold] * n
        kds = [kd_hold] * n
        kps[free_idx] = 0.0
        kds[free_idx] = kd_free
        controller = self._robot.get_articulation_controller()
        controller.set_gains(kps=kps, kds=kds)

    def reset_all_joints(self, position: float = 0.0) -> None:
        """Téléporte tous les joints à une même position, d'un coup, avec
        des vitesses nulles, et réaligne aussi toutes les cibles PD sur
        cette position. Même avertissement que reset_joint() : outil de
        simulation pur, jamais un équivalent réel."""
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction
        n = self._robot.num_dof
        positions = np.full(n, position)
        velocities = np.zeros(n)
        self._robot.set_joint_positions(positions)
        self._robot.set_joint_velocities(velocities)
        self._robot.apply_action(ArticulationAction(joint_positions=positions))

    def reset_joint(self, joint_id: str, position: float = 0.0) -> None:
        """Téléporte un joint à une position donnée, EN CONTOURNANT
        DÉLIBÉRÉMENT AXIS-OS : ni garde-fou de sécurité
        (JointSafetyGuard), ni suivi des mouvements (MotionStateTracker),
        ni passage par le HAL (send_joint_commands()), donc ni par
        HALBridgeServer, le pont qui relaie les commandes des apps
        isolées. Les vitesses de tous les joints sont remises à zéro.

        Pourquoi : ramener chaque joint à une pose de référence connue
        entre deux essais, et empêcher la dérive due à la gravité de
        s'accumuler d'un joint testé au suivant. Ce n'est PAS un
        équivalent de ce qui existe sur un robot réel : aucun robot ne
        peut « téléporter » un joint. Outil de simulation pur, à ne jamais
        confondre avec une commande de mouvement réelle.
        """
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction
        idx = self._joint_index[joint_id]
        positions = np.array(self._robot.get_joint_positions())
        velocities = np.zeros_like(positions)
        positions[idx] = position
        self._robot.set_joint_positions(positions)
        self._robot.set_joint_velocities(velocities)
        # Réaligner aussi la CIBLE du contrôleur PD, pas seulement l'état
        # physique. Pourquoi : sinon, une fois les gains de maintien
        # restaurés (hold_all_except()), le PD reprend sa poursuite vers
        # une ancienne cible résiduelle (par exemple la dernière cible du
        # test de direction en mode position, ou une cible jamais fixée
        # pendant une commande en couple pur). Il en résulte un mouvement
        # parasite pendant qu'on teste un AUTRE joint. Constaté en
        # exécution réelle : left_hip_yaw_joint lisait avant=-0.226 juste
        # après son propre reset, alors qu'aucun couple ne lui avait
        # jamais été appliqué.
        self._robot.apply_action(ArticulationAction(
            joint_positions=[position], joint_indices=[idx]))

    def step(self, n: int = 1):
        """Avance la simulation physique de n pas, sans rendu d'image.
        Thread principal uniquement (voir MainThreadSimPump)."""
        for _ in range(n):
            self._world.step(render=False)

    def apply_command(self, joint_id: str, position: Optional[float] = None,
                       velocity: Optional[float] = None,
                       effort: Optional[float] = None) -> None:
        """Envoie à Isaac Sim la commande d'un joint : position, vitesse
        et/ou couple (effort). Un champ laissé à None n'est pas commandé.
        Thread principal uniquement (voir MainThreadSimPump).

        BUG RÉEL corrigé : cette méthode n'acceptait auparavant que la
        position et la vitesse. Un ActuatorCommand en ControlMode.TORQUE
        produisait donc une ArticulationAction totalement vide (les trois
        champs à None), et RIEN n'était appliqué dans Isaac Sim pour les
        commandes de couple. Constaté en exécution réelle, lors du test de
        direction en mode couple (docs/campagne_validation_isaac.md, §3) :
        les petits mouvements observés étaient du bruit physique résiduel,
        pas l'effet du couple commandé. Ce test, tel qu'exécuté alors,
        n'avait jamais testé le couple.
        """
        from isaacsim.core.utils.types import ArticulationAction
        idx = self._joint_index[joint_id]
        action = ArticulationAction(
            joint_positions=[position] if position is not None else None,
            joint_velocities=[velocity] if velocity is not None else None,
            joint_efforts=[effort] if effort is not None else None,
            joint_indices=[idx],
        )
        self._robot.apply_action(action)

    def read_position(self, joint_id: str) -> float:
        """Lecture rapide de la seule position d'un joint. Pratique pour
        construire des commandes en écart RELATIF plutôt qu'en absolu,
        comme dans les tests de convention de signe
        (docs/campagne_validation_isaac.md, §3)."""
        idx = self._joint_index[joint_id]
        return float(self._robot.get_joint_positions()[idx])

    def read_all_raw(self) -> Dict[str, tuple]:
        """Lit tous les joints d'un coup : {joint_id: (position, vitesse,
        couple appliqué)}. Le couple vaut NaN (« not a number », valeur
        indéfinie) si cette version d'Isaac Sim n'expose pas
        get_applied_joint_efforts().

        THREAD PRINCIPAL UNIQUEMENT : c'est cette méthode que
        MainThreadSimPump appelle pour publier son instantané. Une seule
        lecture groupée par pas de simulation, plutôt qu'une lecture par
        joint (23 pour la variante 23dof), potentiellement incohérentes
        entre elles."""
        positions = self._robot.get_joint_positions()
        velocities = self._robot.get_joint_velocities()
        try:
            efforts = self._robot.get_applied_joint_efforts()
        except AttributeError:
            efforts = None
        out = {}
        for joint_id, idx in self._joint_index.items():
            out[joint_id] = (
                float(positions[idx]),
                float(velocities[idx]),
                float(efforts[idx]) if efforts is not None else float("nan"),
            )
        return out

    def read_feedback(self, joint_id: str) -> ActuatorFeedback:
        """État réel d'un joint, lu depuis le solveur physique d'Isaac
        Sim : POSITION, VITESSE et COUPLE RÉELLEMENT APPLIQUÉ (pas le
        couple commandé).

        Si une pompe est attachée et tourne (mode sandbox, multi-thread),
        la méthode lit son INSTANTANÉ plutôt que d'appeler Isaac Sim.
        Pourquoi : elle est invoquée par JointSafetyGuard (le garde-fou
        qui vérifie chaque commande et chaque retour des joints) depuis le
        thread du pont, et get_joint_positions() y serait aussi peu sûr
        qu'un step() : ces appels ne sont pas « thread-safe », c'est-à-dire
        pas prévus pour être faits depuis un autre thread que le thread
        principal (voir MainThreadSimPump). Tant que l'instantané ne
        contient pas encore ce joint, la lecture se fait directement dans
        Isaac Sim.

        Courant, température et tension ne sont pas simulés : valeurs
        fixes."""
        if self._pump is not None and self._pump.is_running():
            snap = self._pump.snapshot()
            if joint_id in snap:
                pos, vel, eff = snap[joint_id]
                return ActuatorFeedback(
                    joint_id=joint_id, state=ActuatorState.ACTIVE,
                    position=pos, velocity=vel, torque=eff,
                    current=0.0, temperature=25.0, voltage=48.0,
                )
        idx = self._joint_index[joint_id]
        pos = float(self._robot.get_joint_positions()[idx])
        vel = float(self._robot.get_joint_velocities()[idx])
        # applied_joint_efforts = couple réellement exercé par le solveur
        # physique. C'est la seule source fiable pour vérifier que le couple
        # ne dépasse jamais la limite de l'URDF (campagne de validation, §2).
        try:
            eff = float(self._robot.get_applied_joint_efforts()[idx])
        except AttributeError:
            eff = float("nan")  # à vérifier selon la version d'Isaac Sim
        return ActuatorFeedback(
            joint_id=joint_id,
            state=ActuatorState.ACTIVE,
            position=pos,
            velocity=vel,
            torque=eff,
            current=0.0,
            temperature=25.0,
            voltage=48.0,
        )

    def attach_pump(self, pump: "MainThreadSimPump") -> None:
        """Active le mode multi-thread. À partir de là, send_joint_commands()
        dépose les commandes dans la pompe au lieu d'appeler Isaac Sim
        directement, et read_feedback() lit l'instantané publié par la
        pompe.

        Obligatoire dès que l'app tourne via PlayStoreRuntime, le moteur
        d'exécution des apps d'AXIS-OS : l'app y tourne dans un sandbox
        (sous-processus isolé), et ses commandes sont relayées vers le HAL
        par HALBridgeServer, depuis un thread secondaire."""
        self._pump = pump

    def build_hal(self) -> UnitreeG1HAL:
        """Retourne un UnitreeG1HAL dont send_joint_commands() écrit
        réellement vers Isaac Sim via ce pont, au lieu du simulateur
        interne G1SimulatedActuator.

        Technique : la méthode est remplacée sur l'instance pendant
        l'exécution (un « monkeypatch »), délibérément et de façon
        documentée. C'est le point d'intégration unique entre les deux
        piles logicielles. Les commandes passent d'abord par la méthode
        d'origine, qui applique les contrôles d'AXIS-OS ; seules celles
        qu'elle accepte (HALStatus.OK) sont transmises à Isaac Sim."""
        hal = UnitreeG1HAL(backend="sim", variante=self._variante)
        bridge = self

        original_send = hal.send_joint_commands

        def send_via_isaac(commands):
            statuses = original_send(commands)  # garde-fou et suivi AXIS-OS
            accepted = {jid: cmd for jid, cmd in commands.items()
                        if statuses.get(jid) == HALStatus.OK}
            if not accepted:
                return statuses

            if bridge._pump is not None:
                # Chemin multi-thread : on DÉPOSE les commandes et on attend
                # le thread principal. Ne jamais appeler apply_command() ni
                # step() ici : dès que l'app tourne en sandbox,
                # send_via_isaac s'exécute dans le thread de HALBridgeServer.
                # Si la pompe ne confirme pas l'application (arrêt, erreur,
                # délai dépassé), ces joints passent en ERROR_COMM.
                if not bridge._pump.submit(accepted):
                    for jid in accepted:
                        statuses[jid] = HALStatus.ERROR_COMM
                return statuses

            # Chemin mono-thread (scripts de test directs, sans sandbox) :
            # on est déjà sur le thread principal. Un seul pas de simulation
            # pour l'ensemble des commandes.
            for joint_id, cmd in accepted.items():
                bridge.apply_command(joint_id, position=cmd.position,
                                      velocity=cmd.velocity, effort=cmd.torque)
            bridge.step(1)
            return statuses

        hal.send_joint_commands = send_via_isaac  # type: ignore[method-assign]
        return hal


# ---------------------------------------------------------------------------
# Pompe du thread principal : obligatoire dès que l'app tourne dans le
# sandbox, car ses commandes arrivent alors par un autre thread.
# ---------------------------------------------------------------------------

@dataclass
class _SimRequest:
    """Une demande d'actuation à exécuter sur le THREAD PRINCIPAL : les
    commandes, l'événement signalé une fois la demande traitée, l'erreur
    éventuelle, et l'indicateur « commandes appliquées »."""
    commands: Dict[str, Any]
    done: "threading.Event"
    error: Optional[BaseException] = None
    applied: bool = False


class MainThreadSimPump:
    """Fait exécuter tous les appels Isaac Sim sur le thread principal.

    POURQUOI : World.step() et apply_action() ne sont PAS thread-safe
    (rien ne garantit leur bon fonctionnement s'ils sont appelés depuis
    plusieurs threads), et ils bloquent dans du code C SANS RELÂCHER LE
    GIL, le verrou global de l'interpréteur Python, qui ne laisse
    s'exécuter qu'un thread à la fois. Appelés depuis un thread
    secondaire (par exemple HALBridgeServer._loop()), ils gèlent le
    processus ENTIER : aucun autre thread Python ne peut plus s'exécuter,
    y compris le thread principal qui attendrait avec join(timeout=...).
    Il n'existe donc pas de version « prudente, avec délai maximal » de
    cet appel : la seule issue est de ne jamais le faire hors du thread
    principal.

    ARCHITECTURE RÉSULTANTE :
      - thread principal   : run_forever() ; applique les commandes,
                             appelle world.step(), publie un instantané
      - thread du pont     : submit() ; dépose et attend, ne touche
                             JAMAIS une API Isaac
      - thread « mission » : lance PlayStoreRuntime (voir
                             run_mission_in_thread())
      - sous-processus     : l'app sandboxée, inchangée

    Ce découpage rapproche d'ailleurs la simulation du matériel réel :
    sur un vrai G1, la publication DDS (le bus de messages par lequel
    passent les commandes moteur) se fait aussi depuis une boucle dédiée,
    que les threads applicatifs alimentent. Le pont devient donc PLUS
    fidèle au driver réel, pas moins.

    Les durées relevées (tick_apply_ms, tick_step_ms, submit_wait_ms)
    servent aux mesures de latence du pont, résumées par _stats() (voir
    sim/validation/test_s5_latency_probe.py et
    docs/latence_chemin_commande.md).
    """

    def __init__(self, bridge: "IsaacSimBridge"):
        self._bridge = bridge
        self._queue: "queue.Queue[_SimRequest]" = queue.Queue()
        self._state_lock = threading.Lock()
        self._snapshot: Dict[str, tuple] = {}
        self._running = False
        self._main_thread_id: Optional[int] = None
        # Décomposition du coût d'un tick (un tour de boucle), en
        # millisecondes. Pourquoi : distinguer le TRANSPORT (files
        # d'attente, threads : le vrai coût de l'isolation des apps) du PAS
        # DE SIMULATION (bridge.step(), qui n'existe pas sur un robot réel :
        # la physique y est le monde, elle ne coûte rien au logiciel). Sans
        # cette séparation, on attribue à l'architecture un coût qui est en
        # réalité celui du simulateur. idle_ticks compte les tours de
        # boucle passés sans aucune commande reçue.
        self.tick_apply_ms: list = []
        self.tick_step_ms: list = []
        self.submit_wait_ms: list = []
        self.idle_ticks: int = 0

    # ---- appelé depuis N'IMPORTE QUEL thread -----------------------------

    def submit(self, commands: Dict[str, Any], timeout_s: float = 5.0) -> bool:
        """Dépose des commandes et ATTEND leur application par le thread
        principal.

        Retourne True si elles ont été appliquées sans erreur ; False si la
        pompe est arrêtée, si l'application a échoué ou si le délai
        timeout_s expire. Jamais d'attente infinie : sinon, l'arrêt de la
        boucle principale laisserait le thread du pont bloqué pour
        toujours."""
        if not self._running:
            return False
        req = _SimRequest(commands=commands, done=threading.Event())
        _t0 = time.perf_counter()
        self._queue.put(req)
        ok = req.done.wait(timeout=timeout_s)
        # Attente vue DU THREAD APPELANT, du dépôt jusqu'à l'application
        # par le thread principal. C'est le coût que subit réellement
        # l'app, à distinguer du travail effectué (tick_apply_ms +
        # tick_step_ms). Un écart important entre les deux signifie du
        # temps de RÉVEIL DE THREAD, pas du travail.
        self.submit_wait_ms.append((time.perf_counter() - _t0) * 1000)
        if not ok:
            return False
        return req.error is None and req.applied

    def snapshot(self) -> Dict[str, tuple]:
        """Copie du dernier état publié par le thread principal :
        {joint_id: (position, vitesse, couple)}.

        C'est la SEULE source de retour d'état autorisée pour un thread
        secondaire : lire robot.get_joint_positions() depuis le thread du
        pont referait le même appel non thread-safe, en lecture."""
        with self._state_lock:
            return dict(self._snapshot)

    def is_running(self) -> bool:
        """True tant que run_forever() tourne."""
        return self._running

    # ---- THREAD PRINCIPAL UNIQUEMENT -------------------------------------

    def run_forever(self, should_stop, max_seconds: float = 600.0,
                    idle_sleep_s: float = 0.0) -> None:
        """Boucle de simulation. À APPELER DEPUIS LE THREAD PRINCIPAL,
        celui où SimulationApp a été instanciée. Tourne jusqu'à ce que
        should_stop() renvoie True ou que max_seconds soit atteint.

        idle_sleep_s : attente maximale d'une commande à chaque tour
        (0 signifie 2 ms). Un pas de simulation est fait à chaque tour,
        même sans commande."""
        self._main_thread_id = threading.get_ident()
        self._running = True
        t0 = time.time()
        try:
            while not should_stop() and (time.time() - t0) < max_seconds:
                # Attendre l'arrivée d'une commande plutôt que de boucler à
                # vide. Pourquoi : une boucle sans pause monopolise le GIL.
                # Le thread de HALBridgeServer et le sous-processus n'ont
                # alors la main qu'au bon vouloir de l'ordonnanceur Python,
                # ce qui ajoute des dizaines de ms d'attente PURE (ni
                # transport, ni physique) à chaque aller-retour.
                # Queue.get(timeout=) relâche le GIL pendant l'attente, et
                # laisse les autres threads progresser immédiatement.
                try:
                    first = self._queue.get(timeout=idle_sleep_s or 0.002)
                except queue.Empty:
                    first = None
                    self.idle_ticks += 1
                self._tick(first=first)
        finally:
            self._running = False
            # Libérer tout thread encore en attente : sinon, après l'arrêt
            # de la boucle, il reste bloqué jusqu'à l'expiration de son
            # délai.
            self._drain_pending()

    def _tick(self, first: "Optional[_SimRequest]" = None) -> None:
        """Un tour de boucle : applique toutes les demandes en attente,
        fait UN pas de simulation, publie l'instantané, puis libère les
        threads demandeurs. Les durées ne sont relevées que pour les tours
        qui ont traité au moins une demande."""
        _t_apply = time.perf_counter()
        pending = []
        if first is not None:
            self._apply_one(first)
            pending.append(first)
        while True:
            try:
                req = self._queue.get_nowait()
            except queue.Empty:
                break
            self._apply_one(req)
            pending.append(req)
        if pending:
            self.tick_apply_ms.append((time.perf_counter() - _t_apply) * 1000)

        # UN SEUL pas de simulation pour tout le lot. Pourquoi : appeler
        # step() pour chaque commande ferait avancer la physique 23 fois
        # pour un mouvement du corps entier (variante 23dof), avec des
        # états intermédiaires incohérents. Même raisonnement que la
        # publication DDS groupée du driver réel.
        _t_step = time.perf_counter()
        self._bridge.step(1)
        self._publish_snapshot()
        if pending:
            self.tick_step_ms.append((time.perf_counter() - _t_step) * 1000)

        for req in pending:
            req.done.set()

    def _apply_one(self, req: "_SimRequest") -> None:
        """Applique les commandes d'une demande. Une exception n'arrête
        pas la boucle : elle est rangée dans la demande, pour être
        remontée au thread qui l'a déposée."""
        try:
            for joint_id, cmd in req.commands.items():
                self._bridge.apply_command(
                    joint_id,
                    position=getattr(cmd, "position", None),
                    velocity=getattr(cmd, "velocity", None),
                    effort=getattr(cmd, "torque", None),
                )
            req.applied = True
        except BaseException as exc:      # noqa: BLE001 — remonté au demandeur
            req.error = exc

    def _publish_snapshot(self) -> None:
        """Lit l'état de tous les joints et le publie, sous verrou, pour
        les threads secondaires (voir snapshot())."""
        snap = self._bridge.read_all_raw()
        with self._state_lock:
            self._snapshot = snap

    def _drain_pending(self) -> None:
        """Libère, en les marquant en erreur, les demandes restées en file
        à l'arrêt de la boucle."""
        while True:
            try:
                req = self._queue.get_nowait()
            except queue.Empty:
                break
            req.error = RuntimeError("pompe arrêtée avant application")
            req.done.set()


def run_mission_in_thread(fn, *args, **kwargs):
    """Lance fn (typiquement PlayStoreRuntime.start_app ou run_mission)
    dans un thread annexe, et retourne (thread, holder).

    NÉCESSAIRE : le thread principal doit rester libre pour faire tourner
    MainThreadSimPump.run_forever(). Si la mission était lancée
    directement depuis le thread principal, celui-ci resterait bloqué
    dedans, la file de la pompe ne serait jamais vidée, et le thread du
    pont attendrait une application qui n'arriverait jamais : un autre
    interblocage (« deadlock »), pas une correction.

    holder est un dict rempli à la fin : {"done": bool, "result": ...,
    "error": ...}. Utiliser holder["done"] comme condition should_stop()
    de la pompe. Le thread est de type « daemon » : il n'empêche pas le
    programme de se terminer.
    """
    holder: Dict[str, Any] = {"done": False, "result": None, "error": None}

    def _runner():
        try:
            holder["result"] = fn(*args, **kwargs)
        except BaseException as exc:          # noqa: BLE001
            holder["error"] = exc
        finally:
            holder["done"] = True

    t = threading.Thread(target=_runner, name="axis-mission", daemon=True)
    t.start()
    return t, holder


def _stats(xs):
    """Résume une liste de durées : moyenne, médiane (p50), 95e centile
    (p95), maximum et nombre de mesures (n) ; None si la liste est vide.
    Sert aux mesures de latence de la pompe
    (sim/validation/test_s5_latency_probe.py)."""
    if not xs:
        return None
    s = sorted(xs)
    return {"mean": sum(xs) / len(xs), "p50": s[len(s) // 2],
            "p95": s[int(len(s) * 0.95)], "max": s[-1], "n": len(s)}
