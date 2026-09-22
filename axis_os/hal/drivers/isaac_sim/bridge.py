"""
Pont réutilisable entre UnitreeG1HAL et Isaac Sim.

Remplace l'intercepteur de fonction écrit pour une session (voir
docs/isaac_lab_unitree_sim_status.md). Trois exigences tenues :

1. Boucle fermée : les états articulaires reviennent d'Isaac Sim vers
   AXIS-OS comme de vrais ActuatorFeedback, pas seulement l'envoi.
2. Couple RÉELLEMENT appliqué par le solveur physique exposé, pas le
   couple commandé — indispensable pour la suite S1.2.
3. Réimport USD sûr : supprime systématiquement le .usd existant avant
   d'importer, pour ne pas reproduire le blocage `is_homogeneous`.

À exécuter DANS le conteneur Isaac Sim (voir campagne_validation_isaac_axis_os.md
§1). Suppose que /isaac-sim/axis_os existe (copié via `docker cp`) et que
/isaac-sim/g1_23dof_rev_1_0.urdf existe (copié depuis le repo).
"""
from __future__ import annotations

import os
import sys
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.core.types import ActuatorFeedback, ActuatorState, HALStatus
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL, _build_g1_joint_definitions


USD_PATH = "/isaac-sim/g1_axisos.usd"
URDF_PATH = "/isaac-sim/g1_23dof_rev_1_0.urdf"

_simulation_app = None  # référence gardée pour éviter le garbage-collect


def start_simulation_app(headless: bool = False):
    """À appeler EN PREMIER, avant tout autre usage de ce module et avant
    tout import isaacsim.* dans le process. Contrainte du framework
    Carbonite d'Isaac Sim, pas une particularité d'axis_os : sans ça,
    `from isaacsim.core.api import World` échoue avec
    `ModuleNotFoundError: No module named 'isaacsim.core'` même si le
    module est bien installé — l'extension n'est simplement pas encore
    chargée en mémoire.

    Retourne l'objet SimulationApp : garde-le en vie (ne le laisse pas
    être ramassé par le GC) et appelle .close() en fin de script.
    """
    global _simulation_app
    if _simulation_app is not None:
        return _simulation_app
    from isaacsim import SimulationApp
    _simulation_app = SimulationApp({"headless": headless})
    return _simulation_app



def find_missing_meshes(urdf_path: str = URDF_PATH) -> List[str]:
    """Meshes référencés par l'URDF mais absents du disque.

    NÉCESSAIRE, et découvert tardivement : les .STL du G1 ne sont pas
    versionnés (106 Mo, voir urdf/meshes/README.md). Sans eux, Isaac Sim
    importe le robot SANS ERREUR — articulations, masses, limites, tout
    ce qui fait la physique — mais sans AUCUNE géométrie visuelle
    (100 % meshes dans l'URDF du G1) et avec une collision PARTIELLE
    (20 éléments sur 32 sont des meshes).

    Symptôme trompeur, constaté réellement : toutes les suites
    articulaires passaient, et les caméras rendaient un sol et un ciel
    vides. On a cru à un problème d'orientation de caméra, puis à un
    modèle de perception incapable de reconnaître un G1 non texturé —
    il n'y avait simplement pas de robot dans l'image. Les avertissements
    de fichiers introuvables existaient, masqués par « USD Diagnostics
    are currently muted ».
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
        rel = fn.split("://", 1)[-1] if "://" in fn else fn
        if rel.startswith("/"):
            path = rel
        else:
            path = os.path.join(base, rel)
        if not os.path.exists(path):
            missing.add(fn)
    return sorted(missing)

def import_g1_urdf(force_reimport: bool = True, fix_base: bool = False,
                   require_meshes: bool = False):
    """Importe le G1 depuis le vrai URDF officiel AXIS-OS (pas la variante
    générique NVIDIA — évite la divergence de nommage à 5 joints déjà
    rencontrée). Retourne le usd_path généré.

    fix_base : True épingle la base au monde (pas de dynamique de corps
    entier). Nécessaire pour tester la CONVENTION DE SIGNE d'un joint
    (suite S2) sans que le G1, sans gains PD ni couple de maintien,
    s'effondre sous la gravité entre l'import et la première commande —
    constaté en exécution réelle (positions de départ très dispersées,
    incohérentes d'un joint à l'autre). Mettre False pour les suites qui
    testent la dynamique corps entier (S4, S5). Le principe est le même
    que le protocole de mise sous tension réel (§9 de la campagne) :
    robot suspendu, pas debout, pour isoler ce qu'on teste vraiment.

    force_reimport=True supprime le .usd existant avant d'importer : c'est
    le correctif au blocage `AttributeError: 'NoneType' object has no
    attribute 'is_homogeneous'` rencontré en réimportant sur un .usd déjà
    présent sur disque.
    """
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    import shutil

    # Vérifier les meshes AVANT d'importer : l'importateur ne signale pas
    # leur absence de façon visible, et produit un robot physiquement
    # correct mais INVISIBLE, avec une collision partielle.
    missing = find_missing_meshes(URDF_PATH)
    if missing:
        msg = (f"{len(missing)} mesh(es) introuvable(s) à côté de "
               f"{URDF_PATH} (ex. {missing[0]}). Le robot sera INVISIBLE "
               f"aux caméras et sa COLLISION sera PARTIELLE. Récupération : "
               f"voir axis_os/hal/drivers/unitree_g1/urdf/meshes/README.md")
        if require_meshes:
            raise RuntimeError(msg)
        print(f"[bridge] AVERTISSEMENT : {msg}")
        print("[bridge]   -> sans conséquence pour les suites articulaires "
              "en base fixe ; INVALIDE pour toute perception et toute "
              "locomotion.")

    if force_reimport and os.path.exists(USD_PATH):
        # L'importateur URDF peut laisser une arborescence (textures,
        # payload) plutôt qu'un simple fichier plat au chemin usd_path —
        # constaté en exécution réelle (IsADirectoryError). On gère les
        # deux cas plutôt que de supposer l'un ou l'autre.
        if os.path.isdir(USD_PATH):
            shutil.rmtree(USD_PATH)
        else:
            os.remove(USD_PATH)

    config = URDFImporterConfig()
    config.urdf_path = URDF_PATH
    config.usd_path = USD_PATH
    config.fix_base = fix_base
    config.merge_fixed_joints = False
    config.self_collision = False

    importer = URDFImporter(config)
    return importer.import_urdf()


class IsaacSimBridge:
    """Relie UnitreeG1HAL(backend="sim") à un G1 physiquement simulé
    dans Isaac Sim, en remplaçant le G1SimulatedActuator interne par
    des appels réels à robot.apply_action() / la lecture de l'état
    articulaire réel.

    Usage :
        bridge = IsaacSimBridge()
        bridge.setup()                 # charge le G1, le plan de sol
        hal = bridge.build_hal()       # UnitreeG1HAL branché sur Isaac
        hal.initialize_all()
        hal.send_joint_commands({...}) # passe réellement par apply_action()
        bridge.step(60)                # avance la simulation physique
    """

    def __init__(self):
        self._robot = None
        self._world = None
        self._joint_defs = _build_g1_joint_definitions()
        self._joint_index: Dict[str, int] = {}  # joint_id AXIS-OS -> index DOF Isaac
        self._pump: Optional["MainThreadSimPump"] = None
    def setup(self, fix_base: bool = False, require_meshes: bool = False,
              ground_clearance: float = 0.0,
              physics_dt: Optional[float] = None):
        if _simulation_app is None:
            raise RuntimeError(
                "IsaacSimBridge.setup() appelé sans avoir d'abord appelé "
                "start_simulation_app() — obligatoire, voir sa docstring."
            )
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.stage import add_reference_to_stage
        import omni.usd

        # Référencer le USD importé AVANT de créer World() — l'ordre compte,
        # voir plus bas.
        usd_path = import_g1_urdf(fix_base=fix_base,
                                  require_meshes=require_meshes)
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/g1")

        # ÉTAPE RÉELLEMENT MANQUANTE (trouvée par test direct, pas par
        # déduction) : l'URDF importer d'Isaac Sim 6.1.0 génère un
        # VariantSet nommé "Physics" sur le prim référencé. Tant qu'aucune
        # variante n'y est sélectionnée, AUCUN joint physique n'est
        # composé dans le stage — ce n'était ni un problème de timing
        # (mes tentatives précédentes avec simulation_app.update() en
        # boucle ne servaient à rien), ni un problème lié à l'API
        # dépréciée isaacsim.core.api : c'est cette sélection de variante
        # qui manquait, avant même la création de World().
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/World/g1")
        variant_set = prim.GetVariantSet("Physics")
        available = variant_set.GetVariantNames()
        if "physx" in available:
            variant_set.SetVariantSelection("physx")
        elif available:
            # Le nom de variante pourrait changer selon la version de
            # l'importateur URDF — ne pas planter dans ce cas, mais le
            # signaler clairement plutôt que d'échouer en silence.
            print(f"[bridge] AVERTISSEMENT : variante 'physx' absente, "
                  f"disponibles : {available} — pas de sélection automatique, "
                  f"'{available[0]}' pourrait convenir selon le contexte.")
        else:
            print("[bridge] AVERTISSEMENT : pas de VariantSet 'Physics' sur "
                  "/World/g1 — comportement différent de la version "
                  "d'Isaac Sim testée, l'articulation risque d'être "
                  "introuvable au world.reset() suivant.")

        # Un seul plan de sol physique — add_default_ground_plane() suffit
        # et évite le doublon de prim rencontré avec un GroundPlane() en
        # plus au même endroit. La grille par défaut d'Isaac Sim n'a
        # AUCUNE collision : sans ce plan, le G1 tombe à travers.
        # PLACEMENT DANS LA SCÈNE USD, AVANT QUE LA PHYSIQUE NE DÉMARRE.
        #
        # Deux tentatives précédentes ont échoué, et la vérification l'a
        # dit à chaque fois (boîte englobante inchangée de -0.73 à +0.59) :
        #   1. passer position= au Robot après création du World ;
        #   2. relever une ancre de joint attachée au monde — il n'en
        #      existe aucune : la base fixe n'est pas réalisée ainsi.
        # Plutôt que de deviner le mécanisme de fixation, on déplace le
        # prim racine dans la scène AVANT la création du World : quel que
        # soit ce mécanisme, la physique s'initialise depuis la pose
        # authorée à ce moment-là.
        # ground_clearance > 0 : robot SUSPENDU, pieds dans le vide.
        # Indispensable aux suites articulaires (S2) : constaté, une fois
        # le robot posé avec de vraies collisions, le plancher bloquait la
        # rotation des chevilles (-0.063 -> -0.059 pour +0.15 commandé,
        # identique à gauche et à droite). Même principe que le protocole
        # de mise sous tension réel : robot suspendu pour isoler ce qu'on
        # teste.
        self._ground_clearance = ground_clearance
        spawn_z = self._compute_spawn_height(margin=0.005 + ground_clearance)
        self._spawn_z = spawn_z
        self._author_root_translation(spawn_z)
        if fix_base:
            self._shift_world_anchors(spawn_z)

        # physics_dt : pas physique. Par défaut Isaac Sim tourne à 60 Hz ;
        # une politique de marche entraînée à 500 Hz (dt=0.002) tombe à
        # coup sûr sur un pas huit fois plus grossier — et on accuserait
        # la politique. Passer le dt d'ENTRAÎNEMENT de la politique.
        world_kw = {"stage_units_in_meters": 1.0}
        if physics_dt is not None:
            world_kw["physics_dt"] = physics_dt
            world_kw["rendering_dt"] = max(physics_dt, 1.0 / 60.0)
        self._world = World(**world_kw)
        self._world.scene.add_default_ground_plane()

        # Lumière : sans source lumineuse, TOUTE caméra de la scène rend
        # du noir uniforme, indéfiniment. Aucune erreur n'est levée — le
        # tableau a la bonne taille, seuls les octets trahissent le
        # problème. Sans conséquence tant qu'on ne faisait que de la
        # physique (les suites S1-S5 n'ont jamais rendu d'image), mais
        # bloquant dès qu'une caméra existe.
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
        # HAUTEUR D'APPARITION calculée, pas supposée. Constaté en
        # diagnostic : le robot apparaissait avec sa boîte englobante de
        # z=-0.69 à +0.64 — pieds 70 cm SOUS le sol. Sans conséquence tant
        # que les meshes de collision manquaient ; dès qu'ils ont été
        # présents, les pieds pris dans le sol ont fait diverger la
        # physique (« PhysX error: non-finite bounds », en boucle).
        #
        # On mesure la géométrie réelle à l'origine et on relève le robot
        # de ce qu'il faut pour que son point le plus bas touche le sol,
        # plus une marge qui évite toute interpénétration initiale.
        # Placement déjà fait AVANT la création du World (voir plus
        # haut) : on ne repasse PAS de position au Robot, qui écraserait
        # la pose authorée dans la scène.
        self._robot = self._world.scene.add(Robot(
            prim_path="/World/g1", name="g1_axisos"))
        self._world.reset()
        self._verify_grounded(expected=ground_clearance)
        self._robot.initialize()

        if not self._robot.num_dof:
            raise RuntimeError(
                "IsaacSimBridge.setup() : articulation toujours introuvable "
                "après sélection de variante — voir les avertissements "
                "ci-dessus sur le VariantSet."
            )

        # Table de correspondance nom AXIS-OS -> index DOF Isaac. Avec
        # l'URDF officiel importé directement, les noms coïncident
        # exactement (confirmé le 18/09, 23/23) — on vérifie quand même
        # plutôt que de le supposer, pour détecter toute régression.
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
        print(f"[bridge] {len(self._joint_index)}/23 joints mappés avec Isaac Sim.")

    def _author_root_translation(self, z: float) -> None:
        """Écrit la translation du prim racine /World/g1 dans la scène USD."""
        try:
            import omni.usd
            from pxr import Gf, UsdGeom
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath("/World/g1")
            api = UsdGeom.XformCommonAPI(prim)
            if api.SetTranslate(Gf.Vec3d(0.0, 0.0, z)):
                print(f"[bridge] translation authorée sur /World/g1 : z={z:+.3f}")
                return
            # Repli : pile d'opérations incompatible avec XformCommonAPI.
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
        le repère du monde. C'est cette valeur qu'il faut relever.
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
        """Remesure la géométrie APRÈS placement et le dit si les pieds ne
        sont pas au sol. Même principe que look_at() : on ne croit pas ce
        qu'on a demandé, on mesure ce qui est."""
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
        le plus bas repose sur le sol (z=0).

        Mesure la boîte englobante de TOUTE la géométrie, collision
        comprise (purpose guide), puisque c'est elle qui touche le sol.
        Rend 0.0 si la mesure échoue, avec un avertissement : mieux vaut
        un robot à l'ancienne position, signalée, qu'une hauteur
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
        """Règle des gains PD de maintien sur les 23 joints.

        NÉCESSAIRE : l'URDF officiel AXIS-OS ne fournit aucune donnée
        stiffness/damping (confirmé en exécution réelle — le warning
        Isaac Sim 'actuator will be created without gain parameters'
        apparaît sur les 23 joints à chaque import). Sans gains, une
        commande POSITION ne tient rien du tout : le joint devient un
        pendule libre sous la gravité dès la commande suivante, ce qui
        a produit des résultats de test entièrement dominés par le bruit
        gravitationnel plutôt que par la direction réellement commandée.

        kp=80.0, kd=8.0 sont des valeurs GÉNÉRIQUES DE TEST, choisies
        pour tenir une position sans osciller violemment — PAS les gains
        réels du G1 (AXIS-OS n'en connaît pas ; ce sont ceux du firmware
        Unitree, propriétaires). Ne jamais réutiliser ces valeurs comme
        référence de contrôle réel.
        """
        n = self._robot.num_dof
        controller = self._robot.get_articulation_controller()
        controller.set_gains(kps=[kp] * n, kds=[kd] * n)

    def set_pure_torque_mode(self) -> None:
        """Gains à zéro (kd résiduel faible pour éviter l'instabilité
        numérique) — nécessaire avant de tester le mode TORQUE en isolant
        le couple commandé, sans qu'un gain de position résiduel ne
        s'y oppose ou ne s'y ajoute."""
        n = self._robot.num_dof
        controller = self._robot.get_articulation_controller()
        controller.set_gains(kps=[0.0] * n, kds=[1.0] * n)

    def hold_all_except(self, free_joint_id: str, kp_hold: float = 80.0,
                         kd_hold: float = 8.0, kd_free: float = 1.0) -> None:
        """Maintient les 23 joints en position SAUF free_joint_id, laissé
        en couple pur (gain de position nul).

        NÉCESSAIRE pour isoler la réponse d'UN SEUL joint à un couple
        commandé. set_pure_torque_mode() met les 23 joints à zéro
        d'un coup — pendant qu'on teste le joint N dans la boucle, les
        joints 1..N-1 déjà « remis à zéro » retombent sous la gravité
        faute de tenue, et comme le squelette est une chaîne
        cinématique, un parent déplacé change l'orientation effective
        de gravité vue par ses enfants. Constaté en exécution réelle :
        right_hip_roll_joint lisait avant=+0.524 au lieu de 0.0 juste
        après son propre reset, parce que right_hip_pitch_joint (son
        parent dans la chaîne, testé juste avant) avait dérivé à
        1.49 rad entre-temps.
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
        """Téléporte les 23 joints à une position donnée d'un coup —
        même avertissement que reset_joint() : outil de simulation pur,
        jamais un équivalent réel. Réaligne aussi les 23 cibles PD."""
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
        DÉLIBÉRÉMENT AXIS-OS (guard, tracker, pont HAL). Ce n'est PAS un
        équivalent de ce qui existe sur un robot réel — aucun robot ne
        peut « téléporter » un joint. C'est un outil de simulation pur,
        pour ramener chaque joint à une pose de référence connue entre
        deux essais et empêcher la dérive gravitationnelle de s'accumuler
        d'un joint testé au suivant. Ne jamais confondre avec une
        commande de mouvement réelle.
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
        # physique. Sinon, une fois les gains de maintien restaurés
        # (hold_all_except), le PD reprend sa poursuite vers une ancienne
        # cible résiduelle (ex : le dernier 'cible' de S2.1, ou rien de
        # jamais fixé pendant une commande TORQUE pure) — provoquant un
        # mouvement parasite pendant qu'on teste un AUTRE joint. Constaté
        # en exécution réelle : left_hip_yaw_joint lisait avant=-0.226
        # juste après son propre reset, alors qu'aucun couple ne lui
        # avait jamais été appliqué.
        self._robot.apply_action(ArticulationAction(
            joint_positions=[position], joint_indices=[idx]))

    def step(self, n: int = 1):
        for _ in range(n):
            self._world.step(render=False)

    def apply_command(self, joint_id: str, position: Optional[float] = None,
                       velocity: Optional[float] = None,
                       effort: Optional[float] = None) -> None:
        """BUG RÉEL corrigé : cette méthode n'acceptait auparavant que
        position et velocity — un ActuatorCommand en ControlMode.TORQUE
        produisait une ArticulationAction totalement vide (les trois champs
        à None), donc RIEN n'était appliqué à Isaac Sim pour les commandes
        de couple. Constaté en exécution réelle (suite S2.2) : les petits
        mouvements observés étaient du bruit physique résiduel, pas l'effet
        du couple commandé — la suite S2.2 telle qu'exécutée n'a jamais
        testé le couple.
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
        """Lecture rapide de la position seule — pratique pour construire
        des commandes en DELTA relatif plutôt qu'en absolu (voir S2)."""
        idx = self._joint_index[joint_id]
        return float(self._robot.get_joint_positions()[idx])

    def read_all_raw(self) -> Dict[str, tuple]:
        """Lit les 23 joints d'un coup : {joint_id: (pos, vel, effort)}.
        THREAD PRINCIPAL UNIQUEMENT — c'est cette méthode que
        MainThreadSimPump appelle pour publier son instantané. Une
        lecture groupée par pas de simulation, plutôt que 23 lectures
        séparées potentiellement incohérentes entre elles."""
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
        """État réel lu depuis le solveur physique Isaac Sim — POSITION,
        VITESSE ET COUPLE RÉELLEMENT APPLIQUÉ (pas le couple commandé).

        Si une pompe est attachée (mode sandbox / multi-thread), lit son
        INSTANTANÉ plutôt que d'appeler Isaac Sim : cette méthode est
        invoquée par JointSafetyGuard depuis le thread du pont, et
        get_joint_positions() y serait tout aussi non-thread-safe qu'un
        step()."""
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
        # physique — c'est la seule source fiable pour la suite S1.2.
        try:
            eff = float(self._robot.get_applied_joint_efforts()[idx])
        except AttributeError:
            eff = float("nan")  # à vérifier contre ta version d'Isaac Sim
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
        """Active le mode multi-thread : à partir de là, send_joint_commands
        dépose dans la pompe au lieu d'appeler Isaac Sim directement, et
        read_feedback lit l'instantané. Obligatoire dès que l'app tourne
        via PlayStoreRuntime (sandbox + HALBridgeServer)."""
        self._pump = pump

    def build_hal(self) -> UnitreeG1HAL:
        """Retourne un UnitreeG1HAL dont send_joint_commands() écrit
        réellement vers Isaac Sim via ce pont, au lieu du simulateur
        interne G1SimulatedActuator. Monkeypatch délibéré et documenté —
        c'est le point d'intégration unique entre les deux piles."""
        hal = UnitreeG1HAL(backend="sim")
        bridge = self

        original_send = hal.send_joint_commands

        def send_via_isaac(commands):
            statuses = original_send(commands)  # applique le guard + tracker AXIS-OS
            accepted = {jid: cmd for jid, cmd in commands.items()
                        if statuses.get(jid) == HALStatus.OK}
            if not accepted:
                return statuses

            if bridge._pump is not None:
                # Chemin multi-thread : on DÉPOSE et on attend le thread
                # principal. Ne jamais appeler apply_command/step ici —
                # send_via_isaac s'exécute dans le thread HALBridgeServer
                # dès que l'app tourne en sandbox.
                if not bridge._pump.submit(accepted):
                    for jid in accepted:
                        statuses[jid] = HALStatus.ERROR_COMM
                return statuses

            # Chemin mono-thread (scripts de test directs, sans sandbox) :
            # on est déjà sur le thread principal.
            for joint_id, cmd in accepted.items():
                bridge.apply_command(joint_id, position=cmd.position,
                                      velocity=cmd.velocity, effort=cmd.torque)
            bridge.step(1)
            return statuses

        hal.send_joint_commands = send_via_isaac  # type: ignore[method-assign]
        return hal


# ---------------------------------------------------------------------------
# Pompe thread principal — obligatoire dès que l'app tourne dans le sandbox
# ---------------------------------------------------------------------------

@dataclass
class _SimRequest:
    """Une demande d'actuation à exécuter sur le THREAD PRINCIPAL."""
    commands: Dict[str, Any]
    done: "threading.Event"
    error: Optional[BaseException] = None
    applied: bool = False


class MainThreadSimPump:
    """Fait exécuter tous les appels Isaac Sim sur le thread principal.

    POURQUOI : World.step() et apply_action() ne sont PAS thread-safe et
    bloquent dans du code C SANS RELÂCHER LE GIL. Appelés depuis un
    thread secondaire (par ex. HALBridgeServer._loop()), ils gèlent le
    process ENTIER — aucun autre thread Python ne peut plus s'exécuter,
    y compris le thread principal attendant un join(timeout=...). Il n'y
    a donc pas de version « prudente avec timeout » de cet appel : la
    seule issue est de ne jamais le faire hors du thread principal.

    ARCHITECTURE RÉSULTANTE :
      - thread principal  : run_forever() — applique les commandes,
                            appelle world.step(), publie un instantané
      - thread du pont    : submit() — dépose et attend, ne touche
                            JAMAIS une API Isaac
      - thread « mission » : lance PlayStoreRuntime (voir run_mission_in_thread)
      - sous-process      : l'app sandboxée, inchangée

    Ce découpage rapproche d'ailleurs la simulation du matériel réel :
    sur un vrai G1, la publication DDS se fait aussi depuis une boucle
    dédiée que les threads applicatifs alimentent — le pont devient donc
    PLUS fidèle au driver réel, pas moins.
    """

    def __init__(self, bridge: "IsaacSimBridge"):
        self._bridge = bridge
        self._queue: "queue.Queue[_SimRequest]" = queue.Queue()
        self._state_lock = threading.Lock()
        self._snapshot: Dict[str, tuple] = {}
        self._running = False
        self._main_thread_id: Optional[int] = None
        # Décomposition du coût d'un tick, pour distinguer le TRANSPORT
        # (files, threads — le vrai coût de l'isolation) du PAS DE
        # SIMULATION (bridge.step(), qui n'existe pas sur un robot réel :
        # la physique y est le monde, elle ne coûte rien au logiciel).
        # Sans cette séparation, on attribue à l'architecture un coût qui
        # est en réalité celui du simulateur.
        self.tick_apply_ms: list = []
        self.tick_step_ms: list = []
        self.submit_wait_ms: list = []
        self.idle_ticks: int = 0

    # ---- appelé depuis N'IMPORTE QUEL thread -----------------------------

    def submit(self, commands: Dict[str, Any], timeout_s: float = 5.0) -> bool:
        """Dépose des commandes et ATTEND leur application par le thread
        principal. Retourne False si la pompe est arrêtée ou si le délai
        expire — jamais d'attente infinie, sinon l'arrêt de la boucle
        principale laisserait le thread du pont bloqué pour toujours."""
        if not self._running:
            return False
        req = _SimRequest(commands=commands, done=threading.Event())
        _t0 = time.perf_counter()
        self._queue.put(req)
        ok = req.done.wait(timeout=timeout_s)
        # Attente vue DU THREAD APPELANT : dépôt -> application par le
        # thread principal. C'est le coût que subit réellement l'app, à
        # distinguer du travail effectué (tick_apply_ms + tick_step_ms).
        # Un écart important entre les deux signifie du temps de
        # RÉVEIL DE THREAD, pas du travail.
        self.submit_wait_ms.append((time.perf_counter() - _t0) * 1000)
        if not ok:
            return False
        return req.error is None and req.applied

    def snapshot(self) -> Dict[str, tuple]:
        """(position, vitesse, couple) par joint, publié par le thread
        principal. C'est la SEULE source de feedback autorisée pour un
        thread secondaire : lire robot.get_joint_positions() depuis le
        thread du pont referait le même appel non-thread-safe, en
        lecture."""
        with self._state_lock:
            return dict(self._snapshot)

    def is_running(self) -> bool:
        return self._running

    # ---- THREAD PRINCIPAL UNIQUEMENT -------------------------------------

    def run_forever(self, should_stop, max_seconds: float = 600.0,
                    idle_sleep_s: float = 0.0) -> None:
        """Boucle de simulation. À APPELER DEPUIS LE THREAD PRINCIPAL,
        celui où SimulationApp a été instanciée. Tourne jusqu'à ce que
        should_stop() renvoie True ou que max_seconds soit atteint."""
        self._main_thread_id = threading.get_ident()
        self._running = True
        t0 = time.time()
        try:
            while not should_stop() and (time.time() - t0) < max_seconds:
                # Attendre l'arrivée d'une commande plutôt que de boucler
                # à vide. Une boucle sans pause monopolise le GIL : le
                # thread HALBridgeServer et le sous-process n'obtiennent
                # la main qu'au bon vouloir de l'ordonnanceur Python, ce
                # qui ajoute des dizaines de ms d'attente PURE (ni
                # transport, ni physique) à chaque aller-retour.
                # Queue.get(timeout=) relâche le GIL pendant l'attente,
                # laissant les autres threads progresser immédiatement.
                try:
                    first = self._queue.get(timeout=idle_sleep_s or 0.002)
                except queue.Empty:
                    first = None
                    self.idle_ticks += 1
                self._tick(first=first)
        finally:
            self._running = False
            # Libérer tout thread encore en attente, sinon il reste
            # bloqué jusqu'à son timeout après l'arrêt de la boucle.
            self._drain_pending()

    def _tick(self, first: "Optional[_SimRequest]" = None) -> None:
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

        # UN SEUL step pour tout le lot : appeler step() par commande
        # ferait avancer la physique 23 fois pour un mouvement corps
        # entier, avec des états intermédiaires incohérents — même
        # raisonnement que la publication DDS groupée du driver réel.
        _t_step = time.perf_counter()
        self._bridge.step(1)
        self._publish_snapshot()
        if pending:
            self.tick_step_ms.append((time.perf_counter() - _t_step) * 1000)

        for req in pending:
            req.done.set()

    def _apply_one(self, req: "_SimRequest") -> None:
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
        snap = self._bridge.read_all_raw()
        with self._state_lock:
            self._snapshot = snap

    def _drain_pending(self) -> None:
        while True:
            try:
                req = self._queue.get_nowait()
            except queue.Empty:
                break
            req.error = RuntimeError("pompe arrêtée avant application")
            req.done.set()


def run_mission_in_thread(fn, *args, **kwargs):
    """Lance fn (typiquement PlayStoreRuntime.start_app / run_mission)
    dans un thread annexe, et retourne (thread, holder).

    NÉCESSAIRE : le thread principal doit rester libre pour faire
    tourner MainThreadSimPump.run_forever(). Si la mission est lancée
    directement depuis le thread principal, celui-ci reste bloqué
    dedans, la queue de la pompe n'est jamais vidée, et le thread du
    pont attend une application qui n'arrivera jamais — un deadlock
    différent, pas une correction.

    holder est un dict rempli à la fin : {"done": bool, "result": ...,
    "error": ...}. Utiliser holder["done"] comme should_stop() de la
    pompe.
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
    if not xs:
        return None
    s = sorted(xs)
    return {"mean": sum(xs) / len(xs), "p50": s[len(s) // 2],
            "p95": s[int(len(s) * 0.95)], "max": s[-1], "n": len(s)}
