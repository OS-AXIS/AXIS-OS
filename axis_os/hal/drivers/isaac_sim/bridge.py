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
from dataclasses import dataclass
from typing import Dict, Optional

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


def import_g1_urdf(force_reimport: bool = True, fix_base: bool = False):
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
    def setup(self, fix_base: bool = False):
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
        usd_path = import_g1_urdf(fix_base=fix_base)
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
        self._world = World(stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()
        self._robot = self._world.scene.add(Robot(prim_path="/World/g1", name="g1_axisos"))
        self._world.reset()
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
        jamais un équivalent réel."""
        import numpy as np
        n = self._robot.num_dof
        positions = np.full(n, position)
        velocities = np.zeros(n)
        self._robot.set_joint_positions(positions)
        self._robot.set_joint_velocities(velocities)

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
        idx = self._joint_index[joint_id]
        positions = np.array(self._robot.get_joint_positions())
        velocities = np.zeros_like(positions)
        positions[idx] = position
        self._robot.set_joint_positions(positions)
        self._robot.set_joint_velocities(velocities)

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

    def read_feedback(self, joint_id: str) -> ActuatorFeedback:
        """État réel lu depuis le solveur physique Isaac Sim — POSITION,
        VITESSE ET COUPLE RÉELLEMENT APPLIQUÉ (pas le couple commandé)."""
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
            for joint_id, cmd in commands.items():
                if statuses.get(joint_id) == HALStatus.OK:
                    bridge.apply_command(joint_id, position=cmd.position,
                                          velocity=cmd.velocity, effort=cmd.torque)
            bridge.step(1)
            return statuses

        hal.send_joint_commands = send_via_isaac  # type: ignore[method-assign]
        return hal
