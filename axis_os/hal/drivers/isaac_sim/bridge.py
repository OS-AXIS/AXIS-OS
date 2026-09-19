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
from typing import Any, Dict, Optional

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
        self._pump: Optional["MainThreadSimPump"] = None
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

    # ---- appelé depuis N'IMPORTE QUEL thread -----------------------------

    def submit(self, commands: Dict[str, Any], timeout_s: float = 5.0) -> bool:
        """Dépose des commandes et ATTEND leur application par le thread
        principal. Retourne False si la pompe est arrêtée ou si le délai
        expire — jamais d'attente infinie, sinon l'arrêt de la boucle
        principale laisserait le thread du pont bloqué pour toujours."""
        if not self._running:
            return False
        req = _SimRequest(commands=commands, done=threading.Event())
        self._queue.put(req)
        if not req.done.wait(timeout=timeout_s):
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
                self._tick()
                if idle_sleep_s:
                    time.sleep(idle_sleep_s)
        finally:
            self._running = False
            # Libérer tout thread encore en attente, sinon il reste
            # bloqué jusqu'à son timeout après l'arrêt de la boucle.
            self._drain_pending()

    def _tick(self) -> None:
        _t_apply = time.perf_counter()
        pending = []
        while True:
            try:
                req = self._queue.get_nowait()
            except queue.Empty:
                break
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
