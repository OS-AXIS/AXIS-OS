# AXIS-OS — Architecture des services système

> **Note de conception conservée telle quelle.** Elle a été écrite à partir
> du code du 2026-09-15, quand les services système n'existaient pas
> encore, et ajoutée au dépôt le 2026-09-19. Elle s'adresse à l'auteur du
> projet, qu'elle tutoie, et son « aujourd'hui » désigne septembre 2026.
> L'architecture actuelle est décrite dans la section « Architecture » du
> [README](../README.fr.md).

Les chemins et les noms de classes sont ceux du dépôt à cette date. Tout ce
qui est marqué **[existe]** était alors dans le code ; **[à écrire]** ne
l'était pas.

---

## 1. Le principe

Aujourd'hui AXIS-OS a deux couches : le HAL (moteurs, sécurité) et le Play Store
(apps). Une app parle directement aux articulations. C'est pour ça que « va
chercher un verre » est impossible : il n'y a rien entre « bouger le genou
droit » et « aller dans la cuisine ».

L'architecture cible en a quatre :

```
┌─────────────────────────────────────────────────────────┐
│  APPS               métiers spécialisés, tierces         │
│                     déclarent des intentions             │
├─────────────────────────────────────────────────────────┤
│  ORCHESTRATEUR      IA locale + registre d'intentions    │
│                     demande en langage naturel → plan    │
├─────────────────────────────────────────────────────────┤
│  SERVICES SYSTÈME   navigation, perception, préhension,  │
│                     parole, mémoire — partagés, fournis  │
│                     par l'OS, jamais réimplémentés       │
├─────────────────────────────────────────────────────────┤
│  HAL + SÉCURITÉ     drivers, guard, watchdog, e-stop     │
│                     seul chemin vers les moteurs         │
└─────────────────────────────────────────────────────────┘
```

**La règle de décision — service ou app ?**

> Si deux apps de métiers différents en auraient toutes les deux besoin, c'est un
> service système. Si c'est propre à un métier, c'est une app.

Marcher : une app de surveillance et une app de logistique en ont besoin toutes
les deux → service. Trier des colis par code postal : seule la logistique →
app. Cette règle tranche 95 % des cas, applique-la mécaniquement.

**Le parallèle Android, terme à terme :**

| Android | AXIS-OS |
|---|---|
| Linux + drivers | HAL + safety + watchdog |
| System services (LocationManager, AudioManager…) | Services système |
| Intents + Assistant | Orchestrateur + registre d'intentions |
| Apps du Play Store | Apps AXIS-OS |
| `AndroidManifest.xml` (permissions, features) | `AppManifest` (permissions, capabilities, intents) |

---

## 2. Couche 1 — HAL : ce qui manque

Le HAL est solide mais il est **aveugle et sourd**. `ISensor` et `SensorType`
existent déjà (`axis_os/hal/core/interfaces.py`, `types.py`) avec `CAMERA_RGB`,
`CAMERA_DEPTH` et `MICROPHONE` dans l'énumération — mais aucune interface ni
aucun driver derrière. C'est la dépendance bloquante de tout le reste.

### 2.1 Interfaces à ajouter — `axis_os/hal/core/interfaces.py`

```python
class ICamera(ISensor, ABC):
    """Caméra RGB ou RGB-D. Une instance par caméra physique."""

    @abstractmethod
    def read_frame(self) -> Optional[CameraFrame]: ...

    @abstractmethod
    def get_intrinsics(self) -> CameraIntrinsics:
        """Matrice de calibration — nécessaire pour passer du pixel
        à une position 3D. Sans elle, la perception ne peut pas
        donner de coordonnées exploitables par la préhension."""

    @property
    @abstractmethod
    def frame_rate_hz(self) -> float: ...


class IMicrophone(ISensor, ABC):
    @abstractmethod
    def read_chunk(self, duration_ms: int) -> Optional[AudioChunk]: ...


class ISpeaker(ABC):
    """Sortie audio. N'hérite pas d'ISensor — ce n'est pas un capteur."""

    @abstractmethod
    def play(self, audio: AudioChunk) -> HALStatus: ...

    @abstractmethod
    def is_playing(self) -> bool: ...
```

### 2.2 Types à ajouter — `axis_os/hal/core/types.py`

```python
@dataclass(frozen=True)
class CameraFrame:
    sensor_id:    str
    width:        int
    height:       int
    encoding:     str            # "rgb8" | "bgr8" | "depth16"
    data:         bytes
    timestamp_ns: int            # horodatage à la capture, PAS à la lecture

@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float; fy: float         # distances focales, pixels
    cx: float; cy: float         # centre optique
    distortion: tuple[float, ...]

@dataclass(frozen=True)
class AudioChunk:
    sample_rate_hz: int
    channels:       int
    samples:        bytes
    timestamp_ns:   int
```

**Trois points à ne pas rater :**

`timestamp_ns` doit être celui de la capture matérielle, pas celui de l'appel
Python. Toute la fusion capteur (image + IMU + position articulaire) en dépend :
si tu associes une image à une pose du robot prise 80 ms plus tard, la position
3D de l'objet est fausse de plusieurs centimètres.

Le G1 a ses caméras sur le Jetson embarqué, pas sur le bus DDS des moteurs. Le
driver caméra ne passera donc **pas** par `G1DDSHub` — c'est un chemin
totalement séparé. Ne les mélange pas dans le même module.

`read_frame()` ne doit jamais bloquer la boucle de contrôle. Le driver tourne
dans son propre thread et garde la dernière image ; `read_frame()` la retourne
immédiatement, ou `None` si aucune n'est encore arrivée.

### 2.3 Fichiers à créer

```
axis_os/hal/drivers/unitree_g1/g1_camera.py      # ICamera via le Jetson
axis_os/hal/drivers/unitree_g1/g1_audio.py       # IMicrophone + ISpeaker
axis_os/hal/drivers/sim_common/fake_camera.py    # rejoue des images disque
```

Écris le faux driver **en premier**. Il te permet de développer toute la couche
perception sans robot et sans Isaac Sim, et il devient ton banc de test.

---

## 3. Couche 2 — Services système

Nouveau package : `axis_os/services/`.

### 3.1 Le contrat commun

```python
# axis_os/services/base.py

class IService(ABC):
    """Service système. Singleton, démarré par le noyau au boot,
    partagé par toutes les apps et par l'orchestrateur.

    Un service ne commande JAMAIS un actionneur directement : il
    passe par NormalizedRobotHAL, donc par JointSafetyGuard. Cette
    règle n'a aucune exception."""

    @property
    @abstractmethod
    def service_id(self) -> str: ...

    @property
    @abstractmethod
    def capability(self) -> Capability:
        """Capacité déclarée. Si le matériel ne la supporte pas,
        le service n'est pas démarré et les apps qui l'exigent
        ne s'installent pas."""

    @abstractmethod
    def start(self) -> HALStatus: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def health(self) -> ServiceHealth: ...
```

Toute méthode de service qui agit sur le monde retourne un `ServiceResult` —
jamais une exception nue, jamais un booléen :

```python
@dataclass(frozen=True)
class ServiceResult:
    ok:       bool
    value:    Any            = None
    reason:   Optional[str]  = None   # "objet non trouvé", "chemin bloqué"
    partial:  bool           = False  # action entamée puis interrompue
```

`partial=True` est essentiel : « j'ai marché 3 m sur 8 puis j'ai été bloqué »
n'est pas la même chose qu'un échec. L'orchestrateur doit savoir dans quel état
physique il se retrouve pour replanifier.

### 3.2 Les cinq services

#### Perception — `axis_os/services/perception/`

```python
class PerceptionService(IService):

    def find(self, query: str, timeout_s: float = 5.0) -> ServiceResult:
        """Cherche un objet décrit en langage naturel.
        value = list[DetectedObject], trié par confiance."""

    def describe_scene(self) -> ServiceResult:
        """Description textuelle de ce que voit le robot.
        C'est l'entrée principale de l'orchestrateur."""

    def track(self, object_id: str) -> ServiceResult:
        """Suit un objet déjà détecté, image après image."""

@dataclass(frozen=True)
class DetectedObject:
    object_id:    str
    label:        str
    confidence:   float
    bbox:         tuple[int, int, int, int]
    position_3d:  Optional[tuple[float, float, float]]  # repère robot, mètres
    timestamp_ns: int
```

Le modèle derrière `find()` doit être à **vocabulaire ouvert** — capable de
trouver « un verre » sans avoir été entraîné sur une classe « verre ». C'est ce
qui rend le système extensible sans réentraînement. `position_3d` reste `None`
si la caméra est RGB seule : la préhension doit gérer ce cas plutôt que de
supposer la profondeur disponible.

Le modèle est chargé via un `ModelStore`, pas codé en dur dans le service — voir
§6.4, c'est une décision à prendre dès la première ligne.

#### Navigation — `axis_os/services/navigation/`

```python
class NavigationService(IService):

    def walk_to(self, target: NavTarget, timeout_s: float = 60.0) -> ServiceResult:
        """Marche jusqu'à une cible : coordonnées, ou lieu nommé
        ("cuisine") résolu via le service mémoire."""

    def turn(self, angle_rad: float) -> ServiceResult: ...

    def stop(self) -> ServiceResult:
        """Arrêt propre de la marche — posture stable, PAS un e-stop."""

    def current_pose(self) -> ServiceResult: ...

    def is_reachable(self, target: NavTarget) -> ServiceResult:
        """Vérifie AVANT de bouger. L'orchestrateur appelle toujours
        ça d'abord : échouer dans le planificateur coûte zéro,
        échouer à mi-chemin laisse le robot au milieu du couloir."""
```

Trois sous-composants, dans cet ordre de dépendance : une politique de marche
(la politique pré-entraînée que tu as repérée sur Hugging Face), un planificateur
de trajectoire (Nav2), une carte (SLAM). Note que `stop()` n'est **pas** un
e-stop : sur un bipède, couper les moteurs debout, c'est une chute. Un arrêt de
marche est une manœuvre active vers une posture stable.

#### Préhension — `axis_os/services/manipulation/`

```python
class ManipulationService(IService):

    def reach(self, position_3d, arm: str = "auto") -> ServiceResult:
        """Amène la main à une position. Résout l'IK, puis passe
        par move_to_pose() — donc par le garde de sécurité."""

    def grasp(self, obj: DetectedObject) -> ServiceResult: ...

    def release(self) -> ServiceResult: ...

    def is_holding(self) -> ServiceResult: ...
```

C'est ici que vit la cinématique inverse, et c'est ici que se branche Octo via
`IVLAConnector` **[existe]** — dont la signature `predict_action(image,
instruction)` attend déjà exactement ce que la perception produit. La brique est
compatible, elle n'est juste pas reliée.

#### Parole — `axis_os/services/speech/`

```python
class SpeechService(IService):

    def say(self, text: str, blocking: bool = False) -> ServiceResult: ...

    def listen(self, timeout_s: float = 10.0) -> ServiceResult:
        """value = texte transcrit."""

    def set_wake_word(self, word: str) -> ServiceResult: ...
```

Le plus facile des cinq, et celui qui change le plus la perception du produit.
Un robot qui répond « je vais chercher ça » avant d'agir paraît dix fois plus
abouti qu'un robot silencieux qui fait exactement la même chose.

#### Mémoire — `axis_os/services/memory/`

```python
class MemoryService(IService):

    # Mémoire sémantique : les faits stables
    def remember_place(self, name: str, pose) -> ServiceResult: ...
    def recall_place(self, name: str) -> ServiceResult: ...
    def remember_object(self, label: str, place: str) -> ServiceResult: ...

    # Mémoire épisodique : ce qui s'est passé
    def log_episode(self, episode: Episode) -> ServiceResult: ...
    def recall_episodes(self, query: str, limit: int = 10) -> ServiceResult: ...

    # Espace privé par app — cloisonné, comme le stockage Android
    def app_store(self, app_id: str, key: str, value: Any) -> ServiceResult: ...
    def app_load(self, app_id: str, key: str) -> ServiceResult: ...
```

Persisté sur disque, survit au redémarrage. C'est ce service qui débloque PHILIA
— et qui règle au passage le problème de la facturation `ONE_TIME` perdue à
chaque reboot, relevé dans l'audit.

**Cloisonnement obligatoire :** `app_store` est préfixé par `app_id`, et une app
ne peut jamais lire l'espace d'une autre. Sinon une app gratuite peut siphonner
les données d'une app payante. À faire dès la première ligne, pas après.

---

## 4. Couche 3 — Orchestrateur

Nouveau package : `axis_os/orchestrator/`.

### 4.1 Le registre d'intentions

C'est la pièce centrale. Chaque app déclare ce qu'elle sait faire ; chaque
service déclare ses actions primitives. L'orchestrateur ne connaît rien d'autre.

```python
# axis_os/orchestrator/registry.py

@dataclass(frozen=True)
class Intent:
    intent_id:    str        # "logistics.sort_packages"
    description:  str        # "trier des colis par destination"
    examples:     list[str]  # "trie-moi ces colis", "range les paquets"
    parameters:   dict[str, str]
    provider:     str        # app_id, ou service_id
    capabilities: list[Capability]

class IntentRegistry:
    def register_app(self, manifest: AppManifest) -> None: ...
    def register_service(self, service: IService) -> None: ...
    def match(self, utterance: str, top_k: int = 5) -> list[Intent]: ...
```

`match()` n'a pas besoin d'être intelligent au départ : une recherche par
similarité sur les `examples` suffit largement pour les cinquante premières apps.
Tu remplaceras par un embedding le jour où ça sature, et le reste de
l'architecture ne bougera pas.

### 4.2 Le plan

L'orchestrateur ne produit **jamais** de commandes articulaires. Il produit une
liste d'appels de services ou d'apps — c'est l'invariant de sécurité de toute
l'architecture.

```python
@dataclass(frozen=True)
class PlanStep:
    step_id:    str
    provider:   str
    action:     str
    params:     dict[str, Any]
    on_failure: str = "abort"   # "abort" | "retry" | "ask_user" | "skip"

@dataclass(frozen=True)
class Plan:
    goal:        str
    steps:       list[PlanStep]
    reversible:  bool    # peut-on revenir à l'état initial si ça échoue ?
```

Pour « va chercher un verre dans la cuisine » :

```python
Plan(goal="apporter un verre", reversible=True, steps=[
  PlanStep("1", "memory",       "recall_place",  {"name": "cuisine"}),
  PlanStep("2", "navigation",   "walk_to",       {"target": "$1"},     on_failure="abort"),
  PlanStep("3", "perception",   "find",          {"query": "un verre"}, on_failure="ask_user"),
  PlanStep("4", "manipulation", "grasp",         {"obj": "$3"},         on_failure="retry"),
  PlanStep("5", "navigation",   "walk_to",       {"target": "$origin"}),
  PlanStep("6", "manipulation", "release",       {}),
])
```

### 4.3 La boucle

```python
class Orchestrator:
    def handle(self, utterance: str) -> OrchestratorResult:
        # 1. une app déclare-t-elle cette intention ?
        # 2. sinon, le modèle local compose un plan à partir des services
        # 3. valider le plan AVANT d'exécuter
        # 4. exécuter pas à pas, en réévaluant entre chaque étape
        ...

    def validate(self, plan: Plan) -> list[str]:
        """Rejette un plan avant toute action physique :
        - un provider qui n'existe pas
        - une action inconnue de ce provider
        - une capacité absente du robot
        - plus de N étapes (garde-fou anti-boucle du modèle)
        - une étape non réversible sans confirmation humaine"""
```

**Réévaluer entre chaque étape est non négociable.** Le modèle planifie sur une
idée du monde qui date de la seconde d'avant. Après `walk_to`, on regarde à
nouveau avant de saisir. Un plan exécuté d'un bloc sans revérifier, c'est un
robot qui referme la main sur du vide et croit avoir réussi.

---

## 5. Couche 4 — SDK à trois niveaux

C'est la réponse à « illimité mais simple ». Une seule API ne peut pas être les
deux ; trois niveaux empilés, oui.

**Niveau 3 — verbes métier.** Ce que 90 % des développeurs utiliseront :

```python
def run_mission(self, ctx, params):
    ctx.say("je vais chercher un verre")
    glass = ctx.find("un verre")
    ctx.walk_to("cuisine")
    ctx.pick_up(glass)
    ctx.walk_to("salon")
    ctx.put_down()
    return MissionResult(success=True)
```

**Niveau 2 — mouvement.** `move_to_pose()`, `move_arms()` — ce que tu as déjà.
Pour qui veut piloter le geste sans réécrire la marche.

**Niveau 1 — articulations.** Commandes brutes, `ControlMode.TORQUE` inclus.
Pour les labos et les cas que tu n'as pas prévus. **C'est ta garantie de ne
jamais bloquer personne** — mais il reste derrière le `JointSafetyGuard`, sans
exception.

Le niveau 1 doit exiger une permission dédiée (`ACTUATORS_RAW`) et une revue
manuelle à la publication. C'est le niveau avec lequel on casse du matériel.

**Une seule interface, pas deux.** L'audit a relevé que `RobotContext` expose 11
méthodes et `HALBridgeClient` seulement 8. En ajoutant trois niveaux, cette
divergence deviendrait ingérable. Définis une `IRobotContext` abstraite, fais
hériter les deux implémentations, et ajoute un test qui échoue si leurs surfaces
publiques diffèrent. Un test de dix lignes, à écrire avant tout le reste de ce
document.

---

## 6. Services système, apps système, apps tierces

Il y a trois catégories de code sur le robot, et les confondre coûte cher. Le
parallèle Android est exact :

| | Ce que c'est | Android | Visible ? | Désinstallable ? |
|---|---|---|---|---|
| **Services système** | Pas des apps. Du code dans le process de l'OS | LocationManager, AudioManager | Jamais | Impossible, ils *sont* l'OS |
| **Apps système** | De vraies apps, préinstallées et privilégiées | SystemUI, Réglages, Téléphone | Parfois sans icône | Non |
| **Apps tierces** | Apps ordinaires du catalogue | Tout le reste | Oui | Oui |

### 6.1 Les cinq services ne sont pas des apps

Navigation, perception, préhension, parole, mémoire sont des **services système**
— compilés dans AXIS-OS, démarrés au boot, sans manifest, sans facturation, sans
sandbox. Quatre raisons, tirées directement du code existant :

*Latence.* Le sandbox fait passer chaque appel par une socket Unix, avec
encodage et vérification JSON de chaque message. Acceptable pour une commande de mouvement toutes les 100 ms,
catastrophique pour une boucle de perception à 30 images par seconde.

*Durée de vie.* `SandboxRunner` tue le process au bout de `max_runtime_s`. Un
service doit vivre tant que le robot est allumé.

*Concurrence.* Le verrou de mission empêche deux apps de commander le robot en
même temps. Un service doit être appelable **pendant** une mission, par l'app
elle-même.

*Dépendance circulaire.* L'orchestrateur a besoin de la perception pour décider
quoi lancer. Si la perception était une app, il faudrait la lancer pour décider
quoi lancer.

### 6.2 Les trois apps système à écrire

Celles-là sont de vraies apps, avec un manifest, mais signées par la clé
plateforme et non désinstallables.

**Diagnostic et calibration.** La seule autorisée à envoyer des commandes
articulaires brutes pour tester chaque moteur un par un. C'est l'app la plus
dangereuse du système — elle ne doit jamais pouvoir venir d'un tiers.

**Réglages.** Nommer les lieux (« ça, c'est la cuisine »), apprendre la carte,
configurer le réseau, gérer les permissions accordées. C'est ce qui rend le
robot utilisable par quelqu'un qui ne code pas.

**Assistant par défaut.** Le résident minimal (voir §9). Le robot doit savoir
répondre quand on lui parle, même si aucune app n'est installée. C'est
l'équivalent de SystemUI : l'app qu'on ne remarque pas parce qu'elle est
toujours là.

### 6.3 Le mécanisme qui sépare les deux mondes

Android ne se contente pas de marquer certaines apps « non désinstallables ». Il
attribue un **niveau de protection** à chaque permission. Une permission
`signature` n'est accordée qu'à une app signée avec la même clé que le système —
un développeur tiers n'a pas la clé, donc ne peut pas contourner.

La signature Ed25519 existe déjà **[existe]**, donc c'est une petite extension
de `axis_os/store/sdk/manifest.py` :

```python
class ProtectionLevel(Enum):
    NORMAL    = auto()   # accordée automatiquement — ex. ACTUATORS_READ
    DANGEROUS = auto()   # demande confirmation humaine — ex. ACTUATORS_WRITE_ALL
    SIGNATURE = auto()   # réservée aux apps signées par la clé plateforme

PERMISSION_LEVELS: dict[Permission, ProtectionLevel] = {
    Permission.ACTUATORS_READ:       ProtectionLevel.NORMAL,
    Permission.ACTUATORS_WRITE_ARMS: ProtectionLevel.DANGEROUS,
    Permission.ACTUATORS_WRITE_ALL:  ProtectionLevel.DANGEROUS,
    Permission.ACTUATORS_RAW:        ProtectionLevel.SIGNATURE,
    Permission.EMERGENCY_STOP:       ProtectionLevel.SIGNATURE,
}
```

`install()` refuse alors une app tierce déclarant une permission `SIGNATURE` —
**au moment de l'installation, pas au moment de l'appel**. Et tes apps système
sont simplement celles signées par ta clé : pas de liste blanche codée en dur
quelque part, qu'il faudrait maintenir et qui finirait par diverger.

### 6.4 Le piège de la mise à jour — à éviter dès maintenant

Si la perception est compilée dans l'OS, améliorer le modèle de détection exige
une mise à jour complète de l'OS sur chaque robot déployé.

Android a eu exactement ce problème : les correctifs mettaient dix-huit mois à
atteindre les téléphones parce que tout passait par les constructeurs. Google a
dû sortir des morceaux de l'OS en modules mis à jour séparément.

Ne refais pas l'erreur. Sépare dès le premier jour **le service** (le code, dans
l'OS, stable) et **son modèle** (les poids, mis à jour indépendamment) :

```python
class PerceptionService(IService):
    def __init__(self, model_store: ModelStore):
        self._detector = model_store.load("open_vocab_detector")

    def reload_model(self, version: str) -> ServiceResult:
        """Remplace le modèle sans redémarrer l'OS ni le service."""
```

`PerceptionService` ne bouge plus ; le détecteur qu'il charge se télécharge et
se remplace tout seul. Sinon tu seras coincé entre « je ne peux pas améliorer la
détection » et « je dois pousser une mise à jour système pour changer un fichier
de poids ». Ça vaut pour les trois services qui embarquent un modèle :
perception, préhension (Octo), parole.

---

## 7. Le manifest étendu

Deux champs à ajouter dans `axis_os/store/sdk/manifest.py` :

```python
@dataclass
class AppManifest:
    # ... champs existants ...

    capabilities_required: list[Capability] = field(default_factory=list)
    intents:               list[IntentDecl] = field(default_factory=list)
```

```python
class Capability(Enum):
    WALK = auto();  GRASP_SINGLE = auto();  GRASP_DUAL = auto()
    CAMERA_RGB = auto();  CAMERA_DEPTH = auto()
    SPEECH_IN = auto();   SPEECH_OUT = auto()
    NAVIGATION_MAP = auto();  MEMORY_PERSISTENT = auto()
```

`install()` refuse alors une app dont une capacité manque sur ce robot, avec un
message clair : *« cette app exige une caméra de profondeur, ce robot n'en a
pas »*. C'est ce qui permet un catalogue unique pour des robots très différents
— ton objectif d'universalité.

**Ajoute ces deux champs maintenant**, même vides, même inutilisés. Le jour où
des apps tierces existent, changer le format du manifest casse tout le catalogue.
Aujourd'hui ça coûte dix lignes.

---

## 8. Invariants de sécurité

Ces cinq règles ne souffrent aucune exception, quelle que soit la couche ajoutée.

1. **Aucune couche au-dessus du HAL n'écrit jamais dans un actionneur.** Tout
   passe par `NormalizedRobotHAL` → `JointSafetyGuard`. Un service n'est pas
   plus privilégié qu'une app.
2. **L'IA ne produit jamais de valeurs numériques de commande.** Elle produit
   des appels de services nommés. Le pire qu'un modèle qui délire puisse faire,
   c'est un plan absurde mais borné.
3. **Un plan est validé avant d'être exécuté**, et réévalué entre chaque étape.
4. **Un service en défaut ne dégrade jamais en silence.** Perception hors
   service → `find()` retourne `ok=False`, il ne retourne pas une liste vide qui
   ressemble à « rien trouvé ».
5. **Le watchdog, le superviseur, le moniteur indépendant et l'e-stop câblé
   restent au-dessus de tout ça.** Aucun service n'a le droit de les
   contourner, même pour gagner de la latence.

---

## 9. Le résident — PHILIA

Une app aujourd'hui, c'est une mission : on l'appelle, elle s'exécute, elle rend
un résultat, elle meurt. Une identité persistante, c'est l'inverse : elle est là
en permanence, et les missions sont des choses qui lui arrivent.

Ajoute un second type d'app, à côté de `AXISApp` :

```python
class AXISResident(ABC):
    """Tourne en continu entre les missions. Un seul résident actif
    à la fois. Quota permanent mais réduit."""

    @abstractmethod
    def on_idle(self, ctx: IRobotContext) -> None:
        """Appelé régulièrement quand aucune mission n'est active."""

    @abstractmethod
    def on_event(self, ctx, event: SystemEvent) -> None:
        """Personne détectée, mission terminée, batterie faible..."""

    @abstractmethod
    def on_mission_start(self, ctx, app_id: str) -> None:
        """Le résident rend la main. Il observe mais ne commande plus."""
```

Règle d'arbitrage claire : **pendant une mission, le résident perd le contrôle
moteur.** Il peut observer, parler, mémoriser — pas bouger. Sans cette règle,
deux entités commandent le même robot, et c'est exactement le bug de missions
concurrentes que tu as déjà corrigé une fois.

Ne construis pas ça avant d'avoir les services parole et mémoire : un résident
sans mémoire persistante et sans voix n'a rien à faire ni à dire.

---

## 10. Plan de construction

Chaque étape est validable avant de passer à la suivante.

**Étape 0 — unifier l'interface du contexte.** `IRobotContext` + test de
non-divergence. Une journée, et ça t'évite une dette qui devient ingérable dès
l'étape 3.
*Validé quand :* le test échoue si on ajoute une méthode d'un seul côté.

**Étape 1 — le manifest étendu et les niveaux de protection.**
`capabilities_required`, `intents`, et `ProtectionLevel` sur les permissions
(§6.3). Tout se vérifie à l'installation. Quelques dizaines de lignes, aucune
dépendance.
*Validé quand :* une app exigeant `CAMERA_DEPTH` est refusée sur un robot sans,
et une app tierce déclarant `ACTUATORS_RAW` est refusée à l'installation.

**Étape 2 — la caméra.** `ICamera`, `CameraFrame`, le faux driver d'abord, puis
le driver G1. C'est la dépendance bloquante de toute la couche haute.
*Validé quand :* une app reçoit une image horodatée du robot réel, à 15 Hz
minimum, sans perturber la boucle de contrôle.

**Étape 3 — perception + mémoire.** Les deux ensemble : la perception sans
mémoire ne peut rien situer, la mémoire sans perception n'a rien à retenir.
*Validé quand :* le robot répond correctement à « qu'est-ce que tu vois ? » et
retrouve un lieu nommé après un redémarrage complet.

**Étape 4 — parole.** Le moins risqué, le plus visible. Excellente démo pour des
investisseurs, et ça ne peut rien casser physiquement.
*Validé quand :* boucle complète écouter → transcrire → répondre.

**Étape 5 — l'orchestrateur, sans navigation.** Registre d'intentions, plan,
validation, exécution. Limité à perception + préhension + parole + mémoire.
*Validé quand :* « prends le verre sur la table » fonctionne, robot immobile.

**Étape 6 — navigation.** La plus lourde : politique de marche, Nav2, SLAM. Sur
robot suspendu d'abord, puis au sol dans un espace dégagé.
*Validé quand :* « va dans la cuisine » fonctionne sur une carte connue.

C'est seulement après l'étape 6 que « va chercher un verre dans l'autre pièce »
devient un enchaînement de choses qui marchent déjà.

**Les trois apps système (§6.2) ne sont pas une étape séparée** — elles se
construisent au fil de l'eau, chacune quand ce dont elle dépend existe. Le
diagnostic vient tôt, dès l'étape 1 : c'est lui qui te servira à valider chaque
moteur du G1 à la première mise sous tension, et il n'a besoin que du niveau 1
du SDK. Les réglages arrivent avec la mémoire (étape 3), pour nommer les lieux.
L'assistant par défaut arrive avec la parole (étape 4).

---

## 11. Ce que je ne construirais pas maintenant

**Le résident (PHILIA).** Avant les étapes 3 et 4, il n'aurait ni mémoire ni
voix. Conçois l'interface maintenant, écris-la plus tard.

**Le multi-robots.** Le Fleet Manager est déjà documenté comme volontairement
reporté, et cette architecture ne change rien à cette décision.

**Un orchestrateur cloud.** Commence avec le modèle local seul. La latence
réseau dans une boucle de décision robotique est un problème en soi, et tu ne
veux pas que « va chercher un verre » dépende d'une connexion.

**Une place de marché de services système.** Les cinq services sont fournis par
l'OS et par personne d'autre. Le jour où un tiers peut remplacer le service de
navigation, la sécurité n'est plus garantie par personne.

---

## 12. Le mot honnête

L'architecture ci-dessus est solide et te met sur le bon chemin. Mais « aller
chercher un verre dans une pièce inconnue » est aujourd'hui à la frontière de ce
que la robotique sait faire — de la manipulation mobile en environnement non
préparé. Les meilleurs labos du monde le font en démo, dans des conditions
choisies, avec un taux d'échec réel.

Ça ne veut pas dire qu'il ne faut pas viser ça. Ça veut dire deux choses.
D'abord, conçois maintenant pour ce scénario : c'est gratuit aujourd'hui et très
cher à rétrofiter dans deux ans. Ensuite, la première version qui marchera sera
contrainte — carte apprise à l'avance, liste d'objets connue, pièce rangée.

Assume-le et annonce-le tel quel. Un robot qui fait dix choses de façon fiable
vaut infiniment mieux, pour tes premiers clients comme pour tes investisseurs,
qu'un robot qui en promet mille et en rate neuf cents.
