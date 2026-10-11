# GR00T dans AXIS-OS — note d'architecture

Écrit à partir du code au commit `3fb7589`. Chemins et noms de classes
réels du dépôt.

Ce document est la version de référence. Si un résumé de cette analyse
circule ailleurs et le contredit, c'est ce fichier qui fait foi.

---

## 0. Ce qui est déjà en place et qu'il ne faut pas réécrire

| Existe déjà | Où | Rôle pour GR00T |
|---|---|---|
| `IVLAConnector.predict_action(image, instruction) → VLAAction` | `axis_os/store/sdk/ai/connector.py` | Interface d'inférence bas niveau — GR00T l'implémente |
| `VLAAction.target_angles: Dict[str, float]` | idem | Sortie clé par `joint_id`, pas un vecteur brut |
| `VLAAction.to_commands()` | idem | Conversion vers `ActuatorCommand` |
| `IService` / `ServiceResult` / `ServiceHealth` | `axis_os/services/base.py` | Contrat du service à écrire |
| `MemoryService` | `axis_os/services/memory/` | Premier service — modèle à suivre |
| `JointSafetyGuard` | `axis_os/hal/safety/limits.py` | Garde-fou, inchangé |
| `move_to_pose()` | `RobotContext` | Interpolation — réutilisée telle quelle |
| `MainThreadSimPump` | `axis_os/hal/drivers/isaac_sim/bridge.py` | Test contre Isaac Sim |

**`VLAAction.target_angles` contient des POSITIONS**, pas des vitesses.
Donc `position_delta_max` s'applique tel quel — il n'y a rien à
repenser de ce côté.

---

## 1. Le principe de base : correct

> GR00T ne touche jamais un actionneur directement. Il propose une
> action, le système décide.

C'est juste, et c'est la bonne architecture. Rien à changer.

---

## 2. Service ET connecteur — pas l'un à la place de l'autre

La catégorisation initiale en **service système** était bonne, cohérente
avec `MemoryService` et avec le §6 du document d'architecture (un service
est ce dont plusieurs apps de métiers différents auraient besoin).

Le découpage correct est à deux étages :

```
GR00TPolicyService (IService)          <- NOUVEAU, dans axis_os/services/
  - fait tourner la boucle d'inférence EN INTERNE, process séparé
  - expose latest_action() NON BLOQUANT + horodatage
        |
        v
GR00TConnector (IVLAConnector)         <- NOUVEAU, à côté de VLACppConnector
  - predict_action(image, instruction) -> VLAAction
  - parle au serveur GR00T (réseau)
```

**Pourquoi l'asynchronisme est dans le service et pas dans le
connecteur :** `predict_action()` est synchrone par contrat. GR00T à
2-5 Hz bloquerait l'app 200 à 500 ms par appel. Le service absorbe cette
latence ; l'app ne bloque jamais sur l'inférence, elle prend la dernière
action disponible ou rien si elle est périmée.

`IVLAConnector` reste l'interface générique. Ne crée pas d'interface
parallèle spécifique à GR00T.

---

## 3. Latence : l'interpolateur et l'échéance de fraîcheur

**Règle non négociable : la boucle de sécurité n'est jamais conditionnée
à GR00T.**

Trois couches :

1. GR00T publie des consignes à son rythme (asynchrone)
2. Un interpolateur tourne à la fréquence du HAL et rampe entre deux
   consignes — c'est `move_to_pose()`, qui existe déjà
3. **Échéance de fraîcheur** : si aucune nouvelle action n'arrive dans
   N ms, l'interpolateur ne continue PAS la dernière consigne. Il
   maintient, puis amortit.

Le point 3 est le plus important et c'est celui qu'on oublie. Une
consigne périmée exécutée indéfiniment est exactement le mode de
défaillance déjà rencontré deux fois sur ce projet : le bug `publish()`
jamais appelé, et la raison d'être du moniteur indépendant. Même patron,
à réutiliser.

**Aucune commande ne contourne `JointSafetyGuard` dans ce schéma.** Il
n'y a pas de « chemin rapide allégé » pour un flux à haute fréquence, et
il ne faut pas en créer : c'est ainsi qu'on introduit un contournement de
la couche de sécurité qu'on regrette ensuite.

### Modification requise dans le code existant

```python
@dataclass
class VLAAction:
    target_angles:  Dict[str, float] = field(default_factory=dict)
    gripper_closed: bool             = False
    confidence:     Optional[float]  = None
    timestamp_ns:   int              = 0   # <-- À AJOUTER
```

Sans `timestamp_ns`, l'échéance de fraîcheur est impossible à
implémenter. À ajouter maintenant, tant que rien ne consomme cette
dataclass.

---

## 4. L'écart réel entre ce qui est promis et ce qui est garanti

**C'est le point le plus important de ce document.**

> « GR00T n'a AUCUN pouvoir de contourner cette vérification »

Vrai **commande par commande**. Faux au niveau de ce que le robot fait.

`JointSafetyGuard` valide une commande isolée : position dans les
limites, delta sous `position_delta_max`, vitesse, couple, température.
Il ne valide **aucune trajectoire**.

GR00T peut donc émettre une suite de commandes toutes individuellement
valides qui, collectivement :

- amènent le bras dans une personne debout à côté du robot
- déséquilibrent la machine
- font cycler une articulation à delta maximal pendant dix minutes
  (fatigue mécanique, échauffement)

### Conséquence sur l'argumentaire investisseurs

À ne pas dire : « même quand une IA pilote le robot, notre OS reste le
gardien final. » Un investisseur technique verra la faille.

À dire : **« l'IA ne peut jamais violer les limites physiques du
robot. »** Exact, défendable, et ça reste un vrai différenciateur.

### Ce qui fermerait vraiment l'écart

Une **enveloppe cartésienne** : la main est-elle dans un volume autorisé ?
Cela exige de la cinématique directe, qui n'existe pas encore dans le
projet. Cette brique relève du `ManipulationService` du document
d'architecture, pas de GR00T.

C'est le seul point de cette note dont la solution est reportée, et la
raison est explicite : elle dépend d'un composant non encore écrit.

---

## 5. La couche de traduction — le piège connu, en pire

GR00T produit ses actions dans **sa propre convention** : ordre des
joints, normalisation et **signes**, hérités des données d'entraînement
NVIDIA. Rien ne garantit la correspondance avec l'URDF officiel AXIS-OS.

C'est exactement la classe de bug qui a occupé toute la validation S2 —
mais en pire : les conventions de `conventions.py` viennent d'un URDF
maîtrisé, celles de GR00T viennent d'un jeu de données externe.

**Exigence : la table de correspondance GR00T → AXIS-OS doit être validée
par une suite équivalente à S2, joint par joint, dans Isaac Sim, avant
de toucher un robot réel.** Commander une action GR00T isolée, vérifier
que chaque articulation part dans la direction attendue.

L'outillage existe déjà et n'a pas à être réécrit : `hold_all_except()`,
`reset_joint()`, `MainThreadSimPump`, et la structure du script
`sim/validation/test_isaac_s2_normalization.py`.

---

## 6. Permissions : la décision arrive à échéance

Les docs du projet avaient volontairement reporté « la distinction
IA/script dans les permissions ». C'est maintenant.

Le profil de risque est objectivement différent : les mouvements d'une
app scriptée sont auditables à l'avance, ceux d'une app pilotée par IA
ne le sont pas.

```python
# axis_os/store/sdk/manifest.py
ACTUATORS_AI_DRIVEN = auto()   # niveau DANGEROUS

# Capability
AI_POLICY = auto()   # le Play Store refuse l'installation sur un
                     # robot sans GPU capable de faire tourner le modèle
```

---

## 7. Plan de démonstration

Ce n'est **pas** un conseil de renoncer à l'IA. GR00T est la bonne
direction, et c'est ce qui rend le projet intéressant — un OS qui
n'exécute que des scripts n'attire personne.

**En simulation :** montrer l'IA sans réserve. Isaac Sim, GR00T qui
pilote le G1, le garde-fou qui intercepte. Reproductible, relançable.
C'est la meilleure démo.

**Sur robot physique :** prévoir les deux modes dans la même app. Mode
IA par défaut, bascule scriptée disponible. Pas « scripté à la place de
l'IA » — un filet, en cas d'échec public sur du matériel encore mal
maîtrisé.

**La scène la plus forte** n'est pas « l'IA trie des colis » : n'importe
qui avec un G1 et GR00T peut le faire. C'est **pousser l'IA à commander
un mouvement dangereux et montrer l'OS le refuser**, trace de violation
à l'appui. C'est ça, le produit.

---

## 8. Récapitulatif des actions

| # | Action | Dépendance |
|---|---|---|
| 1 | Ajouter `timestamp_ns` à `VLAAction` | aucune — à faire maintenant |
| 2 | Ajouter `ACTUATORS_AI_DRIVEN` + `Capability.AI_POLICY` | aucune |
| 3 | Écrire `GR00TConnector(IVLAConnector)` | serveur GR00T accessible |
| 4 | Écrire `GR00TPolicyService(IService)` + `latest_action()` | 3 |
| 5 | Interpolateur + échéance de fraîcheur | 4 |
| 6 | Suite de validation des signes GR00T (type S2) | 3, Isaac Sim |
| 7 | Enveloppe cartésienne | cinématique directe (non écrite) |

Les points 1 et 2 coûtent quelques lignes aujourd'hui et deviennent
coûteux une fois que du code consomme ces structures.
