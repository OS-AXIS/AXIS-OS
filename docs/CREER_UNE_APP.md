# Créer une app pour AXIS-OS — le guide de A à Z

Ce guide s'adresse à quelqu'un qui veut créer une application pour un
robot qui tourne sous AXIS-OS : un développeur, une entreprise qui a sa
propre IA, un étudiant. **Il n'est pas nécessaire d'avoir un robot, ni un
GPU** : tout ce qui suit se développe et se teste sur un ordinateur
ordinaire.

Tout ce qui est décrit ici existe et est testé dans le dépôt. Ce qui
n'existe pas encore est dit explicitement (§ 12).

---

## Sommaire

1. [Une app AXIS-OS, c'est quoi ?](#1-une-app-axis-os-cest-quoi-)
2. [Ce qu'il faut sur son ordinateur](#2-ce-quil-faut-sur-son-ordinateur)
3. [Première app en 5 minutes](#3-première-app-en-5-minutes)
4. [Anatomie d'une app](#4-anatomie-dune-app)
5. [Le manifeste : ce que l'app déclare](#5-le-manifeste--ce-que-lapp-déclare)
6. [Ce que l'app peut faire : le contexte `ctx`](#6-ce-que-lapp-peut-faire--le-contexte-ctx)
7. [Les règles d'or (apprises à nos dépens)](#7-les-règles-dor-apprises-à-nos-dépens)
8. [Exemple complet : brancher SON IA, sans faire bouger le robot](#8-exemple-complet--brancher-son-ia-sans-faire-bouger-le-robot)
9. [« J'ai créé une IA pour que le robot parle mieux » : où la brancher ?](#9--jai-créé-une-ia-pour-que-le-robot-parle-mieux---où-la-brancher-)
10. [Exemple : une app de tri de colis](#10-exemple--une-app-de-tri-de-colis)
11. [Tester, puis publier](#11-tester-puis-publier)
12. [Ce qui n'existe pas encore](#12-ce-qui-nexiste-pas-encore)
13. [FAQ](#13-faq)

---

## 1. Une app AXIS-OS, c'est quoi ?

Le même principe qu'une app Android sur un téléphone :

| Téléphone Android | Robot sous AXIS-OS |
|---|---|
| Android gère l'écran, le réseau, la batterie | AXIS-OS gère les moteurs, l'équilibre, la sécurité |
| Une app demande « accès à l'appareil photo » | Une app demande `SENSORS_CAMERA`, `LOCOMOTION_WALK`… |
| L'app ne peut pas faire exploser la batterie | Par `ctx`, l'app ne commande aucun moteur sans le contrôle de l'OS : permissions, garde-fou (voir la limite plus bas) |
| Google Play | le Play Store AXIS-OS |

L'idée centrale : **l'app décide QUOI faire, l'OS décide si c'est sûr et
COMMENT le faire.** Une app ne commande jamais un moteur directement :

```
votre app  ──>  RobotContext (ctx)  ──>  services de l'OS  ──>  garde-fou  ──>  moteurs
             permissions vérifiées      marche, vision,       JointSafetyGuard,
             à CHAQUE appel             parole, mémoire       gardien de proximité
```

Chaque app tourne dans un **bac à sable** (un processus séparé, avec des
limites de mémoire, de processeur et de durée). Si elle plante ou se fige,
le robot reste debout et les autres apps continuent.

> **Limite actuelle.** Ce bac à sable protège le robot d'une app buggée,
> pas encore d'une app malveillante. Les garanties de ce guide valent pour
> une app qui passe par `ctx`. Une app délibérément hostile peut
> aujourd'hui sortir de ce cadre et agir hors de la HAL, donc hors du
> garde-fou : voir [SECURITY.md](../SECURITY.md#limites-connues) (limite
> connue n° 1). La correction est planifiée dans
> [isolation_apps.md](isolation_apps.md). En attendant, n'installez sur un
> vrai robot que des apps dont vous avez relu le code.

---

## 2. Ce qu'il faut sur son ordinateur

- Python **3.12** ou plus récent (la version que teste la CI) ;
- le dépôt AXIS-OS, et ses dépendances de base :

```bash
git clone https://github.com/OS-AXIS/AXIS-OS.git
cd AXIS-OS
pip install -r requirements.txt numpy
export PYTHONPATH="$PWD"          # pour que « import axis_os » fonctionne
```

Pas de robot, pas de GPU, pas de simulateur 3D pour commencer.

---

## 3. Première app en 5 minutes

L'outil en ligne de commande génère le squelette d'une app :

```bash
python3 -m axis_os.cli.main init "Mon Assistant" --author "Kenzo" --email "moi@exemple.com"
cd mon_assistant
python3 -m axis_os.cli.main simulate
```

Résultat attendu :

```
[PlayStore] Installation 'Mon Assistant' v0.1.0
  ✓ 'Mon Assistant' installée.
  [com.kenzo.mon_assistant] Prête.
[PlayStore] ▶ Mission FD8FF336 | app=com.kenzo.mon_assistant
  [Mission FD8FF336] ✓ SUCCESS
✓ Simulation réussie
```

`simulate` installe l'app sur un **robot virtuel** (un G1 sans simulateur
3D, dont chaque articulation suit une physique simplifiée), la démarre
dans son bac à sable et lui fait exécuter une mission. Votre app a tourné
exactement comme elle tournera sur un vrai robot, à ceci près qu'aucun
moteur n'existe.

Fichiers générés :

```
mon_assistant/
├── app.py          ← votre code : le manifeste + 3 méthodes
├── manifest.json   ← les métadonnées pour le Play Store
├── test_app.py     ← un test qui installe et lance l'app
└── README.md
```

---

## 4. Anatomie d'une app

Une app est **une classe** qui hérite de `RobotApp` et définit quatre
méthodes :

```python
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission


class MonApp(RobotApp):

    def manifest(self) -> AppManifest:
        """QUI je suis et ce dont j'ai BESOIN. Lu à l'installation."""
        return AppManifest(
            app_id="com.moi.mon_app", name="Mon App", version="1.0.0",
            author="Moi", author_email="moi@exemple.com",
            description="Ce que fait l'app, en une phrase.",
            category=AppCategory.UTILITIES,
            permissions=[Permission.SENSORS_CAMERA],
        )

    def on_start(self, ctx: RobotContext) -> bool:
        """Préparation. Rendre False = l'app refuse de démarrer."""
        ctx.log("Prête.")
        return True

    def run_mission(self, ctx: RobotContext, params: dict) -> MissionResult:
        """Le travail. `params` : ce que l'utilisateur a demandé."""
        objets = ctx.look_for("a cardboard box")
        return MissionResult(success=True, mission_id="", app_id="com.moi.mon_app",
                             duration_s=0.0, items_processed=len(objets))

    def on_stop(self, ctx: RobotContext) -> None:
        """Rangement. Toujours appelée, même après un échec."""
        ctx.log("Arrêt.")
```

Le cycle de vie, géré par le Play Store :

```
install(app)        manifeste vérifié : format, version du SDK, signature,
                    permissions, matériel du robot  ── refus motivé sinon
start_app(app_id)   -> on_start(ctx)                (dans le bac à sable)
run_mission(...)    -> run_mission(ctx, params)     (autant de fois qu'on veut)
stop_app(app_id)    -> on_stop(ctx)
```

`MissionResult` est le compte rendu : `success`, `items_processed` (le
nombre de choses traitées : colis, questions…), `error_message` en cas
d'échec, et `metrics`, un dictionnaire libre pour vos résultats.

---

## 5. Le manifeste : ce que l'app déclare

Le manifeste est un **contrat** : l'utilisateur le voit avant
d'installer, et l'OS le fait respecter pendant l'exécution.

### Les champs

| Champ | Rôle |
|---|---|
| `app_id` | identifiant unique, format `com.auteur.nom` |
| `name`, `version`, `author`, `author_email`, `description`, `category` | ce que voit l'utilisateur |
| `permissions` | ce que l'app a le droit de faire (tableau suivant) |
| `capabilities_required` | ce dont le robot doit être capable : `WALK`, `CAMERA_RGB`, `CAMERA_DEPTH`, `SPEECH_IN`, `GRASP_DUAL`, `NAVIGATION_MAP`, `MEMORY_PERSISTENT`… Un robot qui ne l'a pas : **installation refusée**, avec la raison |
| `hardware` | exigences matérielles fines : `required`/`optional`, nombre minimal d'articulations, modèles compatibles |
| `quota` | limites du bac à sable : `max_cpu_percent`, `max_memory_mb`, `max_runtime_s` (30 s par défaut : à augmenter pour une longue mission). Des nombres finis strictement positifs, sous les plafonds de l'OS (`PLAFONDS_QUOTAS` : 100 %, 1 024 Mo et 3 600 s en production ; 400 %, 8 192 Mo et 7 200 s en simulation et en développement) : au-delà, installation refusée. Ils sont montrés à l'utilisateur qui accepte vos permissions |
| `max_velocity_ratio` | plafond de vitesse des bras, entre 0 et 1 |
| `pricing_model`, `base_price_eur` | gratuit, achat unique, abonnement, ou paiement par mission. Le Play Store prend 30 % : pour toucher 1,40 €, le client paie 2,00 € |
| `code_signature`, `developer_public_key` | signature du code (§ 11) |

### Les permissions

Principe : **ne demandez que ce dont vous avez besoin.** Une app qui
demande de faire marcher le robot alors qu'elle répond à des questions
sera, à juste titre, refusée par l'utilisateur.

| Permission | Ce qu'elle autorise | Niveau |
|---|---|---|
| `ACTUATORS_READ` | lire la position des articulations | normal |
| `SENSORS_IMU` | lire la centrale inertielle | normal |
| `SENSORS_CAMERA` | `look_for`, `describe_scene` | normal |
| `SENSORS_MICROPHONE` | `listen`, `declare_intent` | normal |
| `STORAGE_PERSISTENT` | `remember`, `recall` (mémoire propre à l'app, 1 000 clés et 1 Mio au plus) | normal |
| `WATCHDOG_READ` | recevoir les alertes de sécurité | normal |
| `ACTUATORS_WRITE_ARMS` / `_LEGS` / `_ALL` | commander les bras, les jambes, tout | **dangereux** |
| `LOCOMOTION_WALK` | faire se déplacer le robot | **dangereux** |
| `EMERGENCY_STOP` | déclencher l'arrêt d'urgence | **dangereux** |
| `ACTUATORS_AI_DRIVEN` | mouvements décidés par une IA (et non un script) — confirmée à l'installation, et exigée à l'exécution pour les consignes des actions d'IA du SDK | **dangereux** |
| `ACTUATORS_RAW` | commandes moteur brutes | **réservé** aux apps signées par AXIS-OS |
| `FLEET_BROADCAST` | parler aux autres robots | **réservé** |

Les permissions **dangereuses** exigent une confirmation EXPLICITE à
l'installation : sans elle, l'app est refusée. Le robot fournit cette
confirmation par `PlayStoreRuntime(confirmer_permissions=...)` —
`confirmer_au_terminal` sur un vrai robot (seul « o » ou « oui »
accorde), `accepter_tout_SIMULATION` en simulation et dans les tests
(refusé sur un robot réel et en profil `PRODUCTION`). Une app qui exige `Capability.AI_POLICY` doit
déclarer `ACTUATORS_AI_DRIVEN`, sinon son manifest est invalide (le
runtime ne fournit pas encore `AI_POLICY` : cette règle vaudra pour le
futur service VLA). Déclarez `ACTUATORS_AI_DRIVEN` dès qu'une IA décide
des mouvements : les consignes produites par `VLAAction`, `GR00TAction`
ou `LocomotionAction` (`to_commands()`, `move_to_pose(ctx)`, et
`walk(ctx)` pour une vitesse de marche) sont refusées à l'exécution sans
elle, et l'utilisateur doit le savoir avant d'accepter. Passez par ces
méthodes plutôt que de recopier les nombres du modèle dans `ctx.walk()`
ou `ctx.move_to_pose()` : ce sont elles qui portent la marque « IA ». Les
permissions **réservées** sont, elles, refusées à toute app qui n'est pas
signée par la clé de la plateforme.

Honnêteté sur l'état actuel : `NETWORK_CLOUD`, `NETWORK_LOCAL`,
`SENSORS_FORCE_TORQUE` et `FLEET_BROADCAST` existent mais **ne sont pas
encore appliquées** (le bac à sable ne filtre pas le réseau). Déclarez-les
quand même si votre app s'en sert : l'utilisateur doit savoir que votre
app parle à Internet.

---

## 6. Ce que l'app peut faire : le contexte `ctx`

Tout passe par l'objet `ctx`, reçu dans `on_start`, `run_mission` et
`on_stop`. **Chaque appel vérifie la permission** : sans elle, l'appel
lève `PermissionError`. Aucun appel de `ctx` ne permet d'éviter ce
contrôle ; une app hostile qui sort de `ctx` relève de la limite décrite
au § 1.

| Appel | Permission | Ce que ça fait |
|---|---|---|
| `ctx.log(texte)` | — | journal de mission |
| `ctx.now()`, `ctx.sleep(s)` | — | l'heure **du robot** (simulée en simulation) |
| `ctx.look_for("a red box", frame="world")` | caméra | objets détectés, triés par confiance ; position 3D si caméra de profondeur |
| `ctx.describe_scene()` | caméra | ce que le robot voit, en clair |
| `ctx.declare_intent(nom, mots_cles)` | micro | ajoute du vocabulaire à votre app (256 intentions au plus ; une autre app peut déclarer le même nom sans vous gêner) |
| `ctx.listen(duree_s)` | micro | une **intention** reconnue (`.name`, `.params`, `.utterance` = la phrase), ou `None` |
| `ctx.remember(cle, valeur)`, `ctx.recall(cle)` | stockage | mémoire qui survit au redémarrage, cloisonnée par app (un cloisonnement logique : voir la FAQ), 1 000 clés et 1 Mio au plus : au-delà, `remember` rend `False` |
| `ctx.walk(vx, vy, wz)`, `ctx.stop_walking()` | marche | déplacement ; la consigne doit être **renouvelée** |
| `ctx.move_to_pose({articulation: angle}, duree_s)` | bras / jambes | mouvement progressif, depuis la position réelle |
| `ctx.move_arms(...)`, `ctx.move_legs(...)` | bras / jambes | consignes articulaires, vérifiées par le garde-fou |
| `ctx.get_full_state()` | lecture | position et vitesse de chaque articulation |
| `ctx.emergency_stop()` | arrêt d'urgence | arrête tout le robot, et interrompt la mission en cours |
| `ctx.subscribe_alerts(fonction)` | alertes | être prévenu des alertes de sécurité |

Un service absent du robot (pas de caméra, pas de micro) n'est **pas une
erreur** : l'appel rend une liste vide, `None` ou `False`, et le dit dans
le journal.

---

## 7. Les règles d'or (apprises à nos dépens)

Chacune vient d'un vrai bug, rencontré en développant les apps de
référence.

1. **Mesurez le temps avec `ctx.now()` et attendez avec `ctx.sleep()`,
   jamais avec `time`.** En simulation, le temps du robot n'est pas celui
   de votre montre. Une app qui écoutait « deux secondes » de montre, dans
   une simulation ralentie, prenait ses décisions sur une image périmée.
2. **`None` est une réponse normale.** `listen()` rend `None` quand
   personne ne parle ; `look_for()` rend `[]` quand rien n'est visible.
   Traitez-le comme un cas ordinaire, jamais comme une erreur.
3. **Pour mémoriser une position, utilisez `frame="world"`.** Dans le
   repère du robot, deux colis vus en tournant sur place peuvent tomber au
   même endroit : une app d'inventaire a fusionné deux colis situés à 90°
   l'un de l'autre.
4. **`walk()` n'est pas une commande qu'on lance et qu'on oublie.**
   Renouvelez la consigne régulièrement ; si l'app se tait, le robot
   s'arrête de lui-même, en équilibre. C'est voulu.
5. **Définissez votre classe au niveau du module**, jamais à l'intérieur
   d'une fonction, avec un constructeur sans argument. Chaque appel à
   votre app tourne dans un processus Python neuf qui la recharge depuis
   son module ; il ne retrouve pas une classe locale, et ne recopie de
   l'instance installée que les attributs qui sont de simples données.
   Paramètres de mission et `metrics` doivent eux aussi être des données
   JSON (nombres, chaînes, listes, dicts).
6. **Le garde-fou peut refuser vos consignes**, et il a raison : trop
   rapide, trop loin, hors des butées. `move_arms()` rend le statut de
   chaque articulation ; lisez-le. Aucun appel de `ctx` ne permet de le
   contourner, et l'API n'en offrira pas.
7. **Une intention vocale ne commande jamais un moteur.** `listen()` rend
   une intention ; c'est **votre code** qui décide ce qu'elle déclenche.
   Une phrase qu'aucune app n'a déclarée ne produit rien, et votre app ne
   reçoit que les intentions qu'elle a déclarées elle-même.
9. **Un arrêt d'urgence met fin à votre mission.** Dès qu'il survient,
   tout appel à `ctx` lève `EstopActiveError` ; vous avez une
   demi-seconde pour rendre votre résultat, puis votre processus est
   arrêté. La mission est notée « interrompue par arrêt d'urgence » et
   n'est jamais facturée. Ne comptez pas sur ce délai pour finir un
   travail.
8. **Échouez en le disant.** Un `MissionResult(success=False,
   error_message="…")` clair vaut mieux qu'une app qui invente un résultat
   plausible.

---

## 8. Exemple complet : brancher SON IA, sans faire bouger le robot

Le cas le plus simple et le plus fréquent : une entreprise a sa propre IA
(un assistant, une base de connaissances, un modèle maison) et veut que
les gens puissent lui parler **à travers le robot**. Le robot n'a pas
besoin de bouger.

Le code complet, testé, est dans le dépôt :
- l'app : [`axis_os/apps/assistant_ia/app.py`](../axis_os/apps/assistant_ia/app.py)
- son test, sans robot ni GPU : [`test/test_app_assistant_ia.py`](../test/test_app_assistant_ia.py)

### Ce qu'elle fait

```
« Robot, quels colis sont arrivés ce matin ? »
      │  micro → transcription (Whisper) → intention « parler_a_l_ia »
      ▼
   l'app  ──POST {"question": …, "historique": […]}──>  VOTRE IA
      ▲                                                     │
      └──────────────── {"reponse": "…"} ◄──────────────────┘
      │
   journal de mission + MissionResult.metrics["reponses"]
```

### 1. Le manifeste : le strict nécessaire

```python
permissions=[
    Permission.SENSORS_MICROPHONE,   # écouter
    Permission.STORAGE_PERSISTENT,   # retenir le nombre d'échanges
    Permission.NETWORK_CLOUD,        # dire à l'utilisateur qu'on parle à Internet
],
capabilities_required=[Capability.SPEECH_IN],   # robot sans micro : refusée
quota=ResourceQuota(max_runtime_s=120.0),       # une conversation dure
```

Aucune permission de mouvement : **l'app ne peut pas faire bouger le
robot**, même par erreur, même si votre IA le lui demandait.

### 2. Un mot de réveil

```python
def on_start(self, ctx):
    ok = ctx.declare_intent("parler_a_l_ia", [["robot", "axis"]])
    ok &= ctx.declare_intent("fin_conversation", [["au revoir", "termine"]])
    return ok
```

Seules les phrases contenant « robot » ou « axis » partent vers l'IA. La
conversation de deux collègues à côté du robot n'est **jamais envoyée en
ligne** (c'est vérifié par le test).

### 3. Votre IA, derrière une adresse web

L'app ne dépend d'aucune bibliothèque d'IA. Elle parle à **n'importe quel
service** qui respecte ce contrat minimal :

```
POST <votre adresse>
     {"question": "Robot, combien de palettes au quai 3 ?",
      "historique": [["question précédente", "réponse précédente"]]}
réponse
     {"reponse": "Il y en a 12."}
```

Derrière, vous mettez ce que vous voulez : votre modèle maison, un modèle
hébergé, un agent qui interroge votre logiciel de stock. Pour développer
hors ligne, l'app contient une **IA d'essai** qui répond sans réseau.

### 4. La boucle de conversation

```python
ia = ModeleHTTP(params["ia_url"]) if params.get("ia_url") else ModeleEssai()
for _ in range(params.get("ecoutes_max", 20)):
    intention = ctx.listen(duration_s=4.0)
    if intention is None:
        continue                                   # silence : normal
    if intention.name == "fin_conversation":
        break
    try:
        reponse = ia.repondre(intention.utterance, historique)
    except (urllib.error.URLError, TimeoutError, KeyError, ValueError):
        ctx.log("IA indisponible — question ignorée")   # on le dit, on n'invente pas
        continue
    historique.append((intention.utterance, reponse))
```

### 5. Le lancer

```bash
python3 test/test_app_assistant_ia.py
```

Le test monte un robot virtuel, le vrai Play Store avec son bac à sable, le
vrai service de parole (avec des phrases scriptées à la place du micro),
et une « IA d'entreprise » simulée par un petit serveur local. Il vérifie :
les questions arrivent à l'IA avec leur historique ; la conversation
voisine n'y arrive jamais ; « au revoir » termine ; une IA injoignable
donne un échec **expliqué**, sans réponse inventée ; un robot sans micro
refuse d'installer l'app.

Pour brancher votre vraie IA : `run_mission(..., params={"ia_url":
"https://votre-ia.exemple.com/robot"})`.

---

## 9. « J'ai créé une IA pour que le robot parle mieux » : où la brancher ?

Tout dépend de ce que fait votre IA : chaque cas se branche à un étage
différent.

| Votre IA… | Où elle se branche | État |
|---|---|---|
| **converse** : comprend une question, formule une réponse | dans une **app**, comme au § 8 | ✅ disponible |
| **reconnaît mieux la parole** : meilleure transcription, accents, bruit d'usine | dans l'**OS** : implémenter `ISpeechRecognizer` (une méthode, `transcribe(audio) -> (texte, confiance)`), à la place de Whisper. Toutes les apps en profitent d'un coup | ✅ interface disponible ; branchement par l'intégrateur du robot, pas par une app du Play Store |
| **donne une voix au robot** : synthèse vocale | l'OS a la capacité `SPEECH_OUT`, mais le SDK n'expose **pas encore** `ctx.speak()` | ❌ pas encore (§ 12) |
| **décide des mouvements** : un modèle vision-langage-action | `IVLAConnector` (`axis_os/store/sdk/ai/connector.py`), avec la permission `ACTUATORS_AI_DRIVEN` en plus des permissions de mouvement. Chaque consigne traverse quand même le garde-fou | ⚠️ interface disponible, jamais validée sur robot |

Le principe reste le même partout : **une IA propose, du code
responsable décide, l'OS vérifie.** Aucun modèle, si bon soit-il, n'a de
chemin direct vers un moteur.

---

## 10. Exemple : une app de tri de colis

Trier des colis, c'est : trouver un colis, lire sa destination (couleur,
étiquette), le prendre, le porter au bon poste, le poser, recommencer — et
savoir quoi faire quand ça rate.

### La leçon d'architecture de la démo

La démonstration de tri d'AXIS-OS
([`docs/demo_tri_usine.md`](demo_tri_usine.md)) sépare trois choses, et
c'est ce découpage qu'il faut reproduire :

```
LOGIQUE MÉTIER        quoi faire, et quoi faire quand ça rate
(tri_colis/logique.py)   « colis rouge → poste A », 3 essais de prise puis
                         abandon signalé, poste inaccessible → couleur suspendue
        │  ne parle qu'à une façade à 11 méthodes : chercher, aller_au_poste,
        │  accoster, prendre, poser, bloque_par_securite…
        ▼
FAÇADE                traduit ces méthodes en appels au robot
(tri_colis/facade.py)
        ▼
SERVICES DE L'OS      navigation, marche, bras, vision, sécurité
```

Pourquoi c'est la bonne structure : la **logique métier se teste sans
robot**, en quelques secondes, face à un monde factice qui provoque des
pannes (`test/test_tri_colis.py` : 30 minutes de tri simulées, prises
ratées, humains qui bloquent le passage, poste inaccessible). Un bug de
logique se trouve alors sur votre ordinateur, pas pendant une heure de GPU
louée.

### Ce que vous pouvez faire AUJOURD'HUI avec le SDK

La démo branche sa façade directement sur les services de l'OS
(navigation, manipulation). Ces deux services ne sont **pas encore
exposés** aux apps du Play Store (§ 12). Une app de tri tierce s'écrit
donc aujourd'hui avec `look_for`, `walk` et `move_to_pose`, en gardant le
même découpage :

```python
from axis_os.apps.tri_colis.logique import ConfigTri, TriColis


class FacadeCtx:
    """La façade de la logique de tri, écrite avec le SDK public.
    ESQUISSE : montre la structure, n'a pas tourné sur un robot."""

    def __init__(self, ctx):
        self.ctx = ctx

    def maintenant(self):
        return self.ctx.now()

    def chercher(self, couleur):
        from axis_os.apps.tri_colis.logique import Colis
        return [Colis(couleur, (d.position_3d.x, d.position_3d.y, d.position_3d.z),
                      d.confidence)
                for d in self.ctx.look_for(f"a {couleur} box", frame="world")
                if d.position_3d is not None]

    # aller_au_poste / accoster : ctx.walk(...) renouvelé, jusqu'à la cible
    # prendre / poser : ctx.move_to_pose({...}) avec les postures de bras
    # bloque_par_securite : l'état du gardien de proximité n'est pas
    #   encore exposé aux apps (§ 12) ; en attendant, rendre False et
    #   laisser l'OS arrêter le robot : si un gardien de proximité est
    #   branché sur la marche (voir la FAQ), il le fait de lui-même
    ...


class TriApp(RobotApp):
    def run_mission(self, ctx, params):
        tri = TriColis(FacadeCtx(ctx), ConfigTri(regles={"rouge": "A", "bleu": "B"}))
        while not tri.termine:
            tri.pas()
            ctx.sleep(0.1)
        ...
```

Pour les déplacements réels avec `look_for` et `walk`, partez de
[`axis_os/apps/warehouse_inspector/app.py`](../axis_os/apps/warehouse_inspector/app.py)
: validée dans Isaac Sim, elle cherche un colis, marche jusqu'à lui et
s'arrête devant.

---

## 11. Tester, puis publier

### Trois niveaux de test, du moins cher au plus cher

| Niveau | Comment | Coût | Ce qu'il prouve |
|---|---|---|---|
| 1 | `python3 -m axis_os.cli.main simulate` | secondes | l'app s'installe, démarre, termine sa mission |
| 2 | un test comme `test/test_app_assistant_ia.py` : robot virtuel `UnitreeG1HAL(backend="sim")`, vrais services avec des entrées scriptées | secondes | la logique, les cas d'erreur, les permissions |
| 3 | Isaac Sim, sur une machine GPU (`deploy/machine_gpu/`) | heures GPU | la physique : le robot marche, prend, pose vraiment |

Ne passez au niveau 3 que lorsque les niveaux 1 et 2 sont propres.

### Signer son app

Une signature prouve que le code publié est bien le vôtre et qu'il n'a pas
été modifié. Sur un vrai robot (profil `PRODUCTION`), on signe un
**paquet** : le code et son manifeste déclaré, ensemble. Le robot lit
ainsi votre manifeste sans exécuter une ligne de votre code avant que
l'utilisateur ait accepté :

```python
from pathlib import Path
from axis_os.store.runtime.manifeste_json import manifeste_texte
from axis_os.store.sdk.signing import generate_developer_keypair, sign_app_package
from app import MonApp                  # sur VOTRE machine

cles = generate_developer_keypair()     # UNE fois ; gardez private_key_b64 secrète
source = Path("app.py").read_text(encoding="utf-8")
manifeste = manifeste_texte(MonApp().manifest())       # JSON, sans signature
signature = sign_app_package(manifeste, source, cles.private_key_b64)
# à livrer : app.py, le texte du manifeste, la signature et cles.public_key_b64
# sur le robot :
#   runtime.install_source(source, manifeste=manifeste, code_signature=signature,
#                          developer_public_key=cles.public_key_b64)
```

La clé privée ne quitte jamais votre machine et ne va **jamais** dans un
dépôt Git. Un seul caractère modifié après signature, dans le code ou
dans le manifeste, et l'installation est refusée. C'est le manifeste
déclaré que le robot applique.

Votre clé doit figurer dans le fichier des clés approuvées du robot. La
**première** clé qui installe un `app_id` sur un robot le garde : une
autre clé ne pourra plus l'installer ni le mettre à jour. Pour publier une
nouvelle version : même clé, numéro de version **strictement** plus grand
(`MAJEUR.MINEUR.CORRECTIF`, chiffres sans zéro en tête), puis
`runtime.update_source(...)` avec les mêmes arguments. Un robot refuse une
version plus ancienne que la plus récente qu'il ait installée, même après
un redémarrage. Hors production, `sign_app_file` (le code seul) reste
accepté par `install()`.

### Publier

```bash
python3 -m axis_os.cli.main publish --dry-run
```

Aujourd'hui, `publish` **valide** l'app localement ; le registre en ligne
du Play Store n'est pas encore connecté.

---

## 12. Ce qui n'existe pas encore

Dit franchement, pour ne pas vous faire perdre de temps :

- **Isolation contre une app malveillante** : pas encore. Une app hostile
  peut aujourd'hui agir hors de la HAL
  ([SECURITY.md](../SECURITY.md#limites-connues), limite connue n° 1), et
  le cloisonnement des données par `app_id` n'est pas imposé par le
  système (constat 3 d'[isolation_apps.md](isolation_apps.md), qui donne
  aussi le plan de correction). N'installez que des apps dont vous avez
  relu le code.
- **Faire parler le robot** : pas de `ctx.speak()`. La synthèse vocale est
  la prochaine brique naturelle.
- **Navigation, manipulation et état de sécurité pour les apps** : les
  services existent et sont testés (ils font tourner la démo de tri), mais
  ne sont pas encore exposés dans `ctx` (`ctx.aller_vers(poste)`,
  `ctx.prendre(objet)`, « suis-je arrêté par la sécurité ? »…).
- **Filtrage réseau du bac à sable** : les permissions `NETWORK_*` sont
  déclaratives pour l'instant.
- **Registre en ligne du Play Store** : `publish` valide, sans envoyer.
- **Paiement** : la répartition 70/30 est calculée et affichée à chaque
  mission facturée, mais aucun paiement réel n'est encore branché.
- **Installation par `pip`** : pas encore de paquet ; on utilise
  `PYTHONPATH` (§ 2).
- **Communication entre robots** : conçue (`docs/fleet_manager_design.md`),
  pas codée.

---

## 13. FAQ

**Faut-il un robot pour développer une app ?**
Non. `simulate` et les tests du niveau 2 utilisent un robot virtuel. Seule
la validation physique finale demande Isaac Sim ou un vrai robot.

**Faut-il un GPU ?**
Non, sauf pour le niveau 3 (simulation physique) et pour faire tourner
localement de gros modèles d'IA. Une app qui appelle une IA en ligne n'en
a pas besoin.

**Mon app peut-elle faire tomber le robot ?**
Pas en marchant par `ctx.walk()` : l'équilibre est alors tenu par la
politique de marche de l'OS, et si l'app plante ou se fige, le robot
s'arrête en équilibre. En revanche, la permission dangereuse
`ACTUATORS_WRITE_LEGS` (que `walk()` exige aussi) permet de commander
directement les articulations des jambes, par `move_legs()` ou
`move_to_pose()` ; `ACTUATORS_WRITE_ALL` le permet aussi, par
`move_all()`. Le garde-fou vérifie alors les limites de chaque
articulation, pas l'équilibre : une consigne mal choisie peut faire
tomber le robot. Si le robot a un gardien de proximité (branché par
l'intégrateur, avec un lidar ; aujourd'hui dans la démo de tri
seulement), il ralentit puis arrête la marche près de tout obstacle
inexpliqué, humain ou non (les obstacles fixes de la carte du site ne
comptent pas), quoi que demande l'app.
Enfin, une app délibérément hostile peut aujourd'hui sortir de ce cadre
(limite décrite au § 1).

**Mon app peut-elle lire les données d'une autre app ?**
Pas par `ctx.recall()` : la mémoire est cloisonnée par `app_id`, que
l'app ne choisit pas. Mais ce cloisonnement n'est pas encore imposé par le
système : une app malveillante peut lire les fichiers des autres apps
([isolation_apps.md](isolation_apps.md), constat 3).

**Deux apps peuvent-elles tourner en même temps ?**
Plusieurs apps peuvent être installées et démarrées. Une seule mission à la
fois commande le robot : une seconde demande est refusée, avec la raison.

**Pourquoi mon app plante au démarrage avec `SANDBOX_ERROR` ?**
Presque toujours la règle d'or n° 5 : une classe définie dans une fonction,
un constructeur qui exige des arguments, ou des paramètres qui ne sont pas
des données JSON. Définissez tout au niveau du module.

**Pourquoi `listen()` rend-il presque toujours `None` ?**
Parce que la phrase ne contient pas tous les groupes de mots-clés d'une
intention déclarée. `[["va", "avance"], ["carton", "colis"]]` exige un mot
de CHAQUE groupe : « va vers le carton » passe, « le carton » seul ne passe
pas. C'est voulu : une phrase vague ne doit rien déclencher.

**Mon IA doit-elle être en Python ?**
Non. Avec le contrat HTTP du § 8, elle peut être écrite dans n'importe quel
langage et tourner n'importe où.

**Combien je gagne ?**
Vous fixez `base_price_eur` : c'est votre part. Le client paie
`base_price_eur` divisé par 0,7, arrondi au centime (1,40 € → 2,00 €),
dont 30 % pour le Play Store. Aucun paiement réel n'est encore branché
(§ 12).

**Mon app marchera-t-elle sur un autre robot que le G1 ?**
Si elle ne demande que des capacités (`WALK`, `CAMERA_RGB`, `SPEECH_IN`…)
et pas un modèle précis, oui : c'est tout l'intérêt d'un OS. Le Play Store
refuse l'installation sur un robot qui n'a pas ce que l'app exige. Les
prérequis matériels d'un robot pour AXIS-OS sont dans
[`docs/PREREQUIS_ROBOT.md`](PREREQUIS_ROBOT.md).
