# Creating an app for AXIS-OS — the complete guide

*Version française : [CREER_UNE_APP.md](CREER_UNE_APP.md).*

This guide is for anyone who wants to create an application for a robot
running AXIS-OS: a developer, a company that has its own AI, a student.
**You do not need a robot or a GPU**: everything below is developed and
tested on an ordinary computer.

Everything described here exists and is tested in the repository. What
does not exist yet is stated explicitly (§ 12).

---

## Contents

1. [What is an AXIS-OS app?](#1-what-is-an-axis-os-app)
2. [What you need on your computer](#2-what-you-need-on-your-computer)
3. [Your first app in 5 minutes](#3-your-first-app-in-5-minutes)
4. [Anatomy of an app](#4-anatomy-of-an-app)
5. [The manifest: what the app declares](#5-the-manifest-what-the-app-declares)
6. [What the app can do: the `ctx` context](#6-what-the-app-can-do-the-ctx-context)
7. [The golden rules (learned the hard way)](#7-the-golden-rules-learned-the-hard-way)
8. [Full example: connecting YOUR AI, without moving the robot](#8-full-example-connecting-your-ai-without-moving-the-robot)
9. ["I built an AI to make the robot speak better": where do I connect it?](#9-i-built-an-ai-to-make-the-robot-speak-better-where-do-i-connect-it)
10. [Example: a parcel sorting app](#10-example-a-parcel-sorting-app)
11. [Test, then publish](#11-test-then-publish)
12. [What does not exist yet](#12-what-does-not-exist-yet)
13. [FAQ](#13-faq)

---

## 1. What is an AXIS-OS app?

The same principle as an Android app on a phone:

| Android phone | Robot running AXIS-OS |
|---|---|
| Android manages the screen, the network, the battery | AXIS-OS manages the motors, balance, safety |
| An app asks for "camera access" | An app asks for `SENSORS_CAMERA`, `LOCOMOTION_WALK`… |
| The app cannot make the battery explode | Through `ctx`, the app commands no motor without the OS's control: permissions, safety guard (see the limitation below) |
| Google Play | the AXIS-OS Play Store |

The central idea: **the app decides WHAT to do, the OS decides whether it
is safe and HOW to do it.** An app never commands a motor directly:

```
votre app  ──>  RobotContext (ctx)  ──>  services de l'OS  ──>  garde-fou  ──>  moteurs
             permissions vérifiées      marche, vision,       JointSafetyGuard,
             à CHAQUE appel             parole, mémoire       gardien de proximité
```

Each app runs in a **sandbox** (a separate process, with memory, CPU and
duration limits). If it crashes or hangs, the robot stays standing and the
other apps keep running.

> **Current limitation.** This sandbox protects the robot from a buggy
> app, not yet from a malicious one. The guarantees in this guide apply to
> an app that goes through `ctx`. A deliberately hostile app can currently
> step outside this framework and act outside the HAL, and therefore
> outside the safety guard: see [SECURITY.md](../SECURITY.md#limites-connues)
> (known limitation no. 1). The fix is planned in
> [isolation_apps.md](isolation_apps.md). In the meantime, on a real robot,
> only install apps whose code you have reviewed.

---

## 2. What you need on your computer

- Python **3.12** or newer (the version CI tests);
- the AXIS-OS repository, and its basic dependencies:

```bash
git clone https://github.com/OS-AXIS/AXIS-OS.git
cd AXIS-OS
pip install -r requirements.txt numpy
export PYTHONPATH="$PWD"          # pour que « import axis_os » fonctionne
```

Or install the development kit, the `axis-os-sdk` package (not published
on PyPI; it installs from the repository and provides the `axis`
command):

```bash
pip install .
```

No robot, no GPU, no 3D simulator to get started.

---

## 3. Your first app in 5 minutes

The command-line tool generates the skeleton of an app:

```bash
axis init "Mon Assistant" --author "Kenzo" --email "moi@exemple.com"
cd mon_assistant
python test_app.py      # the app's test, on the light simulated robot
axis validate           # checks the app as the OS will
axis simulate           # a mission on the simulated G1
```

(Without `pip install .`, write `python3 -m axis_os.cli.main` instead of
`axis`.) The generated `app.py` already shows the habits of a good app:
ask the robot what it can do (`ctx.has_capability`), honour a stop request
(`ctx.should_stop`) and report progress (`ctx.report_progress`).

Expected output:

```
[PlayStore] Installation 'Mon Assistant' v0.1.0
  ✓ 'Mon Assistant' installée.
  [com.kenzo.mon_assistant] Prête.
[PlayStore] ▶ Mission FD8FF336 | app=com.kenzo.mon_assistant
  [Mission FD8FF336] ✓ SUCCESS
✓ Simulation réussie
```

`simulate` installs the app on a **virtual robot** (a G1 without a 3D
simulator, each joint following simplified physics), starts it in its
sandbox and has it run a mission. Your app ran exactly as it will run on
a real robot, except that no motor exists.

Generated files:

```
mon_assistant/
├── app.py          ← your code: the manifest + 3 methods
├── manifest.json   ← metadata read by simulate and publish; the
│                     manifest to ship with the code is
│                     manifeste_texte(MyApp().manifest()) ("Signing your app")
├── test_app.py     ← a test that installs and runs the app
└── README.md
```

---

## 4. Anatomy of an app

An app is **a class** that inherits from `RobotApp` and defines four
methods:

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

The lifecycle, managed by the Play Store:

```
install(app)        manifeste vérifié : format, version du SDK, signature,
                    permissions, matériel du robot  ── refus motivé sinon
start_app(app_id)   -> on_start(ctx)                (dans le bac à sable)
run_mission(...)    -> run_mission(ctx, params)     (autant de fois qu'on veut)
stop_app(app_id)    -> on_stop(ctx)
```

`MissionResult` is the report: `success`, `items_processed` (the number
of things handled: parcels, questions…), `error_message` on failure, and
`metrics`, a free-form dictionary for your results.

---

## 5. The manifest: what the app declares

The manifest is a **contract**: the user sees it before installing, and
the OS enforces it at run time.

### The fields

| Field | Role |
|---|---|
| `app_id` | unique identifier, format `com.auteur.nom`: lowercase ASCII, digits and `_` |
| `name`, `version`, `author`, `author_email`, `description`, `category` | what the user sees |
| `permissions` | what the app is allowed to do (next table) |
| `capabilities_required` | what the robot must be capable of: `WALK`, `CAMERA_RGB`, `CAMERA_DEPTH`, `SPEECH_IN`, `GRASP_DUAL`, `NAVIGATION_MAP`, `MEMORY_PERSISTENT`… A robot that lacks it: **installation refused**, with the reason |
| `hardware` | fine-grained hardware requirements: `required`/`optional`, minimum number of joints, compatible models |
| `quota` | sandbox limits: `max_cpu_percent`, `max_memory_mb`, `max_runtime_s` (30 s by default: increase it for a long mission). Finite, strictly positive numbers, under the OS ceilings (`PLAFONDS_QUOTAS`: 100 %, 1,024 MB and 3,600 s in production; 400 %, 8,192 MB and 7,200 s in simulation and development): above them, installation is refused. They are shown to the user who accepts your permissions |
| `max_velocity_ratio` | arm speed cap, between 0 and 1 |
| `pricing_model`, `base_price_eur` | free, one-time purchase, subscription, or pay per mission. The Play Store takes 30%: for you to receive €1.40, the customer pays €2.00 |
| `code_signature`, `developer_public_key` | code signature (§ 11) |

### The permissions

Principle: **only ask for what you need.** An app that asks to make the
robot walk when it answers questions will, rightly, be refused by the
user.

| Permission | What it allows | Level |
|---|---|---|
| `ACTUATORS_READ` | read joint positions | normal |
| `SENSORS_IMU` | read the inertial measurement unit | normal |
| `SENSORS_CAMERA` | `look_for`, `describe_scene` | normal |
| `SENSORS_MICROPHONE` | `listen`, `declare_intent` | normal |
| `STORAGE_PERSISTENT` | `remember`, `recall` (memory private to the app, at most 1,000 keys and 1 MiB) | normal |
| `WATCHDOG_READ` | receive safety alerts | normal |
| `PEOPLE_DETECT` | `people`, `person_in_front`: temporary numbers, distances, directions — never an identity (shown to the human at install: "this app detects the presence of people") | normal |
| `PEOPLE_FOLLOW` | `follow`, `follow_status`, `stop_follow` (also needs `PEOPLE_DETECT` and the rights of `walk`) | **dangerous** |
| `ACTUATORS_WRITE_ARMS` / `_LEGS` / `_ALL` | command the arms, the legs, everything | **dangerous** |
| `LOCOMOTION_WALK` | make the robot move around | **dangerous** |
| `EMERGENCY_STOP` | trigger the emergency stop | **dangerous** |
| `ACTUATORS_AI_DRIVEN` | movements decided by an AI (not a script) — confirmed at installation, and required at run time for the commands of the SDK's AI actions | **dangerous** |
| `ACTUATORS_RAW` | raw motor commands | **reserved** for apps signed by AXIS-OS |
| `FLEET_BROADCAST` | talk to other robots | **reserved** |

**Dangerous** permissions require EXPLICIT confirmation at installation:
without it, the app is refused. The robot provides this confirmation
through `PlayStoreRuntime(confirmer_permissions=...)` —
`confirmer_au_terminal` on a real robot (only "o" or "oui" grants),
`accepter_tout_SIMULATION` in simulation and in tests (refused on a real
robot and under the `PRODUCTION` profile). An app that requires `Capability.AI_POLICY` must declare
`ACTUATORS_AI_DRIVEN`, otherwise its manifest is invalid (the runtime
does not provide `AI_POLICY` yet: this rule will apply to the future VLA
service). Declare `ACTUATORS_AI_DRIVEN` as soon as an AI decides the
movements: the commands produced by `VLAAction`, `GR00TAction` or
`LocomotionAction` (`to_commands()`, `move_to_pose(ctx)`, and `walk(ctx)`
for a walking speed) are refused at run time without it, and the user
must know before accepting. Go through these methods rather than copying
the model's numbers into `ctx.walk()` or `ctx.move_to_pose()`: they are
what carries the "AI" mark. **Reserved** permissions, for their part, are refused
to any app that is not signed with the platform key.

To be honest about the current state: `NETWORK_CLOUD`, `NETWORK_LOCAL`,
`SENSORS_FORCE_TORQUE` and `FLEET_BROADCAST` exist but **are not enforced
yet** (the sandbox does not filter the network). Declare them anyway if
your app uses them: the user must know that your app talks to the
Internet.

---

## 6. What the app can do: the `ctx` context

Everything goes through the `ctx` object, received in `on_start`,
`run_mission` and `on_stop`. **Every call checks the permission**:
without it, the call raises `PermissionError`. No `ctx` call lets you
bypass this check; a hostile app that steps outside `ctx` falls under
the limitation described in § 1.

| Call | Permission | What it does |
|---|---|---|
| `ctx.log(texte)` | — | mission log |
| `ctx.now()`, `ctx.sleep(s)` | — | the **robot's** time (simulated in simulation) |
| `ctx.look_for("a red box", frame="world")` | camera | detected objects, sorted by confidence; 3D position if there is a depth camera |
| `ctx.describe_scene()` | camera | what the robot sees, in plain language |
| `ctx.declare_intent(nom, mots_cles)` | microphone | adds vocabulary to your app (at most 256 intents; another app can declare the same name without getting in your way) |
| `ctx.listen(duree_s)` | microphone | a recognized **intent** (`.name`, `.params`, `.utterance` = the sentence), or `None` |
| `ctx.remember(cle, valeur)`, `ctx.recall(cle)` | storage | memory that survives a restart, partitioned per app (a logical partitioning: see the FAQ), at most 1,000 keys and 1 MiB: beyond that, `remember` returns `False` |
| `ctx.walk(vx, vy, wz)`, `ctx.stop_walking()` | walking | movement; the command must be **renewed** |
| `ctx.move_to_pose({articulation: angle}, duree_s)` | arms / legs | gradual movement, from the actual position |
| `ctx.move_arms(...)`, `ctx.move_legs(...)` | arms / legs | joint commands, checked by the safety guard |
| `ctx.get_full_state()` | read | position and velocity of each joint |
| `ctx.emergency_stop()` | emergency stop | stops the whole robot, and interrupts the running mission |
| `ctx.subscribe_alerts(fonction)` | alerts | be notified of safety alerts |

A service missing from the robot (no camera, no microphone) is **not an
error**: the call returns an empty list, `None` or `False`, and says so in
the log.

### SDK v1: high-level APIs

Each API below is detailed — parameters, permission, refusal codes,
example — in **[API_REFERENCE.md](API_REFERENCE.md)**. Actions that can
fail normally return an `ActionResult` (`ok`, `status`, `code`,
`message`): `if not r: ctx.log(r.code)`. Everything that moves the robot
needs a DANGEROUS permission, confirmed by a human at install, and is
refused during an emergency stop.

| Call | Permission | What it does |
|---|---|---|
| `ctx.robot_info()`, `ctx.has_capability(name)`, `ctx.safety_status()` | `SYSTEM_INFO` | model, versions, capabilities and their state, battery (never invented), emergency stop, proximity guard, quotas left |
| `ctx.go_to(x, y, theta, timeout_s=…)` | `NAVIGATION_GOTO` + those of `walk` | walks to a point of the map, around obstacles, under the proximity guard |
| `ctx.pick(detection, size_m=…)`, `ctx.place(surface)` | `MANIPULATION` + those of the arms | grasps and puts down an object with both arms |
| `ctx.rest_pose()` | arms | the rest pose defined by the driver |
| `ctx.should_stop()`, `ctx.check_cancelled()`, `ctx.report_progress(f, msg)` | `MISSION_LIFECYCLE` | cooperative stop, progress read by the OS |
| `ctx.on_estop(f)`, `ctx.on_alert(f)`, `ctx.on_low_battery(f)` | `MISSION_LIFECYCLE`, `WATCHDOG_READ`, `SYSTEM_INFO` | callbacks, in the app's process |
| `ctx.ask_confirmation(question)` | `HUMAN_CONFIRMATION` | a yes/no question to the human present; the app can never answer it itself |
| `ctx.stream_targets("arms" \| "walk")` | `TELEOP_STREAM` | local teleoperation stream under a deadman |
| `ctx.play_animation("wave")`, `ctx.stop_animation()` | `ANIMATION_PLAY` | keyframe gestures, checked before playing |
| `ctx.speak(text, language=…)` | `AUDIO_OUTPUT` | the robot speaks |
| `ctx.get_setting(name)` | `APP_SETTINGS` | a setting declared in the manifest, chosen by the user |
| `ctx.people()`, `ctx.person_in_front()` | `PEOPLE_DETECT` | the people around: a temporary number, a distance, a direction. No face recognition, no identity, no image kept |
| `ctx.follow(person_id)`, `ctx.follow_status()`, `ctx.stop_follow()` | `PEOPLE_FOLLOW` + `PEOPLE_DETECT` + those of `walk` | the OS follows the person: keeps its distance (1.5 m), slowly, never backwards, under the proximity guard; `LOST` rather than the wrong person. Non-blocking: call `follow_status()` regularly (dead man) |

Complete, tested examples: [examples/](../examples/) — `hello_robot`
(animation + speech), `fetch` (look_for + go_to + pick + place),
`teleop_local` (deadman stream), `follow_me` ("follow me" by voice).

---

## 7. The golden rules (learned the hard way)

Each one comes from a real bug, met while developing the reference apps.

1. **Measure time with `ctx.now()` and wait with `ctx.sleep()`, never
   with `time`.** In simulation, the robot's time is not your wall-clock
   time. An app that listened for "two seconds" of wall-clock time, in a
   slowed-down simulation, made its decisions on a stale image.
2. **`None` is a normal answer.** `listen()` returns `None` when nobody
   is speaking; `look_for()` returns `[]` when nothing is visible. Treat
   it as an ordinary case, never as an error.
3. **To store a position, use `frame="world"`.** In the robot's frame,
   two parcels seen while turning in place can land on the same spot: an
   inventory app merged two parcels located 90° apart.
4. **`walk()` is not a fire-and-forget command.** Renew the command
   regularly; if the app goes silent, the robot stops on its own, balanced.
   This is intended.
5. **Define your class at module level**, never inside a function, and
   give it a constructor without arguments. Each call to your app runs in a
   fresh Python process that reloads it from its module; it cannot find a
   local class, and it only copies the attributes of the installed instance
   that are plain data. Mission parameters and `metrics` must be JSON data
   too (numbers, strings, lists, dicts).
6. **The safety guard may refuse your commands**, and it is right to: too
   fast, too far, beyond the joint limits. `move_arms()` returns the status
   of each joint; read it. No `ctx` call lets you bypass it, and the API
   will not offer one.
7. **A voice intent never commands a motor.** `listen()` returns an
   intent; it is **your code** that decides what it triggers. A sentence
   that no app has declared produces nothing, and your app only receives
   the intents it declared itself.
8. **An emergency stop ends your mission.** As soon as it happens, every
   `ctx` call raises `EstopActiveError`, except `log`,
   `get_recent_alerts`, `stop_walking` and `emergency_stop` (you can still
   log and stop walking); you have half a second to return
   your result, then your process is stopped. The mission is recorded as
   "interrupted by emergency stop" and is never billed. Do not count on
   that delay to finish a job.
9. **Fail and say so.** A clear `MissionResult(success=False,
   error_message="…")` is better than an app that makes up a plausible
   result.

---

## 8. Full example: connecting YOUR AI, without moving the robot

The simplest and most common case: a company has its own AI (an
assistant, a knowledge base, an in-house model) and wants people to be
able to talk to it **through the robot**. The robot does not need to
move.

The complete, tested code is in the repository:
- the app: [`axis_os/apps/assistant_ia/app.py`](../axis_os/apps/assistant_ia/app.py)
- its test, with no robot or GPU: [`test/test_app_assistant_ia.py`](../test/test_app_assistant_ia.py)

### What it does

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

### 1. The manifest: the bare minimum

```python
permissions=[
    Permission.SENSORS_MICROPHONE,   # écouter
    Permission.STORAGE_PERSISTENT,   # retenir le nombre d'échanges
    Permission.NETWORK_CLOUD,        # dire à l'utilisateur qu'on parle à Internet
],
capabilities_required=[Capability.SPEECH_IN],   # robot sans micro : refusée
quota=ResourceQuota(max_runtime_s=120.0),       # une conversation dure
```

No movement permission: **the app cannot make the robot move**, not even
by mistake, not even if your AI asked it to.

### 2. A wake word

```python
def on_start(self, ctx):
    ok = ctx.declare_intent("parler_a_l_ia", [["robot", "axis"]])
    ok &= ctx.declare_intent("fin_conversation", [["au revoir", "termine"]])
    return ok
```

Only sentences containing "robot" or "axis" are sent to the AI. A
conversation between two colleagues next to the robot is **never sent
online** (the test checks this).

### 3. Your AI, behind a web address

The app does not depend on any AI library. It talks to **any service**
that follows this minimal contract:

```
POST <votre adresse>
     {"question": "Robot, combien de palettes au quai 3 ?",
      "historique": [["question précédente", "réponse précédente"]]}
réponse
     {"reponse": "Il y en a 12."}
```

Behind it, you put whatever you want: your in-house model, a hosted
model, an agent that queries your inventory software. For offline
development, the app includes a **test AI** that answers without a
network.

### 4. The conversation loop

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

### 5. Running it

```bash
python3 test/test_app_assistant_ia.py
```

The test sets up a virtual robot, the real Play Store with its sandbox,
the real speech service (with scripted sentences in place of the
microphone), and a "company AI" simulated by a small local server. It
checks that: questions reach the AI with their history; the neighboring
conversation never reaches it; "au revoir" ends the conversation; an
unreachable AI gives an **explained** failure, with no made-up answer; a
robot without a microphone refuses to install the app.

To connect your real AI: `run_mission(..., params={"ia_url":
"https://votre-ia.exemple.com/robot"})`.

---

## 9. "I built an AI to make the robot speak better": where do I connect it?

It all depends on what your AI does: each case connects at a different
layer.

| Your AI… | Where it connects | Status |
|---|---|---|
| **converses**: understands a question, formulates an answer | in an **app**, as in § 8 | ✅ available |
| **recognizes speech better**: better transcription, accents, factory noise | in the **OS**: implement `ISpeechRecognizer` (one method, `transcribe(audio) -> (texte, confiance)`), in place of Whisper. All apps benefit at once | ✅ interface available; connected by the robot integrator, not by a Play Store app |
| **gives the robot a voice**: speech synthesis | the OS has the `SPEECH_OUT` capability, but the SDK does **not yet** expose `ctx.speak()` | ❌ not yet (§ 12) |
| **decides movements**: a vision-language-action model | `IVLAConnector` (`axis_os/store/sdk/ai/connector.py`), with the `ACTUATORS_AI_DRIVEN` permission in addition to the movement permissions. Every command still goes through the safety guard | ⚠️ interface available, never validated on a robot |

The principle is the same everywhere: **an AI proposes, accountable code
decides, the OS checks.** No model, however good, has a direct path to a
motor.

---

## 10. Example: a parcel sorting app

Sorting parcels means: finding a parcel, reading its destination (color,
label), picking it up, carrying it to the right station, putting it
down, starting again — and knowing what to do when it fails.

### The architecture lesson from the demo

The AXIS-OS sorting demo
([`docs/demo_tri_usine.md`](demo_tri_usine.md)) separates three things,
and this split is what you should reproduce:

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

Why this is the right structure: the **business logic is tested without
a robot**, in a few seconds, against a fake world that causes failures
(`test/test_tri_colis.py`: 30 minutes of simulated sorting, failed
grasps, humans blocking the way, unreachable station). A logic bug is
then found on your computer, not during an hour of rented GPU time.

### What you can do TODAY with the SDK

The demo connects its facade directly to the OS services (navigation,
manipulation). These two services are **not yet exposed** to Play Store
apps (§ 12). A third-party sorting app is therefore written today with
`look_for`, `walk` and `move_to_pose`, keeping the same split:

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

For real movement with `look_for` and `walk`, start from
[`axis_os/apps/warehouse_inspector/app.py`](../axis_os/apps/warehouse_inspector/app.py):
validated in Isaac Sim, it looks for a parcel, walks up to it and stops
in front of it.

---

## 11. Test, then publish

### Three levels of testing, from cheapest to most expensive

| Level | How | Cost | What it proves |
|---|---|---|---|
| 0 | `axis validate` | a second | manifest, permissions, quotas, animations, versions: what the OS will refuse |
| 1 | `python test_app.py`: `axis_os.testing.AppTestHarness`, the light simulated robot (real navigation, locomotion, manipulation, proximity guard), or `axis simulate` | seconds | the app installs, starts, completes its mission; error cases, permissions |
| 2 | a test like `test/test_app_assistant_ia.py`: virtual robot `UnitreeG1HAL(backend="sim")`, real services with scripted inputs | seconds | the logic, the error cases, the permissions |
| 3 | Isaac Sim, on a GPU machine (`deploy/machine_gpu/`) | GPU hours | the physics: the robot really walks, picks up, puts down |

Only move to level 3 once levels 1 and 2 are clean.

### Signing your app

A signature proves that the published code is really yours and has not
been modified. On a real robot (`PRODUCTION` profile), you sign a
**package**: the code and its declared manifest, together. The robot thus
reads your manifest without running a line of your code before the user
has accepted:

```python
from pathlib import Path
from axis_os.store.runtime.manifeste_json import manifeste_texte
from axis_os.store.sdk.signing import generate_developer_keypair, sign_app_package
from app import MonApp                  # on YOUR machine

cles = generate_developer_keypair()     # ONCE; keep private_key_b64 secret
source = Path("app.py").read_text(encoding="utf-8")
manifeste = manifeste_texte(MonApp().manifest())       # JSON, without signature
signature = sign_app_package(manifeste, source, cles.private_key_b64)
# to ship: app.py, the manifest text, the signature and cles.public_key_b64
# on the robot:
#   runtime.install_source(source, manifeste=manifeste, code_signature=signature,
#                          developer_public_key=cles.public_key_b64)
```

The private key never leaves your machine and **never** goes into a Git
repository. A single character changed after signing, in the code or in
the manifest, and the installation is refused. The robot applies the
declared manifest.

Your key must be in the robot's approved-keys file. The **first** key
that installs an `app_id` on a robot keeps it: another key can no longer
install or update it. To publish a new version: same key, a **strictly**
greater version number (`MAJOR.MINOR.PATCH`, digits without leading
zeros), then `runtime.update_source(...)` with the same arguments. A robot
refuses a version older than the newest it has installed, even after a
restart, and different content under a version number already installed:
any change, even one line, needs a new number. A version that changes the
pricing model or raises the price must be accepted by the human who
installs it. `update_source()` updates an app installed since the OS
started; after a restart, the new version is installed with
`install_source()`, and its price is then not compared with the old one
(`SECURITY.md`, limit n° 7). Outside production, `sign_app_file` (the code alone) is still
accepted by `install()`.

### Publishing

```bash
python3 -m axis_os.cli.main publish --dry-run
```

Today, `publish` **validates** the app locally; the Play Store's online
registry is not connected yet.

---

## 12. What does not exist yet

Stated plainly, so you do not waste your time:

- **Isolation against a malicious app**: not yet. A hostile app can
  currently act outside the HAL
  ([SECURITY.md](../SECURITY.md#limites-connues), known limitation no. 1),
  and data partitioning by `app_id` is not enforced by the system
  (finding 3 of [isolation_apps.md](isolation_apps.md), which also gives
  the fix plan). Only install apps whose code you have reviewed.
- **A real speech engine**: `ctx.speak()` goes through the
  `ISpeechSynthesizer` interface, but AXIS-OS ships no engine (tests use a
  fake one that records what is said).
- **A gamepad for `stream_targets`**: the stream is local and works, but no
  OS input service reads a real gamepad yet (an app never reads a device
  itself). Teleoperation over the internet, video and VR are out of scope.
- **Sandbox network filtering**: the `NETWORK_*` permissions are
  declarative for now.
- **Online Play Store registry**: `publish` validates, without uploading.
- **Payment**: the 70/30 split is calculated and displayed for each
  billed mission, but no real payment is connected yet.
- **Installation from PyPI**: the `axis-os-sdk` package installs from the
  repository (`pip install .`, § 2), but is not published.
- **Robot-to-robot communication**: designed
  (`docs/fleet_manager_design.md`), not coded.

---

## 13. FAQ

**Do I need a robot to develop an app?**
No. `simulate` and the level 2 tests use a virtual robot. Only the final
physical validation requires Isaac Sim or a real robot.

**Do I need a GPU?**
No, except for level 3 (physics simulation) and for running large AI
models locally. An app that calls an online AI does not need one.

**Can my app make the robot fall?**
Not when walking through `ctx.walk()`: balance is then maintained by the
OS's walking policy, and if the app crashes or hangs, the robot stops
balanced. However, the dangerous permission `ACTUATORS_WRITE_LEGS`
(which `walk()` also requires) allows direct control of the leg joints,
through `move_legs()` or `move_to_pose()`; `ACTUATORS_WRITE_ALL` also
allows it, through `move_all()`. The safety guard then checks each
joint's limits, not balance: a poorly chosen command can make the robot
fall. If the robot has a proximity guard (connected by the
integrator, with a lidar; today only in the sorting demo), it slows down
and then stops walking near any unexplained obstacle, human or not
(fixed obstacles on the site map do not count), whatever the app asks.
Finally, a deliberately hostile app can currently step outside this
framework (limitation described in § 1).

**Can my app read another app's data?**
Not through `ctx.recall()`: memory is partitioned by `app_id`, which the
app does not choose. But this partitioning is not yet enforced by the
system: a malicious app can read other apps' files
([isolation_apps.md](isolation_apps.md), finding 3).

**Can two apps run at the same time?**
Several apps can be installed and started. Only one mission at a time
commands the robot: a second request is refused, with the reason.

**Why does my app crash at startup with `SANDBOX_ERROR`?**
Almost always golden rule no. 5: a class defined inside a function, a
constructor that requires arguments, or parameters that are not JSON data.
Define everything at module level.

**Why does `listen()` almost always return `None`?**
Because the sentence does not contain all the keyword groups of a
declared intent. `[["va", "avance"], ["carton", "colis"]]` requires one
word from EACH group: "va vers le carton" passes, "le carton" alone does
not. This is intended: a vague sentence must not trigger anything.

**Does my AI have to be in Python?**
No. With the HTTP contract from § 8, it can be written in any language
and run anywhere.

**How much do I earn?**
You set `base_price_eur`: that is your share. The customer pays
`base_price_eur` divided by 0.7, rounded to the cent (€1.40 → €2.00),
of which 30% goes to the Play Store. No real payment is connected yet
(§ 12).

**Will my app work on a robot other than the G1?**
If it only asks for capabilities (`WALK`, `CAMERA_RGB`, `SPEECH_IN`…)
and not a specific model, yes: that is the whole point of an OS. The
Play Store refuses installation on a robot that lacks what the app
requires. The hardware requirements for a robot to run AXIS-OS are in
[`docs/PREREQUIS_ROBOT.md`](PREREQUIS_ROBOT.md).
