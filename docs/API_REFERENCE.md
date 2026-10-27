# AXIS-OS App SDK — API reference

SDK API version **1.0.0** (`ctx.api_version`, `axis_os/store/sdk/version.py`).
Guides: [CREATE_AN_APP.md](CREATE_AN_APP.md) (English) ·
[CREER_UNE_APP.md](CREER_UNE_APP.md) (français). Examples:
[examples/](../examples/).

## Contents

1. [Conventions](#1-conventions) — the context, permissions, results, refusal codes
2. [Robot information and safety](#2-robot-information-and-safety) — `robot_info`, `has_capability`, `safety_status`
3. [High-level services](#3-high-level-services) — `go_to`, `pick`, `place`, `rest_pose`
4. [Mission lifecycle](#4-mission-lifecycle) — `should_stop`, `check_cancelled`, `report_progress`, `on_estop`, `on_alert`, `on_low_battery`, `ask_confirmation`
5. [Local teleoperation](#5-local-teleoperation) — `stream_targets`, `TargetStream`
6. [Animations](#6-animations) — format, `play_animation`, `stop_animation`
7. [Speech and settings](#7-speech-and-settings) — `speak`, `get_setting`
8. [Low-level APIs](#8-low-level-apis) — joints, walking, perception, voice, memory
9. [Versions](#9-versions) — `api_version`, `min_sdk_version`
10. [Testing and tools](#10-testing-and-tools) — `axis_os.testing`, the `axis` command

---

## 1. Conventions

### The context

Every hook of an app (`on_start`, `run_mission`, `on_stop`) receives `ctx`,
the app's only way to reach the robot. On a robot, the app runs in its own
process: `ctx` is a client that sends each call over a local JSON bridge to
the OS, which checks the call against a strict schema, then against the
app's permissions, then runs it through the system services and the
normalized HAL. An app never touches a service, a driver or an OS file
directly. The same code runs unchanged in a direct `RobotContext` (tests).

Units: metres, radians, seconds. Joint angles use the *normalized*
convention (same sign on every robot). `ctx.now()` and `ctx.sleep()` follow
the robot's clock (simulated time in simulation): use them instead of
`time.monotonic()`.

### Permissions

Each API requires a permission, declared in the app's manifest
(`AppManifest.permissions`) and checked on every call. A missing permission
raises `PermissionError` (code `PERMISSION_DENIED`).

- **NORMAL**: granted at install.
- **DANGEROUS**: everything that moves the robot. A human confirms it at
  install (in the `SIMULATION` profile, `accepter_tout_SIMULATION` grants
  it without a human — never on a real robot).

A high-level API never has more rights than the low-level APIs it replaces:
`play_animation` also requires the rights of `move_arms`,
`stream_targets("walk")` those of `walk`, and so on.

### Results

APIs whose failure is a normal outcome (navigation, grasping, a question to
a human…) return an `ActionResult`:

```python
r = ctx.go_to(2.0, 1.0, timeout_s=30)
if not r:                                  # bool(r) == r.ok
    ctx.log(f"{r.status} {r.code}: {r.message}")
```

| Field | Meaning |
|---|---|
| `ok` | `True` if the action succeeded |
| `status` | outcome, specific to the API (`ARRIVED`, `HELD`, `TIMEOUT`…) |
| `code` | a refusal code (below), `None` on success |
| `message` | human-readable explanation |
| `data` | API-specific values |

Programming errors raise exceptions instead, all subclasses of
`AxisRefusal` with a `code`: `InvalidArgumentError` (also a `ValueError`),
`PermissionError`, `EstopActiveError`, `MissionCancelled`,
`CapabilityMissingError`, `BusyError`. They cross the bridge unchanged: an
app sees the same exception in its process as in a direct test.

### Refusal codes

`axis_os.store.sdk.results.RefusalCode`:

| Code | Meaning |
|---|---|
| `PERMISSION_DENIED` | permission not declared in the manifest |
| `ESTOP_ACTIVE` | emergency stop active: nothing that moves the robot is accepted |
| `SAFETY_LIMIT` | a safety limit refused it (joint guard, fall, legs not still…) |
| `PROXIMITY_GUARD` | the proximity guard held the robot (a person too close) |
| `CAPABILITY_MISSING` | the robot lacks what this needs (no arms, no navigation, no voice…) |
| `QUOTA_EXCEEDED` | an app quota is exhausted |
| `INVALID_ARGUMENT` | NaN, infinite, out of bounds, wrong type… (raised) |
| `TIMEOUT` | the deadline passed |
| `CANCELLED` | stopped on request (human, OS, end of hook) |
| `BUSY` | the resource is in use (arms holding an object, a stream already open…) |
| `BLOCKED` | no path |
| `OUT_OF_REACH` | out of the arms' reach |
| `FAILED` | the action was tried and failed |
| `HUMAN_DECLINED` | the human said no |
| `HUMAN_UNAVAILABLE` | no human to answer |
| `STALE_TARGET` | stream target too old (or from the future) |
| `RATE_LIMITED` | stream targets too frequent |
| `OUT_OF_BOUNDS` | stream target beyond the stream's limits |
| `STREAM_STOPPED` | the stream is stopped: call `resume()` |

### Emergency stop

During an emergency stop, every API that moves the robot is refused
(`EstopActiveError`), and an action in progress is interrupted (the robot
stops). In the app's process, the OS then stops the process within 0.5 s;
only calls that move nothing are still served (`log`, `robot_info`,
`has_capability`, `safety_status`, `should_stop`, `report_progress`,
`stop_walking`, `stop_animation`, the stream's `metrics` and `close`…).

---

## 2. Robot information and safety

### `ctx.robot_info() -> RobotInfo`

Permission: `SYSTEM_INFO` (NORMAL). Model, manufacturer, OS version, SDK API
version, profile (`simulation`, `development`, `production`), each hardware
capability with its state (`available`, `degraded`, `absent`), the system
services present (`locomotion`, `perception`, `navigation`, `manipulation`,
`speech_in`, `speech_out`, `memory`, `battery`), useful limits
(`max_walk_speed_mps`, `max_turn_rate_radps`, `max_velocity_ratio`…) and the
battery. The battery is the driver's measurement, or `known=False`: the OS
never invents a charge.

```python
info = ctx.robot_info()
ctx.log(f"{info.manufacturer} {info.model}, battery: "
        f"{info.battery.percent if info.battery.known else 'unknown'}")
```

### `ctx.has_capability(name) -> bool`

Permission: `SYSTEM_INFO`. `True` if the hardware capability (`"arms"`,
`"legs"`, `"camera"`, `"speaker"`…) is available, or if the system service
(`"navigation"`, `"manipulation"`…) is present. Refusals:
`INVALID_ARGUMENT` (unknown name, not a text).

### `ctx.safety_status() -> SafetyStatus`

Permission: `SYSTEM_INFO`. `estop_active`, `proximity` (`clear`, `slowing`,
`stopped`, or `unavailable` without a guard or a reading yet) with its distance and reason, `fall_detected`,
`time_remaining_s` in the current hook, remaining persistent storage
(`storage_keys_remaining`, `storage_bytes_remaining`). Still served during
an emergency stop.

---

## 3. High-level services

These calls block until the outcome is known. Their deadline is also bounded
by the time left in the hook (never extended). They reuse the OS services
(navigation, locomotion, manipulation), always under the proximity guard,
the joint guard and the emergency stop.

### `ctx.go_to(x, y, theta=None, *, timeout_s) -> ActionResult`

Permission: `NAVIGATION_GOTO` (DANGEROUS). Walks to `(x, y)` on the map
(world frame, metres), around the map's obstacles, then turns to `theta`
(radians) if given. A still obstacle on the path is avoided when another way
exists; a person stops the robot (proximity guard).

| `status` | `code` | When |
|---|---|---|
| `ARRIVED` | — | arrived |
| `BLOCKED` | `BLOCKED` | no path, or goal inside an obstacle or off the map |
| `TIMEOUT` | `TIMEOUT` or `PROXIMITY_GUARD` | deadline passed (the guard held the robot) |
| `REFUSED` | `ESTOP_ACTIVE`, `CANCELLED`, `SAFETY_LIMIT` (fall), `CAPABILITY_MISSING` | interrupted, or no navigation |

The robot is stopped for every outcome but `ARRIVED`. Raises
`InvalidArgumentError` (NaN, infinite, coordinates beyond ±10 km, `theta`
beyond ±2π, `timeout_s` not in ]0, 600]) and `EstopActiveError`.

```python
r = ctx.go_to(1.5, 0.0, 0.0, timeout_s=60)
```

### `ctx.pick(target, *, size_m=None, timeout_s=15.0) -> ActionResult`

Permission: `MANIPULATION` (DANGEROUS). Grasps an object with both arms.
`target` is a `Detection` seen with `look_for(query, "world")` (then
`size_m=(width, depth, height)` is required: an image does not give an
object's size, and the OS does not guess it), or an
`ObjectSpec(center, size)` (`axis_os.store.sdk.results.object_spec`). The
manipulation service only closes the arms if the object is between the
hands, asks the proximity guard for a bounded exemption around it, then
checks the object followed before reporting it held.

| `status` | `code` |
|---|---|
| `HELD` | — |
| `REFUSED` | `OUT_OF_REACH` (move closer with `go_to`), `BUSY`, `SAFETY_LIMIT`, `CAPABILITY_MISSING`, `ESTOP_ACTIVE`, `CANCELLED` |
| `FAILED` | `FAILED` — grasp not confirmed: the arms reopen gradually |
| `TIMEOUT` | `TIMEOUT` |

Raises `InvalidArgumentError` (detection in the robot frame, missing size,
centre or size NaN / out of bounds, size outside [2 cm, 70 cm]).

### `ctx.place(target=None, *, timeout_s=15.0) -> ActionResult`

Permission: `MANIPULATION`. Puts the held object down. `target`: `None`
(lower to carrying height and open), a surface height in metres, an
`ObjectSpec` of the surface (its top), or a `Detection` in the world frame.
With a surface, the object goes down until it touches it before the arms
open: it is never dropped from above.

`PLACED`; `REFUSED` (`OUT_OF_REACH`: surface too high; `FAILED`: nothing
held; `CAPABILITY_MISSING`, `ESTOP_ACTIVE`, `CANCELLED`); `FAILED` with
`OUT_OF_REACH` (arms at their lowest without touching: the object stays
held); `TIMEOUT`.

### `ctx.rest_pose(*, duration_s=2.0) -> ActionResult`

Permissions: those of `move_to_pose` (`ACTUATORS_READ`,
`ACTUATORS_WRITE_ARMS` on the G1). Returns to the rest pose **defined by the
driver**, never by the app (G1: arms only, never a standing biped's legs),
in interpolated steps under the joint guard. `AT_REST`; `REFUSED`
(`CAPABILITY_MISSING`: no rest pose declared; `BUSY`: the arms hold an
object, `place()` first); `FAILED` (`SAFETY_LIMIT`); `TIMEOUT` (not enough
time left in the hook: the motion is refused rather than sped up).
`duration_s` in [0.5, 30].

---

## 4. Mission lifecycle

### `ctx.should_stop() -> bool` · `ctx.check_cancelled()`

Permission: `MISSION_LIFECYCLE` (NORMAL). `True` when a human or the OS
requests the mission to stop (`PlayStoreRuntime.request_stop(app_id,
source="human" | "os")`), when less than 2 s remain in the hook, or during
an emergency stop. `check_cancelled()` raises `MissionCancelled` (code
`CANCELLED`; the OS records the mission as cancelled), or
`EstopActiveError` during an emergency stop. A long action in progress
(`go_to`, `pick`…) also returns with `CANCELLED`.

This is **cooperative**: it lets the app put things down and return. It
does not replace the forced stop: the emergency stop cuts everything
anyway, and the runtime quota kills an app that does not return.

```python
for step in steps:
    if ctx.should_stop():
        ctx.place(None, timeout_s=10)
        break
    do(step)
```

### `ctx.report_progress(fraction, message="")`

Permission: `MISSION_LIFECYCLE`. `fraction` in [0, 1]; `message` cut to 200
characters, control characters replaced by spaces; the last 50 reports are
kept. The OS reads it with `PlayStoreRuntime.get_progress(app_id)`.
Refusals: `INVALID_ARGUMENT`.

### `ctx.on_estop(callback)` · `ctx.on_alert(callback)` · `ctx.on_low_battery(callback, *, threshold_percent=20.0) -> bool`

Callbacks run **in the app's process**, in a thread that polls the OS every
100 ms (no function crosses the bridge). A callback that raises is logged
and does not stop the others.

- `on_estop(cb)` — `MISSION_LIFECYCLE`. `cb()` once, as soon as an emergency
  stop occurs (also immediately when a call is refused for that reason).
  The robot is already stopped; the process will be stopped within 0.5 s:
  keep it short.
- `on_alert(cb)` — `WATCHDOG_READ`. `cb(alert)` for each new watchdog alert
  (`kind`, `severity`, `message`, `joint_id`), from the subscription on.
- `on_low_battery(cb, threshold_percent=20)` — `SYSTEM_INFO`. `cb(percent)`
  once each time the measured charge falls below the threshold. Returns
  `False`, without subscribing, if the driver does not measure the battery.

Refusals: `INVALID_ARGUMENT` (callback not callable, threshold outside
[1, 99]). Limit: in the app's process, events are delivered between two
blocking calls (during a 30 s `go_to`, an alert waits for it to return).

### `ctx.ask_confirmation(question, *, timeout_s=30.0) -> ActionResult`

Permission: `HUMAN_CONFIRMATION` (NORMAL). Asks the human present a yes/no
question, in the name of the app (the name shown comes from the manifest,
never from the app). The human interface exists only on the OS side: an
app can neither see it nor answer in the human's place.

| `status` | `code` |
|---|---|
| `CONFIRMED` | — (an explicit "yes": nothing else counts as one) |
| `DECLINED` | `HUMAN_DECLINED` |
| `TIMEOUT` | `TIMEOUT` (no answer is never a yes) |
| `REFUSED` | `HUMAN_UNAVAILABLE` (no human on this robot) |

Raises `InvalidArgumentError` (empty question, `timeout_s` not in ]0, 300])
and `EstopActiveError`. In simulation, `axis_os.testing.ScriptedHuman` gives
the answers set by the test.

---

## 5. Local teleoperation

### `ctx.stream_targets(kind, *, deadman_ms=200, max_rate_hz=50, max_age_ms=150, max_speed=None) -> TargetStream`

Permission: `TELEOP_STREAM` (DANGEROUS), plus the rights of the commands it
replaces — `"arms"`: `ACTUATORS_READ`, `ACTUATORS_WRITE_ARMS`; `"walk"`:
`LOCOMOTION_WALK`, `ACTUATORS_WRITE_LEGS`. **Local only**: the stream goes
through the app's bridge, never over the network.

| Parameter | Range | Meaning |
|---|---|---|
| `kind` | `"arms"`, `"walk"` | arm joint targets, or walking velocities `(vx, vy, wz)` |
| `deadman_ms` | [50, 1000] | without a valid target with the deadman held for this long: safe stop |
| `max_rate_hz` | [1, 100] | targets accepted per second at most |
| `max_age_ms` | [10, 1000] | maximum age of a target when it arrives |
| `max_speed` | arms: ]0, 2] rad/s (1); walk: ]0, 0.5] × the robot's max speed (0.3) | the stream's own speed limit |

The OS drops, never clips, and counts: targets too old or from the future
(`STALE_TARGET`), too frequent (`RATE_LIMITED`), out of bounds
(`OUT_OF_BOUNDS`: joint limits, a leg joint, a velocity beyond the stream's
limit). Deadman released, or no valid target for `deadman_ms` (stream lost,
excessive delay): **safe stop** — walking slows down at a bounded
deceleration then stops (the walking controller keeps the balance; the legs
are never cut), arms stop where they are. After a safe stop nothing passes
until the app calls `resume()`. Arms are driven through `SetpointFollower`
and `move_arms` (joint guard on every step), walking through `walk`
(proximity guard). Raises `InvalidArgumentError`, `EstopActiveError`,
`CapabilityMissingError`, `BusyError` (a stream already open).

### `TargetStream`

| Method | |
|---|---|
| `push(target, *, deadman, t=None) -> StreamAck` | `target`: `{joint: position}` or `(vx, vy, wz)`; `deadman`: `True` while the operator holds the button; `t`: robot time the target was read (default: now). `StreamAck(accepted, code, state)`. |
| `resume() -> ActionResult` | after a safe stop (`RESUMED`); refused during an emergency stop |
| `metrics() -> StreamMetrics` | `received`, `accepted`, `rejected` by code, `latency_ms_mean`, `latency_ms_max`, `jitter_ms`, `safe_stops`, `stop_reason`, `state` |
| `close()` | safe stop, then close (also at the end of the hook) |

```python
with ctx.stream_targets("walk", deadman_ms=200, max_speed=0.5) as stream:
    while not ctx.should_stop():
        velocity, button = gamepad.read()
        stream.push(velocity, deadman=button)
        time.sleep(0.02)
```

See [examples/teleop_local](../examples/teleop_local/). Out of scope:
teleoperation over the internet, WebRTC, video, VR headsets.

---

## 6. Animations

### Format

```json
{
  "format": "axis-animation", "format_version": 1,
  "name": "wave", "version": "1.0.0", "robots": ["G1"],
  "description": "...",
  "keyframes": [
    {"t": 0.0, "positions": {"right_shoulder_pitch_joint": 0.0}},
    {"t": 1.0, "positions": {"right_shoulder_pitch_joint": -1.5}}
  ]
}
```

Positions in radians (normalized convention), linear interpolation between
keyframes, per joint. Optional `"stationary_legs_still": true` — "robot at
rest, legs still": without it, any leg joint is refused; with it, legs must
not move from one keyframe to the next, the robot must be standing still
with the described leg posture, and the OS never commands them.

Everything is checked **before playing** (`axis_os.store.sdk.animation`:
`load_animation`, `validate_animation`): format, compatible robot, known
and animatable joints (arms), positions inside the joint limits with a
0.02 rad margin, velocities ≤ 2 rad/s (and the joint's own limit),
accelerations ≤ 10 rad/s², at most 500 keyframes and 60 s, 256 KiB. On the
command line: `axis animation validate <file>`.

The G1 ships `wave` and `point` (`axis_os/hal/drivers/unitree_g1/animations/`).
No head nod: the G1 23 DOF has no neck joint.

### `ctx.play_animation(animation, *, wait=True, timeout_s=None) -> ActionResult`

Permission: `ANIMATION_PLAY` (DANGEROUS), plus those of `move_arms`
(`ACTUATORS_READ`, `ACTUATORS_WRITE_ARMS`). `animation`: the name of an
animation shipped with the robot, a dict in the format above, or an
`Animation`. A file path is not accepted: the OS does not read files for an
app; an app loads its own with `load_animation()`. Each step goes through
`move_arms` (joint guard); the gesture pauses while the proximity guard
holds the robot. First a gentle approach (0.5 rad/s) from the measured pose
to the first keyframe.

`wait=True`: blocks until `PLAYED`, or `timeout_s` (default: the duration
plus 2 s, extended while paused, at most 120 s; then the gesture is stopped:
`TIMEOUT`). `wait=False`: returns `PLAYING` at once. Other outcomes:
`REFUSED` (`ESTOP_ACTIVE`, `CANCELLED`, `SAFETY_LIMIT`, `BUSY`,
`CAPABILITY_MISSING`), `STOPPED`, `FAILED` (`SAFETY_LIMIT`: a step refused
by the joint guard). Raises `InvalidArgumentError` (with the list of
problems) and `EstopActiveError`.

### `ctx.stop_animation() -> ActionResult`

Permission: `ANIMATION_PLAY`. The arms stop where they are (`STOPPED`);
`IDLE` if nothing is playing.

---

## 7. Speech and settings

### `ctx.speak(text, *, language="en") -> ActionResult`

Permission: `AUDIO_OUTPUT` (NORMAL). Says `text` (cut to 500 characters,
control characters replaced) through the OS's `ISpeechSynthesizer`
(`axis_os/services/speech/synthesis.py`; AXIS-OS ships no speech engine and
downloads no model — `FakeSpeechSynthesizer` records what is said).
`language`: `"en"`, `"fr"`, `"fr-FR"`… `SPOKEN`; `REFUSED`
(`CAPABILITY_MISSING`: no synthesizer, or language not supported);
`FAILED`. Raises `InvalidArgumentError` and `EstopActiveError`.

### Settings: `SettingSpec` and `ctx.get_setting(name)`

Declared in the manifest (`settings=[...]`, at most 32):

```python
SettingSpec("volume", "int", 5, min=0, max=10)
SettingSpec("language", "choice", "en", choices=("en", "fr"))
```

Types: `bool`, `int`, `float` (bounded by `min`/`max`), `str` (at most
`max_length`, no control characters), `choice`. The declaration is checked
by `AppManifest.validate()` (the default must respect it). The user picks a
value with `PlayStoreRuntime.set_setting(app_id, name, value)`, checked
against the declaration. `ctx.get_setting(name)` — permission
`APP_SETTINGS` (NORMAL) — returns that value, or the default; raises
`InvalidArgumentError` for an undeclared name.

---

## 8. Low-level APIs

| API | Permission | |
|---|---|---|
| `now()`, `sleep(s)` | — | robot time |
| `log(message)`, `get_mission_log()` | — | bounded, neutralized log |
| `remember(key, value)`, `recall(key, default=None)` | `STORAGE_PERSISTENT` | persistent memory, quota-bounded |
| `listen(duration_s=3.0)` | `SENSORS_MICROPHONE` | voice intent |
| `declare_intent(name, keywords, params=None, description="")` | `SENSORS_MICROPHONE` | |
| `look_for(query=None, frame="base")` | `SENSORS_CAMERA` | `Detection`s; `frame="world"` for `pick` |
| `describe_scene()` | `SENSORS_CAMERA` | |
| `walk(vx, vy, wz)`, `stop_walking()` | `LOCOMOTION_WALK` (+ `ACTUATORS_WRITE_LEGS` for `walk`) | velocity command, under the proximity guard |
| `get_full_state()`, `get_joint_state(joint_id)` | `ACTUATORS_READ` | |
| `get_imu()` | `SENSORS_IMU` | |
| `move_arms(commands)`, `move_legs(commands)`, `move_all(commands)` | `ACTUATORS_WRITE_ARMS` / `_LEGS` / `_ALL` | each command through the joint guard |
| `move_to_pose(target, duration_s=2.0)` | `ACTUATORS_READ` + write rights of the joints | interpolated steps |
| `subscribe_alerts(callback)` | `WATCHDOG_READ` | |
| `emergency_stop()` | `EMERGENCY_STOP` (DANGEROUS) | triggers the robot's emergency stop |

---

## 9. Versions

- `ctx.api_version` — the SDK API version this OS serves (SemVer).
- `AppManifest.min_sdk_version` — the minimum SDK API version the app needs.
  Installation is refused if the OS's API is older or of another MAJOR
  version.
- `AppManifest.sdk_version_min` — the minimum AXIS-OS version (unchanged).

The package `axis-os-sdk` (`pyproject.toml`, `pip install .`, not published
on PyPI) has the version of the API.

---

## 10. Testing and tools

### `axis_os.testing`

A light simulated robot (no GPU, no Isaac Sim) running the real services:
navigation, locomotion, manipulation, proximity guard, watchdog.

```python
from axis_os.testing import AppTestHarness, SimulatedObject, ScriptedHuman

with AppTestHarness(MyApp(), objects=[SimulatedObject("red box", (1.2, 0.0, 0.9))],
                    runtime={"human_interface": ScriptedHuman([True])}) as h:
    result = h.run({"object": "red"})
    assert result.success, result.error_message
    print(h.robot.pose, h.robot.voice.spoken)
```

- `SimulatedRobot(carte=None, start=(0, 0, 0), proximity_guard=True,
  manipulation=True, objects=(), battery_percent=None, grasp_succeeds=True)`
  — `add_person(x, y)`, `clear_people()`, `set_battery(percent)`, `fall()`,
  `pose`, `voice.spoken`, `make_runtime(**kw)`.
- `AppTestHarness(app, robot=None, *, runtime=None, **robot_kw)` — installs,
  starts, `run(params)`, stops. The app runs exactly as on a robot: in its
  own process, through the bridge, under its permissions and quotas.
- `ScriptedHuman(answers)` — answers `ask_confirmation` (`True`, `False`,
  `None` = no answer) and records the questions.

### The `axis` command

| Command | |
|---|---|
| `axis init <name>` | app skeleton (code, manifest, test on the simulated robot) |
| `axis validate [dir]` | manifest, permissions (and the extra rights high-level APIs need), capabilities, quotas against the production caps, `animations/*.json`, versions |
| `axis animation validate <file>` | an animation, exactly as the OS checks it before playing |
| `axis simulate` | a mission on the simulated G1 |
| `axis publish --dry-run` | local packaging check (no store server) |
