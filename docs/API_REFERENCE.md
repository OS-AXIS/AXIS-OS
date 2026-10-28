# AXIS-OS App SDK — API reference

SDK API version **1.2.0** (`ctx.api_version`, `axis_os/store/sdk/version.py`).
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
8. [People: presence and following](#8-people-presence-and-following) — `people`, `person_in_front`, `follow`, `follow_status`, `stop_follow`
9. [Position, places and zones](#9-position-places-and-zones) — `where_am_i`, `places`, `zones`, `go_to(place)`, `on_enter_zone`, `on_exit_zone`, `required_places`
10. [Notifications, sounds and network](#10-notifications-sounds-and-network) — `notify`, `play_sound`, `network_status`
11. [Scheduled missions](#11-scheduled-missions) — `triggers`, the scheduler's rules
12. [Energy and hardware health](#12-energy-and-hardware-health) — return to the charging station, `battery`, `motors_health`, `thermal_status`, `on_thermal_change`, crash reports
13. [Low-level APIs](#13-low-level-apis) — joints, walking, perception, voice, memory
14. [Versions](#14-versions) — `api_version`, `min_sdk_version`
15. [Testing and tools](#15-testing-and-tools) — `axis_os.testing`, the `axis` command

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
on top of its own permission, `go_to` also requires the rights of `walk`,
`pick`/`place`/`play_animation` those of `move_arms`,
`stream_targets("walk")` those of `walk`, and so on.

One context drives the robot at a time through the high-level APIs: while
an app has a `go_to`, a grasp, a rest pose, an animation, a stream or a
`follow` in progress, another app's hook (an `on_start` during a mission,
say) gets `BUSY`. Within one app, the arms have one writer at a time (grasp,
animation or arm stream), and walking has one too (`go_to`, a walk stream
or `follow`).

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
| `TARGET_LOST` | the followed person was lost, or the person number is unknown |
| `DEADMAN` | the app stopped giving news (`follow_status`) within `deadman_s` |
| `FORBIDDEN_ZONE` | the goal is inside a forbidden zone of the site map |
| `RATE_LIMITED_APP` | too many notifications or sounds from this app |

### Emergency stop

During an emergency stop, every API that moves the robot is refused
(`EstopActiveError`), and an action in progress is interrupted (the robot
stops). In the app's process, the OS then stops the process within 0.5 s;
only calls that move nothing are still served (`log`, `robot_info`,
`has_capability`, `safety_status`, `should_stop`, `report_progress`,
`stop_walking`, `stop_animation`, `follow_status`, `stop_follow`, the
stream's `metrics` and `close`, `notify`, `where_am_i`, `places`, `zones`,
`battery`, `motors_health`, `thermal_status`, `network_status`…).

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
(`storage_keys_remaining`, `storage_bytes_remaining`), `thermal` (the
motors' thermal status, see [section 12](#12-energy-and-hardware-health)) and
`compute` (CPU temperature and load, GPU if present; `None` when not
measured). Still served during an emergency stop.

---

## 3. High-level services

These calls block until the outcome is known. Their deadline is also bounded
by the time left in the hook (never extended). They reuse the OS services
(navigation, locomotion, manipulation), always under the proximity guard,
the joint guard and the emergency stop.

### `ctx.go_to(x, y, theta=None, *, timeout_s) -> ActionResult`

Permissions: `NAVIGATION_GOTO` (DANGEROUS), plus those of `walk`
(`LOCOMOTION_WALK`, `ACTUATORS_WRITE_LEGS`). Walks to `(x, y)` on the map
(world frame, metres), around the map's obstacles and the site map's
forbidden zones, then turns to `theta` (radians) if given. `x` may also be
the name of a place of the site map (see [section 9](#9-position-places-and-zones)). A still obstacle on the path is avoided when another way
exists; a person stops the robot (proximity guard).

| `status` | `code` | When |
|---|---|---|
| `ARRIVED` | — | arrived |
| `BLOCKED` | `BLOCKED` | no path, or goal inside an obstacle or off the map |
| `TIMEOUT` | `TIMEOUT` or `PROXIMITY_GUARD` | deadline passed (the guard held the robot) |
| `REFUSED` | `ESTOP_ACTIVE`, `CANCELLED`, `SAFETY_LIMIT` (fall), `CAPABILITY_MISSING`, `BUSY` (a walk stream is open), `FORBIDDEN_ZONE` | interrupted, no navigation, or goal in a forbidden zone |

The robot is stopped for every outcome but `ARRIVED`. Raises
`InvalidArgumentError` (NaN, infinite, coordinates beyond ±10 km, `theta`
beyond ±2π, `timeout_s` not in ]0, 600]) and `EstopActiveError`.

```python
r = ctx.go_to(1.5, 0.0, 0.0, timeout_s=60)
```

### `ctx.pick(target, *, size_m=None, timeout_s=15.0) -> ActionResult`

Permissions: `MANIPULATION` (DANGEROUS), plus those of `move_arms`
(`ACTUATORS_READ`, `ACTUATORS_WRITE_ARMS`). Grasps an object with both arms.
`target` is a `Detection` seen with `look_for(query, "world")` (then
`size_m=(width, depth, height)` is required: an image does not give an
object's size, and the OS does not guess it), or an
`ObjectSpec(center, size)` (`axis_os.store.sdk.results.object_spec`). In
both cases the OS's own perception must have seen an object within 10 cm
of that centre in the last 30 s (`look_for(..., "world")` by this app):
otherwise `REFUSED`/`SAFETY_LIMIT`. The manipulation service only closes
the arms if the object is between the hands, asks the proximity guard for
a bounded exemption around it, then checks the object followed before
reporting it held. The hands must also be within the object's height
(otherwise `OUT_OF_REACH`). When the robot reports its arm joint torques
(the simulated robot of `axis_os.testing`, the Isaac Sim T1 scene), the
grasp is *measured*: reach checked with the arms' kinematics (URDF), hands
placed at the middle of two opposite faces, arms closed until contact
(torque rise or hand stopped), a bounded extra squeeze (never above
10 N·m, nor half the joint safety guard's torque limit), a short lift
that must be followed by the object (one retry), then the object brought
against the torso. Missing, non-finite or stale torque data: no squeeze
(`SAFETY_LIMIT`). Nothing in between the hands at the expected width:
no squeeze (`OUT_OF_REACH`). If the grasp is cancelled (stop requested, end
of the hook, the app's process gone), the arms reopen gradually and the
exemption is lifted: the gesture never goes on without the app.

| `status` | `code` |
|---|---|
| `HELD` | — |
| `REFUSED` | `OUT_OF_REACH` (move closer with `go_to`), `BUSY` (the arms are busy: grasp, animation, arm stream), `SAFETY_LIMIT` (object not seen; arm torque data missing or invalid), `CAPABILITY_MISSING`, `ESTOP_ACTIVE`, `CANCELLED` |
| `FAILED` | `FAILED` — grasp not confirmed: the arms reopen gradually |
| `TIMEOUT` | `TIMEOUT` |

Raises `InvalidArgumentError` (detection in the robot frame, missing size,
centre or size NaN / out of bounds, size outside [2 cm, 70 cm]).

### `ctx.place(target=None, *, timeout_s=15.0) -> ActionResult`

Permissions: those of `pick`. Puts the held object down. `target`: `None`
(lower to carrying height and open), a surface height in metres, an
`ObjectSpec` of the surface (its top), or a `Detection` in the world frame.
With a surface, the object goes down until it touches it before the arms
open: it is never dropped from above. A put-down that has started goes to
its end even if a stop is requested (lowering then opening is the safe way
to finish).

`PLACED`; `REFUSED` (`OUT_OF_REACH`: surface too high; `FAILED`: nothing
held; `CAPABILITY_MISSING`, `ESTOP_ACTIVE`, `CANCELLED`); `FAILED` with
`OUT_OF_REACH` (arms at their lowest without touching: the object stays
held); `TIMEOUT`.

### `ctx.rest_pose(*, duration_s=2.0) -> ActionResult`

Permissions: those of `move_to_pose` (`ACTUATORS_READ`,
`ACTUATORS_WRITE_ARMS` on the G1). Returns to the rest pose **defined by the
driver**, never by the app (G1: arms only, never a standing biped's legs),
in interpolated steps under the joint guard, at 1 rad/s at most (a shorter
`duration_s` is lengthened, never the opposite). `AT_REST`; `REFUSED`
(`CAPABILITY_MISSING`: no rest pose declared; `BUSY`: the arms are busy —
an object held, an animation, an arm stream); `FAILED` (`SAFETY_LIMIT`);
`TIMEOUT` (not enough time left in the hook: the motion is refused rather
than sped up). `duration_s` in [0.5, 30].

---

## 4. Mission lifecycle

### `ctx.should_stop() -> bool` · `ctx.check_cancelled()`

Permission: `MISSION_LIFECYCLE` (NORMAL). `True` when a human or the OS
requests the mission to stop (`PlayStoreRuntime.request_stop(app_id,
source="human" | "os")`), when less than 2 s remain in the hook, or during
an emergency stop. `check_cancelled()` raises `MissionCancelled` (code
`CANCELLED`; the OS records the mission as cancelled), or
`EstopActiveError` during an emergency stop. A long action in progress
(`go_to`, `pick`…) also returns with `CANCELLED`, and an open stream makes a
safe stop and refuses `resume()`. If the app's process dies (crash,
quota), its long action in progress stops too.

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
100 ms (no function crosses the bridge); polling needs at least one of
these permissions. A callback that raises is logged
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
question, in the name of the app: the OS gives the human interface the
app's `app_id` (checked at install) and its manifest name, never anything
the app chooses at run time. The human interface exists only on the OS side: an
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
| `deadman_ms` | [50, 1000] (walk: [50, 500]) | without a valid target with the deadman held for this long: safe stop |
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
`CapabilityMissingError`, `BusyError` (a stream already open; for `"arms"`,
the arms busy with a grasp or an animation; for `"walk"`, a `go_to` in
progress).

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
`move_arms` (joint guard), through a limiter that keeps each joint's
commanded velocity ≤ 2 rad/s and acceleration ≤ 10 rad/s² step by step
(the arm follows the keyframes with a slight lag rather than jerking at
each keyframe; a late thread is never caught up in one step); the gesture
pauses while the proximity guard holds the robot. First a gentle approach
(0.5 rad/s) from the measured pose to the first keyframe.

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

## 8. People: presence and following

AXIS-OS detects people and follows one on request. **It never recognizes
anyone**: no face recognition, no face template, no biometric data. An app
only ever sees a temporary number, a distance and a direction. No image or
crop of a person is written to disk or to a log; appearance descriptors
(clothing colours, never the head) live only in memory and are erased at the
end of each follow, or a few seconds after the person was last seen. The
people service only looks while an app asks (within the last 10 s).

The detector is a model the OS loads (default: RT-DETR R18,
`PekingU/rtdetr_r18vd`, Apache-2.0, pinned revision, safetensors only — see
[THIRD_PARTY.md](../THIRD_PARTY.md)); tests use fake detections.

### `ctx.people() -> list[PersonInfo]`

Permission: `PEOPLE_DETECT` (NORMAL — but shown to the human at install:
*"this app detects the presence of people"*). The people visible now,
nearest first. `PersonInfo`: `person_id` (temporary number, kept while the
person stays visible, never reused), `distance_m` (to the robot's centre),
`bearing_rad` (0 straight ahead, positive to the left), `seen_for_s`,
`confidence`. A person whose distance cannot be estimated (no depth camera
and feet outside the image) is not listed. Empty list when the robot cannot
detect people.

### `ctx.person_in_front(max_distance_m=3.0) -> PersonInfo | None`

Permission: `PEOPLE_DETECT`. The nearest person within ±0.35 rad of the
robot's axis and within `max_distance_m` (in ]0, 8]): "the one in front of
me". Raises `InvalidArgumentError`.

### `ctx.follow(person_id, *, distance_m=1.5, timeout_s=60.0, deadman_s=1.0) -> ActionResult`

Permission: `PEOPLE_FOLLOW` (DANGEROUS), plus the rights of the APIs it
replaces: `PEOPLE_DETECT`, `LOCOMOTION_WALK`, `ACTUATORS_WRITE_LEGS`.
**Non-blocking**: returns `FOLLOWING` as soon as following starts; the OS
runs the loop at the locomotion rate. Every command goes through the
locomotion service, hence through the proximity guard and the joint guard:
following gets **no** exemption from the proximity guard.

- **Distance**: the robot keeps `distance_m`, at least the distance under
  which the proximity guard stops it plus a margin
  (`FollowService.distance_min`, 1.29 m with the default guard), at most
  3 m. Below: `InvalidArgumentError`. The distance used is the shorter of
  the camera's (the followed person) and the lidar's (the nearest obstacle
  ahead).
- **Speed**: at most 0.25 m/s (navigation: 0.30), slower when closing in;
  never backwards, never sideways; turns on the spot when the person is
  far to the side. It never drives into an obstacle of the site map.
- **The person stops**: the robot stops at the target distance.
- **The person disappears**: the robot stops, then searches by turning on
  the spot, for 5 s at most after it last saw the person; then `LOST`.
- **Someone else passes close by**: the robot holds still (the proximity
  guard has priority anyway). It never switches to another person by
  itself: if the other person looks similar (clothing), or if that cannot
  be told, following ends with `LOST` rather than risk following the wrong
  one; same if a lookalike passes where the hidden person could be. After
  a short re-identification (an occlusion), the robot holds still for one
  second (`verifying`) and ends with `LOST` if another similar person is
  visible; each one is reported in `data["reacquisitions"]`. See the known
  limit n° 9 in [SECURITY.md](../SECURITY.md).
- **Dead man**: the app must call `follow_status()` at least every
  `deadman_s` seconds (in [0.3, 2]); otherwise `DEADMAN`.
- Following also stops at the end of the hook that started it, on
  `stop_follow()`, `stop_walking()` (from any app), emergency stop, or a
  stop request.

Refusals: `REFUSED` with `TARGET_LOST` (unknown number, or person no longer
visible), `BUSY` (`go_to`, a walk stream, a grasp in progress, or another
app driving), `CAPABILITY_MISSING` (no people detection, no follow service,
or **no proximity guard**: the OS follows no one without it). Raises
`InvalidArgumentError` and `EstopActiveError`. While following, `walk()`
returns `False`, `go_to`/`pick`/`place` return `BUSY`, and
`stream_targets("walk")` raises `BusyError`.

### `ctx.follow_status() -> ActionResult`

Permission: `PEOPLE_FOLLOW`. Also the dead man's heartbeat. While following:
`FOLLOWING` (`ok`), with `data`: `phase` (`following`, `waiting`,
`searching`, `holding`, `verifying` — still for one second after a
re-identification), `person_id`, `distance_m`, `bearing_rad`,
`target_distance_m`, `reacquisitions`, `elapsed_s`. Afterwards, the outcome
of the last follow: `STOPPED_BY_APP` (`ok`), `LOST` (`TARGET_LOST`),
`TIMEOUT`, `ESTOP` (`ESTOP_ACTIVE`), `CANCELLED`, `DEADMAN`, `FALLEN`
(`SAFETY_LIMIT`), with `data["moved"]`. Otherwise `IDLE`. Served during an
emergency stop.

### `ctx.stop_follow() -> ActionResult`

Permission: `PEOPLE_FOLLOW`. Stops following (the robot stops, balanced) and
returns `STOPPED_BY_APP`, or the outcome if it had already ended, or `IDLE`.
Never refused during an emergency stop: stopping is always allowed.

```python
person = ctx.person_in_front()
if person and ctx.follow(person.person_id, deadman_s=2.0):
    while ctx.follow_status().status == "FOLLOWING":
        intent = ctx.listen(0.5)
        if intent and intent.name == "stop":
            ctx.stop_follow()
```

---

## 9. Position, places and zones

The OS keeps a **site map** (`axis_os/services/navigation/site_map.py`) on top
of the geometric map: **named places** (a point and an optional heading; one
of them may be the *charging station*) and **zones** (polygons):

| Zone kind | What the OS does |
|---|---|
| `forbidden` | the robot never enters it — for **every** walking command: `walk`, `go_to`, `follow`, a `"walk"` stream. The path planner goes around it, and a zone guard on the locomotion service cuts any translation that would bring the robot within its radius of the zone. |
| `slow` | walking speed capped to the zone's `max_speed_mps` inside it and just before entering it |
| `named` | a name only (for `on_enter_zone` / `on_exit_zone`) |

Only the site administrator writes the map, through the OS
(`SiteMap.publish(places, zones, author=...)`, `rollback(version)`): every
publication is checked as a whole (unique names, finite numbers, valid
polygons, no place inside a forbidden zone or within 0.7 m of one) and gets a
new version number; previous versions are kept. **Apps only read it**: no
bridge action writes the map.

### `ctx.where_am_i() -> Location`

Permission: `LOCATION` (NORMAL — shown at install: *"this app knows the
robot's position on the site map"*). `known`, `x`, `y` (m), `heading` (rad),
`uncertainty_m` (`None` when the localization does not provide it — odometry
alone does not), `timestamp` (robot time of the measurement),
`nearest_place` and `nearest_place_distance_m`, `zones` (names of the zones
the robot is in), `map_version`. `Location(known=False)` when the
localization does not answer: a position is never invented. Still served
during an emergency stop.

### `ctx.places() -> list[PlaceInfo]` · `ctx.zones() -> list[ZoneInfo]`

Permission: `LOCATION`. Read only. `PlaceInfo`: `name`, `x`, `y`,
`heading`, `kind` (`place` or `charging_station`). `ZoneInfo`: `name`,
`kind` (`forbidden`, `slow`, `named`), `polygon`, `max_speed_mps`,
`contains(x, y)`. Empty lists without a site map.

### `ctx.go_to(place, *, timeout_s)`

`go_to` also accepts the name of a place: `ctx.go_to("dock 3",
timeout_s=60)` goes there and turns to the place's heading. An unknown name
raises `InvalidArgumentError`. A goal inside a forbidden zone is refused
before planning: `REFUSED` / `FORBIDDEN_ZONE`.

### `ctx.on_enter_zone(name, callback)` · `ctx.on_exit_zone(name, callback)`

Permission: `LOCATION`. `callback(name)` when the robot enters (leaves) the
zone. An unknown zone raises `InvalidArgumentError`.

### Required places (manifest)

`AppManifest.required_places` (up to 32 names): installation is refused,
with a clear message, if one of them is missing from the site map
(*"Lieu(x) requis absent(s) de la carte du site : [...] — l'administrateur
doit les placer"*). A `zone_enter` trigger's zone must exist too.

---

## 10. Notifications, sounds and network

### `ctx.notify(title, message, *, priority="info") -> ActionResult`

Permission: `NOTIFICATIONS` (NORMAL). Tells the operator. `priority`:
`"info"`, `"warning"` or `"urgent"`. Title cut to 80 characters, message to
500, control characters replaced. **The operator always sees who sent it**:
the OS adds the source itself (`[app <name> · <app_id>]`); a text that
presents itself as the OS (*"AXIS-OS"*, also in full-width or with
invisible characters) raises `InvalidArgumentError`, and an app name that
imitates the OS is shown as its `app_id`. At most 6 notifications per
minute per app, 2 of them urgent: beyond, `REFUSED` / `RATE_LIMITED_APP`.
Allowed during an emergency stop (it moves nothing).

| `status` | `code` | When |
|---|---|---|
| `SENT` | — | delivered |
| `REFUSED` | `RATE_LIMITED_APP`, `CAPABILITY_MISSING` (no notification channel) | |
| `FAILED` | `FAILED` | the channel did not take it |

The OS side is an `INotificationSink` (`axis_os/services/notifications.py`);
today `FakeNotificationSink` for the tests, the robot's screen or a phone
later.

### `ctx.play_sound(name, *, volume=0.5) -> ActionResult`

Permission: `AUDIO_OUTPUT`. One of the OS's sounds: `"beep"`, `"alert"`,
`"confirmation"` — an app plays no file of its own. `volume` in [0, 1],
capped by the OS to 0.8; at most 20 sounds per minute (`RATE_LIMITED_APP`).
`status`: `PLAYED`, `REFUSED` (`CAPABILITY_MISSING`: no speaker), `FAILED`.
Raises `InvalidArgumentError` and `EstopActiveError`.

### `ctx.network_status() -> str`

Permission: `SYSTEM_INFO`. `"online"` (a route to the outside exists),
`"local_only"`, `"offline"`, or `"unknown"`. No connection is opened to find
out.

---

## 11. Scheduled missions

An app may declare **triggers** in its manifest; the OS then starts its
mission itself (`params == {"trigger": "<name>"}`), through the Play Store
exactly as a human would: sandbox, quotas, permissions, emergency stop,
mission lock. Permission: `SCHEDULED_MISSIONS` (DANGEROUS, confirmed by a
human at install, where every trigger is shown in plain words).

```python
from axis_os.store.sdk.manifest import TriggerSpec

triggers=[TriggerSpec("evening_round", "daily", at="22:00"),
          TriggerSpec("tour", "interval", every_s=3600),
          TriggerSpec("low", "battery_below", percent=40),
          TriggerSpec("arrival", "zone_enter", zone="dock")]
```

At most 8 triggers; an interval of at least 300 s; `percent` in [5, 95].
The rules (`axis_os/store/runtime/scheduler.py`, `MissionScheduler`):

1. Only installed **and started** apps have active triggers. An app cannot
   add or change one at run time.
2. **One mission at a time.** A trigger that fires during any mission is
   skipped (`busy`), never queued: it fires again at its next occurrence.
3. **Two triggers due together**: one is started, in this order — battery,
   zone, daily, interval, then `app_id` and trigger name; the others are
   skipped (`collision`).
4. **No catch-up**: an occurrence more than 120 s late (robot off, OS busy)
   is `missed` and never started later; an interval restarts from now.
5. Nothing is started during an **emergency stop** or after a **fall**,
   outside the **hours allowed by the administrator**
   (`set_allowed_hours("06:00", "23:00")`, may wrap midnight), while an OS
   service blocks missions (return to the charging station, motor too hot),
   or for an app or trigger a **human disabled** (`set_enabled`,
   `set_paused`).
6. The same trigger never fires twice within 300 s (a zone border crossed
   ten times starts one mission); a battery trigger re-arms once the charge
   is 5 points above its threshold. An unknown battery never fires.

Every decision is in `MissionScheduler.journal`, for the administrator.

---

## 12. Energy and hardware health

### The OS's energy service

`axis_os/services/energy.py` (`EnergyService`), thresholds set by the
administrator (`set_thresholds(return_percent=20, critical_percent=8)`):

- **below the return threshold**, not charging: no new mission and no app
  start; the running mission is asked to stop (`should_stop()`), then
  force-stopped after a grace period (its process is killed, the mission
  returns *"Arrêt forcé par l'OS : …"*); then the OS drives the robot to the
  charging station with the navigation — forbidden and slow zones, proximity
  guard, thermal derating all apply. Missions stay refused until the charge
  is back above the resume threshold (return + 10 by default). No station on
  the map, or the way blocked after several tries: the operator is told
  (urgent).
- **below the critical threshold**: everything stops where it is, balanced,
  and the operator is told (urgent).
- Unknown battery: nothing is invented; the service does nothing.

An app cannot prevent the return: it has no process left to command with.

### `ctx.battery() -> BatteryInfo`

Permission: `HARDWARE_HEALTH` (NORMAL). As measured by the driver:
`known`, `percent`, `charging`, `voltage`, `current_a`, `temperature_c`,
`health_percent`, `cycles` — each `None` when the driver does not measure
it, `known=False` when it cannot read the battery at all.

### `ctx.motors_health() -> list[MotorHealth]`

Permission: `HARDWARE_HEALTH`. Read only. Per joint: `temperature_c`,
`critical_c` (where the OS refuses any command to that joint), `margin_c`,
`torque_nm`, `current_a` (only if the driver really measures it), `status`.
`temperature_c` is `None` when the driver does not *measure* motor
temperature (`IRobotHAL.measures_motor_temperature()`): a fixed or estimated
value is not a measurement. Empty without a health service.

### `ctx.thermal_status() -> str` · `ctx.on_thermal_change(callback) -> bool`

Permission: `HARDWARE_HEALTH`. The robot's worst joint:

| Status | Temperature (for an 80 °C joint) | What the OS does |
|---|---|---|
| `normal` | below 55 °C | — |
| `hot` | 55 to 65 °C | — |
| `throttled` | 65 to 80 °C | walking slowed (down to 50 %), the joint guard refuses commanded velocity and torque beyond the same share of their maximum; warning notification |
| `critical` | 80 °C or more, **or a non-finite or stale reading** | no walking, no command to that joint, the running mission is stopped (cooperative, then forced), new missions refused; urgent notification |
| `unknown` | the driver does not measure it | — |

The critical temperature is the lower of 80 °C and the joint's
`temperature_max` (never higher). `on_thermal_change(callback)` calls
`callback(status)` on every change; it returns `False` without a health
service.

`safety_status()` also gives `thermal` and `compute`: CPU temperature and
load, GPU presence, temperature and load (`None` when not measured).

### Crash reports

When an app crashes, exceeds a quota or hangs, the OS keeps a report for the
administrator (`PlayStoreRuntime.crash_reports()`): app, version, hook,
cause, quota, duration, time and the exception **type** — never its message,
the stack, the mission parameters or the app's log, which may hold what the
app saw or heard. No app can read them.

---

## 13. Low-level APIs

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

## 14. Versions

- `ctx.api_version` — the SDK API version this OS serves (SemVer).
- `AppManifest.min_sdk_version` — the minimum SDK API version the app needs.
  Installation is refused if the OS's API is older or of another MAJOR
  version.
- `AppManifest.sdk_version_min` — the minimum AXIS-OS version (unchanged).

The package `axis-os-sdk` (`pyproject.toml`, `pip install .`, not published
on PyPI) has the version of the API.

---

## 15. Testing and tools

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
  manipulation=True, objects=(), battery_percent=None, grasp_succeeds=True,
  people=(), speech=False, seed=0, time_scale=1.0, site_map=None,
  battery_drain_per_s=0.0, battery_drain_per_m=0.0, charge_per_s=1.0)` —
  the battery drains and recharges within 0.3 m of the site map's charging
  station (no real docking); `set_motor_temperature(joint, celsius)` sets
  what a motor's simulated sensor measures; `notification_sink.delivered`,
  `sounds.played`, `network.state` (fakes); `health` (the hardware health
  service); `add_person(x, y,
  torso=..., legs=...)` (returns a `SimulatedPerson`, seen by the proximity
  guard and the simulated camera), `clear_people()`, `hear(text,
  after_s=0)` (with `speech=True`), `set_battery(percent)`, `fall()`,
  `pose`, `voice.spoken`, `make_runtime(**kw)`. `time_scale=2` runs robot
  time twice as fast as real time.
- `SimulatedPerson(x, y, *, torso, legs)` — `.walk_to(x, y, speed_mps)`,
  `.wait(seconds)` (chainable). Hidden from the camera by map obstacles and
  by nearer people.
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
