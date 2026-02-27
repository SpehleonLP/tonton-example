# TonTon Myopic Control Module — Design Notes

## What This Document Is

Design notes for implementing a myopic locomotion controller as a module within TonTon. Written with full context from the game engine it will integrate into, distilled to what's relevant here.

## The Problem

TonTon analyzes a rigged skeleton+mesh and produces a comprehensive `Output` struct with optional locomotion sections (aerial, terrestrial, aquatic, serpentine, climbing, brachiation, jumping) plus metabolic, behavioral, and sensory data. A single creature can have multiple locomotion modes simultaneously (a dragonfly has aerial + terrestrial + climbing + jumping).

The game engine needs a per-frame steering controller that:
1. Takes a current state and a target
2. Returns acceleration, torque, and a stability metric
3. Is "myopic" — it only cares about this frame, trusts the caller about what mode it's in, and doesn't plan ahead

Something *external* to this module decides what locomotion mode to use. The myopic controller just says "given that you told me I'm walking, here's how to steer, and here's how stable I am." If the caller lied about being on the ground, that's the caller's problem.

## Previous Failed Approaches

1. **PD controller**: Untunable. Robotics PD controllers assume ~1kHz control loops. At game timesteps (16-60Hz) the gains don't transfer. Robotics papers never state their timestep because 1kHz is assumed. Oscillated wildly.

2. **Bird flight control via AI (Sonnet 4.5)**: Given the full TonTon output, the AI tried to use ALL the data and built an over-engineered controller. The model drowned in information.

3. **Control theory framing**: Calling this "myopic control" triggered control-theory approaches (force models, torque computation, stability margins). The actual problem is much simpler.

## Key Insight

This is **constrained steering behaviors** (Craig Reynolds, 1999), not control theory. The controller doesn't simulate forces. It computes desired heading change and speed change, clamps them to what TonTon says is physically possible, and reports how hard it's working relative to its limits.

## Interface

```cpp
namespace TonTon {

enum class LocomotionMode : uint8_t {
    TERRESTRIAL,
    AERIAL,
    AQUATIC,
    SERPENTINE,
    CLIMBING,
    BRACHIATION,
    // JUMPING is a one-shot event, not a sustained mode
};

struct MyopicInput {
    // What mode the caller says we're in. We trust them.
    LocomotionMode mode;

    // Current state (world space)
    glm::vec3 position;
    glm::quat orientation;      // current facing
    glm::vec3 velocity;         // current linear velocity
    glm::vec3 angular_velocity; // current rotation rates (body space)

    // What we want
    glm::vec3 target_position;

    // Optional: desired speed. If < 0, controller picks cruise speed.
    float desired_speed = -1.f;

    // For terrestrial: which gait are we in? (index into speed ranges)
    // 0 = optimal/walk, 1 = sustainable/trot, 2 = sprint/gallop
    int current_gait = 0;

    // Frame time
    float dt;
};

struct MyopicOutput {
    // Body-level commands (NOT per-limb)
    glm::vec3 linear_acceleration;  // world space, how to change velocity
    glm::vec3 angular_acceleration; // body space, how to change orientation

    // The important one:
    // 1.0 = comfortable, well within limits
    // 0.5 = working hard, consider changing gait
    // 0.0 = at limits, strongly suggest mode/gait change
    // <0  = exceeding limits, suggest ragdoll or emergency
    float stability;

    // Hints for the external mode/gait selector (NOT decisions, just data)
    float speed_headroom;      // how close to max speed (0=at max, 1=lots of room)
    float turn_headroom;       // how close to max turn rate
    bool  suggest_gait_change; // current gait can't sustain this
};

// The actual function. One call per creature per frame.
// `analysis` is the full TonTon::Output, computed once at creature creation.
MyopicOutput ComputeMyopicControl(
    const Output& analysis,
    const MyopicInput& input);

} // namespace TonTon
```

## Implementation Approach

### Step 1: Extract the constraint envelope for current mode

The first thing `ComputeMyopicControl` does is pull the 5-6 numbers that matter from the relevant `Analysis_*` section. This is the critical data reduction — the rest of the function never touches raw TonTon data again.

```cpp
struct Envelope {
    float max_speed;
    float min_speed;       // 0 for ground, stall speed for air
    float max_accel;
    float max_brake;       // may differ from max_accel
    float max_turn_rate;   // best available axis, rad/s
    float min_turn_radius; // 0 = can pivot in place
};
```

For **terrestrial**, the envelope depends on current gait:
- gait 0 (walk): max_speed = optimal_speed, turn rate is generous
- gait 1 (trot): max_speed = sustainable_speed, turn rate tighter
- gait 2 (gallop): max_speed = sprint_speed, turn rate = constrained by min_turning_radius

For **aerial**, the envelope comes directly from Analysis_Aerial:
- max_speed = max_flight_speed
- min_speed = min_flight_speed (stall!)
- max_turn_rate = pick the BEST axis for the needed heading change (see below)
- min_turn_radius = from analysis, BUT only applies when banking — yaw-pivoting ignores it

For **aquatic**, similar to aerial but different speed ranges and no stall.

### Step 2: Compute desired heading change

```
direction_to_target = normalize(target_position - position)
angle_error = signed_angle(current_heading, direction_to_target)
```

For aerial mode, decompose into body-space roll/pitch/yaw components. For everything else, it's just yaw.

### Step 3: Pick the turn strategy (aerial only)

This is where the dragonfly-vs-eagle distinction happens. Given the TonTon maneuverability data:

```
max_roll_rate, max_pitch_rate, max_yaw_rate
```

The question: "can yaw alone cover the needed heading change this frame?"

```
needed_yaw_rate = angle_error / some_time_horizon  // e.g. 0.5 seconds
if (abs(needed_yaw_rate) <= max_yaw_rate)
    // flat yaw turn, like the dragonfly — ignore turning radius entirely
else
    // need to bank — now turning radius matters
    // bank angle = atan(v² / (r * g)) for a coordinated turn
    // roll into the bank, let centripetal force handle the turn
```

The dragonfly: yaw_rate = 4.1 rad/s. For almost any heading change, flat yaw is sufficient. It never needs to bank. The turning radius of 0.63m is essentially irrelevant for it.

A large bird: yaw_rate might be 0.3 rad/s but roll_rate is 2.0 rad/s. It banks for everything.

This is NOT the controller deciding to bank or yaw. It's just: "can the cheaper axis handle it? Use that. If not, use the more expensive one." The creature's anatomy makes the decision via TonTon's numbers.

### Step 4: Compute acceleration

```
speed_error = desired_speed - current_speed
desired_accel = clamp(speed_error * gain, -max_brake, max_accel)
```

For aerial: also check stall. If speed < min_flight_speed, stability drops hard.

For terrestrial: if desired_speed > current gait's max, suggest_gait_change = true. Don't clamp to current gait's max — let the stability metric communicate the problem.

### Step 5: Compute stability

This is the most important output. It's a simple ratio of demand to capacity:

```
turn_usage = abs(demanded_turn_rate) / max_turn_rate
speed_usage = current_speed > max_speed
    ? -(current_speed - max_speed) / max_speed   // negative = over limit
    : 1.0 - current_speed / max_speed

// For aerial: stall check adds instability
stall_penalty = (mode == AERIAL && speed < min_speed)
    ? -(min_speed - speed) / min_speed
    : 0

stability = 1.0 - max(turn_usage, 1.0 - speed_headroom) + stall_penalty
```

Stability meanings:
- **1.0**: Cruising straight, well within limits. No concerns.
- **0.7**: Moderate turn or speed. Normal operation.
- **0.3**: Hard turn at high speed, or near gait limit. External system should consider gait change.
- **0.0**: At the edge. Max turn at max speed, or at stall speed in air.
- **Negative**: Beyond limits. External system should switch modes or ragdoll.

### Step 6: Package output

Linear acceleration = forward_acceleration * heading + centripetal terms if banking.
Angular acceleration = turn_rate_change * appropriate_axis.
Stability, speed_headroom, turn_headroom, suggest_gait_change.

Done. One function call. Maybe 100-150 lines of actual logic.

## What This Module Does NOT Do

- **Mode selection**: "should I fly or walk?" — external
- **Gait selection**: "should I trot or gallop?" — external, but stability hints help
- **Foot placement**: handled by CPG + arc interpolation + IK in the engine
- **Limb control**: CPG phase oscillators handle leg timing
- **Trajectory planning**: myopic = this frame only
- **Terrain awareness**: caller's job to know we're on the ground
- **Obstacle avoidance**: caller adds avoidance vectors to target before calling us
- **Transition sequencing**: "how to take off" is a separate state machine using TonTon's takeoff analysis

## Terrestrial Gait Model

TonTon gives three speed points:
- `optimal_speed` — minimum cost of transport (walk)
- `max_sustainable_speed` — aerobic limit (trot/canter)
- `max_sprint_speed` — anaerobic, time-limited (gallop)

Plus `max_sprint_duration_s` and `recovery_time_s`.

The myopic controller doesn't switch gaits. It reports:
- `suggest_gait_change = true` when speed demand exceeds current gait range
- `stability` drops when operating outside comfortable range for current gait

The external gait selector (in the engine, not TonTon) uses these hints plus context (stamina, threat level, brain decisions) to switch gaits, which changes the envelope for next frame.

## Aerial Turn Strategy Detail

The key realization: `min_turning_radius` is the radius for a **sustained banked turn at cruise speed**. It is NOT the minimum turn capability. A creature with high yaw authority can turn tighter than its banking radius by yawing directly. The banking radius only matters when:
1. Yaw authority is insufficient for the demanded heading change
2. The creature is moving fast enough that banking is aerodynamically efficient

For the dragonfly (yaw: 4.1, roll: 2.1, pitch: 0.4 rad/s):
- Almost all turns are flat yaw. Radius effectively 0.
- Banking only at very high speed tight turns where centripetal force helps.

For an albatross (hypothetical yaw: 0.2, roll: 1.5, pitch: 0.3 rad/s):
- Almost all turns are banked. min_turning_radius is the real constraint.
- Yaw authority is decorative — minor course corrections only.

The controller doesn't need a special case. It just checks: `needed_rate <= yaw_rate? use yaw : bank`. The anatomy decides.

## Aquatic Notes

Similar to aerial but:
- No stall speed for most swimmers (except `requires_constant_motion` species like sharks)
- 2D-ish steering (mostly yaw, some pitch for depth changes)
- Buoyancy replaces lift — no stall danger for buoyant creatures
- Current/flow replaces wind

## Serpentine Notes

The hardest mode because turning IS locomotion — a snake can't turn without undulating. The "turn rate" is coupled to forward speed through the body wave parameters. Slower movement = tighter turns but there's a minimum speed to maintain the undulation. `lateral_undulation_speed` is both min and max — serpentine locomotion has a narrow speed band per mode.

Suggest: treat serpentine as having a very constrained envelope (narrow speed range, turn rate coupled to speed) and let stability drop fast when demands exceed it. The external system can switch serpentine sub-modes (lateral undulation vs sidewinding vs concertina) which changes the envelope.

## Implementation Order

1. **Terrestrial first.** Most creatures walk. Simplest envelope (speed + turn rate). Get this working with the CPG and IK in the engine.
2. **Aerial second.** The yaw-vs-bank selection is the interesting part. Speed envelope with stall adds complexity.
3. **Aquatic third.** Similar to aerial once aerial works.
4. **Serpentine last.** Weird coupling between speed and turning. Needs the most thought.
5. **Climbing/brachiation**: Simplified terrestrial with different constraints. Low priority.

## Integration With Game Engine

The engine has:
- **CPG library** (Kuramoto-coupled phase oscillators) — handles leg synchronization, produces per-limb phase [0,2π)
- **SPARK IK** — solves joint angles from foot targets (already working)
- **Collision system** — handles ragdoll when stability goes negative (gravity, bounce, rolling, buoyancy already implemented)
- **Cellular automaton atmosphere** — provides wind vectors for aerial drag
- **Fluid system** — provides water levels and current for aquatic mode

The myopic controller's output feeds into the creature's velocity/orientation update. The CPG runs independently to synchronize limbs. Foot placement is a game-dev cheat: feet follow parametric arcs keyed to CPG phase, with stride length from TonTon parameterization. SPARK IK solves the joints. Body bob and shoulder roll are sin waves keyed to average leg phase.

The myopic controller doesn't know about any of this. It just outputs body-level acceleration and torque. The engine applies it.

## Testing

The tonton-example project can test this without the full engine:
- Create a TonTon::Output for known creatures (dragonfly, penguin, horse, snake, eagle)
- Feed scripted MyopicInput sequences (approach target, orbit target, turn 180, speed up, slow down)
- Verify: stability stays positive during reasonable maneuvers
- Verify: stability goes negative during impossible maneuvers (penguin tries to sprint-turn)
- Verify: dragonfly yaw-turns, eagle bank-turns, for same heading change input
- Verify: gait_change suggested when speed exceeds current gait range
- Print trajectories, plot in gnuplot or similar

## Dimensional Analysis

Use TonTon's existing `Quantity<M,L,T,Temp,Stage>` compile-time unit system for all physics in the implementation. This catches unit errors at compile time. The MyopicInput/Output structs in the public API can use plain floats with documented units for simplicity, but internal computation should use typed quantities.

## Summary

The myopic controller is ~150 lines of logic:
1. Extract 5-6 constraint numbers from TonTon output based on current mode
2. Compute angle to target and speed error
3. For aerial: pick yaw vs bank based on which axis has authority
4. Clamp turn rate and acceleration to envelope
5. Compute stability as demand/capacity ratio
6. Return acceleration, torque, stability, hints

Not control theory. Not PID. Not force simulation. Just clamped steering with a stability readout. The hard part was always the data reduction from TonTon's full output, not the steering math.
