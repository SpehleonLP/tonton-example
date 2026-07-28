// ============================================================================
// TonTon myopic control tests
//
// Uses the same load/analyze harness as tonton_plausibility_tests.cpp (copied
// verbatim; it is static/anonymous-namespace there and cannot be shared).
// ============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <deque>
#include <string>
#include <tuple>
#include <vector>

#include "get_armatures_from_files.h"
#include "tonton_builder.h"
#include "tonton_input.h"
#include "tonton_analysis.h"
#include "tonton_skinnedmesh.h"
#include "tonton_wordlist.h"

#include "tonton_myopic.h"
#include "Control/tonton_envelope.h"
#include "Control/tonton_steer.h"
#include "Control/tonton_launch.h"

#ifndef TONTON_SAMPLE_MODELS_DIR
#define TONTON_SAMPLE_MODELS_DIR "sample models"
#endif

using namespace TonTon;

namespace {

// ---------------------------------------------------------------------------
// Environments (copied from tonton_main.cpp so the test does not need main()).
// ---------------------------------------------------------------------------
Environment EarthAir()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.225f;
    env.fluidViscosity_Pa_s = 1.81e-5f;
    env.gravity_m_s2 = 9.81f;
    env.fluidPressure_Pa = 101325.0f;
    env.temperature_K = 293.15f; // 20 C
    return env;
}

Environment EarthOcean()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1025.0f;   // seawater
    env.fluidViscosity_Pa_s = 0.00107f; // seawater @ 20 C
    env.gravity_m_s2 = 9.81f;
    env.fluidPressure_Pa = 101325.0f + (1025.0f * 9.81f * 30.0f); // 30 m depth
    env.temperature_K = 293.15f;
    return env;
}

enum class Env { Air, Ocean };

// Mirror the CLI defaults exactly so results match `tonton-analyze`.
Input MakeDefaultInput(Env which)
{
    Input input;
    input.environment = (which == Env::Ocean) ? EarthOcean() : EarthAir();

    input.scale              = length_b_to_m(1.0f);
    input.average_density    = 0.5f;
    input.structure_vs_weight = 0.5f;
    input.muscle_quality     = 0.5f;
    input.feather_quality    = 0.5f;
    input.metabolic_efficiency = 0.5f;
    input.stability_vs_speed = 0.5f;
    input.activity_level     = 0.5f;
    input.scaling_strategy   = 0.5f;
    input.climbing_ability   = 0.5f;

    input.behavior.coloration          = 0.0f;
    input.behavior.aggression_adjustment = 0.5f;
    input.behavior.activity_adjustment = 0.5f;
    input.behavior.endurance_vs_power  = 0.5f;
    input.behavior.risk_tolerance      = 0.5f;
    input.behavior.social_tendency     = 0.5f;
    input.behavior.seasonal_behavior   = 0.5f;
    input.behavior.activity_pattern    = 0.5f;
    input.behavior.adaptability        = 0.0f;

    input.mana.water  = 0.0f;
    input.mana.fire   = 0.0f;
    input.mana.earth  = 0.0f;
    input.mana.air    = 0.0f;
    input.mana.aether = 0.5f;
    input.mana.shadow = 0.0f;
    return input;
}

// ---------------------------------------------------------------------------
// Analyze a model once and cache the result (loading + volumetric analysis is
// not free). The whole chain is kept alive in a static store so the returned
// Output stays valid for the duration of the test run.
// ---------------------------------------------------------------------------
struct AnalysisHolder {
    std::vector<InputFile> files;
    Input input;
    counted_ptr<const Output> output;
};

const Output* Analyze(const std::string& filename, Env env)
{
    static std::deque<AnalysisHolder> store;
    static std::map<std::string, const Output*> cache;

    std::string key = filename + (env == Env::Ocean ? "|ocean" : "|air");
    auto it = cache.find(key);
    if (it != cache.end()) return it->second;

    std::string path = std::string(TONTON_SAMPLE_MODELS_DIR) + "/" + filename;

    AnalysisHolder holder;
    holder.input = MakeDefaultInput(env);

    std::vector<const char*> args = { path.c_str() };
    holder.files = GetArmaturesFromFiles({ args.data(), args.data() + args.size() });

    const Output* result = nullptr;
    if (!holder.files.empty() && holder.files[0].memo && holder.files[0].memo->size() > 0) {
        holder.input.builder = Builder::Factory(holder.files[0].memo->at(0));
        holder.output = Output::Factory(holder.input);
        result = holder.output.get();
    }

    store.push_back(std::move(holder));
    cache[key] = result;
    return result;
}

// ---------------------------------------------------------------------------
// A freshly-analysed Output that the CALLER owns exclusively -- not the shared
// cache above. Two tests need this:
//   - driving a non-default Input (the mana axis), which the cache's key does
//     not distinguish;
//   - mutating the result, which would corrupt every later test if done to the
//     cached instance.
// `holder` must outlive the returned pointer: it keeps the mesh chain alive.
// ---------------------------------------------------------------------------
counted_ptr<const Output> AnalyzeFresh(const std::string& filename,
                                       Input input,
                                       AnalysisHolder& holder)
{
    std::string path = std::string(TONTON_SAMPLE_MODELS_DIR) + "/" + filename;
    holder.input = std::move(input);

    std::vector<const char*> args = { path.c_str() };
    holder.files = GetArmaturesFromFiles({ args.data(), args.data() + args.size() });
    if (holder.files.empty() || !holder.files[0].memo || holder.files[0].memo->size() == 0)
        return {};

    holder.input.builder = Builder::Factory(holder.files[0].memo->at(0));
    holder.output = Output::Factory(holder.input);
    return holder.output;
}

} // namespace

TEST(MyopicEnvelope, TerrestrialGaitsAreOrdered)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->terrestrial.has_value());

	auto walk   = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 0, 9.81f);
	auto trot   = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 1, 9.81f);
	auto gallop = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 2, 9.81f);

	ASSERT_TRUE(walk.has_value());
	ASSERT_TRUE(trot.has_value());
	ASSERT_TRUE(gallop.has_value());

	EXPECT_LT(float(walk->max_speed), float(trot->max_speed));
	EXPECT_LT(float(trot->max_speed), float(gallop->max_speed));
}

TEST(MyopicEnvelope, TerrestrialInvariants)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	for (int gait = 0; gait <= 2; ++gait) {
		auto env = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, gait, 9.81f);
		ASSERT_TRUE(env.has_value()) << "gait " << gait;

		EXPECT_GT(float(env->max_speed), 0.f)         << "gait " << gait;
		EXPECT_GE(float(env->min_speed), 0.f)         << "gait " << gait;
		EXPECT_LT(float(env->min_speed), float(env->max_speed)) << "gait " << gait;
		EXPECT_GT(float(env->max_accel), 0.f)         << "gait " << gait;
		EXPECT_GT(float(env->max_lateral_accel), 0.f) << "gait " << gait;

		// tau must be finite and positive or the slew produces NaN.
		EXPECT_GT(float(env->tau_linear), 0.f)        << "gait " << gait;
		EXPECT_TRUE(std::isfinite(float(env->tau_linear))) << "gait " << gait;
	}
}

TEST(MyopicEnvelope, AbsentModeReturnsNullopt)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_FALSE(out->aerial.has_value()) << "cat should not fly";

	EXPECT_FALSE(ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f).has_value());
}

namespace {

// A minimal envelope for pure control-law tests. No Output involved, which is
// the point: Steer cannot see one.
Envelope TestEnvelope()
{
	Envelope e;
	e.max_speed         = velocity_m_s{10.f};
	e.min_speed         = velocity_m_s{0.f};
	e.max_accel         = acceleration_m_s2{5.f};
	e.max_brake         = acceleration_m_s2{5.f};
	e.max_lateral_accel = acceleration_m_s2{8.f};
	e.min_turn_radius   = length_m{2.f};
	e.tau_linear        = time_s{0.25f};
	return e;
}

// Same shape as TestEnvelope but with a fast slew (tau well under a 16 Hz
// frame). Exercises the dt > tau_linear regime, where the stopping-angle
// bound alone (error / tau_linear) is not the tighter constraint -- added in
// review round 2 after that regime was found to overshoot (up to 83%, with
// repeated sign flips) under the round-1 bound.
Envelope FastTauEnvelope()
{
	Envelope e = TestEnvelope();
	e.tau_linear = time_s{0.025f};
	return e;
}

// Integrate a pure heading-tracking maneuver and return the final heading error.
float SimulateTurn(float dt, float total_time_s, float initial_error_rad)
{
	Envelope env = TestEnvelope();
	SteerState state{};
	float error = initial_error_rad;
	float speed = 5.f;

	const int steps = int(total_time_s / dt);
	for (int i = 0; i < steps; ++i) {
		SteerCommand cmd;
		cmd.angle_error_rad   = error;
		cmd.current_speed_m_s = speed;
		cmd.desired_speed_m_s = speed;
		cmd.dt_s              = dt;

		SteerResult r = Steer(env, state, cmd);
		error -= r.turn_rate_rad_s * dt;
	}
	return error;
}

// Same maneuver as SimulateTurn, but returns the heading error at an
// intermediate wall-clock time instead of running to the end. The terminal
// error alone cannot detect framerate-dependent response *shape*: the greedy
// term forces exact termination for any law that finishes inside the
// simulated window, so by the end of a long-enough run every law's error
// converges to (near) zero regardless of how it got there. Mid-slew, at
// t=0.3s here, the laws are still separating and a dt-dependent law (a bare
// constant lerp factor, the literal 2025 failure) is caught red-handed.
float SimulateTurnAtTime(float dt, float target_time_s, float initial_error_rad)
{
	Envelope env = TestEnvelope();
	SteerState state{};
	float error = initial_error_rad;
	float speed = 5.f;

	const int steps = static_cast<int>(std::lround(target_time_s / dt));
	for (int i = 0; i < steps; ++i) {
		SteerCommand cmd;
		cmd.angle_error_rad   = error;
		cmd.current_speed_m_s = speed;
		cmd.desired_speed_m_s = speed;
		cmd.dt_s              = dt;

		SteerResult r = Steer(env, state, cmd);
		error -= r.turn_rate_rad_s * dt;
	}
	return error;
}

} // namespace

// THE regression test for the 2025 failure. A dt-dependent control law
// (a naive lerp, or any PD gain) diverges here.
TEST(MyopicFramerate, TrajectoryConvergesAcrossTimesteps)
{
	const float total = 2.0f;
	const float start = 1.5f; // rad

	float e16  = SimulateTurn(1.f / 16.f,  total, start);
	float e30  = SimulateTurn(1.f / 30.f,  total, start);
	float e60  = SimulateTurn(1.f / 60.f,  total, start);
	float e120 = SimulateTurn(1.f / 120.f, total, start);

	EXPECT_NEAR(e16,  e60, 0.05f) << "16 Hz diverges from 60 Hz";
	EXPECT_NEAR(e30,  e60, 0.05f) << "30 Hz diverges from 60 Hz";
	EXPECT_NEAR(e120, e60, 0.05f) << "120 Hz diverges from 60 Hz";

	// Mid-trajectory check (added in review round 1): the terminal-only
	// assertions above pass for a bare constant lerp factor (alpha=0.10,
	// ignoring dt) at all four rates -- exact termination hides the shape
	// difference. At t=0.3s the real controller's four rates still agree to
	// within ~0.04 rad, while the constant-lerp controller's disagree by
	// ~0.09-0.14 rad at the same point (measured with a standalone stub;
	// see task-2-report.md). 0.05 rad separates the two cleanly.
	const float mid_t = 0.3f;
	float m16  = SimulateTurnAtTime(1.f / 16.f,  mid_t, start);
	float m30  = SimulateTurnAtTime(1.f / 30.f,  mid_t, start);
	float m60  = SimulateTurnAtTime(1.f / 60.f,  mid_t, start);
	float m120 = SimulateTurnAtTime(1.f / 120.f, mid_t, start);

	EXPECT_NEAR(m16,  m60, 0.05f) << "mid-trajectory (t=0.3s): 16 Hz diverges from 60 Hz";
	EXPECT_NEAR(m30,  m60, 0.05f) << "mid-trajectory (t=0.3s): 30 Hz diverges from 60 Hz";
	EXPECT_NEAR(m120, m60, 0.05f) << "mid-trajectory (t=0.3s): 120 Hz diverges from 60 Hz";
}

// A PD controller cannot pass this. Clamped-greedy-plus-slew cannot fail it.
TEST(MyopicNoOscillation, HeadingErrorIsMonotone)
{
	Envelope env = TestEnvelope();
	SteerState state{};
	float error = 1.5f;
	float prev_error = error;
	const float dt = 1.f / 60.f;

	for (int i = 0; i < 600; ++i) { // 10 seconds
		SteerCommand cmd;
		cmd.angle_error_rad   = error;
		cmd.current_speed_m_s = 5.f;
		cmd.desired_speed_m_s = 5.f;
		cmd.dt_s              = dt;

		SteerResult r = Steer(env, state, cmd);
		error -= r.turn_rate_rad_s * dt;

		ASSERT_GE(error, -0.01f) << "overshot into negative error at step " << i;
		ASSERT_LE(error, prev_error + 1e-4f) << "error grew at step " << i;
		prev_error = error;
	}
	EXPECT_NEAR(error, 0.f, 0.02f) << "did not converge";
}

// Added in review round 2: the dt > tau_linear regime. HeadingErrorIsMonotone
// above only exercises tau=0.25 at dt=1/60 (dt < tau), where the
// stopping-angle bound (error / tau_linear) alone is the tighter, correct
// constraint. A creature with a fast slew relative to its frame time --
// tau_linear = optimal_speed / max_acceleration can easily land under 16 ms
// -- was found to overshoot under that bound alone: up to 83% past zero,
// with repeated sign flips, at tau=0.025 driven at 16 Hz. This test pins the
// same monotonicity and no-sign-crossing properties in that regime.
TEST(MyopicNoOscillation, HeadingErrorIsMonotoneWhenDtExceedsTau)
{
	Envelope env = FastTauEnvelope(); // tau_linear = 0.025s
	SteerState state{};
	float error = 1.5f;
	float prev_error = error;
	const float dt = 1.f / 16.f; // dt (0.0625s) > tau_linear (0.025s)

	for (int i = 0; i < 160; ++i) { // 10 seconds
		SteerCommand cmd;
		cmd.angle_error_rad   = error;
		cmd.current_speed_m_s = 5.f;
		cmd.desired_speed_m_s = 5.f;
		cmd.dt_s              = dt;

		SteerResult r = Steer(env, state, cmd);
		error -= r.turn_rate_rad_s * dt;

		ASSERT_GE(error, -0.01f) << "overshot into negative error at step " << i;
		ASSERT_LE(error, prev_error + 1e-4f) << "error grew at step " << i;
		prev_error = error;
	}
	EXPECT_NEAR(error, 0.f, 0.02f) << "did not converge";
}

TEST(MyopicSteer, RespectsLateralAccelBudget)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 3.0f; // demand far more than possible
	cmd.current_speed_m_s = 8.f;
	cmd.desired_speed_m_s = 8.f;
	cmd.dt_s              = 1.f / 60.f;

	// omega_max = a_lat / v = 8 / 8 = 1.0 rad/s. The slew means the FIRST frame
	// is well under that; run to steady state before asserting the clamp.
	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, cmd);

	EXPECT_LE(std::fabs(r.turn_rate_rad_s), 1.0f + 1e-3f);
}

TEST(MyopicSteer, StandingCreatureCanPivot)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 1.0f;
	cmd.current_speed_m_s = 0.f;   // standing still
	cmd.desired_speed_m_s = 0.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_GT(r.turn_rate_rad_s, 0.f) << "a standing animal must be able to turn";
	EXPECT_TRUE(std::isfinite(r.turn_rate_rad_s));
}

// Added in task 5 after a perturbation test found this behaviour unasserted:
// reintroducing the rejected `max_lateral_accel / 1e-3` divisor at zero speed
// passed the entire suite. That divisor is not a guard, it is a claim -- it
// fabricates an omega_max of ~8000 rad/s and then reports the real demand as a
// comfortable 0.25% of it. Skipping the limit is the settled behaviour: at zero
// speed the centripetal budget does not apply, so there is NO turning-authority
// ratio to report and the turn channel must read exactly "not applicable"
// (headroom 1.0), not "99.75% idle against an invented bound".
TEST(MyopicSteer, StandingCreatureReportsNoFabricatedTurnBudget)
{
	Envelope env = TestEnvelope();      // max_lateral_accel = 8, tau = 0.25
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 5.0f;       // turn_stop_bound = 20 rad/s
	cmd.current_speed_m_s = 0.f;
	cmd.desired_speed_m_s = 0.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	// With the fabricated divisor this reads 1 - 20/8000 = 0.9975.
	EXPECT_NEAR(r.turn_headroom, 1.f, 1e-6f)
		<< "a fabricated omega_max leaked into the turn channel";
	EXPECT_TRUE(std::isfinite(r.turn_headroom));
}

// Added in review round 1 (C2): every test above passed with the exponential
// slew deleted outright (rate = clamp(error/dt, +/-omega_max), no SlewAlpha,
// no Approach, no SteerState) -- the 1-exp(-dt/tau) shape this whole task
// exists to establish was unasserted. This pins the FIRST frame from a
// zeroed state to exactly demand * (1 - exp(-dt/tau)), at two different dt,
// which only holds if the slew is actually applied.
TEST(MyopicSteer, FirstFrameMatchesExponentialSlew)
{
	for (float dt : {1.f / 60.f, 1.f / 30.f}) {
		Envelope env = TestEnvelope();
		SteerState state{}; // zeroed: prev_turn_rate_rad_s == 0

		SteerCommand cmd;
		cmd.angle_error_rad   = 1.0f; // small enough that no bound clips the demand
		cmd.current_speed_m_s = 5.f;
		cmd.desired_speed_m_s = 5.f;
		cmd.dt_s              = dt;

		// Recompute the expected demand exactly as Steer does: greedy,
		// clamped by the lateral-accel budget at this speed.
		const float omega_max = float(env.max_lateral_accel) / cmd.current_speed_m_s;
		const float greedy     = cmd.angle_error_rad / dt;
		const float demand     = std::clamp(greedy, -omega_max, omega_max);
		const float alpha      = 1.f - std::exp(-dt / float(env.tau_linear));
		const float expected   = demand * alpha;

		SteerResult r = Steer(env, state, cmd);
		EXPECT_NEAR(r.turn_rate_rad_s, expected, 1e-4f) << "dt=" << dt;
	}
}

TEST(MyopicStability, CruisingStraightIsComfortable)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;
	cmd.current_speed_m_s = 2.f;   // well under max_speed 10
	cmd.desired_speed_m_s = 2.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_GT(r.stability, 0.7f);
	EXPECT_GT(r.speed_headroom, 0.7f);
}

TEST(MyopicStability, AtMaxSpeedStabilityReachesZero)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;
	cmd.current_speed_m_s = 10.f;  // exactly max_speed
	cmd.desired_speed_m_s = 10.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_NEAR(r.stability, 0.f, 0.02f);
	EXPECT_NEAR(r.speed_headroom, 0.f, 0.02f);
}

TEST(MyopicStability, ExceedingMaxSpeedGoesNegative)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;
	cmd.current_speed_m_s = 15.f;  // 1.5x max_speed
	cmd.desired_speed_m_s = 15.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_LT(r.stability, 0.f);
}

TEST(MyopicStability, DemandingMoreThanTheGaitSuggestsAChange)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;
	cmd.current_speed_m_s = 5.f;
	cmd.desired_speed_m_s = 20.f;  // twice max_speed
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_TRUE(r.suggest_gait_change);
}

// All four tests above hold angle_error_rad == 0 and rely on TestEnvelope's
// min_speed == 0, so u_turn and u_stall are structurally zero throughout and
// only u_speed is ever exercised -- a stub computing solely u_speed would
// pass every test above. These three pin down the turn and stall channels
// specifically, since u_turn's numerator (turn_stop_bound, not turn_demand
// or the delivered turn_rate) was the one real judgment call in this task.
//
// TestEnvelope: max_lateral_accel = 8 m/s^2, tau_linear = 0.25 s, max_speed
// = 10 m/s. At speed = 5 m/s (comfortably inside max_speed, so u_speed = 0.5
// cannot by itself drive stability negative): omega_max = 8/5 = 1.6 rad/s,
// and turn_stop_bound = angle_error_rad / max(tau, dt) = angle_error_rad /
// 0.25 = 4 * angle_error_rad (dt = 1/60 << tau here, so max(tau,dt) == tau).

TEST(MyopicStability, TurnSaturationDrivesStabilityNegative)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 2.0f;  // turn_stop_bound = 8.0 rad/s
	cmd.current_speed_m_s = 5.f;   // omega_max = 1.6 rad/s -> u_turn = 5.0
	cmd.desired_speed_m_s = 5.f;   // u_speed = 0.5, alone cannot go negative
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	EXPECT_LT(r.turn_headroom, 0.f);
	EXPECT_LT(r.stability, 0.f);
	// Isolates the turn term as the cause: u_speed = 0.5 alone bottoms out
	// at stability = 0.5 (positive), and TestEnvelope's min_speed == 0 keeps
	// u_stall structurally zero, so only u_turn can explain a value this
	// far negative. If u_turn were stubbed to 0, stability would be 0.5.
	EXPECT_LT(r.stability, -1.f);
}

TEST(MyopicStability, TurnHeadroomVariesInNormalRegime)
{
	Envelope env = TestEnvelope();
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.2f;  // turn_stop_bound = 0.8 rad/s
	cmd.current_speed_m_s = 5.f;   // omega_max = 1.6 rad/s -> u_turn = 0.5
	cmd.desired_speed_m_s = 5.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	// If u_turn were stubbed to 0, turn_headroom would read exactly 1.0
	// instead of ~0.5 -- outside this tolerance.
	EXPECT_NEAR(r.turn_headroom, 0.5f, 0.05f);
	EXPECT_GT(r.turn_headroom, 0.f);
	EXPECT_LT(r.turn_headroom, 1.f);
}

TEST(MyopicStability, BelowMinSpeedDrivesStabilityNegative)
{
	Envelope env = TestEnvelope();
	env.min_speed = velocity_m_s{3.f}; // e.g. a stall-limited flyer/shark
	SteerState state{};

	SteerCommand cmd;
	cmd.angle_error_rad   = 0.f;   // keep u_turn structurally zero
	cmd.current_speed_m_s = 1.f;   // below min_speed -> u_stall = 3/1 = 3
	cmd.desired_speed_m_s = 1.f;
	cmd.dt_s              = 1.f / 60.f;

	SteerResult r = Steer(env, state, cmd);
	// u_speed = 1/10 = 0.1 alone cannot explain a negative result -- if
	// u_stall were stubbed to 0, stability would read 0.9 (positive).
	EXPECT_LT(r.stability, 0.f);
}


// ---------------------------------------------------------------------------
// Aerial envelope.
//
// The reference flyer here is the DRAGONFLY, not the bat. batto.glb is affected
// by the documented clade-misclassification red (Species.Bat): it is detected as
// bare CHORDATA rather than MAMMALIA, so its metabolic block is built from the
// wrong coefficients. The consequence is that its own analysis layer already
// reports `can_sustain_level_flight == false` and its mechanical cruise power
// (280 W) exceeds its available muscle power (153 W) -- i.e. TonTon says this
// bat cannot fly level, so there is by definition no power surplus with which to
// accelerate. See AerialPresenceFollowsPowerSurplus below, which asserts that
// consistency directly rather than papering over it.
// ---------------------------------------------------------------------------
namespace {

// Closed-form load-factor limit, recomputed in the test from the analysis
// fields so that any change to the implementation's formula (or to the speed it
// is evaluated at) shows up as a failure rather than being absorbed.
float ExpectedLoadFactor(const Analysis_Aerial& a, float g)
{
	const float v_c = float(a.cruise_speed_m_s);
	const float v_s = float(a.min_flight_speed_m_s);
	const float r   = float(a.min_turning_radius_m);
	const float n_aero   = std::pow(v_c / v_s, 2.f);
	const float n_radius = std::sqrt(1.f + std::pow((v_c * v_c) / (g * r), 2.f));
	return std::min(n_aero, n_radius);
}

// Closed-form accelerating budget: the MECHANICAL surplus (muscle mechanical
// output minus the mechanical power level cruise already consumes) turned into
// an acceleration via P = F*v, F = m*a.
float ExpectedMaxAccel(const Output& o)
{
	const auto& a = *o.aerial;
	const float surplus = std::max(0.f,
		float(o.metabolic.available_muscle_power_W)
		- float(a.flapping_power_mechanical_W));
	return surplus / (float(o.physical.body_mass_kg) * float(a.cruise_speed_m_s));
}

} // namespace

TEST(MyopicEnvelope, AerialInvariants)
{
	const Output* out = Analyze("dragonfly.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value()) << "dragonfly should fly";

	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	EXPECT_GT(float(env->max_speed), 0.f);
	EXPECT_GT(float(env->min_speed), 0.f) << "a flyer has a stall speed";
	EXPECT_LT(float(env->min_speed), float(env->max_speed));

	// The derived-from-power acceleration must be real, not zero or NaN.
	EXPECT_GT(float(env->max_accel), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->max_accel)));
	EXPECT_GT(float(env->max_brake), 0.f) << "a flyer that cannot decelerate cannot be steered";
	EXPECT_GT(float(env->tau_linear), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->tau_linear)));

	// A flyer always has a finite minimum turn radius (it cannot pivot in place,
	// unlike a standing quadruped, for which 0 is legitimate).
	EXPECT_GT(float(env->min_turn_radius), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->min_turn_radius)));

	ASSERT_TRUE(env->aerial.has_value());
	EXPECT_GE(env->aerial->n_max, 1.f) << "load factor cannot be below 1";
	EXPECT_TRUE(std::isfinite(env->aerial->n_max));
	EXPECT_GT(float(env->aerial->max_roll_rate), 0.f);
	EXPECT_GT(float(env->aerial->max_yaw_rate), 0.f);

	// UPSTREAM GAP: max_pitch_rate_rad_s is 0 for every sample -- the analysis
	// layer never populates it (max_roll_rate_rad_s and max_yaw_rate_rad_s are
	// populated, pitch is not). Do NOT assert > 0: that would be asserting a
	// number TonTon does not compute. Assert only that it is a usable float, and
	// leave Task 5 to route around a zero pitch authority explicitly.
	EXPECT_GE(float(env->aerial->max_pitch_rate), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->aerial->max_pitch_rate)));

	// AerialAuthority's speeds must be the same numbers the envelope reports.
	EXPECT_FLOAT_EQ(float(env->aerial->stall_speed), float(env->min_speed));
	EXPECT_LT(float(env->aerial->stall_speed), float(env->aerial->cruise_speed));
	EXPECT_LT(float(env->aerial->cruise_speed), float(env->max_speed));

	// n_max == 1 exactly means g*sqrt(n^2-1) == 0: an animal that flies but
	// cannot turn. That is never a physical answer, only a symptom of a bad
	// load-factor input being caught by a floor. Assert it here so the fallback
	// can never silently hide a non-physical intermediate again.
	EXPECT_GT(env->aerial->n_max, 1.f) << "a flyer whose load factor floors at 1 cannot bank";
	EXPECT_GT(float(env->max_lateral_accel), 0.f) << "a flyer must be able to turn";
	EXPECT_TRUE(std::isfinite(float(env->max_lateral_accel)));
}

// Pins the load factor to its closed form, computed here from the analysis
// fields (no species-specific magic numbers). Run at two gravities so that both
// budgets get to be the binding one: at 9.81 the stated-radius budget binds for
// this sample, at lunar gravity the radius budget relaxes and the aerodynamic
// V-n budget binds. Without the second case a change to the aerodynamic branch
// alone (e.g. evaluating it at max speed rather than cruise) would be invisible.
TEST(MyopicEnvelope, AerialLoadFactorMatchesClosedForm)
{
	const Output* out = Analyze("dragonfly.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());
	const auto& a = *out->aerial;

	for (float g : {9.81f, 1.62f}) {
		auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, g);
		ASSERT_TRUE(env.has_value()) << "g = " << g;
		ASSERT_TRUE(env->aerial.has_value()) << "g = " << g;
		const float expected = ExpectedLoadFactor(a, g);
		EXPECT_NEAR(env->aerial->n_max, expected, 1e-3f * std::max(1.f, expected))
			<< "g = " << g << ", n_max = " << env->aerial->n_max
			<< ", closed form = " << expected;

		// max_lateral_accel must be the banking identity of that same n_max, and
		// min_turn_radius must be reconciled with it (never optimistic).
		EXPECT_NEAR(float(env->max_lateral_accel),
		            g * std::sqrt(std::max(0.f, expected * expected - 1.f)),
		            1e-2f * g) << "g = " << g;
		EXPECT_GE(float(env->min_turn_radius),
		          std::pow(float(a.cruise_speed_m_s), 2.f) / float(env->max_lateral_accel) - 1e-3f)
			<< "stated turn radius is tighter than the load factor allows, g = " << g;
	}
}

// Pins max_accel to its closed form. Kills both "AccelFromPower returns a
// constant" and "AccelFromPower divides by mass^2": neither reproduces
// surplus / (m * v_cruise).
TEST(MyopicEnvelope, AerialMaxAccelMatchesClosedForm)
{
	const Output* out = Analyze("dragonfly.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());

	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	const float expected = ExpectedMaxAccel(*out);
	ASSERT_GT(expected, 0.f) << "sample has no power surplus; pick another flyer";
	EXPECT_NEAR(float(env->max_accel), expected, 1e-3f * expected)
		<< "max_accel = " << float(env->max_accel) << ", closed form = " << expected;

	// tau is that acceleration worked against cruise speed.
	EXPECT_NEAR(float(env->tau_linear),
	            float(out->aerial->cruise_speed_m_s) / expected,
	            1e-3f * float(out->aerial->cruise_speed_m_s) / expected);
}

TEST(MyopicEnvelope, AerialAccelerationIsPlausible)
{
	const Output* out = Analyze("dragonfly.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	// A flying animal accelerates within roughly an order of magnitude of g.
	// Far outside this band means the P = F*v derivation has a units error.
	EXPECT_GT(float(env->max_accel), 0.1f);
	EXPECT_LT(float(env->max_accel), 100.f);

	// Banked turns of biological flyers sit around 1.5-4 g; dragonflies are the
	// extreme end, measured at 3-9 g in prey-capture turns. Past ~10 g would
	// mean the load-factor derivation has lost its footing.
	ASSERT_TRUE(env->aerial.has_value());
	EXPECT_LT(env->aerial->n_max, 10.f) << "n_max = " << env->aerial->n_max;
}

// The bat: an envelope exists if and only if there is a mechanical power
// surplus. This is deliberately written as a consistency check rather than
// "batto returns nullopt", so that it stays correct once the Species.Bat clade
// bug is fixed and the bat acquires a real surplus.
TEST(MyopicEnvelope, AerialPresenceFollowsPowerSurplus)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());

	const float expected = ExpectedMaxAccel(*out);
	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	EXPECT_EQ(env.has_value(), expected > 0.f)
		<< "surplus-derived accel = " << expected;

	// And the surplus must agree with the analysis layer's own verdict: no
	// surplus <=> it says the animal cannot hold level flight.
	EXPECT_EQ(expected > 0.f, out->aerial->can_sustain_level_flight)
		<< "envelope power budget disagrees with can_sustain_level_flight";
}


// ---------------------------------------------------------------------------
// Bank-versus-yaw decision and the load-factor stall coupling.
//
// Two synthetic flyers with deliberately opposite authority profiles. Both are
// given the SAME load-factor ceiling and the same lateral budget, so the only
// thing that can decide between banking and yawing is the TIME each axis needs
// -- which is the whole point of the decision rule.
// ---------------------------------------------------------------------------
namespace {

Envelope DragonflyLikeEnvelope()   // huge yaw authority, tiny radius
{
	Envelope e = TestEnvelope();
	e.min_speed = velocity_m_s{1.f};
	AerialAuthority a;
	a.max_roll_rate  = omega_rad_s{2.1f};
	a.max_pitch_rate = omega_rad_s{0.4f};
	a.max_yaw_rate   = omega_rad_s{4.1f};
	a.n_max          = 2.f;
	a.stall_speed    = velocity_m_s{1.f};
	a.cruise_speed   = velocity_m_s{4.f};
	e.aerial = a;
	e.max_lateral_accel = acceleration_m_s2{9.81f * std::sqrt(3.f)};
	return e;
}

Envelope AlbatrossLikeEnvelope()   // negligible yaw, strong roll
{
	Envelope e = TestEnvelope();
	// TestEnvelope's max_speed of 10 m/s is below this flyer's own cruise speed
	// of 15, which made u_speed read 1.5 and suggest_gait_change fire
	// unconditionally at its own cruise -- an incoherent fixture. 20 m/s puts
	// cruise comfortably inside the envelope. Verified to move no assertion in
	// any existing test (see task-5-fix-report.md).
	e.max_speed = velocity_m_s{20.f};
	e.min_speed = velocity_m_s{8.f};
	AerialAuthority a;
	a.max_roll_rate  = omega_rad_s{1.5f};
	a.max_pitch_rate = omega_rad_s{0.3f};
	a.max_yaw_rate   = omega_rad_s{0.2f};
	a.n_max          = 2.f;
	a.stall_speed    = velocity_m_s{8.f};
	a.cruise_speed   = velocity_m_s{15.f};
	e.aerial = a;
	e.max_lateral_accel = acceleration_m_s2{9.81f * std::sqrt(3.f)};
	return e;
}

SteerCommand TurnCommand(float error, float speed)
{
	SteerCommand c;
	c.angle_error_rad   = error;
	c.current_speed_m_s = speed;
	c.desired_speed_m_s = speed;
	c.dt_s              = 1.f / 60.f;
	c.gravity_m_s2      = 9.81f;
	return c;
}

} // namespace

TEST(MyopicBank, DragonflyYawsForTheSameTurnAnAlbatrossBanks)
{
	Envelope dragonfly = DragonflyLikeEnvelope();
	Envelope albatross = AlbatrossLikeEnvelope();
	SteerState s1{}, s2{};

	SteerResult rd = Steer(dragonfly, s1, TurnCommand(1.57f, 4.f));
	SteerResult ra = Steer(albatross, s2, TurnCommand(1.57f, 15.f));

	EXPECT_EQ(rd.strategy, TurnStrategy::YAW);
	EXPECT_EQ(ra.strategy, TurnStrategy::BANK);
}

TEST(MyopicStall, HardTurnNearStallDrivesStabilityNegative)
{
	Envelope env = AlbatrossLikeEnvelope();
	SteerState state{};

	// Flying just above stall (8.0) and demanding a hard turn. The load factor
	// raises the effective stall speed by sqrt(n), so this must go unstable.
	//
	// MEASURED (see task-5-report.md): n(8.6) = 1.1556, phi_max = 0.5250 rad,
	// effective stall speed 8.600 m/s, u_stall = 1.0000, u_turn = 18.16,
	// stability = -17.16. Note WHICH channel supplies the negativity: with the
	// speed-dependent load factor the stall term saturates at exactly 1 here
	// (see BankingIsCappedByTheLoadFactorTheSpeedAllows below for why that is
	// an identity, not a coincidence), so the demand that cannot be met is
	// surfaced by the turn channel. Both are real; do not read this assertion
	// as isolating the stall term.
	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(3.0f, 8.6f));

	EXPECT_EQ(r.strategy, TurnStrategy::BANK);
	EXPECT_GT(r.bank_angle_rad, 0.f);
	EXPECT_LT(r.stability, 0.f) << "banking near stall must be unstable";

	// The bank is held down to what 8.6 m/s can actually support, not to the
	// cruise-time n_max = 2 (which would allow acos(1/2) = 1.047 rad).
	EXPECT_NEAR(r.bank_angle_rad, std::acos(1.f / std::pow(8.6f / 8.f, 2.f)), 1e-3f);
	EXPECT_LT(r.bank_angle_rad, std::acos(0.5f) - 0.4f);
}

// Isolates the stall channel from the turn channel, and pins the identity the
// speed-dependent load factor creates.
//
// The aerodynamic boundary is n(v) = (v/Vs)^2, and banking raises the stall
// speed to Vs/sqrt(cos phi) with cos phi = 1/n. Substituting: the effective
// stall speed at the maximum permitted bank is Vs / (Vs/v) = v EXACTLY. So a
// flyer banking as hard as its speed allows sits precisely ON its stall
// boundary -- u_stall == 1, the "at the limit" reading -- and can never be
// driven past it by banking, because the bank is capped by the very load
// factor that defines the boundary. Falling BELOW the stall speed is what puts
// it past the limit, and there no bank is available at all.
//
// The identity is asserted on the EFFECTIVE STALL SPEED implied by the bank
// actually achieved, not through `stability`. Reason: after the G1 fix the bank
// target is derived from the turn DEMAND (phi = atan(omega_desired * v / g)),
// so pegging the bank at phi_max requires a demand at or beyond the banked-turn
// authority -- which necessarily makes u_turn >= 1 and pins stability at or
// below zero for a reason that has nothing to do with stall. Reading u_stall
// off `stability` at max bank is therefore no longer possible; reading it off
// the reported bank angle is exact.
TEST(MyopicStall, BankingIsCappedByTheLoadFactorTheSpeedAllows)
{
	const float Vs = 8.f;   // AlbatrossLikeEnvelope stall speed
	auto probe = [](float v) {
		Envelope env = AlbatrossLikeEnvelope();
		SteerState state{};
		SteerResult r{};
		// error 3.0 with tau 0.25 gives a demand of 12 rad/s, far beyond any
		// banked-turn authority here, so the bank pegs at phi_max.
		for (int i = 0; i < 600; ++i) r = Steer(env, state, TurnCommand(3.0f, v));
		return r;
	};
	// Effective stall speed under load factor n = 1/cos(phi): Vs / sqrt(cos phi).
	auto v_stall_eff = [Vs](float phi) { return Vs / std::sqrt(std::cos(phi)); };

	// Above stall, banking at the limit: the effective stall speed rises to
	// EXACTLY the current airspeed, i.e. u_stall == 1, sitting on the boundary.
	SteerResult above = probe(8.6f);
	EXPECT_EQ(above.strategy, TurnStrategy::BANK);
	EXPECT_NEAR(above.bank_angle_rad, std::acos(1.f / std::pow(8.6f / Vs, 2.f)), 1e-3f);
	EXPECT_NEAR(v_stall_eff(above.bank_angle_rad), 8.6f, 1e-3f)
		<< "max-bank at speed v sits exactly on the stall boundary";

	// Same again well above stall: the ceiling n_max binds instead of the
	// v^2 boundary, so there is genuine margin left (u_stall < 1).
	SteerResult fast = probe(15.f);
	EXPECT_NEAR(fast.bank_angle_rad, std::acos(0.5f), 1e-3f);
	EXPECT_LT(v_stall_eff(fast.bank_angle_rad), 15.f)
		<< "n_max binds, so stall margin remains";

	// Below the stall speed: no bank is available (n(v) floors at 1) and the
	// creature is past the floor on speed alone.
	SteerResult below = probe(7.f);
	EXPECT_NEAR(below.bank_angle_rad, 0.f, 1e-4f);
	EXPECT_LT(below.stability, 0.f) << "below stall speed is past the limit";
}

// The bank target is the bank that DELIVERS the demanded turn, capped by
// phi_max -- not phi_max unconditionally. A moderate turn is a moderate bank,
// and then the stall margin is genuinely intact (u_stall < 1).
TEST(MyopicStall, AModerateTurnIsAModerateBankAndKeepsStallMargin)
{
	Envelope env = AlbatrossLikeEnvelope();
	SteerState state{};

	// error 0.25 rad, tau 0.25 -> demand 1.0 rad/s, inside the banked authority
	// at 12 m/s (1.416 rad/s), so the bank settles strictly below phi_max.
	SteerResult r{};
	for (int i = 0; i < 600; ++i) r = Steer(env, state, TurnCommand(0.25f, 12.f));

	ASSERT_EQ(r.strategy, TurnStrategy::BANK);
	EXPECT_GT(r.bank_angle_rad, 0.f);
	EXPECT_LT(r.bank_angle_rad, std::acos(0.5f) - 0.1f)
		<< "a sub-authority turn demand must not peg the bank at the load-factor limit";
	// The bank is exactly the one that delivers the demanded turn rate.
	EXPECT_NEAR(r.turn_rate_rad_s, 1.0f, 1e-3f);
	EXPECT_NEAR(r.bank_angle_rad, std::atan(1.0f * 12.f / 9.81f), 1e-3f);
	EXPECT_GT(r.stability, 0.f) << "a moderate turn well above stall keeps its margin";
}

TEST(MyopicStall, SameSpeedWingsLevelIsStable)
{
	Envelope env = AlbatrossLikeEnvelope();
	SteerState state{};

	// Identical speed, no turn demanded -> comfortably above stall.
	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(0.f, 8.6f));

	EXPECT_GT(r.stability, 0.f) << "wings-level at the same speed must be fine";
}

// ---------------------------------------------------------------------------
// Additions beyond the plan's four assertions. Each one exists because the plan
// tests above survive a specific stub (see task-5-report.md's perturbation
// table); these are the ones that do not.
// ---------------------------------------------------------------------------

// The load factor is a function of SPEED, not a stored cruise-time constant.
// The aerodynamic V-n boundary is n(v) = (v/Vstall)^2, so AT the stall speed
// the available load factor is exactly 1 -- zero bank angle, zero banked turn
// rate. A flyer scraping along at stall cannot bank at all and must yaw.
//
// This is the test that fails if n(v) is stubbed to n_max: with n_max = 2 the
// albatross would find t_bank = 1.44 s against t_yaw = 7.85 s and choose BANK.
TEST(MyopicBank, LoadFactorFallsToOneAtStallSpeed)
{
	Envelope env = AlbatrossLikeEnvelope();   // stall 8, n_max 2, yaw 0.2
	SteerState state{};

	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(1.57f, 8.f));

	EXPECT_EQ(r.strategy, TurnStrategy::YAW)
		<< "at exactly the stall speed the available load factor is 1: no bank is possible";
	EXPECT_NEAR(r.bank_angle_rad, 0.f, 1e-4f);

	// ...and well above stall the same flyer, same demand, banks. Same envelope,
	// same error: only the speed differs, so only n(v) can explain the flip.
	SteerState fast{};
	SteerResult rf{};
	for (int i = 0; i < 300; ++i) rf = Steer(env, fast, TurnCommand(1.57f, 15.f));
	EXPECT_EQ(rf.strategy, TurnStrategy::BANK);
	EXPECT_GT(rf.bank_angle_rad, 0.5f);
}

// Roll-in is a process. The bank angle must arrive at its target over several
// frames at no more than max_roll_rate, never in one step. Fails outright if
// the slew is removed and bank_angle_rad snaps to phi_target.
TEST(MyopicBank, RollInIsAProcessNotAnInstant)
{
	Envelope env = AlbatrossLikeEnvelope(); // max_roll_rate = 1.5 rad/s
	SteerState state{};
	const float dt = 1.f / 60.f;
	const float max_step = 1.5f * dt;       // 0.025 rad

	float prev = 0.f;
	for (int i = 0; i < 10; ++i) {
		SteerResult r = Steer(env, state, TurnCommand(1.57f, 15.f));
		ASSERT_LE(r.bank_angle_rad - prev, max_step + 1e-5f)
			<< "bank angle jumped faster than max_roll_rate at step " << i;
		ASSERT_GT(r.bank_angle_rad, prev - 1e-6f) << "bank angle went backwards at step " << i;
		prev = r.bank_angle_rad;
	}
	// After 10 frames at 1.5 rad/s it can only have reached ~0.25 rad, nowhere
	// near phi_max = acos(1/2) = 1.047 rad.
	EXPECT_NEAR(prev, 10.f * max_step, 1e-3f);
	EXPECT_LT(prev, 0.5f);

	// Given enough frames it does arrive at the load-factor limit.
	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(1.57f, 15.f));
	EXPECT_NEAR(r.bank_angle_rad, std::acos(0.5f), 1e-3f);
}

// The bank angle is SIGNED by the heading error, so a renderer can use it
// directly and a reversal rolls back through wings-level instead of teleporting.
TEST(MyopicBank, BankAngleIsSignedByTheHeadingError)
{
	Envelope env = AlbatrossLikeEnvelope();
	SteerState left{}, right{};

	SteerResult rl{}, rr{};
	for (int i = 0; i < 300; ++i) {
		rl = Steer(env, left,  TurnCommand(+1.57f, 15.f));
		rr = Steer(env, right, TurnCommand(-1.57f, 15.f));
	}

	EXPECT_EQ(rl.strategy, TurnStrategy::BANK);
	EXPECT_EQ(rr.strategy, TurnStrategy::BANK);
	EXPECT_GT(rl.bank_angle_rad, 0.f);
	EXPECT_LT(rr.bank_angle_rad, 0.f);
	EXPECT_NEAR(rl.bank_angle_rad, -rr.bank_angle_rad, 1e-4f);

	// The turn itself follows the error's sign, and the two are mirror images:
	// nothing about the maneuver may depend on which way it goes.
	EXPECT_GT(rl.turn_rate_rad_s, 0.f);
	EXPECT_LT(rr.turn_rate_rad_s, 0.f);
	EXPECT_NEAR(rl.turn_rate_rad_s, -rr.turn_rate_rad_s, 1e-4f);
	EXPECT_NEAR(rl.stability, rr.stability, 1e-4f);

	// Reversing the demand rolls back THROUGH wings-level rather than jumping
	// to the mirrored bank: there must be a frame where |bank| is small.
	float min_abs_bank = std::fabs(rl.bank_angle_rad);
	for (int i = 0; i < 300; ++i) {
		SteerResult r = Steer(env, left, TurnCommand(-1.57f, 15.f));
		min_abs_bank = std::min(min_abs_bank, std::fabs(r.bank_angle_rad));
	}
	EXPECT_LT(min_abs_bank, 0.05f) << "bank teleported across wings-level";
}

// Framerate independence of the NEW state variable. Verified empirically at
// four timesteps rather than asserted from the formula, per the brief.
TEST(MyopicBank, RollInIsFramerateIndependent)
{
	auto bank_after = [](float dt, float wall_clock_s) {
		Envelope env = AlbatrossLikeEnvelope();
		SteerState state{};
		SteerResult r{};
		const int steps = static_cast<int>(std::lround(wall_clock_s / dt));
		for (int i = 0; i < steps; ++i) {
			SteerCommand cmd = TurnCommand(1.57f, 15.f);
			cmd.dt_s = dt;
			r = Steer(env, state, cmd);
		}
		return r.bank_angle_rad;
	};

	// 0.5 s of roll-in at 1.5 rad/s = 0.75 rad, still short of phi_max (1.047),
	// so the answer is genuinely mid-slew and not pinned by the clamp.
	const float t = 0.5f;
	const float b16  = bank_after(1.f / 16.f,  t);
	const float b30  = bank_after(1.f / 30.f,  t);
	const float b60  = bank_after(1.f / 60.f,  t);
	const float b120 = bank_after(1.f / 120.f, t);

	// Bracket it: genuinely mid-slew, neither pinned at phi_max nor stuck at 0
	// (all-zeros would otherwise make the agreement assertions vacuous).
	EXPECT_NEAR(b60, 1.5f * t, 0.03f);
	EXPECT_LT(b60, std::acos(0.5f) - 0.05f) << "clamped: pick a shorter wall clock";
	EXPECT_NEAR(b16,  b60, 0.02f);
	EXPECT_NEAR(b30,  b60, 0.02f);
	EXPECT_NEAR(b120, b60, 0.02f);
}

// Non-aerial envelopes keep reporting GROUND and never acquire a bank angle:
// the aerial path must not leak into the terrestrial one.
TEST(MyopicBank, GroundEnvelopesStayGround)
{
	Envelope env = TestEnvelope(); // no aerial authority
	SteerState state{};

	SteerResult r{};
	for (int i = 0; i < 60; ++i) r = Steer(env, state, TurnCommand(1.57f, 5.f));

	EXPECT_EQ(r.strategy, TurnStrategy::LATERAL);
	EXPECT_FLOAT_EQ(r.bank_angle_rad, 0.f);
}

// The stall channel at and below a standstill. The old form read
// min_speed/speed with a `speed > 1e-3 ? ... : 0` guard, so a creature with a
// speed floor sitting at ZERO speed -- the worst possible stall -- reported as
// perfectly idle. It must instead read as fully saturated or worse, stay
// finite (an infinite stability poisons every downstream consumer), and be
// MONOTONE: slower must never read better than faster.
TEST(MyopicStall, StandstillWithASpeedFloorIsFullySaturated)
{
	Envelope env = TestEnvelope();
	env.min_speed = velocity_m_s{3.f};

	auto stability_at = [&](float v) {
		SteerState state{};
		SteerCommand cmd = TurnCommand(0.f, v); // zero error -> u_turn is 0
		return Steer(env, state, cmd).stability;
	};

	const float s0   = stability_at(0.f);
	const float s015 = stability_at(0.15f);
	const float s150 = stability_at(1.5f);
	const float s300 = stability_at(3.0f);   // exactly the floor
	const float s600 = stability_at(6.0f);

	EXPECT_TRUE(std::isfinite(s0)) << "stability must stay finite at rest";
	EXPECT_LT(s0, 0.f) << "at rest with a speed floor is past the limit, not idle";

	// Monotone in speed all the way down to zero -- no hole, no inversion.
	EXPECT_LT(s0,   s015);
	EXPECT_LT(s015, s150);
	EXPECT_LT(s150, s300);
	EXPECT_LT(s300, s600);

	// At exactly the floor the stall term is 1 ("at the limit"), so stability
	// there is governed by u_speed (3/10) alone: 1 - max(0.3, 1) = 0.
	EXPECT_NEAR(s300, 0.f, 1e-4f);
}

// ---------------------------------------------------------------------------
// Review round: turn rate under BANK is the KINEMATIC CONSEQUENCE of the bank,
// not an independently commanded channel.
// ---------------------------------------------------------------------------

// The bug this pins: a left->right reversal used to report turn_rate == 0 for
// ~15 frames while bank_angle_rad was still +0.77 to +1.02 rad. A consumer
// rolling the mesh by bank_angle_rad and yawing by turn_rate_rad_s would draw a
// 59-degree-banked flyer travelling in a dead straight line. A bank angle FORCES
// a turn rate: omega = g tan(phi) / v. Right-banked always means right-turning.
TEST(MyopicBank, TurnRateIsTheKinematicConsequenceOfBank)
{
	Envelope env = AlbatrossLikeEnvelope();
	SteerState state{};
	const float v = 15.f;
	const float g = 9.81f;

	SteerResult r{};
	for (int i = 0; i < 600; ++i) r = Steer(env, state, TurnCommand(+1.57f, v));
	ASSERT_EQ(r.strategy, TurnStrategy::BANK);
	ASSERT_NEAR(r.bank_angle_rad, std::acos(0.5f), 1e-3f);

	// Reverse the demand and watch every frame of the roll-through.
	int banked_but_straight = 0;
	int sign_disagreements  = 0;
	float worst_residual    = 0.f;
	for (int i = 0; i < 300; ++i) {
		r = Steer(env, state, TurnCommand(-1.57f, v));
		ASSERT_EQ(r.strategy, TurnStrategy::BANK);

		const float expected = g * std::tan(r.bank_angle_rad) / v;
		worst_residual = std::max(worst_residual,
		                          std::fabs(r.turn_rate_rad_s - expected));
		if (std::fabs(r.bank_angle_rad) > 0.05f) {
			if (std::fabs(r.turn_rate_rad_s) < 1e-4f) ++banked_but_straight;
			if (r.bank_angle_rad * r.turn_rate_rad_s <= 0.f) ++sign_disagreements;
		}
	}

	EXPECT_EQ(banked_but_straight, 0)
		<< "frames claiming a banked flyer travelling in a straight line";
	EXPECT_EQ(sign_disagreements, 0)
		<< "frames where the bank and the turn disagree about direction";
	EXPECT_LT(worst_residual, 1e-4f)
		<< "turn rate is not g*tan(bank)/v";

	// Ends up mirrored, still consistent.
	EXPECT_NEAR(r.bank_angle_rad, -std::acos(0.5f), 1e-3f);
	EXPECT_LT(r.turn_rate_rad_s, 0.f);
}

// Perturbation N5 from the review: sourcing the reported turn rate from the
// bank TARGET rather than the bank ACHIEVED survived the whole suite. Mid
// roll-in the flyer genuinely cannot turn as hard as it eventually will.
TEST(MyopicBank, ReportedTurnRateLagsRollIn)
{
	Envelope env = AlbatrossLikeEnvelope();
	const float v = 15.f;
	const float g = 9.81f;

	SteerState mid{};
	SteerResult rm{};
	for (int i = 0; i < 10; ++i) rm = Steer(env, mid, TurnCommand(1.57f, v));

	SteerState done{};
	SteerResult rs{};
	for (int i = 0; i < 600; ++i) rs = Steer(env, done, TurnCommand(1.57f, v));

	ASSERT_EQ(rm.strategy, TurnStrategy::BANK);
	ASSERT_EQ(rs.strategy, TurnStrategy::BANK);
	ASSERT_LT(rm.bank_angle_rad, rs.bank_angle_rad); // genuinely mid-roll-in

	EXPECT_LT(rm.turn_rate_rad_s, rs.turn_rate_rad_s * 0.5f)
		<< "mid roll-in turn rate must lag the settled rate, not match it";
	EXPECT_NEAR(rm.turn_rate_rad_s, g * std::tan(rm.bank_angle_rad) / v, 1e-4f);
	EXPECT_NEAR(rs.turn_rate_rad_s, g * std::tan(rs.bank_angle_rad) / v, 1e-4f);
}

// Perturbation N2 from the review: deleting `err / omega_bank` from t_bank --
// so the bank/yaw decision no longer depends on the heading error at all --
// survived the whole suite. t_bank is a time to COMPLETE the turn, not merely a
// time to roll in: rolling in costs a fixed 0.70 s here, so it only pays off
// once the turn is long enough for the higher banked rate to earn that back.
//
// yaw is raised to 0.8 rad/s (from the albatross fixture's 0.2) purely to widen
// the discriminating window: BANK requires err > 1.90 rad with the completion
// term and only err > 0.56 rad without it, so 1.0 rad separates them cleanly.
TEST(MyopicBank, StrategyAccountsForTimeToCompleteTheTurn)
{
	Envelope env = AlbatrossLikeEnvelope();
	env.aerial->max_yaw_rate = omega_rad_s{0.8f};
	const float v = 15.f;

	// Small turn: rolling in (0.70 s) costs more than the whole yawed turn
	// (1.25 s vs 1.58 s banked). Yaw.
	SteerState small_state{};
	SteerResult small{};
	for (int i = 0; i < 300; ++i) small = Steer(env, small_state, TurnCommand(1.0f, v));
	EXPECT_EQ(small.strategy, TurnStrategy::YAW)
		<< "a short turn does not repay the roll-in";

	// Large turn on the SAME flyer at the SAME speed: only the error differs,
	// so only the completion term can explain the flip. Bank.
	SteerState big_state{};
	SteerResult big{};
	for (int i = 0; i < 300; ++i) big = Steer(env, big_state, TurnCommand(3.0f, v));
	EXPECT_EQ(big.strategy, TurnStrategy::BANK)
		<< "a long turn repays the roll-in";
}

// G1 follow-up: the anti-overshoot bound now shapes the bank TARGET rather than
// the delivered rate, and roll-in lag sits between them. Verified by integrating
// a real maneuver at six timesteps rather than asserted.
//
// Renamed in review round 2: it never asserted the ABSENCE of overshoot -- it
// asserts overshoot is BOUNDED (>= -0.06 rad on a 1.57 rad maneuver), that the
// excursion is a single lobe (<= 1 sign crossing), and that the maneuver
// converges. The old name claimed the opposite of what the block comment below
// says honestly.
//
// It also now carries the dt-parameterised pin for G1c (`turn_follows_bank`
// keyed on the BANK, not on the strategy). That predicate was completely
// unpinned: reverting it to `strategy == TurnStrategy::BANK` left all 42 tests
// green, because the pathology is framerate-dependent AND non-monotone in dt,
// so no single-rate test can see it. Frames with |bank| > 0.05 rad reporting
// |turn_rate| < 1e-4 on this exact maneuver:
//
//     dt      fixed   G1c reverted   pre-G1
//     1/16      0          0            5
//     1/30      0          0           10
//     1/60      0          0           19   <-- the only rate the old test ran
//     1/120     0         22           38
//     1/240     0          0           76
//     1/480     0         80          153
//
// Hence the rate list below must stay at six rates spanning 16..480 Hz.
TEST(MyopicBank, BankedTurnOvershootIsBoundedAndSingleLobed)
{
	auto run = [](float dt) {
		Envelope env = AlbatrossLikeEnvelope();
		// max_yaw_rate = 0 makes t_yaw infinite, so BANK is chosen for the
		// WHOLE maneuver including the tail. Without this the last ~0.17 rad
		// is handed back to YAW and the test measures the handoff rather than
		// the banked law.
		env.aerial->max_yaw_rate = omega_rad_s{0.f};
		SteerState state{};
		float error = 1.57f;
		float worst = 0.f;   // most negative error reached
		int   flips = 0;     // heading-error sign crossings
		int   banked_but_straight = 0;  // G1c pin, see comment above
		const int steps = static_cast<int>(std::lround(12.f / dt));
		for (int i = 0; i < steps; ++i) {
			SteerCommand cmd = TurnCommand(error, 15.f);
			cmd.dt_s = dt;
			SteerResult r = Steer(env, state, cmd);
			if (std::fabs(r.bank_angle_rad) > 0.05f &&
			    std::fabs(r.turn_rate_rad_s) < 1e-4f) ++banked_but_straight;
			const float prev = error;
			error -= r.turn_rate_rad_s * dt;
			if (prev * error < 0.f) ++flips;
			worst = std::min(worst, error);
		}
		return std::tuple<float, float, int, int>{error, worst, flips,
		                                          banked_but_straight};
	};

	// MEASURED (task-5-fix-report.md), overshoot in rad on a 1.57 rad maneuver:
	//   16 Hz -0.0464 | 30 Hz -0.0339 | 60 Hz -0.0268 | 120 Hz -0.0231 | 480 Hz -0.0201
	// BANK therefore DOES overshoot, by 1.3-3.0%, and this is reported rather
	// than clamped away: the bound shapes the bank TARGET, and a roll-rate-
	// limited flyer physically cannot unwind its bank the instant the error
	// reaches zero, so it keeps turning while it rolls out. Adding a clamp on
	// the delivered rate to hide it would recreate the "banked flyer flying
	// straight" contradiction this whole fix removed. It converges with dt to
	// ~-0.020 rad, is a SINGLE lobe (one sign crossing, then monotone), and
	// settles at ~1e-19.
	for (float dt : {1.f / 16.f, 1.f / 30.f, 1.f / 60.f,
	                 1.f / 120.f, 1.f / 240.f, 1.f / 480.f}) {
		auto [final_err, worst, flips, banked_but_straight] = run(dt);
		EXPECT_GE(worst, -0.06f) << "banked turn overshot too far at dt=" << dt;
		EXPECT_LE(flips, 1) << "banked turn oscillated at dt=" << dt;
		EXPECT_NEAR(final_err, 0.f, 1e-3f) << "did not converge at dt=" << dt;
		EXPECT_EQ(banked_but_straight, 0)
			<< "frames claiming a banked flyer travelling in a straight line, "
			   "at dt=" << dt;
	}
}

// G3: a flat/skidding yawed turn still has to be paid for with lateral force.
// It is the WEAKEST lateral mechanism a flyer has, so max_yaw_rate alone is not
// a bound -- the turn must also fit inside a_lat(v) = g*sqrt(n(v)^2 - 1).
TEST(MyopicBank, YawIsBoundedByTheLateralForceBudget)
{
	Envelope env = DragonflyLikeEnvelope();
	env.aerial->max_yaw_rate = omega_rad_s{20.f};  // absurd nominal yaw authority
	SteerState state{};
	const float v = 4.f;

	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(1.57f, v));

	ASSERT_EQ(r.strategy, TurnStrategy::YAW);
	// n(4) clamps to n_max = 2 -> a_lat = g*sqrt(3) = 16.99, /v = 4.248 rad/s.
	const float a_lat = 9.81f * std::sqrt(3.f);
	EXPECT_NEAR(r.turn_rate_rad_s, a_lat / v, 1e-3f)
		<< "yaw rate escaped the lateral-force budget";
	EXPECT_LT(r.turn_headroom, 0.f)
		<< "the demand exceeds the lateral budget, so the turn channel is saturated";
}

// ---------------------------------------------------------------------------
// G2: zero turn authority is not "no limit" and is not "idle".
// ---------------------------------------------------------------------------

// A flyer with no yaw authority and no roll authority, MOVING. It has no way to
// turn at all. Previously omega_max == 0 was read as "no limit applies", the
// demand clamp was skipped entirely, and the creature pirouetted at 6.28 rad/s
// (the anti-overshoot bound, which is not an authority bound) while reporting
// turn_headroom = +1.0, i.e. perfectly idle.
TEST(MyopicSteer, FlyerWithNoTurnAuthorityCannotTurn)
{
	Envelope env = AlbatrossLikeEnvelope();
	env.aerial->max_yaw_rate  = omega_rad_s{0.f};
	env.aerial->max_roll_rate = omega_rad_s{0.f};
	SteerState state{};

	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(1.57f, 15.f));

	EXPECT_NEAR(r.turn_rate_rad_s, 0.f, 1e-6f)
		<< "a flyer with no turn authority must not pirouette";
	EXPECT_NEAR(r.bank_angle_rad, 0.f, 1e-6f);
	EXPECT_LE(r.turn_headroom, 0.f)
		<< "zero authority against a real demand is saturated, not idle";
	EXPECT_TRUE(std::isfinite(r.turn_headroom));
	EXPECT_LT(r.stability, 0.f);
}

// Same hole on the ground path: a moving creature with no lateral budget.
TEST(MyopicSteer, GroundCreatureWithNoLateralBudgetCannotTurn)
{
	Envelope env = TestEnvelope();
	env.max_lateral_accel = acceleration_m_s2{0.f};
	SteerState state{};

	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, TurnCommand(1.57f, 5.f));

	EXPECT_NEAR(r.turn_rate_rad_s, 0.f, 1e-6f)
		<< "no lateral budget while moving means no turn";
	EXPECT_LE(r.turn_headroom, 0.f);
	EXPECT_TRUE(std::isfinite(r.turn_headroom));
}

// ...but the zero-authority reading must NOT swallow the settled standing case:
// below kSpeedEpsilon no lateral limit applies at all (a standing animal pivots
// in place), so the turn channel reports "not applicable", not "saturated".
TEST(MyopicSteer, ZeroAuthorityReadingDoesNotSwallowTheStandingCase)
{
	Envelope env = TestEnvelope();
	env.max_lateral_accel = acceleration_m_s2{0.f};
	SteerState state{};

	SteerResult r = Steer(env, state, TurnCommand(1.0f, 0.f));
	EXPECT_GT(r.turn_rate_rad_s, 0.f) << "a standing animal must still pivot";
	EXPECT_NEAR(r.turn_headroom, 1.f, 1e-6f);
}


// The lateral-force cross-check on yaw (G3) is derived from the load factor,
// a_lat = g*sqrt(n^2 - 1), which is identically 0 at zero gravity for EVERY n.
// Applied naively that says a zero-g flyer cannot turn at all -- but that is the
// formula being undefined without a weight vector, not a physical bound: a wing
// or tail makes side force whether or not anything is falling. TonTon explicitly
// supports low- and zero-gravity worlds, so the budget must read as unknown
// there, leaving max_yaw_rate to stand alone.
TEST(MyopicBank, ZeroGravityDoesNotAbolishYawAuthority)
{
	Envelope env = DragonflyLikeEnvelope();
	SteerState state{};

	SteerCommand cmd = TurnCommand(1.57f, 4.f);
	cmd.gravity_m_s2 = 0.f;

	SteerResult r{};
	for (int i = 0; i < 300; ++i) r = Steer(env, state, cmd);

	// Banking trades weight for centripetal force, so with no weight there is
	// nothing to trade: the strategy must fall back to yaw, not to paralysis.
	EXPECT_EQ(r.strategy, TurnStrategy::YAW);
	EXPECT_NEAR(r.bank_angle_rad, 0.f, 1e-6f) << "there is no bank without weight";
	EXPECT_NEAR(r.turn_rate_rad_s, float(env.aerial->max_yaw_rate), 1e-3f)
		<< "yaw authority is the only bound that survives zero gravity";
	EXPECT_TRUE(std::isfinite(r.turn_headroom));
}

// B1: the companion case the test above cannot reach. `ZeroGravityDoesNot-
// AbolishYawAuthority` starts wings level, so it never enters the
// `turn_follows_bank` path at all. Start from a SETTLED bank instead and then
// remove gravity.
//
// `turn_follows_bank` hands the delivered turn rate to the bank whenever a bank
// is being carried. At g = 0 the bank produces g*tan(phi)/v == 0 for every phi,
// so without the `gravity > 0` conjunct the yaw command is discarded in favour
// of a hard zero for the entire roll-out: measured 41 consecutive frames
// (0.68 s) of exactly zero turn rate on a flyer rolled to 1.02 rad with
// 4.0 rad/s of yaw available, and 4.30 rad of heading swept in 2 s against
// 7.31 rad with the bank correctly ignored. The bank owns the turn only where
// the bank can produce one.
TEST(MyopicBank, SettledBankDoesNotVetoYawWhenGravityIsRemoved)
{
	Envelope env = AlbatrossLikeEnvelope();
	SteerState state{};
	const float v  = 15.f;
	const float dt = 1.f / 60.f;

	// Settle into a full banked turn at 1 g.
	SteerResult r{};
	for (int i = 0; i < 600; ++i) r = Steer(env, state, TurnCommand(+1.57f, v));
	ASSERT_EQ(r.strategy, TurnStrategy::BANK);
	ASSERT_NEAR(r.bank_angle_rad, std::acos(0.5f), 1e-3f);

	// Same creature, same carried bank, now in free fall -- and with real yaw
	// authority, which is the whole question.
	env.aerial->max_yaw_rate = omega_rad_s{4.f};

	int   banked_but_straight = 0;
	float swept = 0.f;
	for (int i = 0; i < 120; ++i) {          // 2 s
		SteerCommand cmd = TurnCommand(+1.57f, v);
		cmd.gravity_m_s2 = 0.f;
		r = Steer(env, state, cmd);
		if (std::fabs(r.bank_angle_rad) > 0.05f &&
		    std::fabs(r.turn_rate_rad_s) < 1e-4f) ++banked_but_straight;
		swept += std::fabs(r.turn_rate_rad_s) * dt;
	}

	EXPECT_EQ(banked_but_straight, 0)
		<< "a bank that cannot produce a turn must not veto the yaw that can";
	EXPECT_GT(swept, 6.5f)
		<< "zero-g yaw authority (4 rad/s over 2 s) was thrown away";
	EXPECT_NEAR(r.turn_rate_rad_s, 4.f, 1e-3f);
	EXPECT_NEAR(r.bank_angle_rad, 0.f, 1e-6f) << "the bank still rolls out";
}

// F3: the reviewer measured a ~560x collapse in turn authority across g -> 0 and
// proposed blending or flooring it. The collapse is a FIXTURE artefact: it holds
// stall_speed fixed while sweeping gravity. In the real pipeline stall_speed
// comes from the analysis run at that world's gravity, and level flight (L = W)
// gives v_s ~ sqrt(g). Scale it as the pipeline would and the invariant appears:
//
//     a_lift = g * n(v) = g * (v/v_s)^2 = rho*S*CL_max*v^2/(2m)   -- exact, g-free
//     a_lat  = g * sqrt(n^2 - 1) = sqrt(a_lift^2 - g^2)
//
// So the quantity that is gravity-invariant is the TOTAL LIFT ACCELERATION, and
// this test pins that to 1e-3 relative. The lateral budget is what is left after
// holding the creature up, so it RISES as g falls -- monotonically, by a factor
// of 1.3 over a 6x gravity change, not 560x, and toward a_lift as its ceiling.
// That is the physically correct behaviour and is asserted too, deliberately
// rather than being papered over with a blend.
TEST(MyopicBank, LiftAccelIsGravityInvariantWhenStallSpeedScales)
{
	const float v  = 10.f;
	const float vs_1g = 8.f;     // stall speed measured at Earth gravity

	// Read the lateral budget off the delivered rate: put the flyer deep in the
	// v^2-limited band (n_max far away) with nominal yaw authority far above the
	// budget, so the force budget is the only thing that can bind.
	auto measure = [&](float g) {
		Envelope env = DragonflyLikeEnvelope();
		env.max_speed = velocity_m_s{20.f};
		env.aerial->n_max        = 100.f;
		env.aerial->max_yaw_rate = omega_rad_s{100.f};
		// The pipeline's own scaling: v_s^2 = 2mg/(rho S CL_max).
		env.aerial->stall_speed  = velocity_m_s{vs_1g * std::sqrt(g / 9.81f)};

		SteerState state{};
		SteerResult r{};
		for (int i = 0; i < 600; ++i) {
			SteerCommand cmd = TurnCommand(1.57f, v);
			cmd.gravity_m_s2 = g;
			r = Steer(env, state, cmd);
		}
		EXPECT_EQ(r.strategy, TurnStrategy::YAW) << "at g=" << g;
		const float a_lat = r.turn_rate_rad_s * v;
		return std::pair<float, float>{a_lat, std::sqrt(a_lat * a_lat + g * g)};
	};

	auto [lat_earth, lift_earth] = measure(9.81f);
	auto [lat_mars,  lift_mars ] = measure(3.71f);
	auto [lat_luna,  lift_luna ] = measure(1.62f);

	// The invariant. rho*S*CL_max*v^2/(2m) = 9.81 * (10/8)^2 = 15.328 m/s^2.
	const float expected_lift = 9.81f * (v / vs_1g) * (v / vs_1g);
	for (auto [name, lift] : {std::pair<const char*, float>{"earth", lift_earth},
	                          {"mars", lift_mars}, {"luna", lift_luna}}) {
		EXPECT_NEAR(lift, expected_lift, 1e-3f * expected_lift)
			<< "lift acceleration is not gravity-invariant on " << name;
	}

	// ...and the residue, in the physically correct direction: less weight to
	// hold up leaves more lift for turning. No cliff.
	EXPECT_GT(lat_mars, lat_earth);
	EXPECT_GT(lat_luna, lat_mars);
	EXPECT_LT(lat_luna, expected_lift);            // a_lift is the ceiling
	EXPECT_LT(lat_luna, lat_earth * 1.5f)
		<< "spread across a 6x gravity change must stay small, not 560x";
}


// ===========================================================================
// Task 6: launch planning and the ComputeMyopicControl entry point.
// ===========================================================================

namespace {

// Facing +Z with +Y up. A target at world bearing `b` (measured the same way,
// so bearing 0 is dead ahead) sits far enough away that a few frames of travel
// do not appreciably move it.
glm::vec3 TargetAtBearing(float b, float distance_m = 1000.f)
{
	return distance_m * glm::vec3(std::sin(b), 0.f, std::cos(b));
}

MyopicInput GroundChase(float bearing_rad, float speed_m_s, float dt_s)
{
	MyopicInput in;
	in.mode            = LocomotionMode::TERRESTRIAL;
	in.target_mode     = LocomotionMode::TERRESTRIAL;
	in.substrate       = Substrate::GROUND;
	in.orientation     = glm::quat(1.f, 0.f, 0.f, 0.f); // forward = +Z
	in.position        = glm::vec3(0.f);
	in.target_position = TargetAtBearing(bearing_rad);
	in.velocity_m_s    = glm::vec3(0.f, 0.f, speed_m_s);
	in.desired_speed_m_s = speed_m_s;
	in.dt_s            = dt_s;
	return in;
}

// Integrate heading only (position held at the origin, target 1 km out) and
// return the signed heading error remaining after `t_end_s`. Position is held
// fixed on purpose: this measures the CONTROL LAW, not the pursuit geometry.
float SimulateEntryPointHeading(const Output& analysis, float dt, float t_end_s,
                                float initial_bearing_rad, float speed_m_s = 3.f)
{
	MyopicState st{};
	float yaw = 0.f; // creature heading, about +Y; 0 = facing +Z
	const int steps = int(std::lround(t_end_s / dt));
	for (int i = 0; i < steps; ++i) {
		MyopicInput in = GroundChase(initial_bearing_rad, speed_m_s, dt);
		in.orientation  = glm::angleAxis(yaw, glm::vec3(0.f, 1.f, 0.f));
		in.velocity_m_s = (in.orientation * glm::vec3(0.f, 0.f, 1.f)) * speed_m_s;
		MyopicOutput o = ComputeMyopicControl(analysis, in, st);
		yaw += o.angular_velocity_rad_s.y * dt;
	}
	return initial_bearing_rad - yaw;
}

} // namespace

// --- Launch planning -------------------------------------------------------

// The plan asserted "a hovering insect launches vertically" and expected
// readiness == 1 from a standstill. The analysis layer disagrees: dragonfly.glb
// classifies as RUNNING_TAKEOFF, with a takeoff run of 0.020 m against a
// minimum flight speed of 2.631 m/s. Two centimetres of runway IS morally a
// vertical launch, so the numbers are not absurd -- but the MODE is what
// PlanLaunch dispatches on, and reporting a launch the analysis did not
// classify would be inventing biology at the control layer. Pinned so the
// disagreement stays visible instead of being smoothed over.
TEST(MyopicLaunch, DragonflyIsClassifiedRunningTakeoffNotVerticalLaunch)
{
	const Output* out = Analyze("dragonfly.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());
	ASSERT_EQ(out->aerial->takeoff.mode,
	          Analysis_TakeoffAnalysis::TakeoffMode::RUNNING_TAKEOFF)
		<< "if this ever becomes VERTICAL_LAUNCH the expectations below are wrong";
	EXPECT_LT(float(out->aerial->takeoff.takeoff_run_distance_m), 0.05f)
		<< "the 'runway' the analysis asks for is 2 cm";

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;
	in.substrate   = Substrate::GROUND;

	const float v_min = float(out->aerial->min_flight_speed_m_s);

	LaunchPlan cold = PlanLaunch(*out, in, 0.f);
	EXPECT_TRUE(cold.feasible) << "flat ground is a runway";
	EXPECT_TRUE(cold.accelerate_along_heading);
	EXPECT_NEAR(cold.readiness, 0.f, 1e-6f) << "no airspeed, no lift";
	EXPECT_EQ(cold.blocking_reason, BlockingReason::NEEDS_RUNWAY_SPEED);
	EXPECT_NEAR(cold.required_airspeed_m_s, v_min, 1e-4f);

	LaunchPlan half = PlanLaunch(*out, in, 0.5f * v_min);
	EXPECT_NEAR(half.readiness, 0.5f, 1e-4f);

	LaunchPlan hot = PlanLaunch(*out, in, v_min);
	EXPECT_NEAR(hot.readiness, 1.f, 1e-4f);
	EXPECT_EQ(hot.blocking_reason, BlockingReason::NONE);
}

TEST(MyopicLaunch, NonFlyerReportsNoAerialAnalysis)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_FALSE(out->aerial.has_value());

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;

	LaunchPlan p = PlanLaunch(*out, in, 0.f);
	EXPECT_FALSE(p.feasible);
	EXPECT_EQ(p.blocking_reason, BlockingReason::NO_AERIAL_ANALYSIS);
	EXPECT_EQ(p.readiness, 0.f);
}

// batto.glb IS RUNNING_TAKEOFF (verified by the ASSERT below), which is the
// only takeoff mode whose readiness depends on airspeed at all -- every other
// arm returns a constant and would make a monotonicity sweep vacuous. So the
// sweep is kept AND given teeth: readiness must actually rise, and it must
// reach saturation, not merely fail to fall.
TEST(MyopicLaunch, ReadinessIsMonotoneAndActuallyRisesWithAirspeed)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());
	ASSERT_EQ(out->aerial->takeoff.mode,
	          Analysis_TakeoffAnalysis::TakeoffMode::RUNNING_TAKEOFF)
		<< "a constant-readiness mode would make this sweep prove nothing";

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;
	in.substrate   = Substrate::GROUND;

	float prev = -1.f;
	int   rises = 0;
	float max_readiness = 0.f;
	for (float v = 0.f; v <= 30.f; v += 2.f) {
		LaunchPlan p = PlanLaunch(*out, in, v);
		EXPECT_GE(p.readiness, prev) << "readiness dropped at airspeed " << v;
		EXPECT_LE(p.readiness, 1.f);
		EXPECT_GE(p.readiness, 0.f);
		if (p.readiness > prev + 1e-6f) ++rises;
		max_readiness = std::max(max_readiness, p.readiness);
		prev = p.readiness;
	}
	EXPECT_GE(rises, 3) << "a constant readiness is trivially monotone";
	EXPECT_NEAR(max_readiness, 1.f, 1e-4f)
		<< "30 m/s must be enough runway speed for this creature";
	EXPECT_NEAR(PlanLaunch(*out, in, 0.f).readiness, 0.f, 1e-6f);
}

TEST(MyopicLaunch, RunningTakeoffNeedsARunwaySubstrate)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_FALSE(out->aerial->takeoff.can_use_water_taxi);

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;

	for (Substrate s : {Substrate::WATER, Substrate::PERCH, Substrate::CLIFF_EDGE}) {
		in.substrate = s;
		LaunchPlan p = PlanLaunch(*out, in, 100.f); // plenty of airspeed
		EXPECT_FALSE(p.feasible)                  << "substrate " << int(s);
		EXPECT_FALSE(p.accelerate_along_heading)  << "substrate " << int(s);
		EXPECT_EQ(p.readiness, 0.f)               << "substrate " << int(s);
	}

	in.substrate = Substrate::GROUND;
	EXPECT_TRUE(PlanLaunch(*out, in, 100.f).feasible);
}

// penguin.glb in air is IMPOSSIBLE with wing_loading_ok == false, which is the
// first constraint FirstFailedConstraint tests -- so this pins the IMPOSSIBLE
// arm and the constraint ordering together.
TEST(MyopicLaunch, ImpossibleTakeoffReportsItsFirstFailedConstraint)
{
	const Output* out = Analyze("penguin.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());
	ASSERT_EQ(out->aerial->takeoff.mode,
	          Analysis_TakeoffAnalysis::TakeoffMode::IMPOSSIBLE);
	ASSERT_FALSE(out->aerial->takeoff.constraints.wing_loading_ok);

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;
	in.substrate   = Substrate::GROUND;

	LaunchPlan p = PlanLaunch(*out, in, 1000.f); // no amount of speed helps
	EXPECT_FALSE(p.feasible);
	EXPECT_EQ(p.readiness, 0.f);
	EXPECT_EQ(p.blocking_reason, BlockingReason::WING_LOADING);
}

// --- Airspeed discipline ---------------------------------------------------

// The airspeed-versus-ground-speed inversion is the likeliest bug in the
// module, so it gets its own test.
TEST(MyopicAirspeed, HeadwindMakesLaunchEasierThanTailwind)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	MyopicState st_head{}, st_tail{};

	MyopicInput base;
	base.mode         = LocomotionMode::TERRESTRIAL;
	base.target_mode  = LocomotionMode::AERIAL;
	base.substrate    = Substrate::GROUND;
	base.velocity_m_s = glm::vec3(5.f, 0.f, 0.f); // same ground speed both times
	base.dt_s         = 1.f / 60.f;

	MyopicInput headwind = base;
	headwind.medium_velocity_m_s = glm::vec3(-5.f, 0.f, 0.f); // blowing against us

	MyopicInput tailwind = base;
	tailwind.medium_velocity_m_s = glm::vec3(5.f, 0.f, 0.f);  // blowing with us

	MyopicOutput h = ComputeMyopicControl(*out, headwind, st_head);
	MyopicOutput t = ComputeMyopicControl(*out, tailwind, st_tail);

	EXPECT_GT(h.transition_readiness, t.transition_readiness)
		<< "a headwind must make takeoff easier at the same ground speed";

	// And pin the arithmetic, not just the ordering: airspeed is 10 and 0 m/s
	// respectively against a 7.867 m/s requirement.
	const float v_min = float(out->aerial->min_flight_speed_m_s);
	EXPECT_NEAR(h.transition_readiness, std::min(1.f, 10.f / v_min), 1e-4f);
	EXPECT_NEAR(t.transition_readiness, 0.f, 1e-6f);
}

TEST(MyopicAirspeed, StillAirReadinessSitsBetweenHeadAndTailwind)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	const float v_min = float(out->aerial->min_flight_speed_m_s);

	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;
	in.substrate   = Substrate::GROUND;
	in.velocity_m_s = glm::vec3(5.f, 0.f, 0.f);

	MyopicState st{};
	MyopicOutput o = ComputeMyopicControl(*out, in, st);
	EXPECT_NEAR(o.transition_readiness, 5.f / v_min, 1e-4f);
}

// --- The entry point -------------------------------------------------------

TEST(MyopicEntryPoint, ProducesFiniteOutputForEveryModel)
{
	struct Case { const char* file; Env env; };
	const Case cases[] = {
		{"cat.glb", Env::Air}, {"dragonfly.glb", Env::Air},
		{"batto.glb", Env::Air}, {"penguin.glb", Env::Ocean},
	};

	for (auto const& c : cases) {
		const Output* out = Analyze(c.file, c.env);
		ASSERT_NE(out, nullptr) << c.file;

		MyopicState state{};
		MyopicInput in;
		in.mode            = LocomotionMode::TERRESTRIAL;
		in.target_mode     = LocomotionMode::TERRESTRIAL;
		in.target_position = glm::vec3(10.f, 0.f, 3.f);
		in.dt_s            = 1.f / 60.f;

		MyopicOutput o = ComputeMyopicControl(*out, in, state);

		// NOTE, because it is easy to misread this fixture: velocity_m_s is
		// left at zero, so every one of these four is STANDING. A standing
		// animal pivots in place (no moving-frame constraint applies), so all
		// three demand/capacity ratios read 0 and stability is exactly 1 for
		// every model. This case therefore says nothing about turn saturation;
		// the moving sweep below is where the interesting numbers are.
		// (The heading error is also not the ~18 degrees it looks like:
		// forward is +Z, so a target at (10, 0, 3) is atan2(10, 3) = 73.3
		// degrees off the nose.)
		EXPECT_EQ(o.stability, 1.f) << c.file << ": a standing pivot is unconstrained";

		EXPECT_TRUE(std::isfinite(o.linear_acceleration_m_s2.x)) << c.file;
		EXPECT_TRUE(std::isfinite(o.linear_acceleration_m_s2.y)) << c.file;
		EXPECT_TRUE(std::isfinite(o.linear_acceleration_m_s2.z)) << c.file;
		EXPECT_TRUE(std::isfinite(o.angular_velocity_rad_s.y)) << c.file;
		EXPECT_TRUE(std::isfinite(o.stability)) << c.file;
		EXPECT_TRUE(std::isfinite(o.bank_angle_rad)) << c.file;
		// Every one of these four has a terrestrial section, so a control
		// solution must actually have been computed.
		EXPECT_EQ(o.blocking_reason, BlockingReason::NONE) << c.file;

		std::cerr << "[standing] " << c.file << " stability=" << o.stability
		          << " speed_headroom=" << o.speed_headroom
		          << " turn_headroom=" << o.turn_headroom << "\n";

		// The same maneuver actually MOVING, at half the gait's top speed.
		// This is the case Task 5 flagged: u_turn's numerator is the unclamped
		// desired rate error/max(tau, dt), so a 73-degree heading change that
		// the creature will nevertheless execute perfectly well reads deeply
		// negative. Measured, reported, deliberately NOT clamped -- the
		// numerator is what makes the turn channel capable of exceeding 1 at
		// all. Only finiteness is asserted; the magnitudes are diagnostics.
		auto envelope = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 0, 9.81f);
		ASSERT_TRUE(envelope.has_value()) << c.file;
		const float v = 0.5f * float(envelope->max_speed);

		MyopicState moving_state{};
		MyopicInput mv = in;
		mv.velocity_m_s      = glm::vec3(0.f, 0.f, v);
		mv.desired_speed_m_s = v;
		MyopicOutput m = ComputeMyopicControl(*out, mv, moving_state);

		EXPECT_TRUE(std::isfinite(m.stability)) << c.file;
		EXPECT_TRUE(std::isfinite(m.angular_velocity_rad_s.y)) << c.file;
		EXPECT_TRUE(std::isfinite(m.linear_acceleration_m_s2.z)) << c.file;
		std::cerr << "[moving @" << v << " m/s] " << c.file
		          << " stability=" << m.stability
		          << " speed_headroom=" << m.speed_headroom
		          << " turn_headroom=" << m.turn_headroom
		          << " turn_rate=" << m.angular_velocity_rad_s.y << "\n";
	}
}

// C3: "no envelope for this mode" is a CALLER error and must not be encoded in
// a physical output channel.
TEST(MyopicEntryPoint, MissingModeIsReportedByBlockingReasonNotStability)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_FALSE(out->aerial.has_value());

	MyopicState st{};
	MyopicInput in;
	in.mode        = LocomotionMode::AERIAL;   // the cat cannot fly
	in.target_mode = LocomotionMode::AERIAL;
	in.dt_s        = 1.f / 60.f;

	MyopicOutput o = ComputeMyopicControl(*out, in, st);
	EXPECT_EQ(o.blocking_reason, BlockingReason::MODE_UNAVAILABLE);
	EXPECT_EQ(o.angular_velocity_rad_s.y, 0.f);
	EXPECT_EQ(o.linear_acceleration_m_s2, glm::vec3(0.f));
	// The state must not have been advanced by a frame that computed nothing.
	EXPECT_EQ(st.prev_turn_rate_rad_s, 0.f);
	EXPECT_EQ(st.prev_accel_m_s2, 0.f);
}

// ...and the other half of C3: a genuine stability of -1 is an ordinary
// reading (a creature demanding twice the turn authority it owns) and must be
// reported with blocking_reason NONE. Bisected rather than hardcoded so the
// test does not bake in cat.glb's exact lateral budget.
TEST(MyopicEntryPoint, ARealNegativeOneStabilityIsNotAnErrorSentinel)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	auto stability_at = [&](float bearing) {
		MyopicState st{};
		MyopicInput in = GroundChase(bearing, 3.f, 1.f / 60.f);
		return ComputeMyopicControl(*out, in, st).stability;
	};

	// Monotone decreasing in |bearing| through the turn-saturated band.
	ASSERT_GT(stability_at(0.f), -1.f);
	ASSERT_LT(stability_at(1.f), -1.f);

	float lo = 0.f, hi = 1.f;
	for (int i = 0; i < 60; ++i) {
		const float mid = 0.5f * (lo + hi);
		(stability_at(mid) > -1.f ? lo : hi) = mid;
	}

	MyopicState st{};
	MyopicInput in = GroundChase(0.5f * (lo + hi), 3.f, 1.f / 60.f);
	MyopicOutput o = ComputeMyopicControl(*out, in, st);

	EXPECT_NEAR(o.stability, -1.f, 1e-3f);
	EXPECT_EQ(o.blocking_reason, BlockingReason::NONE)
		<< "stability == -1 is a turn the creature cannot hold, not an error";
}

// C2: the state round-trip must preserve the SIGN of the acceleration. A
// magnitude round-trip feeds a braking creature back into the slew as though it
// had been accelerating forward just as hard, which both weakens and
// non-monotonically perturbs the deceleration ramp.
TEST(MyopicEntryPoint, BrakingAccelerationKeepsItsSignAcrossFrames)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	// The world is held fixed on purpose: this isolates the state round-trip
	// from the pursuit dynamics.
	MyopicInput in = GroundChase(0.f, 5.f, 1.f / 60.f);
	in.desired_speed_m_s = 0.5f;

	MyopicState st{};
	const glm::vec3 forward(0.f, 0.f, 1.f);

	float prev_mag = 0.f;
	for (int i = 0; i < 5; ++i) {
		MyopicOutput o = ComputeMyopicControl(*out, in, st);
		const float along = glm::dot(o.linear_acceleration_m_s2, forward);

		EXPECT_LT(along, 0.f) << "frame " << i << ": braking must decelerate";
		EXPECT_LT(st.prev_accel_m_s2, 0.f)
			<< "frame " << i << ": the stored history must stay signed";
		// The ramp must never stutter, and must actually be building while it
		// still has headroom -- it pins at the stopping-distance bound
		// (speed_error / max(tau, dt)) once it reaches it, which for the cat
		// is 135.34 m/s^2 by frame 4.
		EXPECT_GE(std::fabs(along), prev_mag)
			<< "frame " << i << ": the braking ramp must not stutter";
		if (i < 3) {
			EXPECT_GT(std::fabs(along), prev_mag)
				<< "frame " << i << ": the braking ramp must build";
		}
		prev_mag = std::fabs(along);
	}
}

// C4 + the heading-error sign. The controller must close the heading error,
// not open it: the plan's atan2 argument order gave the angle from the target
// back to the heading, which steers away at exactly the commanded rate.
TEST(MyopicEntryPoint, TurnsTowardTheTargetAndClosesTheError)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	for (float bearing : {0.6f, -0.6f}) {
		MyopicState st{};
		MyopicInput in = GroundChase(bearing, 3.f, 1.f / 60.f);
		MyopicOutput o = ComputeMyopicControl(*out, in, st);
		EXPECT_GT(o.angular_velocity_rad_s.y * bearing, 0.f)
			<< "turn rate must share the sign of the heading error (bearing "
			<< bearing << ")";

		const float remaining = SimulateEntryPointHeading(*out, 1.f / 60.f, 3.f, bearing);
		EXPECT_LT(std::fabs(remaining), 0.1f * std::fabs(bearing))
			<< "heading error must close, not diverge (bearing " << bearing << ")";
	}
}

// C4: a creature pointing straight up has no heading at all. glm::normalize of
// a zero-length flattened forward is NaN, and NaN in `forward` poisons every
// output field the caller uses to decide whether to ragdoll.
TEST(MyopicEntryPoint, VerticalOrientationDoesNotProduceNaN)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	MyopicState st{};
	MyopicInput in = GroundChase(0.7f, 3.f, 1.f / 60.f);
	// Nose straight up: forward becomes (0, 1, 0), whose horizontal part is 0.
	in.orientation  = glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.f, 0.f, 0.f));
	in.velocity_m_s = glm::vec3(0.f, 3.f, 0.f);   // and climbing, so no fallback either

	MyopicOutput o = ComputeMyopicControl(*out, in, st);

	EXPECT_TRUE(std::isfinite(o.linear_acceleration_m_s2.x));
	EXPECT_TRUE(std::isfinite(o.linear_acceleration_m_s2.y));
	EXPECT_TRUE(std::isfinite(o.linear_acceleration_m_s2.z));
	EXPECT_TRUE(std::isfinite(o.angular_velocity_rad_s.y));
	EXPECT_TRUE(std::isfinite(o.stability));
	EXPECT_EQ(o.angular_velocity_rad_s.y, 0.f) << "no heading, nothing to steer toward";
	EXPECT_EQ(o.linear_acceleration_m_s2, glm::vec3(0.f))
		<< "no heading, no direction to push in";

	// Target directly overhead is the same degeneracy on the other side.
	MyopicState st2{};
	MyopicInput up = GroundChase(0.f, 3.f, 1.f / 60.f);
	up.target_position = glm::vec3(0.f, 100.f, 0.f);
	MyopicOutput o2 = ComputeMyopicControl(*out, up, st2);
	EXPECT_TRUE(std::isfinite(o2.stability));
	EXPECT_EQ(o2.angular_velocity_rad_s.y, 0.f);
}

// C4, the other half: the acceleration is emitted in the HEADING plane, the
// same plane the turn rate lives in. A pitched-up creature does not get a free
// climb out of a pure speed command.
TEST(MyopicEntryPoint, AccelerationStaysInTheHeadingPlane)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	MyopicState st{};
	MyopicInput in = GroundChase(0.f, 1.f, 1.f / 60.f);
	in.desired_speed_m_s = 5.f;
	// Pitched 40 degrees nose-up.
	in.orientation = glm::angleAxis(-0.7f, glm::vec3(1.f, 0.f, 0.f));

	MyopicOutput o = ComputeMyopicControl(*out, in, st);
	ASSERT_GT(glm::length(o.linear_acceleration_m_s2), 0.f);
	EXPECT_EQ(o.linear_acceleration_m_s2.y, 0.f);
	EXPECT_GT(o.linear_acceleration_m_s2.z, 0.f) << "still pushing along the heading";
}

// C1: Task 5's bank model is invisible unless the entry point reports it.
TEST(MyopicEntryPoint, ReportsStrategyAndBankAngle)
{
	const Output* ground = Analyze("cat.glb", Env::Air);
	ASSERT_NE(ground, nullptr);
	{
		MyopicState st{};
		MyopicInput in = GroundChase(0.6f, 3.f, 1.f / 60.f);
		MyopicOutput o = ComputeMyopicControl(*ground, in, st);
		EXPECT_EQ(o.strategy, TurnStrategy::LATERAL);
		EXPECT_EQ(o.bank_angle_rad, 0.f);
	}

	const Output* flyer = Analyze("dragonfly.glb", Env::Air);
	ASSERT_NE(flyer, nullptr);
	auto air = ExtractEnvelope(*flyer, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(air.has_value()) << "dragonfly must have an aerial envelope";

	MyopicState st{};
	// Seeded mid-roll: a legitimate cold start for a caller resuming a banked
	// flyer. The dragonfly yaws rather than banks, so the target bank is 0 and
	// the reported angle must be the PARTIALLY rolled-out value -- neither the
	// seed (no roll-out happened) nor zero (roll-out is not instant).
	const float seed = 0.5f;
	st.bank_angle_rad = seed;

	MyopicInput in = GroundChase(0.6f, 8.f, 1.f / 60.f);
	in.mode        = LocomotionMode::AERIAL;
	in.target_mode = LocomotionMode::AERIAL;

	MyopicOutput o = ComputeMyopicControl(*flyer, in, st);
	EXPECT_NE(o.strategy, TurnStrategy::LATERAL) << "an aerial envelope yaws or banks";
	EXPECT_EQ(o.bank_angle_rad, st.bank_angle_rad) << "output must mirror the state";
	EXPECT_GT(o.bank_angle_rad, 0.f)   << "roll-out is a process, not an instant";
	EXPECT_LT(o.bank_angle_rad, seed)  << "and it must actually make progress";

	const float roll_step = float(air->aerial->max_roll_rate) / 60.f;
	EXPECT_NEAR(o.bank_angle_rad, seed - roll_step, 1e-5f);

	// Landing readout is populated in aerial mode and only there.
	EXPECT_NEAR(o.touchdown_speed_m_s, float(air->min_speed), 1e-5f);
}

// The entry point now owns the state round-trip, so dt-independence has to be
// re-verified THROUGH it and not only through Steer.
TEST(MyopicFramerate, EntryPointTrajectoryConvergesAcrossTimesteps)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	const float bearing = 0.8f;
	const float t_mid   = 0.25f; // mid-slew: the laws are still separating here

	const float e30  = SimulateEntryPointHeading(*out, 1.f / 30.f,  t_mid, bearing);
	const float e60  = SimulateEntryPointHeading(*out, 1.f / 60.f,  t_mid, bearing);
	const float e120 = SimulateEntryPointHeading(*out, 1.f / 120.f, t_mid, bearing);

	EXPECT_NEAR(e30,  e60,  0.05f * bearing);
	EXPECT_NEAR(e60,  e120, 0.05f * bearing);

	// And the endpoint agrees too.
	const float f30  = SimulateEntryPointHeading(*out, 1.f / 30.f,  3.f, bearing);
	const float f120 = SimulateEntryPointHeading(*out, 1.f / 120.f, 3.f, bearing);
	EXPECT_NEAR(f30, f120, 0.02f * bearing);
}

// ===========================================================================
// Task 6 review round: the launch run, the frame rule, the dispatch seam.
// ===========================================================================

// --- G3: the launch dispatch seam ------------------------------------------
//
// `Output`'s constructor is private, so four of the six PlanLaunch arms had no
// reachable fixture at all and could be gutted to `feasible = false,
// readiness = 0` with the whole suite still green. PlanLaunch(LaunchFacts) is
// the seam: the same dispatch, over exactly the fields it reads.
namespace {

using TakeoffMode = Analysis_TakeoffAnalysis::TakeoffMode;

LaunchFacts BaseFacts(TakeoffMode m)
{
	LaunchFacts f;
	f.mode                   = m;
	f.required_airspeed_m_s  = 8.f;
	f.airspeed_m_s           = 0.f;
	f.gravity_m_s2           = 9.81f;
	f.substrate              = Substrate::GROUND;
	f.wing_loading_ok  = true;
	f.power_loading_ok = true;
	f.aspect_ratio_ok  = true;
	f.leg_strength_ok  = true;
	return f;
}

} // namespace

TEST(MyopicLaunchDispatch, VerticalLaunchNeedsSomethingToStandOn)
{
	LaunchFacts f = BaseFacts(TakeoffMode::VERTICAL_LAUNCH);

	LaunchPlan ground = PlanLaunch(f);
	EXPECT_TRUE(ground.feasible);
	EXPECT_NEAR(ground.readiness, 1.f, 1e-6f);
	EXPECT_EQ(ground.blocking_reason, BlockingReason::NONE);
	EXPECT_FALSE(ground.accelerate_along_heading) << "no runway is involved";

	f.substrate = Substrate::WATER;
	LaunchPlan afloat = PlanLaunch(f);
	EXPECT_FALSE(afloat.feasible) << "open water bears no standing wingbeat";
	EXPECT_EQ(afloat.readiness, 0.f);
	EXPECT_EQ(afloat.blocking_reason, BlockingReason::NEEDS_SOLID_SUBSTRATE);

	f.can_use_water_taxi = true;
	EXPECT_TRUE(PlanLaunch(f).feasible) << "a water-taxiing flyer is the exception";
}

TEST(MyopicLaunchDispatch, JumpLaunchRefusesAnUncomputedRequirement)
{
	// The measured case: dragonfly.glb reports required_jump_velocity_m_s == 0
	// against 3.896 m/s available, and `required <= available` cleared it for a
	// jump launch it was never sized for. Zero is "not computed", not "free".
	LaunchFacts f = BaseFacts(TakeoffMode::JUMP_LAUNCH);
	f.has_jump_analysis           = true;
	f.available_jump_velocity_m_s = 3.896f;
	f.required_jump_velocity_m_s  = 0.f;

	LaunchPlan p = PlanLaunch(f);
	EXPECT_FALSE(p.jump_feasible);
	EXPECT_FALSE(p.feasible);
	EXPECT_EQ(p.readiness, 0.f);
	EXPECT_EQ(p.blocking_reason, BlockingReason::JUMP_REQUIREMENT_UNKNOWN);
}

TEST(MyopicLaunchDispatch, JumpLaunchComparesTheLegsAgainstTheRequirement)
{
	LaunchFacts f = BaseFacts(TakeoffMode::JUMP_LAUNCH);
	f.has_jump_analysis           = true;
	f.required_jump_velocity_m_s  = 3.f;

	f.available_jump_velocity_m_s = 4.f;
	LaunchPlan strong = PlanLaunch(f);
	EXPECT_TRUE(strong.jump_feasible);
	EXPECT_TRUE(strong.feasible);
	EXPECT_NEAR(strong.readiness, 1.f, 1e-6f);
	EXPECT_EQ(strong.blocking_reason, BlockingReason::NONE);
	EXPECT_EQ(strong.jump_direction, glm::vec3(0.f, 1.f, 0.f));
	EXPECT_NEAR(strong.required_jump_velocity_m_s, 3.f, 1e-6f);

	f.available_jump_velocity_m_s = 2.f;
	LaunchPlan weak = PlanLaunch(f);
	EXPECT_FALSE(weak.jump_feasible);
	EXPECT_EQ(weak.readiness, 0.f);
	EXPECT_EQ(weak.blocking_reason, BlockingReason::LEG_STRENGTH);

	// No jumping section at all: the legs are unquantified, so nothing clears.
	f.available_jump_velocity_m_s = 4.f;
	f.has_jump_analysis = false;
	EXPECT_FALSE(PlanLaunch(f).jump_feasible);
}

TEST(MyopicLaunchDispatch, JumpLaunchNeedsSomethingToPushOff)
{
	LaunchFacts f = BaseFacts(TakeoffMode::JUMP_LAUNCH);
	f.has_jump_analysis           = true;
	f.required_jump_velocity_m_s  = 3.f;
	f.available_jump_velocity_m_s = 4.f;
	f.substrate                   = Substrate::WATER;

	LaunchPlan p = PlanLaunch(f);
	EXPECT_FALSE(p.feasible) << "legs cannot push off open water";
	EXPECT_EQ(p.readiness, 0.f);
	EXPECT_EQ(p.blocking_reason, BlockingReason::NEEDS_SOLID_SUBSTRATE);
}

TEST(MyopicLaunchDispatch, RunningTakeoffScalesWithAirspeedAndNeedsARunway)
{
	LaunchFacts f = BaseFacts(TakeoffMode::RUNNING_TAKEOFF);

	f.airspeed_m_s = 0.f;
	EXPECT_NEAR(PlanLaunch(f).readiness, 0.f, 1e-6f);
	f.airspeed_m_s = 4.f;
	EXPECT_NEAR(PlanLaunch(f).readiness, 0.5f, 1e-6f);
	f.airspeed_m_s = 20.f;
	LaunchPlan hot = PlanLaunch(f);
	EXPECT_NEAR(hot.readiness, 1.f, 1e-6f) << "clamped, never above 1";
	EXPECT_TRUE(hot.accelerate_along_heading);
	EXPECT_EQ(hot.blocking_reason, BlockingReason::NONE);

	for (Substrate s : {Substrate::WATER, Substrate::PERCH, Substrate::CLIFF_EDGE}) {
		f.substrate = s;
		LaunchPlan p = PlanLaunch(f);
		EXPECT_FALSE(p.feasible)                 << "substrate " << int(s);
		EXPECT_FALSE(p.accelerate_along_heading) << "substrate " << int(s);
		EXPECT_EQ(p.readiness, 0.f)              << "substrate " << int(s);
	}
	f.substrate = Substrate::WATER;
	f.can_use_water_taxi = true;
	EXPECT_TRUE(PlanLaunch(f).feasible) << "a pelican's runway is the lake";
}

// G6: required_drop_m used to be assigned takeoff_run_distance_m, which
// tonton_analysis.h:236 documents as "Required runway length" -- a HORIZONTAL
// distance reported in a field named for a vertical one. A cliff launch trades
// height for airspeed: v = sqrt(2gh), so h = v_stall^2 / (2g).
TEST(MyopicLaunchDispatch, CliffDropIsDerivedFromTheStallSpeedAndGravity)
{
	LaunchFacts f = BaseFacts(TakeoffMode::CLIFF_LAUNCH);
	f.substrate = Substrate::CLIFF_EDGE;

	LaunchPlan p = PlanLaunch(f);
	EXPECT_TRUE(p.feasible);
	EXPECT_NEAR(p.readiness, 1.f, 1e-6f);
	EXPECT_NEAR(p.required_drop_m, (8.f * 8.f) / (2.f * 9.81f), 1e-4f);

	// Halving gravity doubles the drop; a horizontal run distance would not move.
	f.gravity_m_s2 = 9.81f * 0.5f;
	EXPECT_NEAR(PlanLaunch(f).required_drop_m, (8.f * 8.f) / 9.81f, 1e-4f);

	// Free fall is undefined without gravity: report no drop, never divide.
	f.gravity_m_s2 = 0.f;
	LaunchPlan weightless = PlanLaunch(f);
	EXPECT_TRUE(std::isfinite(weightless.required_drop_m));
	EXPECT_EQ(weightless.required_drop_m, 0.f);

	f.gravity_m_s2 = 9.81f;
	f.substrate    = Substrate::GROUND;
	LaunchPlan flat = PlanLaunch(f);
	EXPECT_FALSE(flat.feasible);
	EXPECT_EQ(flat.blocking_reason, BlockingReason::NEEDS_ELEVATION);
}

TEST(MyopicLaunchDispatch, AssistedLaunchNeedsAPerch)
{
	LaunchFacts f = BaseFacts(TakeoffMode::ASSISTED_LAUNCH);

	f.substrate = Substrate::PERCH;
	LaunchPlan perched = PlanLaunch(f);
	EXPECT_TRUE(perched.feasible);
	EXPECT_NEAR(perched.readiness, 1.f, 1e-6f);
	EXPECT_EQ(perched.blocking_reason, BlockingReason::NONE);

	for (Substrate s : {Substrate::GROUND, Substrate::WATER, Substrate::CLIFF_EDGE}) {
		f.substrate = s;
		LaunchPlan p = PlanLaunch(f);
		EXPECT_FALSE(p.feasible)    << "substrate " << int(s);
		EXPECT_EQ(p.readiness, 0.f) << "substrate " << int(s);
		EXPECT_EQ(p.blocking_reason, BlockingReason::NEEDS_PERCH) << "substrate " << int(s);
	}
}

// G5: ClassifyMode (tonton_takeoffanalysis.cpp:324-380) reaches IMPOSSIBLE from
// threshold misses with every constraint flag true. An infeasible plan whose
// blocking_reason is NONE is uninterpretable.
TEST(MyopicLaunchDispatch, ImpossibleAlwaysNamesAReason)
{
	LaunchFacts f = BaseFacts(TakeoffMode::IMPOSSIBLE); // all four flags true
	LaunchPlan p = PlanLaunch(f);
	EXPECT_FALSE(p.feasible);
	EXPECT_EQ(p.readiness, 0.f);
	EXPECT_NE(p.blocking_reason, BlockingReason::NONE)
		<< "an infeasible plan must never report 'nothing is blocking you'";
	EXPECT_EQ(p.blocking_reason, BlockingReason::TAKEOFF_IMPOSSIBLE);

	// ...and a named constraint still wins over the generic reason, in order.
	struct Case { bool LaunchFacts::* flag; BlockingReason reason; };
	const Case cases[] = {
		{&LaunchFacts::wing_loading_ok,  BlockingReason::WING_LOADING},
		{&LaunchFacts::power_loading_ok, BlockingReason::POWER_LOADING},
		{&LaunchFacts::aspect_ratio_ok,  BlockingReason::ASPECT_RATIO},
		{&LaunchFacts::leg_strength_ok,  BlockingReason::LEG_STRENGTH},
	};
	for (auto const& c : cases) {
		LaunchFacts g = BaseFacts(TakeoffMode::IMPOSSIBLE);
		g.*(c.flag) = false;
		EXPECT_EQ(PlanLaunch(g).blocking_reason, c.reason);
	}
}

// G6, reported for the real models: what a cliff launch would actually cost
// them. Neither classifies as CLIFF_LAUNCH, so this drives the seam directly
// with each model's own stall speed rather than inventing one.
TEST(MyopicLaunchDispatch, CliffDropReportedForTheSampleFlyers)
{
	for (const char* file : {"batto.glb", "dragonfly.glb"}) {
		const Output* out = Analyze(file, Env::Air);
		ASSERT_NE(out, nullptr) << file;
		ASSERT_TRUE(out->aerial.has_value()) << file;

		const float v_stall = float(out->aerial->min_flight_speed_m_s);
		LaunchFacts f = BaseFacts(TakeoffMode::CLIFF_LAUNCH);
		f.required_airspeed_m_s = v_stall;
		f.substrate             = Substrate::CLIFF_EDGE;

		LaunchPlan p = PlanLaunch(f);
		EXPECT_NEAR(p.required_drop_m, v_stall * v_stall / (2.f * 9.81f), 1e-4f);
		EXPECT_GT(p.required_drop_m, 0.f);
		std::cerr << "[cliff drop] " << file << " v_stall=" << v_stall
		          << " m/s -> required_drop=" << p.required_drop_m << " m\n";
	}
}

// --- G1: the launch run must be able to finish -----------------------------
namespace {

struct LaunchRunResult {
	float readiness{0.f};
	bool  suggest_gait_change{false};
	float ground_speed{0.f};
	float airspeed{0.f};
	float env_max_speed{0.f};
	float required_airspeed{0.f};
};

// Run the entry point in a closed loop, integrating the acceleration it
// commands. Heading is held: while the launch precondition is unmet the
// controller suppresses steering anyway, so the run is a straight line.
LaunchRunResult SimulateLaunchRun(const Output& o, int gait, int frames = 3000,
                                  glm::vec3 wind = glm::vec3(0.f),
                                  float desired_speed_m_s = -1.f)
{
	const float dt = 1.f / 60.f;
	MyopicState st{};
	MyopicInput in;
	in.mode                = LocomotionMode::TERRESTRIAL;
	in.target_mode         = LocomotionMode::AERIAL;
	in.substrate           = Substrate::GROUND;
	in.current_gait        = gait;
	in.dt_s                = dt;
	in.target_position     = TargetAtBearing(0.6f);
	in.medium_velocity_m_s = wind;
	in.velocity_m_s        = glm::vec3(0.f);
	in.desired_speed_m_s   = desired_speed_m_s;

	MyopicOutput out;
	for (int i = 0; i < frames; ++i) {
		out = ComputeMyopicControl(o, in, st);
		in.velocity_m_s += out.linear_acceleration_m_s2 * dt;
	}

	auto env = ExtractEnvelope(o, LocomotionMode::TERRESTRIAL, gait, 9.81f);
	LaunchRunResult r;
	r.readiness           = out.transition_readiness;
	r.suggest_gait_change = out.suggest_gait_change;
	r.ground_speed        = glm::length(in.velocity_m_s);
	r.airspeed            = glm::length(in.velocity_m_s - wind);
	r.env_max_speed       = env.has_value() ? float(env->max_speed) : 0.f;
	r.required_airspeed   = float(o.aerial->min_flight_speed_m_s);
	return r;
}

} // namespace

// The launch run used to command `env->max_speed` -- the TERRESTRIAL envelope's
// top speed at the current gait -- while readiness was measured against
// min_flight_speed_m_s, an AERIAL figure. The two are unrelated, and for both
// sample flyers the ground number is far smaller, so the run converged at
// 5-27% readiness and stayed there for ever, silently:
//
//   batto     min_flight 7.867  gait0 env_max 0.579 -> readiness 0.074
//                               gait1 0.827 -> 0.105   gait2 2.089 -> 0.266
//   dragonfly min_flight 2.631  gait2 env_max 0.139 -> readiness 0.053
//
// RUNNING_TAKEOFF is the mode both sample flyers classify as, so that was the
// entire launch feature. Command the airspeed the launch actually requires and
// the run completes.
TEST(MyopicLaunch, LaunchRunReachesFullReadiness)
{
	for (const char* file : {"batto.glb", "dragonfly.glb"}) {
		const Output* out = Analyze(file, Env::Air);
		ASSERT_NE(out, nullptr) << file;
		ASSERT_TRUE(out->aerial.has_value()) << file;
		ASSERT_EQ(out->aerial->takeoff.mode, TakeoffMode::RUNNING_TAKEOFF) << file;

		for (int gait = 0; gait <= 2; ++gait) {
			LaunchRunResult r = SimulateLaunchRun(*out, gait);
			std::cerr << "[launch run] " << file << " gait" << gait
			          << " env_max=" << r.env_max_speed
			          << " required=" << r.required_airspeed
			          << " -> airspeed=" << r.airspeed
			          << " readiness=" << r.readiness
			          << " suggest_gait_change=" << r.suggest_gait_change << "\n";

			EXPECT_NEAR(r.readiness, 1.f, 1e-3f)
				<< file << " gait" << gait << ": the takeoff run never completes";
			EXPECT_NEAR(r.airspeed, r.required_airspeed, 1e-2f * r.required_airspeed)
				<< file << " gait" << gait;
		}
	}
}

// ...and when the gait's own envelope cannot cover the flight speed, that must
// be AUDIBLE. It used to be silence: suggest_gait_change was false in every one
// of the six cases above, so a caller had no way to learn the goal was
// unreachable at this gait.
TEST(MyopicLaunch, AGaitThatCannotReachFlightSpeedSaysSo)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	const float required = float(out->aerial->min_flight_speed_m_s);
	for (int gait = 0; gait <= 2; ++gait) {
		auto env = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, gait, 9.81f);
		ASSERT_TRUE(env.has_value());
		ASSERT_LT(float(env->max_speed), required)
			<< "gait " << gait << " unexpectedly covers flight speed; pick another";

		MyopicState st{};
		MyopicInput in;
		in.mode         = LocomotionMode::TERRESTRIAL;
		in.target_mode  = LocomotionMode::AERIAL;
		in.substrate    = Substrate::GROUND;
		in.current_gait = gait;
		in.dt_s         = 1.f / 60.f;

		MyopicOutput o = ComputeMyopicControl(*out, in, st);
		EXPECT_TRUE(o.suggest_gait_change)
			<< "gait " << gait << ": a run that this gait cannot finish must not be silent";
		EXPECT_EQ(o.blocking_reason, BlockingReason::NEEDS_RUNWAY_SPEED) << "gait " << gait;
	}
}

// G8: steering was suppressed for the whole launch run, so a creature already
// at flight speed flew straight past its target.
TEST(MyopicLaunch, SteeringResumesOnceTheLaunchPreconditionIsMet)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	const float v_min = float(out->aerial->min_flight_speed_m_s);

	MyopicInput in;
	in.mode            = LocomotionMode::TERRESTRIAL;
	in.target_mode     = LocomotionMode::AERIAL;
	in.substrate       = Substrate::GROUND;
	in.dt_s            = 1.f / 60.f;
	in.target_position = TargetAtBearing(0.6f);

	{	// Still building airspeed: head down the runway, do not turn.
		MyopicState st{};
		MyopicInput slow = in;
		slow.velocity_m_s = glm::vec3(0.f, 0.f, 0.5f * v_min);
		MyopicOutput o = ComputeMyopicControl(*out, slow, st);
		ASSERT_LT(o.transition_readiness, 1.f);
		EXPECT_EQ(o.angular_velocity_rad_s.y, 0.f)
			<< "the run owns the heading until it has made its airspeed";
	}
	{	// At flight speed: the launch is ready, so steer at the target again.
		MyopicState st{};
		MyopicInput fast = in;
		fast.velocity_m_s = glm::vec3(0.f, 0.f, v_min);
		MyopicOutput o = ComputeMyopicControl(*out, fast, st);
		ASSERT_NEAR(o.transition_readiness, 1.f, 1e-4f);
		EXPECT_GT(o.angular_velocity_rad_s.y, 0.f)
			<< "readiness 1.0 and still flying straight past the target";
	}
}

// --- G2: the frame rule ----------------------------------------------------
//
// A mode's envelope is evaluated against the speed relative to what that mode
// pushes AGAINST. Legs push against the ground; a wind blowing past a standing
// cat is not a speed its grip budget knows about. Measured before the fix:
// a standing cat in a 10 m/s crosswind had its pivot rate collapse from 14.19
// to 0.053 rad/s and reported stability -132.3.
TEST(MyopicFrame, CrosswindDoesNotStopAStandingCreaturePivoting)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	auto settle = [&](glm::vec3 wind) {
		MyopicState st{};
		MyopicOutput o;
		for (int i = 0; i < 600; ++i) {
			MyopicInput in = GroundChase(0.8f, 0.f, 1.f / 60.f);
			in.velocity_m_s        = glm::vec3(0.f);  // standing
			in.desired_speed_m_s   = 0.f;
			in.medium_velocity_m_s = wind;
			o = ComputeMyopicControl(*out, in, st);
		}
		return o;
	};

	MyopicOutput still  = settle(glm::vec3(0.f));
	MyopicOutput breezy = settle(glm::vec3(10.f, 0.f, 0.f));

	std::cerr << "[frame] standing still air turn=" << still.angular_velocity_rad_s.y
	          << " stability=" << still.stability
	          << " | 10 m/s crosswind turn=" << breezy.angular_velocity_rad_s.y
	          << " stability=" << breezy.stability << "\n";

	EXPECT_NEAR(breezy.angular_velocity_rad_s.y, still.angular_velocity_rad_s.y, 1e-5f)
		<< "a breeze must not decide whether a cat can turn on the spot";
	EXPECT_NEAR(breezy.stability, still.stability, 1e-5f);
	EXPECT_EQ(breezy.stability, 1.f) << "a standing pivot is unconstrained";
}

// The other direction: a tailwind that cancels a runner's airspeed must not
// hand it a standing creature's pivot authority. Measured before the fix: a cat
// running 5 m/s in a 5 m/s tailwind pivoted at 14.19 rad/s and was commanded
// 62.7 m/s^2 as though it were stationary.
TEST(MyopicFrame, TailwindDoesNotEraseARunnersGripBudget)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	auto settle = [&](glm::vec3 wind) {
		MyopicState st{};
		MyopicOutput o;
		for (int i = 0; i < 600; ++i) {
			MyopicInput in = GroundChase(0.8f, 5.f, 1.f / 60.f);
			in.medium_velocity_m_s = wind;
			o = ComputeMyopicControl(*out, in, st);
		}
		return o;
	};

	MyopicOutput still = settle(glm::vec3(0.f));
	MyopicOutput tail  = settle(glm::vec3(0.f, 0.f, 5.f)); // airspeed exactly 0

	std::cerr << "[frame] running 5 m/s still air turn=" << still.angular_velocity_rad_s.y
	          << " stability=" << still.stability
	          << " accel=" << still.linear_acceleration_m_s2.z
	          << " | 5 m/s tailwind turn=" << tail.angular_velocity_rad_s.y
	          << " stability=" << tail.stability
	          << " accel=" << tail.linear_acceleration_m_s2.z << "\n";

	EXPECT_NEAR(tail.angular_velocity_rad_s.y, still.angular_velocity_rad_s.y, 1e-5f)
		<< "a tailwind must not buy a sprinting cat a standing pivot";
	EXPECT_NEAR(tail.stability, still.stability, 1e-5f);
	EXPECT_NEAR(tail.linear_acceleration_m_s2.z, still.linear_acceleration_m_s2.z, 1e-4f)
		<< "and must not command it to accelerate as if it were stationary";

	// Independently: the running case really is grip-limited, i.e. the numbers
	// above are not equal merely because both collapsed to the standing case.
	MyopicState st{};
	MyopicInput standing = GroundChase(0.8f, 0.f, 1.f / 60.f);
	standing.velocity_m_s      = glm::vec3(0.f);
	standing.desired_speed_m_s = 0.f;
	MyopicOutput pivot;
	for (int i = 0; i < 600; ++i) pivot = ComputeMyopicControl(*out, standing, st);
	EXPECT_LT(std::fabs(still.angular_velocity_rad_s.y),
	          0.1f * std::fabs(pivot.angular_velocity_rad_s.y));
}

// The launch is the ONE deliberate exception: readiness is about generating
// lift, so it is airspeed no matter what mode the creature launches from. This
// duplicates MyopicAirspeed.HeadwindMakesLaunchEasierThanTailwind's intent from
// the other side -- a wind that changes nothing on the ground channel must
// still move readiness.
TEST(MyopicFrame, LaunchReadinessStaysAnAirspeedEvenFromTheGround)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	const float v_min = float(out->aerial->min_flight_speed_m_s);

	MyopicInput in;
	in.mode         = LocomotionMode::TERRESTRIAL;
	in.target_mode  = LocomotionMode::AERIAL;
	in.substrate    = Substrate::GROUND;
	in.dt_s         = 1.f / 60.f;
	// Same ground speed both times. Both airspeeds are chosen to land strictly
	// inside the [0, 1] clamp so the arithmetic is pinned, not just the order.
	in.velocity_m_s = glm::vec3(0.f, 0.f, 3.f);

	MyopicState a{}, b{};
	MyopicInput head = in; head.medium_velocity_m_s = glm::vec3(0.f, 0.f, -2.f);
	MyopicInput tail = in; tail.medium_velocity_m_s = glm::vec3(0.f, 0.f,  2.f);

	MyopicOutput h = ComputeMyopicControl(*out, head, a);
	MyopicOutput t = ComputeMyopicControl(*out, tail, b);

	EXPECT_NEAR(h.transition_readiness, 5.f / v_min, 1e-4f);
	EXPECT_NEAR(t.transition_readiness, 1.f / v_min, 1e-4f);
	EXPECT_GT(h.transition_readiness, t.transition_readiness);
}

// --- G4: MODE_UNAVAILABLE must not read as fully comfortable ---------------
TEST(MyopicEntryPoint, ModeUnavailableReportsNoAuthorityNotFullComfort)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	// Modes cat.glb genuinely has no analysis section for. CLIMBING was in this
	// list until Task 7: the cat DOES have an Analysis_Climbing (cats climb), it
	// simply had no envelope arm yet, so the mode read as unavailable for the
	// wrong reason. BRACHIATION replaces it -- no sample model has arms long
	// enough to produce an Analysis_Brachiation.
	ASSERT_FALSE(out->aerial.has_value());
	ASSERT_FALSE(out->aquatic.has_value());
	ASSERT_FALSE(out->serpentine.has_value());
	ASSERT_FALSE(out->brachiation.has_value());
	for (LocomotionMode m : {LocomotionMode::AERIAL, LocomotionMode::AQUATIC,
	                         LocomotionMode::SERPENTINE, LocomotionMode::BRACHIATION}) {
		MyopicState st{};
		MyopicInput in;
		in.mode        = m;
		in.target_mode = m;
		in.dt_s        = 1.f / 60.f;

		MyopicOutput o = ComputeMyopicControl(*out, in, st);
		ASSERT_EQ(o.blocking_reason, BlockingReason::MODE_UNAVAILABLE) << int(m);
		EXPECT_EQ(o.stability, 0.f)
			<< "mode " << int(m) << ": a mode the creature does not have is not comfortable";
		EXPECT_EQ(o.speed_headroom, 0.f) << "mode " << int(m);
		EXPECT_EQ(o.turn_headroom, 0.f)  << "mode " << int(m);
	}
}

// --- G3/PT1: the FlattenDirection velocity fallback -------------------------
//
// VerticalOrientationDoesNotProduceNaN makes the velocity vertical too, so it
// never enters the fallback and the branch could be deleted outright. Here the
// creature is nose-up but genuinely travelling along +X, and +X is the only
// heading it has.
TEST(MyopicEntryPoint, NoHeadingFallsBackToTheDirectionOfTravel)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	MyopicState st{};
	MyopicInput in = GroundChase(0.f, 3.f, 1.f / 60.f); // target dead ahead on +Z
	in.orientation  = glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.f, 0.f, 0.f));
	in.velocity_m_s = glm::vec3(3.f, 0.f, 0.f);         // ...but moving along +X
	in.desired_speed_m_s = 5.f;                         // ...and asked to speed up

	MyopicOutput o = ComputeMyopicControl(*out, in, st);

	// Heading +X, target +Z: the signed error is -pi/2, so the turn is negative.
	EXPECT_LT(o.angular_velocity_rad_s.y, 0.f)
		<< "with the fallback deleted there is no heading and no turn at all";
	// ...and the push goes along the travel direction, not along +Z.
	EXPECT_GT(o.linear_acceleration_m_s2.x, 0.f);
	EXPECT_EQ(o.linear_acceleration_m_s2.z, 0.f);
	EXPECT_EQ(o.linear_acceleration_m_s2.y, 0.f);
}

// --- G3/PT3: the jump fields must actually be forwarded --------------------
TEST(MyopicEntryPoint, ForwardsTheLaunchPlansJumpFields)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);

	MyopicState st{};
	MyopicInput in;
	in.mode        = LocomotionMode::TERRESTRIAL;
	in.target_mode = LocomotionMode::AERIAL;
	in.substrate   = Substrate::GROUND;
	in.dt_s        = 1.f / 60.f;

	LaunchPlan plan = PlanLaunch(*out, in, 0.f);
	MyopicOutput o  = ComputeMyopicControl(*out, in, st);

	EXPECT_EQ(o.jump_direction, plan.jump_direction);
	EXPECT_EQ(o.jump_direction, glm::vec3(0.f, 1.f, 0.f))
		<< "an unpopulated jump_direction is a zero vector, which is not 'up'";
	EXPECT_EQ(o.jump_feasible, plan.jump_feasible);
	EXPECT_EQ(o.required_jump_velocity_m_s, plan.required_jump_velocity_m_s);
	EXPECT_EQ(o.transition_readiness, plan.readiness);

	// A creature not transitioning to AERIAL gets no launch plan at all, so the
	// fields stay at their (zero) defaults -- which is why the check above needs
	// a target_mode of AERIAL to mean anything.
	MyopicState st2{};
	MyopicInput ground = in;
	ground.target_mode = LocomotionMode::TERRESTRIAL;
	MyopicOutput g = ComputeMyopicControl(*out, ground, st2);
	EXPECT_EQ(g.jump_direction, glm::vec3(0.f));
}

// --- G9: the published diagnostic must not depend on the frame rate --------
//
// u_turn's numerator used to be turn_stop_bound = error / max(tau_linear, dt),
// which is dt-proportional for any creature with tau < dt. Measured before the
// split, at a 0.8 rad heading error:
//
//   cat       tau 0.0333 s   16 Hz  -27.15   60 Hz  -51.91   240 Hz  -51.91
//   batto     tau 0.0339 s   16 Hz   -7.33   60 Hz  -14.37   240 Hz  -14.37
//   dragonfly tau 0.00087 s  16 Hz  -45.12   60 Hz -171.96   240 Hz -690.82
//
// while the TRAJECTORY those numbers describe varied by 0.12% over the same
// range. The anti-overshoot clamp still uses max(tau, dt) -- that is adjudicated
// and untouched -- but "the rate I want" is a property of the creature and the
// error, not of how often the caller asks.
TEST(MyopicFramerate, StabilityIsFramerateIndependent)
{
	struct Case { const char* file; Env env; };
	const Case cases[] = {
		{"cat.glb", Env::Air}, {"dragonfly.glb", Env::Air},
		{"batto.glb", Env::Air}, {"penguin.glb", Env::Ocean},
	};
	const float rates[] = {16.f, 60.f, 240.f};

	for (auto const& c : cases) {
		const Output* out = Analyze(c.file, c.env);
		ASSERT_NE(out, nullptr) << c.file;
		auto env = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 0, 9.81f);
		ASSERT_TRUE(env.has_value()) << c.file;
		const float v = 0.5f * float(env->max_speed);

		float first = 0.f;
		for (int k = 0; k < 3; ++k) {
			const float dt = 1.f / rates[k];
			MyopicState st{};
			MyopicOutput o;
			for (int i = 0; i < int(std::lround(4.f * rates[k])); ++i) {
				MyopicInput in = GroundChase(0.8f, v, dt);
				o = ComputeMyopicControl(*out, in, st);
			}
			std::cerr << "[dt-independence] " << c.file << " tau="
			          << float(env->tau_linear) << " " << rates[k] << " Hz stability="
			          << o.stability << " turn_headroom=" << o.turn_headroom << "\n";
			if (k == 0) first = o.stability;
			else EXPECT_NEAR(o.stability, first, 1e-3f * std::fabs(first) + 1e-4f)
				<< c.file << " at " << rates[k] << " Hz";
		}
	}
}

// ===========================================================================
// Fix round 2. Every test below exists because a named perturbation of the
// shipped code left the previous 79 green.
// ===========================================================================

// --- H1: the launch run's wind conversion ----------------------------------
//
// `required_ground = required_airspeed + dot(wind, forward)` is one line, and
// both flipping its sign to `-=` and deleting it outright left the whole suite
// green. LaunchRunReachesFullReadiness runs in still air, where the term is
// exactly 0; LaunchReadinessStaysAnAirspeedEvenFromTheGround is a one-frame
// check that never reaches the speed floor at all.
//
// Closed loop, wind along the heading (+Z, the direction the run holds). The
// conversion is pinned from BOTH sides at once: the converged GROUND speed must
// be R -/+ 3 while the achieved AIRSPEED is R either way. The sign flip breaks
// the ground speed and the airspeed together; the deletion leaves the ground
// speed at R and pushes the airspeed to R -/+ 3.
TEST(MyopicLaunch, WindAlongTheRunwayShiftsGroundSpeedNotAirspeed)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value());
	ASSERT_EQ(out->aerial->takeoff.mode, TakeoffMode::RUNNING_TAKEOFF);

	const float R = float(out->aerial->min_flight_speed_m_s);
	const float w = 3.f;
	ASSERT_GT(R, w) << "the headwind must not swallow the whole requirement here";

	struct Case { const char* name; float wind_z; float expected_ground; };
	const Case cases[] = {
		{"headwind", -w, R - w},   // dot(wind, +Z) = -3 -> a shorter run
		{"tailwind", +w, R + w},   // dot(wind, +Z) = +3 -> a longer one
		{"still",    0.f, R},
	};

	for (auto const& c : cases) {
		LaunchRunResult r = SimulateLaunchRun(*out, /*gait=*/2, /*frames=*/3000,
		                                      glm::vec3(0.f, 0.f, c.wind_z));
		std::cerr << "[launch wind] " << c.name << " ground=" << r.ground_speed
		          << " airspeed=" << r.airspeed << " readiness=" << r.readiness
		          << " (required airspeed " << r.required_airspeed << ")\n";

		EXPECT_NEAR(r.ground_speed, c.expected_ground, 1e-2f * c.expected_ground)
			<< c.name << ": the run converged on the wrong GROUND speed";
		EXPECT_NEAR(r.airspeed, R, 1e-2f * R)
			<< c.name << ": the wing must see the same airspeed in every wind";
		EXPECT_NEAR(r.readiness, 1.f, 1e-3f) << c.name;
	}
}

// The floor-at-0 branch, which nothing reached: a headwind stronger than the
// whole requirement means the creature is already at flight speed STANDING
// STILL. `required` goes negative, std::max clamps it to 0, and the launch then
// asks for nothing at all -- so the speed channel goes back to being the
// caller's, here the gait's own top speed.
TEST(MyopicLaunch, AHeadwindStrongerThanTheRequirementHandsBackTheSpeedChannel)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	const float R = float(out->aerial->min_flight_speed_m_s);

	const float wind = R + 2.f;   // blowing straight down the runway at us
	auto env = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 2, 9.81f);
	ASSERT_TRUE(env.has_value());
	const float gait_top = float(env->max_speed);
	ASSERT_LT(gait_top, R) << "this gait must not cover flight speed on its own";

	LaunchRunResult r = SimulateLaunchRun(*out, /*gait=*/2, /*frames=*/3000,
	                                      glm::vec3(0.f, 0.f, -wind));
	std::cerr << "[launch floor 0] wind=" << wind << " ground=" << r.ground_speed
	          << " gait_top=" << gait_top << " airspeed=" << r.airspeed
	          << " readiness=" << r.readiness << "\n";

	EXPECT_NEAR(r.ground_speed, gait_top, 1e-2f * gait_top)
		<< "with the requirement already met, the speed channel belongs to the caller";
	EXPECT_NEAR(r.readiness, 1.f, 1e-4f);
	EXPECT_GT(r.airspeed, R) << "standing in this wind is already flying";
}

// --- H5: the requirement is a FLOOR, not a replacement ---------------------
//
// Turning `std::max(floor, wish)` into a bare `floor` left 79/79 green: every
// launch fixture in the suite either leaves desired_speed_m_s at -1 or asks for
// less than flight speed, so the two forms agreed everywhere. A caller that
// wants MARGIN over the stall -- or is chasing something down the runway -- is
// the case that distinguishes them.
TEST(MyopicLaunch, TheLaunchRequirementIsAFloorNotACeiling)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	const float R = float(out->aerial->min_flight_speed_m_s);

	const float wanted = 2.f * R;
	LaunchRunResult fast = SimulateLaunchRun(*out, 2, 3000, glm::vec3(0.f), wanted);
	std::cerr << "[launch floor] asked " << wanted << " required " << R
	          << " -> ground " << fast.ground_speed << "\n";
	EXPECT_NEAR(fast.ground_speed, wanted, 1e-2f * wanted)
		<< "a caller asking for margin over the stall had its number thrown away";

	// ...and the floor still binds in the other direction: asking for less than
	// flight speed does NOT get you a takeoff run that stalls short.
	LaunchRunResult slow = SimulateLaunchRun(*out, 2, 3000, glm::vec3(0.f), 0.25f * R);
	EXPECT_NEAR(slow.ground_speed, R, 1e-2f * R)
		<< "the launch requirement stopped being a floor";
	EXPECT_NEAR(slow.readiness, 1.f, 1e-3f);
}

// --- H2: the LaunchFacts wiring -------------------------------------------
//
// G3 made the DISPATCH testable and in doing so created an untested seam
// between it and its only caller. Every MyopicEntryPoint test that compared
// ComputeMyopicControl's output against PlanLaunch(*out, in, v) was comparing
// two values that had both been through this same adapter, so a corrupted field
// cancelled out on both sides. Measured: clearing can_use_water_taxi, swapping
// the required and available jump velocities, and hardcoding gravity to 9.81
// each left 79/79 green.
//
// The oracle has to be the test body reading `Output` itself.
namespace {

// Deliberately spelled out rather than factored: this IS the assertion.
LaunchFacts ExpectedFacts(const Output& o, const MyopicInput& in, float airspeed)
{
	const Analysis_Aerial&           a = *o.aerial;
	const Analysis_TakeoffAnalysis&  t = a.takeoff;

	LaunchFacts f;
	f.mode                        = t.mode;
	f.required_airspeed_m_s       = float(a.min_flight_speed_m_s);
	f.airspeed_m_s                = airspeed;
	f.gravity_m_s2                = in.gravity_m_s2;
	f.required_jump_velocity_m_s  = float(t.required_jump_velocity_m_s);
	f.has_jump_analysis           = o.jumping.has_value();
	f.available_jump_velocity_m_s = o.jumping.has_value()
		? float(o.jumping->takeoff_velocity_m_s) : 0.f;
	f.substrate                   = in.substrate;
	f.can_use_water_taxi          = t.can_use_water_taxi;
	f.wing_loading_ok             = t.constraints.wing_loading_ok;
	f.power_loading_ok            = t.constraints.power_loading_ok;
	f.aspect_ratio_ok             = t.constraints.aspect_ratio_ok;
	f.leg_strength_ok             = t.constraints.leg_strength_ok;
	return f;
}

void ExpectSamePlan(const LaunchPlan& a, const LaunchPlan& b, const std::string& where)
{
	EXPECT_EQ(a.feasible,                   b.feasible)                   << where;
	EXPECT_FLOAT_EQ(a.readiness,            b.readiness)                  << where;
	EXPECT_EQ(a.blocking_reason,            b.blocking_reason)            << where;
	EXPECT_FLOAT_EQ(a.required_airspeed_m_s, b.required_airspeed_m_s)     << where;
	EXPECT_FLOAT_EQ(a.required_drop_m,      b.required_drop_m)            << where;
	EXPECT_FLOAT_EQ(a.required_jump_velocity_m_s,
	                b.required_jump_velocity_m_s)                         << where;
	EXPECT_EQ(a.jump_direction,             b.jump_direction)             << where;
	EXPECT_EQ(a.jump_feasible,              b.jump_feasible)              << where;
	EXPECT_EQ(a.accelerate_along_heading,   b.accelerate_along_heading)   << where;
}

} // namespace

TEST(MyopicLaunchDispatch, TheAdapterWiresEveryFactFromTheAnalysis)
{
	struct Case { const char* file; Env env; };
	const Case models[] = {
		{"batto.glb", Env::Air}, {"dragonfly.glb", Env::Air},
		{"penguin.glb", Env::Ocean}, {"penguin.glb", Env::Air},
	};
	const Substrate substrates[] = {
		Substrate::GROUND, Substrate::WATER, Substrate::PERCH, Substrate::CLIFF_EDGE };
	// 3.71 is Mars. A hardcoded 9.81 in the adapter cannot hide behind it, and
	// 0 exercises the free-fall guard through the wiring rather than around it.
	const float gravities[] = {9.81f, 3.71f, 0.f};
	const float airspeeds[] = {0.f, 3.f, 100.f};

	int taxi_true = 0, jump_sections = 0, quantified_jumps = 0;

	for (auto const& m : models) {
		const Output* o = Analyze(m.file, m.env);
		ASSERT_NE(o, nullptr) << m.file;
		ASSERT_TRUE(o->aerial.has_value()) << m.file;

		if (o->aerial->takeoff.can_use_water_taxi) ++taxi_true;
		if (o->jumping.has_value()) {
			++jump_sections;
			if (float(o->aerial->takeoff.required_jump_velocity_m_s) > 0.f)
				++quantified_jumps;
		}

		for (Substrate s : substrates)
		for (float g : gravities)
		for (float v : airspeeds) {
			MyopicInput in;
			in.mode         = LocomotionMode::TERRESTRIAL;
			in.target_mode  = LocomotionMode::AERIAL;
			in.substrate    = s;
			in.gravity_m_s2 = g;

			const std::string where = std::string(m.file)
				+ " substrate=" + std::to_string(int(s))
				+ " g=" + std::to_string(g) + " v=" + std::to_string(v);

			std::optional<LaunchFacts> got = MakeLaunchFacts(*o, in, v);
			ASSERT_TRUE(got.has_value()) << where;
			const LaunchFacts want = ExpectedFacts(*o, in, v);

			// Field by field against a source that does not share the adapter.
			EXPECT_EQ(got->mode, want.mode) << where;
			EXPECT_FLOAT_EQ(got->required_airspeed_m_s, want.required_airspeed_m_s) << where;
			EXPECT_FLOAT_EQ(got->airspeed_m_s, want.airspeed_m_s) << where;
			EXPECT_FLOAT_EQ(got->gravity_m_s2, want.gravity_m_s2) << where
				<< " -- gravity is a world fact the caller owns; do not hardcode 9.81";
			EXPECT_FLOAT_EQ(got->required_jump_velocity_m_s,
			                want.required_jump_velocity_m_s) << where;
			EXPECT_FLOAT_EQ(got->available_jump_velocity_m_s,
			                want.available_jump_velocity_m_s) << where;
			EXPECT_EQ(got->has_jump_analysis, want.has_jump_analysis) << where;
			EXPECT_EQ(got->substrate, want.substrate) << where;
			EXPECT_EQ(got->can_use_water_taxi, want.can_use_water_taxi) << where;
			EXPECT_EQ(got->wing_loading_ok,  want.wing_loading_ok)  << where;
			EXPECT_EQ(got->power_loading_ok, want.power_loading_ok) << where;
			EXPECT_EQ(got->aspect_ratio_ok,  want.aspect_ratio_ok)  << where;
			EXPECT_EQ(got->leg_strength_ok,  want.leg_strength_ok)  << where;

			// ...and the whole plan, so a field the dispatch reads but the list
			// above forgets is still caught.
			ExpectSamePlan(PlanLaunch(want), PlanLaunch(*o, in, v), where);
		}
	}

	// The sweep only pins a boolean if some model actually sets it: if every
	// sample reported false, `can_use_water_taxi = false` would be invisible
	// here too. Same for the two jump fields, whose SWAP is only observable
	// where they differ.
	EXPECT_GT(taxi_true, 0)
		<< "no sample reports can_use_water_taxi; this sweep cannot pin that field";
	EXPECT_GT(jump_sections, 0) << "no sample has a jumping section";

	// The jump-velocity swap, pinned directly rather than through the sweep:
	// required and available must differ somewhere for the swap to show, and the
	// dispatch only compares them in the JUMP_LAUNCH arm, which no sample enters.
	int distinguishing = 0;
	for (auto const& m : models) {
		const Output* o = Analyze(m.file, m.env);
		if (!o || !o->jumping.has_value()) continue;
		const float req = float(o->aerial->takeoff.required_jump_velocity_m_s);
		const float ava = float(o->jumping->takeoff_velocity_m_s);
		std::cerr << "[jump wiring] " << m.file << " required=" << req
		          << " available=" << ava << "\n";
		if (std::fabs(req - ava) > 1e-6f) ++distinguishing;
	}
	EXPECT_GT(distinguishing, 0)
		<< "required and available jump velocity are equal in every sample, so a "
		   "swap between them would be unobservable";
	(void)quantified_jumps;
}

// --- H3: SpeedFrameOf's AERIAL arm ----------------------------------------
//
// Moving AERIAL from MEDIUM to GROUND *alone* left 79/79 green -- only swapping
// it with TERRESTRIAL bit, and then only through the two TERRESTRIAL tests. No
// test flew a creature in a wind at all. batto has no aerial envelope (negative
// power surplus; see MyopicEnvelope.AerialPresenceFollowsPowerSurplus and the
// documented Species.Bat clade red), so this uses the dragonfly.
TEST(MyopicFrame, AFlyersEnvelopeIsEvaluatedAgainstTheAirNotTheGround)
{
	const Output* out = Analyze("dragonfly.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(env.has_value()) << "this test needs a creature that actually flies";
	const float stall = float(env->min_speed);
	ASSERT_GT(stall, 0.f);

	// Fly along +Z at `ground` with the air itself moving at `wind` along +Z.
	auto settle = [&](float ground, float wind) {
		MyopicState st{};
		MyopicOutput o;
		for (int i = 0; i < 600; ++i) {
			MyopicInput in;
			in.mode                = LocomotionMode::AERIAL;
			in.target_mode         = LocomotionMode::AERIAL;
			in.orientation         = glm::quat(1.f, 0.f, 0.f, 0.f); // forward +Z
			in.target_position     = TargetAtBearing(0.8f);
			in.velocity_m_s        = glm::vec3(0.f, 0.f, ground);
			in.medium_velocity_m_s = glm::vec3(0.f, 0.f, wind);
			in.desired_speed_m_s   = ground - wind;   // hold this airspeed
			in.dt_s                = 1.f / 60.f;
			o = ComputeMyopicControl(*out, in, st);
		}
		return o;
	};

	// Same AIRSPEED, very different ground speeds. Under the frame rule these
	// are the same flight condition and must read identically.
	MyopicOutput calm  = settle(2.0f * stall, 0.f);
	MyopicOutput blown = settle(4.0f * stall, 2.0f * stall);

	std::cerr << "[frame aerial] stall=" << stall
	          << " | airspeed 2*stall, no wind: turn=" << calm.angular_velocity_rad_s.y
	          << " stability=" << calm.stability << " bank=" << calm.bank_angle_rad
	          << " | same airspeed, 2*stall tailwind: turn=" << blown.angular_velocity_rad_s.y
	          << " stability=" << blown.stability << " bank=" << blown.bank_angle_rad << "\n";

	EXPECT_NEAR(blown.angular_velocity_rad_s.y, calm.angular_velocity_rad_s.y, 1e-5f)
		<< "the same airspeed is the same flight condition, whatever the ground does";
	EXPECT_NEAR(blown.stability,      calm.stability,      1e-4f);
	EXPECT_NEAR(blown.speed_headroom, calm.speed_headroom, 1e-4f);
	EXPECT_NEAR(blown.turn_headroom,  calm.turn_headroom,  1e-4f);
	EXPECT_NEAR(blown.bank_angle_rad, calm.bank_angle_rad, 1e-5f);

	// And the other direction, so the equalities above are not two cases
	// collapsing onto the same wrong answer: the SAME ground speed with a
	// tailwind that puts the wing below its stall speed is a different, worse
	// flight condition. Under a ground-frame reading it would be identical to
	// `calm`.
	MyopicOutput stalled = settle(2.0f * stall, 1.5f * stall); // airspeed 0.5*stall
	std::cerr << "[frame aerial] below stall: stability=" << stalled.stability
	          << " speed_headroom=" << stalled.speed_headroom << "\n";
	EXPECT_LT(stalled.stability, calm.stability - 1e-3f)
		<< "a tailwind that erases the airspeed must be visible to a WING";
	EXPECT_LT(stalled.stability, 0.f) << "below stall is not a comfortable place to be";
}

// --- H4: the launch readouts must reach a caller ---------------------------
//
// required_drop_m was derived correctly in LaunchPlan and copied nowhere:
// MyopicOutput had no drop field, so a CLIFF_LAUNCH creature's caller learned
// only NEEDS_ELEVATION and could never find out how much.
TEST(MyopicEntryPoint, ForwardsTheRequiredDrop)
{
	for (const char* file : {"batto.glb", "dragonfly.glb"}) {
		const Output* out = Analyze(file, Env::Air);
		ASSERT_NE(out, nullptr) << file;
		ASSERT_TRUE(out->aerial.has_value()) << file;
		const float v_stall = float(out->aerial->min_flight_speed_m_s);

		for (float g : {9.81f, 3.71f}) {
			MyopicState st{};
			MyopicInput in;
			in.mode         = LocomotionMode::TERRESTRIAL;
			in.target_mode  = LocomotionMode::AERIAL;
			in.substrate    = Substrate::CLIFF_EDGE;
			in.gravity_m_s2 = g;
			in.dt_s         = 1.f / 60.f;

			MyopicOutput o = ComputeMyopicControl(*out, in, st);
			const float expected = v_stall * v_stall / (2.f * g);
			EXPECT_NEAR(o.required_drop_m, expected, 1e-3f * expected)
				<< file << " at g = " << g;
			EXPECT_GT(o.required_drop_m, 0.f) << file;
			std::cerr << "[drop] " << file << " g=" << g << " v_stall=" << v_stall
			          << " -> required_drop=" << o.required_drop_m << " m\n";
		}

		// Not transitioning to AERIAL: no launch plan, no drop.
		MyopicState st2{};
		MyopicInput ground;
		ground.mode        = LocomotionMode::TERRESTRIAL;
		ground.target_mode = LocomotionMode::TERRESTRIAL;
		ground.dt_s        = 1.f / 60.f;
		EXPECT_EQ(ComputeMyopicControl(*out, ground, st2).required_drop_m, 0.f) << file;
	}
}

// `feasible` is NOT redundant with `blocking_reason == NONE`. A RUNNING_TAKEOFF
// creature reports NEEDS_RUNWAY_SPEED in two completely different situations,
// and only `launch_feasible` separates them: accelerating down a real runway
// (keep going) versus standing somewhere it can never take off from (give up).
TEST(MyopicEntryPoint, LaunchFeasibilityIsNotTheSameFactAsTheBlockingReason)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_EQ(out->aerial->takeoff.mode, TakeoffMode::RUNNING_TAKEOFF);
	ASSERT_FALSE(out->aerial->takeoff.can_use_water_taxi);
	const float v_min = float(out->aerial->min_flight_speed_m_s);

	MyopicInput base;
	base.mode        = LocomotionMode::TERRESTRIAL;
	base.target_mode = LocomotionMode::AERIAL;
	base.dt_s        = 1.f / 60.f;

	MyopicState a{};
	MyopicInput running = base;
	running.substrate    = Substrate::GROUND;
	running.velocity_m_s = glm::vec3(0.f, 0.f, 0.5f * v_min);
	MyopicOutput mid = ComputeMyopicControl(*out, running, a);

	MyopicState b{};
	MyopicInput perched = base;
	perched.substrate = Substrate::PERCH;
	MyopicOutput stuck = ComputeMyopicControl(*out, perched, b);

	// Identical reason, opposite verdicts. This is the whole point.
	ASSERT_EQ(mid.blocking_reason,   BlockingReason::NEEDS_RUNWAY_SPEED);
	ASSERT_EQ(stuck.blocking_reason, BlockingReason::NEEDS_RUNWAY_SPEED);
	EXPECT_TRUE(mid.launch_feasible)
		<< "a run that is halfway to flight speed is going to work";
	EXPECT_FALSE(stuck.launch_feasible)
		<< "no amount of patience gets a running takeoff off a perch";

	// And it is false by default when no launch is being planned at all.
	MyopicState c{};
	MyopicInput ground = base;
	ground.target_mode = LocomotionMode::TERRESTRIAL;
	EXPECT_FALSE(ComputeMyopicControl(*out, ground, c).launch_feasible);
}

// --- H6: can_use_water_taxi means a RUNNING takeoff, and nothing else ------
//
// The flag is set from `output.aquatic.has_value()` and documented as "run on
// water surface (pelicans)". Using it to clear a standing LEG THRUST off open
// water borrowed evidence from a different behaviour: a pelican pattering
// across a lake with its wings carrying the weight says nothing about whether
// its legs could launch it from one. Nothing in the analysis layer speaks to
// leg thrust against a non-solid substrate, so JUMP_LAUNCH refuses water flat.
TEST(MyopicLaunchDispatch, WaterTaxiDoesNotBuyALegThrust)
{
	LaunchFacts f = BaseFacts(TakeoffMode::JUMP_LAUNCH);
	f.has_jump_analysis           = true;
	f.required_jump_velocity_m_s  = 3.f;
	f.available_jump_velocity_m_s = 4.f;   // legs are plenty strong
	f.substrate                   = Substrate::WATER;
	f.can_use_water_taxi          = true;  // ...and it taxis

	LaunchPlan p = PlanLaunch(f);
	EXPECT_FALSE(p.feasible)
		<< "taxiing is a running takeoff; it is not evidence about leg thrust";
	EXPECT_EQ(p.readiness, 0.f);
	EXPECT_EQ(p.blocking_reason, BlockingReason::NEEDS_SOLID_SUBSTRATE);

	// The same flag on the same substrate DOES clear the two behaviours it
	// actually describes -- a run across the surface, and a standing wingbeat,
	// which asks the water only to hold the creature up.
	LaunchFacts run = BaseFacts(TakeoffMode::RUNNING_TAKEOFF);
	run.substrate          = Substrate::WATER;
	run.can_use_water_taxi = true;
	EXPECT_TRUE(PlanLaunch(run).feasible) << "a pelican's runway is the lake";

	LaunchFacts hover = BaseFacts(TakeoffMode::VERTICAL_LAUNCH);
	hover.substrate          = Substrate::WATER;
	hover.can_use_water_taxi = true;
	EXPECT_TRUE(PlanLaunch(hover).feasible) << "buoyancy holds a floating bird up";

	// ...and solid ground still accepts the jump, so the refusal above is about
	// the water and not about the arm being gutted.
	f.substrate = Substrate::GROUND;
	EXPECT_TRUE(PlanLaunch(f).feasible);
}

// ============================================================================
// Task 7: aquatic, serpentine, climbing and brachiation envelopes
// ============================================================================

// --- The plan's Step 1 tests ----------------------------------------------

TEST(MyopicEnvelope, AquaticInvariants)
{
	const Output* out = Analyze("penguin.glb", Env::Ocean);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aquatic.has_value());

	auto env = ExtractEnvelope(*out, LocomotionMode::AQUATIC, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	std::cerr << "[aquatic penguin] max_speed=" << float(env->max_speed)
	          << " min_speed=" << float(env->min_speed)
	          << " max_accel=" << float(env->max_accel)
	          << " max_brake=" << float(env->max_brake)
	          << " a_lat=" << float(env->max_lateral_accel)
	          << " r=" << float(env->min_turn_radius)
	          << " tau=" << float(env->tau_linear) << "\n";

	EXPECT_GT(float(env->max_speed), 0.f);
	EXPECT_LT(float(env->min_speed), float(env->max_speed));
	EXPECT_GT(float(env->max_accel), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->tau_linear)));
	EXPECT_GT(float(env->tau_linear), 0.f);
	EXPECT_FALSE(env->aerial.has_value()) << "swimmers have no load factor";

	// AQUATIC is the ONLY one of Task 7's four arms with a real, measured
	// turning radius -- serpentine, climbing and brachiation all report the
	// no-constraint sentinel 0. Nothing pinned that it is actually used: quietly
	// replacing it with the sentinel here (i.e. discarding the one measurement)
	// was invisible to the suite. It has to be a positive number, and it has to
	// be the analysis's own number.
	EXPECT_GT(float(env->min_turn_radius), 0.f)
		<< "the aquatic arm has a stated turning radius; it must not fall back "
		   "to the no-constraint sentinel";
	EXPECT_FLOAT_EQ(float(env->min_turn_radius),
	                float(out->aquatic->min_turning_radius_m));
}

// The plan asserts `requires_constant_motion` on shark.glb in Env::Ocean. It is
// FALSE there, and no input can make it true: tonton_aquatic.cpp:224 requires
// body_density > 1.05 * fluid_density, body_density() (tonton_input.h:103) tops
// out at 1050 kg/m^3, and 1.05 * 1025 = 1076.25 kg/m^3. So in seawater the flag
// is unreachable -- see the follow-up findings. Env::Air is the only medium in
// which this sample exercises the branch, and what is under test here is the
// ENVELOPE ARM (does a min-swim-speed floor reach Envelope::min_speed?), not
// the fluid. Both media are asserted so the finding stays visible.
TEST(MyopicEnvelope, SharkRequiresConstantMotion)
{
	const Output* air = Analyze("shark.glb", Env::Air);
	ASSERT_NE(air, nullptr);
	ASSERT_TRUE(air->aquatic.has_value());
	ASSERT_TRUE(air->aquatic->requires_constant_motion);
	ASSERT_GT(float(air->aquatic->min_swim_speed_m_s), 0.f);

	auto env = ExtractEnvelope(*air, LocomotionMode::AQUATIC, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	// A shark's minimum swim speed is a real floor, expressed as min_speed so
	// the existing stability term reports it. No transition logic needed.
	EXPECT_GT(float(env->min_speed), 0.f);
	EXPECT_FLOAT_EQ(float(env->min_speed), float(air->aquatic->min_swim_speed_m_s));
	EXPECT_LT(float(env->min_speed), float(env->max_speed));

	// Documents the finding above rather than hiding it: in seawater the same
	// creature reports no floor at all.
	const Output* sea = Analyze("shark.glb", Env::Ocean);
	ASSERT_NE(sea, nullptr);
	ASSERT_TRUE(sea->aquatic.has_value());
	EXPECT_FALSE(sea->aquatic->requires_constant_motion)
		<< "if this ever becomes true, the density-cap finding is fixed and this "
		   "test should move wholesale to Env::Ocean";
	auto sea_env = ExtractEnvelope(*sea, LocomotionMode::AQUATIC, 0, 9.81f);
	ASSERT_TRUE(sea_env.has_value());
	EXPECT_FLOAT_EQ(float(sea_env->min_speed), 0.f);
}

TEST(MyopicEnvelope, SerpentineInvariants)
{
	const Output* out = Analyze("eel.glb", Env::Ocean);
	ASSERT_NE(out, nullptr);
	if (!out->serpentine.has_value()) GTEST_SKIP() << "eel has no serpentine section";

	auto env = ExtractEnvelope(*out, LocomotionMode::SERPENTINE, 0, 9.81f);
	ASSERT_TRUE(env.has_value());
	std::cerr << "[serpentine eel] max_speed=" << float(env->max_speed)
	          << " min_speed=" << float(env->min_speed)
	          << " max_accel=" << float(env->max_accel)
	          << " tau=" << float(env->tau_linear) << "\n";
	EXPECT_GT(float(env->max_speed), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->tau_linear)));

	// The envelope is the undulation speed, not some rescaled version of it.
	EXPECT_FLOAT_EQ(float(env->max_speed),
	                float(out->serpentine->lateral_undulation_speed_m_s));
	EXPECT_GT(float(env->min_speed), 0.f);
	EXPECT_LT(float(env->min_speed), float(env->max_speed));
	EXPECT_FALSE(env->aerial.has_value());

	// --- PLACEHOLDER PIN (J3) ---------------------------------------------
	// The serpentine max_accel is `undulation_speed / 1 second`. That divisor is
	// a PLACEHOLDER for a quantity Analysis_Serpentine does not export (the
	// undulation frequency, which tonton_serpentine.cpp:186 computes and drops
	// on the floor), NOT a derived time. Shipped deliberately and adjudicated --
	// but nothing observed it, so it could be changed to any other divisor with
	// the suite staying green, which would silently rescale every `stability`
	// and `turn_headroom` a snake reports.
	//
	// These two assertions make the placeholder VISIBLE. They are pinning a
	// known-provisional value, and they are EXPECTED to fail the day the
	// frequency is exported and max_accel becomes the honest `speed * f`. When
	// that happens, update them deliberately -- do not delete them.
	const float undulation = float(out->serpentine->lateral_undulation_speed_m_s);
	EXPECT_FLOAT_EQ(float(env->max_accel), undulation)
		<< "placeholder: max_accel is undulation_speed / 1 s";
	EXPECT_FLOAT_EQ(float(env->max_brake), undulation);
	EXPECT_FLOAT_EQ(float(env->tau_linear), 1.f)
		<< "placeholder: tau is 1 s by construction, not by derivation";
}

// --- C1: the aquatic acceleration is a MECHANICAL SURPLUS -----------------
//
// The plan fed `metabolic.max_rate_W` -- a whole-organism metabolic rate -- into
// a = P/(m v). That is the exact defect Task 4 removed from the aerial arm.
//
// J1: the first fix mirrored the AERIAL arm syntactically -- muscle power minus
// the cruise cost -- but that is not the same OPERATION. Aerial subtracts a real
// aerodynamic DEMAND derived independently of muscle power (184% of it for the
// bat, which correctly zeroes the surplus). Both aquatic power figures are
// BUDGET ALLOCATIONS of the same available_muscle_power_W, so
// `muscle - 0.08*muscle` was a fixed 0.92*muscle on every sample forever, and it
// assumed 2.3x the budget the burst SPEED bounding the same envelope came from.
// The arm now spends out of the burst budget: 0.4*muscle - 0.08*muscle.
//
// J4: the PROVENANCE of both exported fields is pinned against
// available_muscle_power_W directly, not recomputed from the fields themselves.
// Without that, rewiring either field to a different multiple of the muscle
// power left the suite green, because the expectation was derived from the very
// field under test.
TEST(MyopicEnvelope, AquaticAccelerationIsAMechanicalSurplus)
{
	for (const char* file : {"penguin.glb", "shark.glb", "eel.glb"}) {
		const Output* out = Analyze(file, Env::Ocean);
		ASSERT_NE(out, nullptr) << file;
		ASSERT_TRUE(out->aquatic.has_value()) << file;
		const auto& a = *out->aquatic;

		auto env = ExtractEnvelope(*out, LocomotionMode::AQUATIC, 0, 9.81f);
		ASSERT_TRUE(env.has_value()) << file;

		// --- J4: PROVENANCE, asserted against available_muscle_power_W --------
		// Not "the field is smaller than the muscle power" (which 0.08, 0.4 and
		// a rewired-by-mistake 0.9 all satisfy), but the exact allocation
		// tonton_aquatic.cpp documents. Rewiring either field to the other's
		// multiple must be a RED here, not a silently different envelope.
		const float muscle = float(out->metabolic.available_muscle_power_W);
		ASSERT_GT(muscle, 0.f) << file;
		ASSERT_GT(float(a.swim_power_mechanical_W), 0.f) << file;
		ASSERT_GT(float(a.swim_power_burst_mechanical_W), 0.f) << file;

		EXPECT_NEAR(float(a.swim_power_mechanical_W), 0.08f * muscle,
		            1e-4f * muscle)
			<< file << ": swim_power_mechanical_W is the 8% CRUISE allocation "
			           "of available_muscle_power_W (tonton_aquatic.cpp)";
		EXPECT_NEAR(float(a.swim_power_burst_mechanical_W), 0.4f * muscle,
		            1e-4f * muscle)
			<< file << ": swim_power_burst_mechanical_W is the 40% BURST "
			           "allocation of available_muscle_power_W, and is the same "
			           "budget burst_speed_m_s is derived from";

		// --- J1: the surplus is spent out of the BURST budget -----------------
		// max_speed is burst_speed_m_s, which tonton_aquatic.cpp derives from
		// 0.4*muscle. An acceleration derived from a LARGER budget than the top
		// speed bounding it would make the envelope internally inconsistent.
		const float surplus = std::max(0.f,
			float(a.swim_power_burst_mechanical_W)
			- float(a.swim_power_mechanical_W));
		const float expect = surplus / (float(out->physical.body_mass_kg)
		                              * float(a.cruise_speed_m_s));

		std::cerr << "[c1 " << file << "] muscle=" << muscle
		          << "W cruise_mech=" << float(a.swim_power_mechanical_W)
		          << "W burst_mech=" << float(a.swim_power_burst_mechanical_W)
		          << "W metabolic_max=" << float(out->metabolic.max_rate_W)
		          << "W mass=" << float(out->physical.body_mass_kg)
		          << "kg cruise=" << float(a.cruise_speed_m_s)
		          << "m/s -> max_accel=" << float(env->max_accel)
		          << " (" << float(env->max_accel) / 9.81f << " g)"
		          << " tau_linear=" << float(env->tau_linear) << "\n";

		EXPECT_NEAR(float(env->max_accel), expect, expect * 1e-4f) << file;

		// ...and specifically NOT the 0.92*muscle the pre-J1 arm computed. That
		// value is 2.3x this one on every sample, so the assertion above is not
		// two formulas agreeing by accident.
		const float pre_j1 = (muscle - float(a.swim_power_mechanical_W))
		                   / (float(out->physical.body_mass_kg)
		                    * float(a.cruise_speed_m_s));
		EXPECT_GT(pre_j1, float(env->max_accel) * 2.f)
			<< file << ": subtracting an 8% allocation from the quantity it is "
			           "an allocation OF is not a surplus";

		// ...and specifically NOT the plan's metabolic formula. These differ by
		// more than an order of magnitude on every sample, so the assertion
		// above is not two formulas agreeing by accident.
		const float plan_value = float(out->metabolic.max_rate_W)
			/ (float(out->physical.body_mass_kg) * float(a.cruise_speed_m_s));
		EXPECT_GT(std::fabs(float(env->max_accel) - plan_value), 1.f)
			<< file << ": a metabolic rate is not a mechanical power";

		// tau is the cruise speed divided by exactly that acceleration.
		EXPECT_NEAR(float(env->tau_linear),
		            float(a.cruise_speed_m_s) / float(env->max_accel),
		            1e-5f) << file;
	}
}

// --- C2: braking is never weaker than accelerating under water ------------
TEST(MyopicEnvelope, AquaticBrakeIsAFloorNotACeiling)
{
	const Output* out = Analyze("shark.glb", Env::Ocean);
	ASSERT_NE(out, nullptr);
	auto env = ExtractEnvelope(*out, LocomotionMode::AQUATIC, 0, 9.81f);
	ASSERT_TRUE(env.has_value());
	EXPECT_GE(float(env->max_brake), float(env->max_accel))
		<< "drag adds to whatever the muscles do; braking cannot be the weaker "
		   "channel";
	EXPECT_GT(float(env->max_brake), 0.f);
}

// --- C3: min_turn_radius == 0 is a sentinel, not a measurement -------------
//
// Serpentine, climbing and brachiation all report 0. It has to mean "this mode
// states no radius constraint", and every consumer has to survive it: a
// fabricated v^2/0 bound would be an infinite lateral budget, and a naive
// v/radius turn-rate bound would divide by zero.
TEST(MyopicEnvelope, ZeroTurnRadiusIsANoConstraintSentinel)
{
	struct Case { const char* file; Env env; LocomotionMode mode; };
	const Case cases[] = {
		{"eel.glb", Env::Ocean, LocomotionMode::SERPENTINE},
		{"cat.glb", Env::Air,   LocomotionMode::CLIMBING},
	};

	int checked = 0;
	for (const Case& c : cases) {
		const Output* out = Analyze(c.file, c.env);
		ASSERT_NE(out, nullptr) << c.file;
		auto env = ExtractEnvelope(*out, c.mode, 0, 9.81f);
		ASSERT_TRUE(env.has_value()) << c.file;
		++checked;

		EXPECT_FLOAT_EQ(float(env->min_turn_radius), 0.f) << c.file;
		// The sentinel must fall through to the acceleration budget, finite and
		// positive -- not to infinity and not to zero.
		EXPECT_TRUE(std::isfinite(float(env->max_lateral_accel))) << c.file;
		EXPECT_GT(float(env->max_lateral_accel), 0.f) << c.file;
		EXPECT_FLOAT_EQ(float(env->max_lateral_accel), float(env->max_accel)) << c.file;

		// And it must survive a whole steer, at rest and at speed.
		for (float v : {0.f, 0.5f * float(env->max_speed), float(env->max_speed)}) {
			MyopicState st{};
			MyopicInput in;
			in.mode            = c.mode;
			in.target_mode     = c.mode;
			in.target_position = TargetAtBearing(1.2f);
			in.velocity_m_s    = glm::vec3(0.f, 0.f, v);
			in.dt_s            = 1.f / 60.f;
			MyopicOutput o = ComputeMyopicControl(*out, in, st);
			EXPECT_TRUE(std::isfinite(o.angular_velocity_rad_s.y)) << c.file << " v=" << v;
			EXPECT_TRUE(std::isfinite(o.linear_acceleration_m_s2.x)) << c.file << " v=" << v;
			EXPECT_TRUE(std::isfinite(o.stability)) << c.file << " v=" << v;
			EXPECT_TRUE(std::isfinite(o.turn_headroom)) << c.file << " v=" << v;
		}
	}
	EXPECT_EQ(checked, 2);
}

// --- C5: an AQUATIC envelope is evaluated against the WATER ----------------
//
// The mirror of MyopicFrame.AFlyersEnvelopeIsEvaluatedAgainstTheAirNotTheGround
// for a fin. A fish holding station in a river is the case with no coverage.
TEST(MyopicFrame, ASwimmersEnvelopeIsEvaluatedAgainstTheWaterNotTheGround)
{
	const Output* out = Analyze("shark.glb", Env::Ocean);
	ASSERT_NE(out, nullptr);
	auto env = ExtractEnvelope(*out, LocomotionMode::AQUATIC, 0, 9.81f);
	ASSERT_TRUE(env.has_value());
	const float cruise = float(out->aquatic->cruise_speed_m_s);
	ASSERT_GT(cruise, 0.f);

	auto settle = [&](float ground, float current) {
		MyopicState st{};
		MyopicOutput o;
		for (int i = 0; i < 600; ++i) {
			MyopicInput in;
			in.mode                = LocomotionMode::AQUATIC;
			in.target_mode         = LocomotionMode::AQUATIC;
			in.orientation         = glm::quat(1.f, 0.f, 0.f, 0.f); // forward +Z
			in.target_position     = TargetAtBearing(0.8f);
			in.velocity_m_s        = glm::vec3(0.f, 0.f, ground);
			in.medium_velocity_m_s = glm::vec3(0.f, 0.f, current);
			in.desired_speed_m_s   = ground - current;
			in.dt_s                = 1.f / 60.f;
			o = ComputeMyopicControl(*out, in, st);
		}
		return o;
	};

	MyopicOutput still  = settle(cruise, 0.f);
	MyopicOutput drift  = settle(3.f * cruise, 2.f * cruise); // same water speed

	std::cerr << "[frame aquatic] cruise=" << cruise
	          << " | still: turn=" << still.angular_velocity_rad_s.y
	          << " stability=" << still.stability
	          << " | in a 2x-cruise current at the same water speed: turn="
	          << drift.angular_velocity_rad_s.y
	          << " stability=" << drift.stability << "\n";

	EXPECT_NEAR(drift.angular_velocity_rad_s.y, still.angular_velocity_rad_s.y, 1e-5f)
		<< "a fin only ever sees the water";
	EXPECT_NEAR(drift.stability,      still.stability,      1e-4f);
	EXPECT_NEAR(drift.speed_headroom, still.speed_headroom, 1e-4f);
	EXPECT_NEAR(drift.turn_headroom,  still.turn_headroom,  1e-4f);

	// The pair above IS the frame discriminator: `still` and `drift` have
	// different GROUND speeds (cruise vs 3x cruise) and the same WATER speed, so
	// flipping AQUATIC to the GROUND frame reddens it. Everything below is a
	// different kind of check.
	//
	// J8 -- HONESTY NOTE about the case below: this third case is NOT a frame
	// discriminator, and it would not survive being treated as one. It changes
	// `desired_speed_m_s` (ground - current) from +2.535 to -5.07 as well as
	// changing the frame, so a ground-frame reading would ALSO produce a
	// different answer here and the assertion would still pass. What it is, and
	// all it is: a live-fire check that a creature swept BACKWARDS through the
	// water -- negative water speed, negative desired speed -- still settles to a
	// finite, materially different control output instead of degenerating. Keep
	// it for that; do not cite it as evidence about the frame.
	MyopicOutput upstream = settle(cruise, 3.f * cruise);
	std::cerr << "[frame aquatic] into a 3x-cruise current: turn="
	          << upstream.angular_velocity_rad_s.y
	          << " stability=" << upstream.stability << "\n";
	EXPECT_GT(std::fabs(upstream.stability - still.stability), 1e-3f)
		<< "the current has to be visible to a FIN";
}

// --- Presence: each arm is reached, and only when its section exists -------
TEST(MyopicEnvelope, RemainingModesAppearExactlyWhenTheirAnalysisDoes)
{
	struct Row { const char* file; Env env; };
	const Row rows[] = {
		{"batto.glb", Env::Air},   {"cat.glb", Env::Air},
		{"dragonfly.glb", Env::Air}, {"treefrog.glb", Env::Air},
		{"eel.glb", Env::Ocean},   {"penguin.glb", Env::Ocean},
		{"shark.glb", Env::Ocean}, {"penguin.glb", Env::Air},
	};

	int aquatic = 0, serpentine = 0, climbing = 0, brachiation = 0;
	for (const Row& r : rows) {
		const Output* out = Analyze(r.file, r.env);
		ASSERT_NE(out, nullptr) << r.file;

		const std::pair<LocomotionMode, bool> expected[] = {
			{LocomotionMode::AQUATIC,     out->aquatic.has_value()},
			{LocomotionMode::SERPENTINE,  out->serpentine.has_value()},
			{LocomotionMode::CLIMBING,    out->climbing.has_value()},
			{LocomotionMode::BRACHIATION, out->brachiation.has_value()},
		};
		for (auto [mode, has_section] : expected) {
			auto env = ExtractEnvelope(*out, mode, 0, 9.81f);
			EXPECT_EQ(env.has_value(), has_section)
				<< r.file << " mode=" << int(mode);
			if (!env.has_value()) continue;
			switch (mode) {
			case LocomotionMode::AQUATIC:     ++aquatic; break;
			case LocomotionMode::SERPENTINE:  ++serpentine; break;
			case LocomotionMode::CLIMBING:    ++climbing; break;
			case LocomotionMode::BRACHIATION: ++brachiation; break;
			default: break;
			}
			// Universal envelope sanity for every arm Task 7 adds.
			EXPECT_GT(float(env->max_speed), 0.f) << r.file << " mode=" << int(mode);
			EXPECT_GE(float(env->min_speed), 0.f) << r.file << " mode=" << int(mode);
			EXPECT_LT(float(env->min_speed), float(env->max_speed)) << r.file << " mode=" << int(mode);
			EXPECT_GT(float(env->max_accel), 0.f) << r.file << " mode=" << int(mode);
			EXPECT_TRUE(std::isfinite(float(env->max_accel))) << r.file << " mode=" << int(mode);
			EXPECT_GT(float(env->tau_linear), 0.f) << r.file << " mode=" << int(mode);
			EXPECT_TRUE(std::isfinite(float(env->tau_linear))) << r.file << " mode=" << int(mode);
			// Only aerial gets a load factor: buoyancy cancels weight for
			// swimmers and legs re-plant each stride for everything else.
			EXPECT_FALSE(env->aerial.has_value()) << r.file << " mode=" << int(mode);
		}
	}

	std::cerr << "[coverage] aquatic=" << aquatic << " serpentine=" << serpentine
	          << " climbing=" << climbing << " brachiation=" << brachiation << "\n";
	EXPECT_GT(aquatic, 0);
	EXPECT_GT(serpentine, 0);
	EXPECT_GT(climbing, 0);
	// brachiation is deliberately NOT asserted > 0: no sample model has arms
	// long enough (>= 0.8 body lengths, tonton_climbing.cpp:456) to produce an
	// Analysis_Brachiation, so that arm has no sample coverage at all. Raised as
	// a follow-up finding rather than papered over with a fake assertion.
	EXPECT_EQ(brachiation, 0)
		<< "a sample gained a brachiation section -- give that arm real coverage";
}

// --- The climbing envelope tracks the analysis's climb speed ---------------
TEST(MyopicEnvelope, ClimbingUsesTheStatedClimbSpeed)
{
	const Output* out = Analyze("cat.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->climbing.has_value());
	auto env = ExtractEnvelope(*out, LocomotionMode::CLIMBING, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	std::cerr << "[climbing cat] max_speed=" << float(env->max_speed)
	          << " max_accel=" << float(env->max_accel)
	          << " tau=" << float(env->tau_linear) << "\n";

	EXPECT_FLOAT_EQ(float(env->max_speed), float(out->climbing->max_climb_speed_m_s));
	EXPECT_FLOAT_EQ(float(env->min_speed), 0.f) << "a climber can hang still";
	// A climber stops by gripping, so braking is at least as strong as thrust.
	EXPECT_GE(float(env->max_brake), float(env->max_accel));
	// The climbing envelope is genuinely slower than the same cat's walk.
	auto walk = ExtractEnvelope(*out, LocomotionMode::TERRESTRIAL, 0, 9.81f);
	ASSERT_TRUE(walk.has_value());
	EXPECT_LT(float(env->max_speed), float(walk->max_speed));

	// --- PLACEHOLDER PIN (J3) ---------------------------------------------
	// As in SerpentineInvariants: the climbing max_accel is
	// `max_climb_speed / 1 second`, a placeholder for a stride rate
	// Analysis_Climbing does not expose, not a derivation. Unobserved, the
	// divisor could be changed to anything and the suite stayed green, silently
	// rescaling every stability and turn_headroom a climbing cat reports.
	// EXPECTED to fail, and to be updated deliberately, the day climbing gains
	// a real acceleration or stride frequency.
	const float climb = float(out->climbing->max_climb_speed_m_s);
	EXPECT_FLOAT_EQ(float(env->max_accel), climb)
		<< "placeholder: max_accel is max_climb_speed / 1 s";
	EXPECT_FLOAT_EQ(float(env->tau_linear), 1.f)
		<< "placeholder: tau is 1 s by construction, not by derivation";
}

// --- C4: the non-aerial turn strategy is not a claim about the substrate ---
//
// TurnStrategy's first enumerator used to be GROUND, and MyopicOutput defaults
// to it, so a swimming shark reported "GROUND" -- which a caller rolling a mesh
// or picking an animation would read as a statement about the seabed. The
// enumerator means "not a banked or yawed AERIAL turn"; LATERAL says that.
TEST(MyopicEntryPoint, NonAerialModesReportALateralTurnNotAGroundOne)
{
	struct Row { const char* file; Env env; LocomotionMode mode; };
	const Row rows[] = {
		{"shark.glb",  Env::Ocean, LocomotionMode::AQUATIC},
		{"eel.glb",    Env::Ocean, LocomotionMode::SERPENTINE},
		{"cat.glb",    Env::Air,   LocomotionMode::CLIMBING},
		{"cat.glb",    Env::Air,   LocomotionMode::TERRESTRIAL},
	};

	for (const Row& r : rows) {
		const Output* out = Analyze(r.file, r.env);
		ASSERT_NE(out, nullptr) << r.file;
		auto env = ExtractEnvelope(*out, r.mode, 0, 9.81f);
		ASSERT_TRUE(env.has_value()) << r.file << " mode=" << int(r.mode);
		ASSERT_FALSE(env->aerial.has_value()) << r.file << " mode=" << int(r.mode);

		MyopicState st{};
		MyopicInput in;
		in.mode            = r.mode;
		in.target_mode     = r.mode;
		in.target_position = TargetAtBearing(1.0f);
		in.velocity_m_s    = glm::vec3(0.f, 0.f, 0.5f * float(env->max_speed));
		in.dt_s            = 1.f / 60.f;
		MyopicOutput o = ComputeMyopicControl(*out, in, st);

		EXPECT_EQ(o.strategy, TurnStrategy::LATERAL) << r.file << " mode=" << int(r.mode);
		EXPECT_FLOAT_EQ(o.bank_angle_rad, 0.f) << r.file << " mode=" << int(r.mode);
	}
}

// ============================================================================
// Task 7 review round (J5-J7): gaps the reviewer's mutants walked through
// ============================================================================

// --- J6: an Envelope whose floor is at or above its ceiling is not usable ---
//
// `Envelope` carried no min_speed < max_speed invariant, and the aquatic arm can
// invert it through a SUPPORTED input. tonton_aquatic.cpp scales
// cruise_speed_m_s and burst_speed_m_s by exp2(mana.air) at the return statement
// but does NOT scale min_swim_speed_m_s, which was computed as cruise*0.5 before
// the scaling. So a negative mana.air shrinks the ceiling out from under a fixed
// floor and the envelope turns inside out -- at which point every headroom and
// stability term downstream is meaningless or sign-flipped.
//
// The gate lives in ExtractEnvelope and reports ABSENCE, the same way
// UsableAccel already handles a nonsensical envelope. It is applied to every arm
// rather than only this one: it is a statement about what an Envelope is, not
// about the aquatic derivation. (Verified: at the default inputs no arm's band
// is inverted on any sample, so it changes no current behaviour.)
//
// The unscaled min_swim_speed_m_s itself is an UPSTREAM inconsistency and is
// reported as a follow-up, not patched here.
TEST(MyopicEnvelope, AnInvertedSpeedBandIsReportedAsAbsence)
{
	// The shark in air is the sample that has a min_swim_speed floor at all
	// (requires_constant_motion; see SharkRequiresConstantMotion for why
	// seawater cannot reach that branch).
	AnalysisHolder base_holder;
	Input base_in = MakeDefaultInput(Env::Air);
	auto base = AnalyzeFresh("shark.glb", base_in, base_holder);
	ASSERT_TRUE(base);
	ASSERT_TRUE(base->aquatic.has_value());
	ASSERT_TRUE(base->aquatic->requires_constant_motion);
	ASSERT_LT(float(base->aquatic->min_swim_speed_m_s),
	          float(base->aquatic->burst_speed_m_s))
		<< "control: at mana.air = 0 the band is the right way round";
	ASSERT_TRUE(ExtractEnvelope(*base, LocomotionMode::AQUATIC, 0, 9.81f).has_value())
		<< "control: the same sample DOES yield an envelope at default mana, so a "
		   "nullopt below is about the band and not about the sample";

	// Now drive the supported mana axis down. exp2(-8) = 1/256 on the speeds,
	// nothing on the floor.
	AnalysisHolder bent_holder;
	Input bent_in = MakeDefaultInput(Env::Air);
	bent_in.mana.air = -8.f;
	auto bent = AnalyzeFresh("shark.glb", bent_in, bent_holder);
	ASSERT_TRUE(bent);
	ASSERT_TRUE(bent->aquatic.has_value());

	std::cerr << "[j6 shark air] mana.air=0: min=" 
	          << float(base->aquatic->min_swim_speed_m_s)
	          << " burst=" << float(base->aquatic->burst_speed_m_s)
	          << " | mana.air=-8: min="
	          << float(bent->aquatic->min_swim_speed_m_s)
	          << " burst=" << float(bent->aquatic->burst_speed_m_s) << "\n";

	// The upstream inconsistency, asserted so the finding is visible in the
	// suite: the floor did not move, the ceiling did, and they crossed.
	EXPECT_FLOAT_EQ(float(bent->aquatic->min_swim_speed_m_s),
	                float(base->aquatic->min_swim_speed_m_s))
		<< "min_swim_speed_m_s is not scaled by exp2(mana.air) -- if this ever "
		   "stops being true, the upstream inconsistency is fixed and this test "
		   "needs a different way to invert the band";
	ASSERT_GE(float(bent->aquatic->min_swim_speed_m_s),
	          float(bent->aquatic->burst_speed_m_s))
		<< "the band must actually be inverted for this test to test anything";

	// ...and the control layer refuses it rather than handing a caller an
	// envelope it cannot reason about.
	EXPECT_FALSE(ExtractEnvelope(*bent, LocomotionMode::AQUATIC, 0, 9.81f).has_value())
		<< "an inverted speed band is not a usable envelope";
}

// --- J7: the brachiation arm, and what could NOT be covered ----------------
//
// No sample model has arms >= 0.8 body lengths (tonton_climbing.cpp:456), so no
// Analysis_Brachiation is ever produced and the arm is verified by inspection
// only -- the reviewer's "BRACHIATION arm returns nullopt" mutant is invisible
// to the entire suite. The self-repealing EXPECT_EQ(brachiation, 0) tripwire in
// RemainingModesAppearExactlyWhenTheirAnalysisDoes stays; a gibbon-shaped sample
// is the real fix.
//
// WHAT WAS TRIED AND WHY IT DOES NOT WORK. The plan was to analyse a real model
// into an Output this test owns exclusively and install a synthetic brachiation
// section on it, so the SHIPPED ExtractEnvelope arm would run. Output's private
// constructor is not the blocker (Output::Factory is public). The blocker is
// TonTon::optional (include/tonton_optional.hpp): it is not std::optional but an
// ARENA-OFFSET type. It stores a uint32_t byte offset from `this` into a memory
// arena that Output::Factory allocates; copy and move assignment are both
// `= delete`, and the sole mutator, `load()`, throws unless the optional lives
// inside the arena span it is handed and would need the caller to place the
// object at a controlled address above `this`. There is no way to populate a
// section from outside Output::Factory without changing production code, and
// nothing was weakened to get there.
//
// WHAT IS COVERED INSTEAD. The shape the arm emits -- a = v*f (the arm alone
// among the three non-fluid ones needs no placeholder, because swing_frequency_Hz
// is a real exported quantity), the no-constraint sentinel radius, and no aerial
// authority -- is hand-built here and driven through the real Steer, exactly as
// the Envelope tests above do. This proves the emitted shape is safe downstream;
// it does NOT prove ExtractEnvelope emits it. That gap is reported, not papered
// over.
TEST(MyopicSteer, ABrachiatorsEnvelopeShapeIsSafeDownstream)
{
	// The arm's arithmetic, spelled out: a brachiator changes speed once per
	// swing. 3 m/s at 0.8 Hz.
	const float swing_speed = 3.0f;
	const float swing_freq  = 0.8f;

	Envelope e;
	e.max_speed         = velocity_m_s{swing_speed};
	e.min_speed         = velocity_m_s{0.f};          // can hang motionless
	e.max_accel         = acceleration_m_s2{swing_speed * swing_freq};
	e.max_brake         = e.max_accel;
	e.min_turn_radius   = length_m{0.f};              // no-constraint SENTINEL
	e.max_lateral_accel = e.max_accel;                // LateralBudget fall-through
	e.tau_linear        = e.max_speed / e.max_accel;
	// No AerialAuthority: a brachiator does not bank.
	ASSERT_FALSE(e.aerial.has_value());

	EXPECT_FLOAT_EQ(float(e.max_accel), 2.4f);
	EXPECT_FLOAT_EQ(float(e.tau_linear), 1.25f);

	// The sentinel must not produce an infinite lateral budget downstream, and
	// the whole envelope must steer finitely at rest, mid-band and at the
	// ceiling -- including the v = 0 case, where a v^2/r or v/r bound would be
	// degenerate.
	for (float v : {0.f, 0.5f * swing_speed, swing_speed}) {
		SteerState st{};
		SteerCommand cmd;
		cmd.angle_error_rad   = 1.2f;
		cmd.current_speed_m_s = v;
		cmd.desired_speed_m_s = swing_speed;
		cmd.dt_s              = 1.f / 60.f;
		SteerResult r;
		for (int i = 0; i < 300; ++i) r = Steer(e, st, cmd);

		EXPECT_TRUE(std::isfinite(r.turn_rate_rad_s)) << "v=" << v;
		EXPECT_TRUE(std::isfinite(r.accel_m_s2))      << "v=" << v;
		EXPECT_TRUE(std::isfinite(r.stability))       << "v=" << v;
		EXPECT_TRUE(std::isfinite(r.turn_headroom))   << "v=" << v;
		EXPECT_TRUE(std::isfinite(r.speed_headroom))  << "v=" << v;
		EXPECT_LE(std::fabs(r.accel_m_s2), float(e.max_accel) + 1e-4f) << "v=" << v;
		// A brachiator does not bank: no aerial authority means a LATERAL turn.
		EXPECT_EQ(r.strategy, TurnStrategy::LATERAL) << "v=" << v;
		EXPECT_FLOAT_EQ(r.bank_angle_rad, 0.f)       << "v=" << v;
	}
}
