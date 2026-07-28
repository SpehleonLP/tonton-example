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

TEST(MyopicEnvelope, AerialInvariants)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	ASSERT_TRUE(out->aerial.has_value()) << "bat should fly";

	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	EXPECT_GT(float(env->max_speed), 0.f);
	EXPECT_GT(float(env->min_speed), 0.f) << "a flyer has a stall speed";
	EXPECT_LT(float(env->min_speed), float(env->max_speed));

	// The derived-from-power acceleration must be real, not zero or NaN.
	EXPECT_GT(float(env->max_accel), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->max_accel)));
	EXPECT_GT(float(env->tau_linear), 0.f);
	EXPECT_TRUE(std::isfinite(float(env->tau_linear)));

	ASSERT_TRUE(env->aerial.has_value());
	EXPECT_GE(env->aerial->n_max, 1.f) << "load factor cannot be below 1";
	EXPECT_TRUE(std::isfinite(env->aerial->n_max));
	EXPECT_GT(float(env->aerial->max_roll_rate), 0.f);

	// n_max == 1 exactly means g*sqrt(n^2-1) == 0: an animal that flies but
	// cannot turn. That is never a physical answer, only a symptom of a bad
	// load-factor input being caught by the n>=1 floor. Assert it here so the
	// clamp can never silently hide a non-physical intermediate again.
	EXPECT_GT(env->aerial->n_max, 1.f) << "a flyer whose load factor floors at 1 cannot bank";
	EXPECT_GT(float(env->max_lateral_accel), 0.f) << "a flyer must be able to turn";
	EXPECT_TRUE(std::isfinite(float(env->max_lateral_accel)));
}

TEST(MyopicEnvelope, AerialAccelerationIsPlausible)
{
	const Output* out = Analyze("batto.glb", Env::Air);
	ASSERT_NE(out, nullptr);
	auto env = ExtractEnvelope(*out, LocomotionMode::AERIAL, 0, 9.81f);
	ASSERT_TRUE(env.has_value());

	// A flying animal accelerates within roughly an order of magnitude of g.
	// Far outside this band means the P = F*v derivation has a units error.
	EXPECT_GT(float(env->max_accel), 0.1f);
	EXPECT_LT(float(env->max_accel), 100.f);

	// Banked turns of biological flyers sit around 1.5-4 g. Anything past ~10 g
	// would mean the load-factor derivation has lost its footing.
	ASSERT_TRUE(env->aerial.has_value());
	EXPECT_LT(env->aerial->n_max, 10.f) << "n_max = " << env->aerial->n_max;
}
