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

	EXPECT_EQ(r.strategy, TurnStrategy::GROUND);
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
// a real maneuver at four timesteps rather than asserted.
TEST(MyopicBank, BankedTurnDoesNotOvershoot)
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
		const int steps = static_cast<int>(std::lround(12.f / dt));
		for (int i = 0; i < steps; ++i) {
			SteerCommand cmd = TurnCommand(error, 15.f);
			cmd.dt_s = dt;
			SteerResult r = Steer(env, state, cmd);
			const float prev = error;
			error -= r.turn_rate_rad_s * dt;
			if (prev * error < 0.f) ++flips;
			worst = std::min(worst, error);
		}
		return std::tuple<float, float, int>{error, worst, flips};
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
	for (float dt : {1.f / 16.f, 1.f / 30.f, 1.f / 60.f, 1.f / 120.f, 1.f / 480.f}) {
		auto [final_err, worst, flips] = run(dt);
		EXPECT_GE(worst, -0.06f) << "banked turn overshot too far at dt=" << dt;
		EXPECT_LE(flips, 1) << "banked turn oscillated at dt=" << dt;
		EXPECT_NEAR(final_err, 0.f, 1e-3f) << "did not converge at dt=" << dt;
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
