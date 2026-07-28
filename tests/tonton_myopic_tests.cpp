// ============================================================================
// TonTon myopic control tests
//
// Uses the same load/analyze harness as tonton_plausibility_tests.cpp (copied
// verbatim; it is static/anonymous-namespace there and cannot be shared).
// ============================================================================

#include <gtest/gtest.h>

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
