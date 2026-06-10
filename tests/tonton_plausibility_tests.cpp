// ============================================================================
// TonTon plausibility tests
//
// These tests load the real sample models through the SAME pipeline the
// `tonton-analyze` CLI uses (GetArmaturesFromFiles -> Builder::Factory ->
// Output::Factory) and assert that the computed biomechanical values are
// *vaguely plausible* for the species in question.
//
// Two kinds of checks:
//   * Invariants.*  - universal sanity (finite, positive mass/volume, ordered
//                     speeds, normalized [0,1] fields, ...). These should ALWAYS
//                     hold for any creature; a failure is a real bug.
//   * Species.*     - per-animal biological expectations (a cat is an endotherm
//                     mammal that walks; a shark swims and has no swim bladder;
//                     a bat flies; ...).
//
// Some assertions are expected to FAIL against the current engine -- that is the
// point: they pin down "plausible-looking but wrong" output. Each such check is
// tagged `// KNOWN BUG:` with a short explanation so that fixing the rule turns
// the test green.
//
// Environment matters: aquatic species are analyzed in seawater, everything
// else in air. (Buoyancy/sink-rate computed in air are physically meaningless
// for a fish.)
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

double f(float q) { return q; }
template <class Q> double f(const Q& q) { return static_cast<double>(static_cast<float>(q)); }

bool finite(double x) { return std::isfinite(x); }

// ===========================================================================
// INVARIANTS  (parameterized over every model)
// ===========================================================================
struct ModelCase {
    std::string label;
    std::string file;
    Env env;
};

class Invariants : public ::testing::TestWithParam<ModelCase> {};

TEST_P(Invariants, ModelLoadsAndIsSane)
{
    const auto& mc = GetParam();
    const Output* out = Analyze(mc.file, mc.env);
    ASSERT_NE(out, nullptr) << mc.label << ": failed to load/analyze " << mc.file;

    const auto& phys = out->physical;

    // --- Physical basics -----------------------------------------------------
    EXPECT_TRUE(finite(f(phys.body_mass_kg)));
    EXPECT_TRUE(finite(f(phys.body_volume_m3)));
    EXPECT_TRUE(finite(f(phys.body_length_m)));
    EXPECT_GT(f(phys.body_mass_kg), 0.0);
    EXPECT_GT(f(phys.body_volume_m3), 0.0);
    EXPECT_GT(f(phys.body_length_m), 0.0);
    EXPECT_GT(f(phys.surface_area_m2), 0.0);
    EXPECT_GT(f(phys.cross_sectional_area_m2), 0.0);
    EXPECT_GT(phys.fineness_ratio(), 0.0f);

    // Body density: no terrestrial/aquatic animal sits outside ~[200,1500] kg/m^3.
    double density = f(phys.body_mass_kg) / f(phys.body_volume_m3);
    EXPECT_GE(density, 200.0)  << mc.label << ": implausibly low body density " << density;
    EXPECT_LE(density, 1500.0) << mc.label << ": implausibly high body density " << density;

    // --- Metabolic -----------------------------------------------------------
    const auto& met = out->metabolic;
    EXPECT_GT(f(met.basal_rate_W), 0.0);
    EXPECT_GE(f(met.max_rate_W), f(met.basal_rate_W));
    EXPECT_GE(met.aerobic_scope(), 1.0f);
    EXPECT_GT(f(met.muscle_mass_kg), 0.0);
    EXPECT_LT(f(met.muscle_mass_kg), f(phys.body_mass_kg))
        << mc.label << ": muscle mass exceeds body mass";

    // --- Diagnostics ---------------------------------------------------------
    EXPECT_GE(out->diagnostics.overall_confidence, 0.0f);
    EXPECT_LE(out->diagnostics.overall_confidence, 1.0f);

    // --- Vision (always present) --------------------------------------------
    if (out->sensory.vision.has_value()) {
        const auto& v = *out->sensory.vision;
        EXPECT_GE(v.acuity, 0.0f);
        EXPECT_LE(v.acuity, 1.0f);
        // binocular_overlap is documented as 0..1 (0=none, 1=full). It is
        // currently stored in radians, so it falls outside [0,1] for nearly
        // every model.
        // KNOWN BUG (sensory): binocular_overlap stored in radians, not a 0..1 fraction.
        EXPECT_GE(f(v.binocular_overlap), 0.0)
            << mc.label << ": binocular_overlap " << f(v.binocular_overlap) << " < 0";
        EXPECT_LE(f(v.binocular_overlap), 1.0)
            << mc.label << ": binocular_overlap " << f(v.binocular_overlap) << " > 1 (radians bug)";
        EXPECT_GE(f(v.detection_range_m), 0.0);
        EXPECT_TRUE(finite(f(v.detection_range_m)));
    }

    // --- Hearing -------------------------------------------------------------
    if (out->sensory.hearing.has_value()) {
        const auto& h = *out->sensory.hearing;
        EXPECT_GE(f(h.frequency_range_Hz_min), 0.0);
        EXPECT_GT(f(h.frequency_range_Hz_max), f(h.frequency_range_Hz_min));
    }

    // --- Aquatic speeds ------------------------------------------------------
    if (out->aquatic.has_value()) {
        const auto& aq = *out->aquatic;
        EXPECT_TRUE(finite(f(aq.burst_speed_m_s)));
        EXPECT_GE(f(aq.burst_speed_m_s), f(aq.cruise_speed_m_s))
            << mc.label << ": burst < cruise";
        EXPECT_LT(f(aq.burst_speed_m_s), 50.0)
            << mc.label << ": burst swim speed " << f(aq.burst_speed_m_s) << " m/s is absurd";
        EXPECT_GE(f(aq.sink_rate_m_s), 0.0);
        EXPECT_LT(f(aq.sink_rate_m_s), 50.0)
            << mc.label << ": sink rate " << f(aq.sink_rate_m_s) << " m/s is absurd";
    }

    // --- Terrestrial speeds --------------------------------------------------
    if (out->terrestrial.has_value()) {
        const auto& t = *out->terrestrial;
        EXPECT_TRUE(finite(f(t.max_sprint_speed_m_s)));
        EXPECT_GE(f(t.max_sprint_speed_m_s), f(t.max_sustainable_speed_m_s))
            << mc.label << ": sprint < sustainable";
        EXPECT_GE(f(t.max_sustainable_speed_m_s), f(t.optimal_speed_m_s))
            << mc.label << ": sustainable < optimal";
        EXPECT_LT(f(t.max_sprint_speed_m_s), 50.0)
            << mc.label << ": land sprint " << f(t.max_sprint_speed_m_s) << " m/s is absurd";
        EXPECT_GE(t.posture, 0.0f);
        EXPECT_LE(t.posture, 1.0f);
    }

    // --- Aerial --------------------------------------------------------------
    if (out->aerial.has_value()) {
        const auto& a = *out->aerial;
        EXPECT_GT(f(a.wingbeat_frequency_Hz), 0.0);
        EXPECT_TRUE(finite(f(a.cruise_speed_m_s)));
        EXPECT_GT(a.wings.size(), 0u);
    }
}

INSTANTIATE_TEST_SUITE_P(
    SampleModels, Invariants,
    ::testing::Values(
        ModelCase{"dragonfly", "dragonfly.glb", Env::Air},
        ModelCase{"cat",       "cat.glb",       Env::Air},
        ModelCase{"shark",     "shark.glb",     Env::Ocean},
        ModelCase{"eel",       "eel.glb",       Env::Ocean},
        ModelCase{"penguin",   "penguin.glb",   Env::Ocean},
        ModelCase{"treefrog",  "treefrog.glb",  Env::Air},
        ModelCase{"bat",       "batto.glb",     Env::Air}),
    [](const ::testing::TestParamInfo<ModelCase>& i) { return i.param.label; });

// ===========================================================================
// SPECIES-SPECIFIC PLAUSIBILITY
// ===========================================================================

// --- Dragonfly: tiny ectothermic arthropod flier, 4 wings, can hover --------
TEST(Species, Dragonfly)
{
    const Output* out = Analyze("dragonfly.glb", Env::Air);
    ASSERT_NE(out, nullptr);

    EXPECT_GT(f(out->physical.body_mass_kg), 0.0002);
    EXPECT_LT(f(out->physical.body_mass_kg), 0.02);   // < 20 g

    EXPECT_FALSE(out->metabolic.is_endotherm());
    EXPECT_TRUE(HasFlag(out->physical.clade, CladeFlags::ARTHROPODA));

    ASSERT_TRUE(out->aerial.has_value()) << "dragonfly should fly";
    EXPECT_GE(out->aerial->wings.size(), 4u) << "dragonfly has 4 wings";
    EXPECT_TRUE(out->aerial->can_hover) << "dragonflies hover";
    // Empirical: dragonfly wingbeat ~20-30 Hz cruising, up to ~90 Hz at takeoff.
    EXPECT_GT(f(out->aerial->wingbeat_frequency_Hz), 15.0);
    EXPECT_LT(f(out->aerial->wingbeat_frequency_Hz), 90.0);
    // Empirical: top flight speed ~90 km/h (~25 m/s).
    EXPECT_LT(f(out->aerial->max_flight_speed_m_s), 30.0);
}

// --- Cat: small endothermic mammal, erect quadruped, terrestrial only -------
TEST(Species, Cat)
{
    const Output* out = Analyze("cat.glb", Env::Air);
    ASSERT_NE(out, nullptr);

    EXPECT_GT(f(out->physical.body_mass_kg), 1.0);
    EXPECT_LT(f(out->physical.body_mass_kg), 8.0);

    EXPECT_TRUE(out->metabolic.is_endotherm());
    EXPECT_GT(f(out->metabolic.body_temperature_K), 304.0); // ~31-40 C
    EXPECT_LT(f(out->metabolic.body_temperature_K), 315.0);
    EXPECT_TRUE(HasFlag(out->physical.clade, CladeFlags::MAMMALIA));

    ASSERT_TRUE(out->terrestrial.has_value()) << "cat walks";
    EXPECT_FALSE(out->aerial.has_value())  << "cats do not fly";
    EXPECT_FALSE(out->aquatic.has_value()) << "cat is not an aquatic animal";
    EXPECT_LT(out->terrestrial->posture, 0.4f) << "cat stands erect, not sprawling";
    // Empirical: domestic cat top speed ~48 km/h (~13 m/s).
    EXPECT_GT(f(out->terrestrial->max_sprint_speed_m_s), 8.0);
    EXPECT_LT(f(out->terrestrial->max_sprint_speed_m_s), 18.0);
}

// --- Great white shark: large ectothermic fish, BCF swimmer, NO swim bladder
TEST(Species, GreatWhiteShark)
{
    const Output* out = Analyze("shark.glb", Env::Ocean);
    ASSERT_NE(out, nullptr);

    EXPECT_GT(f(out->physical.body_mass_kg), 300.0);
    EXPECT_LT(f(out->physical.body_mass_kg), 1500.0);

    EXPECT_FALSE(out->metabolic.is_endotherm()); // regional endothermy aside
    EXPECT_FALSE(out->aerial.has_value());

    ASSERT_TRUE(out->aquatic.has_value()) << "shark swims";
    const auto& aq = *out->aquatic;
    EXPECT_EQ(aq.primary_mode, Analysis_Aquatic::PropulsionMode::BODY_CAUDAL_FIN);
    // Empirical: great white burst ~15.6 m/s (35 mph), cruise ~1.5 m/s.
    EXPECT_GT(f(aq.burst_speed_m_s), 5.0);
    EXPECT_LT(f(aq.burst_speed_m_s), 20.0);
    EXPECT_NEAR(f(aq.neutral_buoyancy_density_kg_m3), 1025.0, 200.0);

    // KNOWN BUG (aquatic): heterocercal-tail / swim-bladder heuristic gives the
    // shark a swim bladder. Sharks (cartilaginous fish) have none -- they rely on
    // an oily liver + dynamic lift, hence "must keep moving".
    EXPECT_FALSE(aq.has_swim_bladder) << "sharks have no swim bladder";
}

// --- Eel: very elongate ectothermic fish, undulatory, no swim bladder --------
TEST(Species, Eel)
{
    const Output* out = Analyze("eel.glb", Env::Ocean);
    ASSERT_NE(out, nullptr);

    EXPECT_FALSE(out->metabolic.is_endotherm());
    EXPECT_GT(out->physical.fineness_ratio(), 8.0f) << "eel is very elongate";

    ASSERT_TRUE(out->aquatic.has_value()) << "eel swims";
    EXPECT_LT(f(out->aquatic->burst_speed_m_s), 30.0);
    EXPECT_FALSE(out->aquatic->has_swim_bladder);
}

// --- Penguin: endothermic bird; swims by underwater "flight" (wings),
//     cannot do powered aerial flight. -------------------------------------
TEST(Species, PenguinSwims)
{
    const Output* out = Analyze("penguin.glb", Env::Ocean);
    ASSERT_NE(out, nullptr);

    EXPECT_GT(f(out->physical.body_mass_kg), 15.0);
    EXPECT_LT(f(out->physical.body_mass_kg), 60.0);
    EXPECT_TRUE(out->metabolic.is_endotherm());
    EXPECT_TRUE(HasFlag(out->physical.clade, CladeFlags::AVES));

    ASSERT_TRUE(out->aquatic.has_value()) << "penguin swims";
    const auto& aq = *out->aquatic;
    // Empirical: emperor penguin swims ~2-3 m/s typical, ~4 m/s max (they do not
    // porpoise). A burst much above this is too fast.
    EXPECT_GT(f(aq.cruise_speed_m_s), 1.0);
    EXPECT_LT(f(aq.burst_speed_m_s), 10.0);

    // Penguins "fly" underwater: thrust comes from the pectoral wings (lift-based),
    // not from leg paddling or body/tail undulation. The wing-propelled diving
    // mode (MEDIAN_PAIRED_FIN) is the biologically correct classification.
    // KNOWN GAP (aquatic): wing-propelled divers are classified PADDLE_LIMBS;
    // there is no lift-based "underwater flight" propulsion mode.
    EXPECT_EQ(aq.primary_mode, Analysis_Aquatic::PropulsionMode::MEDIAN_PAIRED_FIN)
        << "penguins fly underwater with their wings (not paddle/BCF)";
}

TEST(Species, PenguinCannotFlyInAir)
{
    const Output* out = Analyze("penguin.glb", Env::Air);
    ASSERT_NE(out, nullptr);
    if (out->aerial.has_value()) {
        EXPECT_FALSE(out->aerial->can_sustain_level_flight)
            << "penguins cannot perform powered aerial flight";
    }
}

// --- Treefrog: small ectothermic amphibian, climbs (toe pads) and jumps ------
TEST(Species, Treefrog)
{
    const Output* out = Analyze("treefrog.glb", Env::Air);
    ASSERT_NE(out, nullptr);

    EXPECT_GT(f(out->physical.body_mass_kg), 0.005);
    EXPECT_LT(f(out->physical.body_mass_kg), 0.2);

    // KNOWN BUG (wordlist/clade): the treefrog is classified MAMMALIA and modeled
    // as an endotherm. Frogs are ectothermic amphibians.
    EXPECT_FALSE(HasFlag(out->physical.clade, CladeFlags::MAMMALIA))
        << "a frog is not a mammal";
    EXPECT_FALSE(out->metabolic.is_endotherm())
        << "frogs are ectotherms";

    EXPECT_TRUE(out->climbing.has_value()) << "treefrogs climb with adhesive toe pads";

    // Misclassification leaks into hearing: 90 kHz ceiling is a small-mammal
    // value; frogs hear to roughly 5-8 kHz.
    if (out->sensory.hearing.has_value()) {
        // KNOWN BUG: hearing ceiling driven by mammal misclassification.
        EXPECT_LT(f(out->sensory.hearing->frequency_range_Hz_max), 20000.0)
            << "frog hearing should not reach ultrasonic range";
    }
}

// --- Bat: small endothermic mammal, the archetypal powered flier -------------
TEST(Species, Bat)
{
    const Output* out = Analyze("batto.glb", Env::Air);
    ASSERT_NE(out, nullptr);

    // The heaviest real bats (Indian/large flying fox) top out near ~1.6 kg.
    // This model reports ~1.9 kg, i.e. it may be over-scaled -- so the mass bound
    // is intentionally loose. The structural checks below are scale-independent
    // and are the real findings.
    EXPECT_GT(f(out->physical.body_mass_kg), 0.005);
    EXPECT_LT(f(out->physical.body_mass_kg), 2.5);

    // KNOWN BUG (clade/wing detection): the bat is detected as a generic
    // CHORDATA ectotherm with no wings, so it gets no flight analysis at all.
    EXPECT_TRUE(out->metabolic.is_endotherm())
        << "bats are endothermic mammals";
    EXPECT_TRUE(HasFlag(out->physical.clade, CladeFlags::MAMMALIA))
        << "a bat is a mammal";
    EXPECT_TRUE(out->aerial.has_value())
        << "the defining trait of a bat is powered flight";

    // Bats do roost/climb head-down -- this part the engine gets right.
    EXPECT_TRUE(out->climbing.has_value());
}

// --- Physics: a jump cannot release more kinetic energy than the muscles can do.
// Use the cat: a confirmed terrestrial jumper that already passes its species test.
// (The treefrog is the natural choice biologically, but it is currently
//  clade-misclassified -- a separate, out-of-scope bug -- so the cat is the
//  reliable land jumper here.)
TEST(Physics, JumpEnergyConserved) {
    const Output* op = Analyze("cat.glb", Env::Air);
    ASSERT_NE(op, nullptr);
    const Output& o = *op;
    ASSERT_TRUE(o.jumping.has_value());
    const auto& j = *o.jumping;
    float m = float(o.physical.body_mass_kg);
    float v = float(j.takeoff_velocity_m_s);
    float ke = 0.5f * m * v * v;
    // Muscle work available for one jump: muscle mass x work density (~250 J/kg upper bound).
    float muscle_mass = float(o.metabolic.muscle_mass_kg);
    float max_work = muscle_mass * 250.0f;
    EXPECT_LE(ke, max_work) << "takeoff KE exceeds muscle work -- energy fabricated";
}

// Dig speed must now be a physical, bounded quantity. Previously the formula
// cancelled cross-section and collapsed to a morphology-independent constant
// (stroke_len*freq). It is now derived from P = F*v against soil penetration
// resistance, then clamped to the empirical fossorial range (<= 0.01 m/s).
// We loop every sample model; for any that produces a digging output we assert
// the speed is finite, strictly positive, and below a generous 1.0 m/s ceiling.
TEST(Physics, DigSpeedIsPhysicalAndBounded) {
    const ModelCase models[] = {
        {"dragonfly", "dragonfly.glb", Env::Air},
        {"cat",       "cat.glb",       Env::Air},
        {"shark",     "shark.glb",     Env::Ocean},
        {"eel",       "eel.glb",       Env::Ocean},
        {"penguin",   "penguin.glb",   Env::Ocean},
        {"treefrog",  "treefrog.glb",  Env::Air},
        {"bat",       "batto.glb",     Env::Air},
    };

    int diggers = 0;
    for (const auto& mc : models) {
        const Output* out = Analyze(mc.file, mc.env);
        ASSERT_NE(out, nullptr) << mc.label << ": failed to load/analyze " << mc.file;

        if (!out->specialized.digging.has_value())
            continue;

        ++diggers;
        const auto& dig = *out->specialized.digging;
        double v = f(dig.max_dig_speed_m_s);
        EXPECT_TRUE(finite(v)) << mc.label << ": dig speed not finite";
        EXPECT_GT(v, 0.0)      << mc.label << ": dig speed not positive (" << v << ")";
        EXPECT_LT(v, 1.0)      << mc.label << ": dig speed implausibly high (" << v << " m/s)";
    }

    if (diggers == 0) {
        // Coverage gap: none of the 7 sample models are fossorial, so the
        // model-driven path exercises no digging output. The P=F*v formula is
        // still covered by the library build; add a fossorial sample model to
        // close this gap.
        GTEST_SKIP() << "no sample model produces a digging output";
    }
}

} // namespace
