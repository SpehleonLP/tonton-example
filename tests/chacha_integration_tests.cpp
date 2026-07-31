// Integration tests exercising the full ChaCha pipeline (extraction ->
// analyze() -> bridge output) against real glTF models from the ChaCha
// test corpus, plus a couple of synthetic cases the real corpus does not
// exercise. Lives in tonton-example (not the ChaCha library) because it
// needs fx-gltf to load the .glb files and the bridge to write AGI output;
// ChaCha itself must stay parser-free.

#include <gtest/gtest.h>
#include "fx/gltf.h"
#include "chacha.h"
#include "chacha_fxgltf_bridge.h"
#include <nlohmann/json.hpp>
#include <glm/gtc/quaternion.hpp>
#include <chrono>
#include <filesystem>
#include <map>
#include <set>
#include <cmath>
#include <cstdio>

namespace {

fx::gltf::Document load(const char* name)
{
    std::filesystem::path p = std::filesystem::path(CHACHA_TESTDATA_DIR) / name;
    // sophia-2_9.glb is 41MB; fx::gltf's default quotas cap file/buffer size
    // at 32MB, so without raising them here sophia fails to load with a
    // quota-exceeded exception (chacha_main.cpp raises the same quotas for
    // the same reason -- see its OpenFile()).
    fx::gltf::ReadQuotas quotas;
    quotas.MaxFileSize = 512u * 1024u * 1024u;
    quotas.MaxBufferByteLength = 512u * 1024u * 1024u;
    return fx::gltf::LoadFromBinary(p, quotas);
}

struct Analysis {
    ChaChaFxGltf::ExtractedAnimations anims;
    ChaChaFxGltf::ExtractedSkeleton   skel;
    std::vector<ChaCha::Articulation> articulations;
};

Analysis run(const fx::gltf::Document& doc)
{
    Analysis a;
    a.anims = ChaChaFxGltf::extract_animation_channels(doc);
    a.skel  = ChaChaFxGltf::extract_skeleton(doc);
    a.articulations = ChaCha::analyze(
        a.anims.channels, a.anims.animations, a.skel.as_skeleton());
    return a;
}

const ChaCha::Articulation* find_named(
    const fx::gltf::Document& doc,
    const std::vector<ChaCha::Articulation>& arts,
    const std::string& node_name)
{
    for (const auto& a : arts)
        if (a.node >= 0 && a.node < static_cast<int>(doc.nodes.size())
            && doc.nodes[a.node].name == node_name)
            return &a;
    return nullptr;
}

const ChaCha::Articulation* find_by_node(
    const std::vector<ChaCha::Articulation>& arts, int node)
{
    for (const auto& a : arts)
        if (a.node == node) return &a;
    return nullptr;
}

} // namespace

// ---------------------------------------------------------------------------
// Treefrog: an artist-authored "AGI Configuration" animation is ground
// truth. It declares the stage set, the order, and the range for these two
// joints directly, so the pipeline must reproduce it faithfully rather than
// approximate it.
//
// Measured (see task-15 report): Head and Spine.003 each emit exactly 3
// stages, in slot order xRotate, zRotate, yRotate, at +-15.0 degrees
// (observed range: 14.999994 to 15.000007 degrees). This supersedes the
// original brief, which asserted +-13.4 deg with a +-2.0 deg tolerance --
// a bound that happened to pass only 0.4 degrees away from failing. A tight
// tolerance here means a future regression that nudges the recovered range
// gets caught instead of absorbed into slack.
// ---------------------------------------------------------------------------
TEST(ChaChaIntegration, TreefrogConfigurationIsHonoured)
{
    auto doc = load("treefrog.glb");
    auto a   = run(doc);

    for (const char* joint : {"Head", "Spine.003"}) {
        const auto* art = find_named(doc, a.articulations, joint);
        ASSERT_NE(art, nullptr) << joint;
        ASSERT_EQ(art->stages.size(), 3u) << joint;
        EXPECT_EQ(art->stages[0].type, ChaCha::StageType::xRotate) << joint;
        EXPECT_EQ(art->stages[1].type, ChaCha::StageType::zRotate) << joint;
        EXPECT_EQ(art->stages[2].type, ChaCha::StageType::yRotate) << joint;
        for (const auto& s : art->stages) {
            EXPECT_NEAR(glm::degrees(s.min_value), -15.0f, 0.1f) << joint;
            EXPECT_NEAR(glm::degrees(s.max_value),  15.0f, 0.1f) << joint;
        }
    }
}

// ---------------------------------------------------------------------------
// Treefrog: the property actually worth pinning about "resting" joints is
// not a coarse population fraction (measured: the configuration animation
// drives 82 of 84 nodes with translation+rotation+scale channels, and 56 of
// those clear the noise threshold -- so a "less than half the nodes" bound
// is false, not just loose) but the underlying causal fact: a node with no
// motion in its scanned animation gets no articulation at all.
// ---------------------------------------------------------------------------
TEST(ChaChaIntegration, TreefrogRestingJointsAreLocked)
{
    auto doc = load("treefrog.glb");
    auto a   = run(doc);

    std::set<int> animated_nodes;
    for (const auto& ch : a.anims.channels) animated_nodes.insert(ch.node);

    // Nodes with zero animation channels of any kind must never get an
    // articulation -- there is nothing to scan, so the pipeline has no
    // basis for emitting one. This is a hard invariant, not a fraction.
    int unanimated_checked = 0;
    for (int node = 0; node < static_cast<int>(doc.nodes.size()); ++node) {
        if (animated_nodes.count(node)) continue;
        EXPECT_EQ(find_by_node(a.articulations, node), nullptr)
            << "node " << node << " (" << doc.nodes[node].name
            << ") has no animation channels at all and must not get an articulation";
        ++unanimated_checked;
    }
    // Measured: 84 nodes total, 82 animated -- so exactly 2 unanimated nodes
    // exist to exercise the invariant above. Pinned exactly (not just ">0")
    // so a corpus-file change that adds channels to one of them is visible
    // rather than silently shrinking the population this test covers.
    EXPECT_EQ(unanimated_checked, 2)
        << "expected exactly 2 unanimated nodes in treefrog.glb to exercise "
           "the locked-joint invariant; corpus file may have changed";

    // The count itself: not a fraction of total nodes (that bound is false
    // per the measurement above) but the actual number of animated nodes
    // whose motion clears the noise threshold. 56 is the measured baseline
    // for this exact corpus file with default Options.
    //
    // Honest caveat (found in review): this is the weakest regression guard
    // in this suite. It survived a review mutation that raised
    // rotation_threshold_rad 30x, because treefrog's translation and scale
    // stages keep most of these 56 articulations' node populated regardless
    // of what happens to the rotation threshold specifically -- so this
    // number is dominated by translation/scale motion, not a precise probe
    // of rotation-threshold behavior. Treat a break here as "something in
    // extraction, node indexing, or the overall threshold pipeline changed,
    // go look" rather than "the rotation threshold changed." Deriving 56 any
    // more structurally would mean reimplementing the per-property
    // peak-to-peak threshold comparison from chacha_analyzer.cpp here,
    // which would just make this test fail in lockstep with the same bug
    // instead of catching it.
    EXPECT_LE(a.articulations.size(), animated_nodes.size())
        << "cannot emit more articulations than nodes with any animation channel";
    EXPECT_EQ(a.articulations.size(), 56u)
        << "measured baseline for treefrog.glb with default Options; if this "
           "changes intentionally (threshold tuning, new stage types, etc.) "
           "update the constant, but a silent drift here is worth investigating";
}

// ---------------------------------------------------------------------------
// Ruling 3: pointing_vector is not a per-joint anatomical value to assert
// per-joint. For THIS document, infer_pointing_vectors settles on a single
// vector shared by every articulation -- that's the outcome this test pins.
//
// It is not a general design invariant, and the test does not claim one:
// chacha_pointing.cpp has per-joint fallback loops (see lines ~125-164) for
// when no axis clears the dominant-axis fraction, so a different model
// could legitimately produce divergent per-joint pointing vectors without
// that being a bug. Treefrog happens to converge on a single dominant axis
// for all 56 articulations, so uniformity is what a correct run of THIS
// document should produce; if a future change accidentally made the vote
// per-joint when it should still be uniform here (e.g. reusing a per-node
// local axis instead of the model-wide vote), this test would catch the
// first divergent pair.
// ---------------------------------------------------------------------------
TEST(ChaChaIntegration, PointingVectorIsAModelWideVoteNotPerJoint)
{
    auto doc = load("treefrog.glb");
    auto a   = run(doc);
    ASSERT_FALSE(a.articulations.empty());

    const glm::vec3 first = a.articulations.front().pointing_vector;
    for (const auto& art : a.articulations)
        EXPECT_EQ(art.pointing_vector, first)
            << "pointing_vector must be the single model-wide vote, not a "
               "per-joint value -- node " << art.node << " diverged";
}

// ---------------------------------------------------------------------------
// Scorpion is also where the reduced-DOF subsystem (the closed-form 1-DOF
// solve, the Gauss-Newton 2-DOF solve, and the residual acceptance gate in
// chacha_reduced.cpp/chacha_dp.cpp) gets exercised end-to-end. Before this
// fix round, nothing in this suite asserted anything about DOF counts on
// any model except sophia's trivial "at least one non-3-DOF-or-not" check
// -- and sophia has zero reduced-DOF joints (see SophiaAnalysis), so the
// entire reduced-DOF path could be deleted from the library and this suite
// would stay green. Pinning scorpion's histogram closes that hole: it has
// a real, measured mix of all three DOF counts.
// ---------------------------------------------------------------------------
TEST(ChaChaIntegration, ScorpionAnalysis)
{
    auto doc = load("emporer scorpion.glb");
    auto a   = run(doc);
    ASSERT_FALSE(a.articulations.empty());
    // Measured baseline (see task-15 report): 45.
    EXPECT_EQ(a.articulations.size(), 45u);

    int hist[4] = {0, 0, 0, 0};
    for (const auto& art : a.articulations)
        if (art.dof_count < 4) hist[art.dof_count]++;
    std::printf("[scorpion] dof histogram 0/1/2/3 = %d/%d/%d/%d\n",
                hist[0], hist[1], hist[2], hist[3]);
    // Measured baseline (see task-15 report): 11 one-DOF, 16 two-DOF,
    // 18 three-DOF. Correction: this does NOT exercise select_candidate's
    // reduced-DOF search or its residual acceptance gate (chacha_reduced.cpp
    // / chacha_search.cpp). Scorpion's motion comes from its "AGI
    // Configuration" / "AGI Configuration.001" animation pair, so every one
    // of these 45 articulations -- including the 11 one-DOF and 16 two-DOF
    // results -- is produced by solve_configuration (chacha_config.cpp),
    // which reads dof_count directly off how many distinct axes the artist's
    // authored phases touched. That path never calls select_candidate. This
    // histogram is still useful evidence that reduced-DOF *output* (dof_count
    // < 3) is common and correctly reported, but it says nothing about the
    // search/candidate-solve/residual-gate path in chacha_search.cpp and
    // chacha_reduced.cpp, which has no real-data coverage in this suite (see
    // SophiaAnalysis's dof histogram comment) and is exercised only by
    // ChaCha's own synthetic unit tests.
    EXPECT_EQ(hist[0], 0);
    EXPECT_EQ(hist[1], 11);
    EXPECT_EQ(hist[2], 16);
    EXPECT_EQ(hist[3], 18);
}

// Sophia (mixamo rig, 41MB) takes roughly 17.7s per analyze() call in a
// Release build. The brief's original layout ran three separate TESTs each
// re-loading and re-analyzing sophia from scratch (~53s just for sophia,
// dominating the whole suite's runtime for no added coverage -- the three
// checks are independent assertions over the SAME analyze() result, not
// independent scenarios). Folded into one TEST that analyzes once and
// performs all three checks below, cutting the suite's wall time from ~56s
// to ~19s. Each check is still its own SCOPED_TRACE'd block so a failure
// still names which property broke.
//
// 1) The objective correctness measure: applying the emitted stages at
//    their solved angles must reproduce the original keyframes, within a
//    bound tight enough to catch a dropped stage (fix round, Finding 1).
// 2) Constant unit scale channels must not produce scale stages.
// 3) The DOF histogram is pinned exactly at 0/0/0/40 (fix round, Finding 2):
//    sophia's mocap motion is genuinely all-3-DOF (confirmed in review --
//    not a solver defect; see the histogram check's own comment), so this
//    pins that specific, correct outcome rather than asserting something
//    trivially true of any histogram. It does NOT exercise the reduced-DOF
//    (1-DOF/2-DOF) search path -- that's pinned on scorpion instead, see
//    ScorpionAnalysis, since sophia has no reduced-DOF joints to exercise it
//    with.
TEST(ChaChaIntegration, SophiaAnalysis)
{
    auto doc = load("sophia-2_9.glb");
    auto a   = run(doc);
    ASSERT_FALSE(a.articulations.empty());
    // Measured baseline (see task-15 report): 40.
    EXPECT_EQ(a.articulations.size(), 40u);

    {
    SCOPED_TRACE("round trip");
    std::map<int, const ChaCha::Articulation*> by_node;
    for (const auto& art : a.articulations) by_node[art.node] = &art;

    auto axis_of = [](ChaCha::StageType t) -> int {
        switch (t) {
        case ChaCha::StageType::xRotate: return 0;
        case ChaCha::StageType::yRotate: return 1;
        case ChaCha::StageType::zRotate: return 2;
        default: return -1;
        }
    };

    int    sampled = 0;
    double worst   = 0.0;

    for (const auto& ch : a.anims.channels) {
        if (ch.property != ChaCha::Property::Rotation) continue;
        auto it = by_node.find(ch.node);
        if (it == by_node.end()) continue;

        // Project the observed rest-relative rotation onto the emitted
        // stage axes and measure what's left over. This bounds gross
        // decomposition failure (see the EXPECT_LT below for how tight);
        // it is NOT a general "missing axis" detector by itself -- a
        // dropped stage still leaves the others to soak up some of the
        // rotation, so the residual only blows up past a certain bound.
        // The bound is chosen tight enough (1.5 rad) to actually catch a
        // dropped-stage mutation; see the comment on that EXPECT_LT.
        const glm::quat rest_inv =
            glm::inverse(glm::normalize(a.skel.rest_rotations[ch.node]));

        const int n = static_cast<int>(ch.times.size());
        for (int i = 0; i < n; i += 7) {
            const float* v = ch.values.data() + static_cast<size_t>(i) * 4;
            glm::quat key(v[3], v[0], v[1], v[2]);
            if (glm::dot(key, key) < 1e-8f) continue;
            glm::quat rel = rest_inv * glm::normalize(key);

            // Project onto the emitted axes and measure what is left over.
            glm::quat residual = rel;
            for (const auto& s : it->second->stages) {
                const int ax = axis_of(s.type);
                if (ax < 0) continue;
                glm::vec3 axis(0.0f); axis[ax] = 1.0f;
                const float qv[3] = {residual.x, residual.y, residual.z};
                const float angle = 2.0f * std::atan2(qv[ax], residual.w);
                residual = glm::inverse(glm::angleAxis(angle, axis)) * residual;
            }
            float d = std::fabs(residual.w);
            if (d > 1.0f) d = 1.0f;
            worst = std::max(worst, static_cast<double>(2.0f * std::acos(d)));
            ++sampled;
        }
    }

    EXPECT_GT(sampled, 1000);
    std::printf("[sophia] sampled=%d worst residual=%.2f deg\n",
                sampled, worst * 57.29577951);
    // Measured worst residual on the real pipeline is ~1.19 rad (68.12 deg;
    // mixamo rigs have genuine off-axis motion at several joints, so this is
    // not near zero). 3.2 rad (the brief's original bound, essentially "not
    // pi") is not discriminating: a mutation that drops 2 of the 3 stages
    // from every articulation on every model still lands at ~3.1 rad and
    // passes. 1.5 rad leaves real margin above the measured 1.19 rad while
    // catching that mutation (verified: see task-15 report, fix round).
    EXPECT_LT(worst, 1.5);
    }

    {
    SCOPED_TRACE("constant scale channels produce no scale stages");
    for (const auto& art : a.articulations)
        for (const auto& s : art.stages)
            EXPECT_FALSE(s.type == ChaCha::StageType::xScale
                      || s.type == ChaCha::StageType::yScale
                      || s.type == ChaCha::StageType::zScale)
                << "constant unit scale should filter out";
    }

    {
    SCOPED_TRACE("dof histogram");
    int hist[4] = {0, 0, 0, 0};
    float worst_conditioning_proxy = 0.0f;
    for (const auto& art : a.articulations) {
        if (art.dof_count < 4) hist[art.dof_count]++;
        worst_conditioning_proxy = std::max(worst_conditioning_proxy, art.fit_residual_rad);
    }
    std::printf("[sophia] dof histogram 0/1/2/3 = %d/%d/%d/%d, worst residual %.3f rad\n",
                hist[0], hist[1], hist[2], hist[3], worst_conditioning_proxy);
    // Sophia is genuinely all-3-DOF (measured and confirmed in review): this
    // mixamo mocap rig carries real off-axis motion at every joint, not a
    // solver defect -- the best 1-DOF residual anywhere in the rig (left
    // elbow) is 0.741 rad against a 0.02 rad acceptance gate, nowhere close
    // to passing. `EXPECT_GT(hist[1]+hist[2]+hist[3], 0)` would be trivially
    // satisfied by hist[3] alone and catch nothing; pin the exact histogram
    // instead so a regression that misclassifies DOF (in either direction)
    // is visible here. The reduced-DOF path itself (1-DOF/2-DOF candidates)
    // is exercised and pinned on scorpion instead, see ScorpionAnalysis --
    // sophia has no reduced-DOF joints to pin.
    EXPECT_EQ(hist[0], 0);
    EXPECT_EQ(hist[1], 0);
    EXPECT_EQ(hist[2], 0);
    EXPECT_EQ(hist[3], 40);
    }
}

// ---------------------------------------------------------------------------
// Synthetic: repeated StageType through the bridge's naming path.
//
// Six of the twelve rotation charts are proper-Euler with a repeated axis
// (first == third), so roughly a third of general three-DOF joints emit the
// same StageType twice (e.g. zRotate in slots 0 and 2). No articulation in
// any of the three real corpus models hits this path -- ChaCha's own unit
// tests (Analyze.ProperEulerRepeatedStageSurvivesInSlotOrder) pin that
// analyze() itself never dedupes the repeated stage, but the bridge's own
// occurrence-counting logic (chacha_fxgltf_bridge.cpp, write_agi_articulations)
// that turns "same StageType twice" into unique AGI stage names ("zRotate",
// "zRotate2") is a SEPARATE piece of code with its own array indexed by
// StageType, and nothing anywhere -- corpus or unit test -- exercises it at
// occurrence > 0. A bug there (e.g. keying stages_json by StageType and
// overwriting the first occurrence, or mislabeling both as "zRotate") would
// pass every other test in this suite and in ChaCha's own suite.
//
// This builds a joint whose rest-relative rotation is a genuine Z-X-Z
// proper-Euler motion (large range on the repeated Z axis, moderate range
// on the middle X axis, small range on the second Z occurrence -- the same
// well-conditioned shape ChaCha's own analyzer test uses, chosen so the
// chart search reliably picks the Z-X-Z chart over a Tait-Bryan alternative)
// and checks the AGI JSON the bridge actually emits.
// ---------------------------------------------------------------------------
TEST(ChaChaIntegration, RepeatedStageTypeGetsUniqueNamesInSlotOrder)
{
    std::vector<int>       parents{-1};
    std::vector<glm::quat> rest{glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    std::vector<glm::vec3> trans{glm::vec3(0.0f)};
    std::vector<glm::vec3> scale{glm::vec3(1.0f)};
    ChaCha::Skeleton skeleton{parents, rest, trans, scale};

    std::vector<float> times;
    std::vector<float> values;
    const int n = 40;
    for (int i = 0; i <= n; ++i) {
        const float f = static_cast<float>(i);
        const float t1 = glm::radians(f * 1.0f);        // Z, 1st occurrence: 0..40 deg
        const float t2 = glm::radians(f * 0.7f + 5.0f);  // X, middle: 5..33 deg
        const float t3 = glm::radians(f * 0.15f);        // Z, 2nd occurrence: 0..6 deg
        glm::quat q = glm::angleAxis(t1, glm::vec3(0, 0, 1))
                    * glm::angleAxis(t2, glm::vec3(1, 0, 0))
                    * glm::angleAxis(t3, glm::vec3(0, 0, 1));
        values.insert(values.end(), {q.x, q.y, q.z, q.w});
        times.push_back(f / 30.0f);
    }

    ChaCha::AnimationChannel ch;
    ch.node = 0; ch.animation = 0; ch.property = ChaCha::Property::Rotation;
    ch.times = times; ch.values = values;

    std::vector<ChaCha::AnimationChannel> chans{ch};
    std::vector<ChaCha::Animation> anims{ChaCha::Animation{"rom"}};

    auto articulations = ChaCha::analyze(chans, anims, skeleton);
    ASSERT_EQ(articulations.size(), 1u);
    ASSERT_EQ(articulations[0].stages.size(), 3u);
    ASSERT_EQ(articulations[0].stages[0].type, ChaCha::StageType::zRotate);
    ASSERT_EQ(articulations[0].stages[1].type, ChaCha::StageType::xRotate);
    ASSERT_EQ(articulations[0].stages[2].type, ChaCha::StageType::zRotate)
        << "test setup assumption broken: chart search did not pick the "
           "Z-X-Z proper-Euler chart -- adjust angle magnitudes";

    fx::gltf::Document doc;
    doc.nodes.resize(1);
    doc.nodes[0].name = "TestJoint";
    ChaChaFxGltf::write_agi_articulations(doc, articulations);

    const auto& agi = doc.extensionsAndExtras["extensions"]["AGI_articulations"]["articulations"];
    ASSERT_EQ(agi.size(), 1u);
    const auto& stages = agi[0]["stages"];
    ASSERT_EQ(stages.size(), 3u);

    const std::string name0 = stages[0]["name"].get<std::string>();
    const std::string name1 = stages[1]["name"].get<std::string>();
    const std::string name2 = stages[2]["name"].get<std::string>();

    EXPECT_EQ(name0, "zRotate");
    EXPECT_EQ(name1, "xRotate");
    EXPECT_EQ(name2, "zRotate2");
    EXPECT_NE(name0, name2) << "repeated StageType must not collapse to duplicate names";
}

// ---------------------------------------------------------------------------
// Synthetic: a genuine scale stage.
//
// None of the three corpus models produces a scale stage (sophia's scale
// channels are constant unit scale and get filtered; see the "constant
// scale channels produce no scale stages" check in SophiaAnalysis above),
// so the bridge's "scale is a bare multiplicative ratio, not degrees" branch
// (is_rotation_stage() gating the rad_to_deg conversion in
// write_agi_articulations) is completely unexercised by real data. A
// copy-paste bug applying the rotation conversion to scale stages too would
// multiply these values by ~57.3 and nothing in this suite besides this
// test would notice.
// ---------------------------------------------------------------------------
TEST(ChaChaIntegration, ScaleStageIsBareRatioNotDegrees)
{
    std::vector<int>       parents{-1};
    std::vector<glm::quat> rest{glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    std::vector<glm::vec3> trans{glm::vec3(0.0f)};
    std::vector<glm::vec3> scale{glm::vec3(1.0f)};
    ChaCha::Skeleton skeleton{parents, rest, trans, scale};

    std::vector<float> times;
    std::vector<float> values;
    const int n = 20;
    for (int i = 0; i <= n; ++i) {
        const float f = static_cast<float>(i) / static_cast<float>(n);
        // A pulse on Y scale: 1.0 -> 1.3 -> 1.0, well clear of the default
        // 0.01 scale threshold and clear of kMinRestScale degeneracy (rest
        // is 1.0 on every axis).
        const float y = 1.0f + 0.3f * std::sin(3.14159265f * f);
        values.insert(values.end(), {1.0f, y, 1.0f});
        times.push_back(f);
    }

    ChaCha::AnimationChannel ch;
    ch.node = 0; ch.animation = 0; ch.property = ChaCha::Property::Scale;
    ch.times = times; ch.values = values;

    std::vector<ChaCha::AnimationChannel> chans{ch};
    std::vector<ChaCha::Animation> anims{ChaCha::Animation{"rom"}};

    auto articulations = ChaCha::analyze(chans, anims, skeleton);
    ASSERT_EQ(articulations.size(), 1u);

    const ChaCha::Stage* yscale = nullptr;
    for (const auto& s : articulations[0].stages)
        if (s.type == ChaCha::StageType::yScale) yscale = &s;
    ASSERT_NE(yscale, nullptr)
        << "test setup assumption broken: no yScale stage emitted for a "
           "0.3-ratio pulse well clear of the noise threshold";

    // Ratio space: rest is 1.0, so max_value should sit near 1.3, not
    // glm::degrees(1.3) (~74.5) and not (1.3 - 1.0) (an additive delta,
    // which scale explicitly is not -- see chacha_analyzer.cpp's
    // is_scale comment).
    EXPECT_NEAR(yscale->max_value, 1.3f, 0.05f);
    EXPECT_NEAR(yscale->min_value, 1.0f, 0.05f);

    fx::gltf::Document doc;
    doc.nodes.resize(1);
    doc.nodes[0].name = "ScaleJoint";
    ChaChaFxGltf::write_agi_articulations(doc, articulations);

    const auto& agi = doc.extensionsAndExtras["extensions"]["AGI_articulations"]["articulations"];
    ASSERT_EQ(agi.size(), 1u);
    bool found = false;
    for (const auto& sj : agi[0]["stages"]) {
        if (sj["type"] != "yScale") continue;
        found = true;
        const float written_max = sj["maximumValue"].get<float>();
        // No unit conversion at all: the JSON value must equal the raw
        // Stage value. If someone applied rad_to_deg here it would come out
        // around 74.5 instead.
        EXPECT_NEAR(written_max, yscale->max_value, 1e-4f);
        EXPECT_LT(written_max, 10.0f)
            << "written scale value looks like it went through a degrees "
               "conversion";
    }
    ASSERT_TRUE(found) << "no yScale stage found in written AGI JSON";
}

// ---------------------------------------------------------------------------
// Performance guard: catch a silent multi-x regression in analyze(), not
// small drift.
//
// The search is 49 joints x 12 charts x 88 animations x ~157 frames on this
// model (select_candidate additionally evaluates 3 one-DOF and 6 two-DOF
// candidates per joint, all scaling the same way -- see chacha_search.cpp).
//
// Measured directly on this machine, both configurations built from the same
// source with nothing else changed:
//   RelWithDebInfo (the CMakeLists.txt default): five runs of analyze() alone
//     on sophia-2_9.glb ranged 18968-21876 ms.
//   Debug (-DCMAKE_BUILD_TYPE=Debug, otherwise identical): two runs ranged
//     76779-87704 ms -- roughly a 4x Debug/RelWithDebInfo ratio, not the 6x
//     an earlier draft of this comment assumed from a second-hand figure
//     ("essentially exactly 120000 ms" for an unoptimised build). That
//     assumption did not hold up against direct measurement, and the 120000 ms
//     bound it produced sat far enough above a genuine Debug build (~77-88s)
//     that it would not have caught someone accidentally building Debug, nor
//     a real 3-4x algorithmic regression -- both would have passed.
//
// 60000 ms is chosen instead: about 2.7x the slower end of the measured
// RelWithDebInfo ceiling (headroom for a slower machine), while sitting well
// below the measured Debug range, so it reliably fails a Debug build and
// would also catch a genuine 3x algorithmic regression on RelWithDebInfo.
// Both properties were verified directly, not assumed: this test, built and
// run against RelWithDebInfo, passes at 60000 ms; the same test, built and
// run against an unmodified Debug configuration, fails at 60000 ms (measured
// ~77-88s against the 60s bound). Still not a tuned performance test -- it
// is a guard against catastrophic regressions and misconfigured build types,
// not a bound to alert on small drift.
TEST(ChaChaIntegration, SophiaAnalysisCompletesInReasonableTime)
{
    auto doc   = load("sophia-2_9.glb");
    auto anims = ChaChaFxGltf::extract_animation_channels(doc);
    auto skel  = ChaChaFxGltf::extract_skeleton(doc);

    const auto start = std::chrono::steady_clock::now();
    auto arts = ChaCha::analyze(anims.channels, anims.animations, skel.as_skeleton());
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    std::printf("[sophia] analyze() took %lld ms for %zu articulations\n",
                static_cast<long long>(elapsed), arts.size());
    EXPECT_LT(elapsed, 60000);
}

// ---------------------------------------------------------------------------
// remove_agi_animations must not corrupt bufferView references that don't
// come from a plain accessor.bufferView: an image's embedded bufferView, and
// an accessor's sparse indices/values bufferViews. Neither is counted or
// remapped by the naive "accessor.bufferView" walk, so any document with an
// embedded image or a sparse accessor gets its bufferView indices scrambled
// (or left pointing past the end of the shrunk array) as soon as AGI
// animations are stripped -- silently, since no test model has either
// feature. This test builds a minimal synthetic document exercising both,
// and was confirmed to fail against the pre-fix code (image.bufferView
// stayed at its stale pre-removal index, and the sparse bufferViews were
// left pointing one past the end of the surviving bufferViews array).
// ---------------------------------------------------------------------------
TEST(ChaChaIntegration, RemoveAgiAnimationsPreservesImageAndSparseBufferViews)
{
    fx::gltf::Document doc;

    // Three bufferViews:
    //   [0] referenced only by an accessor that becomes orphaned once the
    //       AGI animation is stripped -- must be removed.
    //   [1] referenced by an embedded image -- must survive and be remapped.
    //   [2] referenced by a sparse accessor's indices AND values -- must
    //       survive and be remapped.
    doc.bufferViews.resize(3);
    doc.bufferViews[0].buffer = 0;
    doc.bufferViews[0].byteLength = 4;
    doc.bufferViews[1].buffer = 0;
    doc.bufferViews[1].byteLength = 4;
    doc.bufferViews[2].buffer = 0;
    doc.bufferViews[2].byteLength = 4;

    // accessors[0]: used only by the AGI animation's sampler -> orphaned.
    fx::gltf::Accessor agi_only_accessor;
    agi_only_accessor.bufferView = 0;
    agi_only_accessor.componentType = fx::gltf::Accessor::ComponentType::Float;
    agi_only_accessor.type = fx::gltf::Accessor::Type::Vec3;
    agi_only_accessor.count = 1;

    // accessors[1]: used by a normal (non-AGI) animation's sampler, so it
    // survives. It has no direct bufferView of its own (bufferView == -1,
    // a sparse-only accessor), but its sparse indices/values reference
    // bufferViews[2].
    fx::gltf::Accessor sparse_accessor;
    sparse_accessor.bufferView = -1;
    sparse_accessor.componentType = fx::gltf::Accessor::ComponentType::Float;
    sparse_accessor.type = fx::gltf::Accessor::Type::Vec3;
    sparse_accessor.count = 1;
    sparse_accessor.sparse.count = 1;
    sparse_accessor.sparse.indices.bufferView = 2;
    sparse_accessor.sparse.values.bufferView = 2;

    doc.accessors = { agi_only_accessor, sparse_accessor };

    // animations[0]: "AGI Configuration" -- gets stripped, orphaning
    // accessors[0].
    fx::gltf::Animation agi_anim;
    agi_anim.name = "AGI Configuration";
    fx::gltf::Animation::Sampler agi_sampler;
    agi_sampler.input = 0;
    agi_sampler.output = 0;
    agi_anim.samplers = { agi_sampler };
    fx::gltf::Animation::Channel agi_channel;
    agi_channel.sampler = 0;
    agi_channel.target.node = 0;
    agi_channel.target.path = "rotation";
    agi_anim.channels = { agi_channel };

    // animations[1]: ordinary animation, keeps accessors[1] alive.
    fx::gltf::Animation normal_anim;
    normal_anim.name = "Walk";
    fx::gltf::Animation::Sampler normal_sampler;
    normal_sampler.input = 1;
    normal_sampler.output = 1;
    normal_anim.samplers = { normal_sampler };
    fx::gltf::Animation::Channel normal_channel;
    normal_channel.sampler = 0;
    normal_channel.target.node = 0;
    normal_channel.target.path = "translation";
    normal_anim.channels = { normal_channel };

    doc.animations = { agi_anim, normal_anim };

    // images[0]: embedded (no uri), referencing bufferViews[1].
    fx::gltf::Image image;
    image.bufferView = 1;
    image.mimeType = "image/png";
    doc.images = { image };

    ChaChaFxGltf::remove_agi_animations(doc);

    // The AGI animation is gone; the normal one remains.
    ASSERT_EQ(doc.animations.size(), 1u);
    EXPECT_NE(doc.animations[0].name.find("Walk"), std::string::npos);

    // Only the sparse-referencing accessor survives.
    ASSERT_EQ(doc.accessors.size(), 1u);
    const auto& surviving_accessor = doc.accessors[0];
    EXPECT_EQ(surviving_accessor.bufferView, -1);

    // Two bufferViews survive (the orphaned one is gone).
    ASSERT_EQ(doc.bufferViews.size(), 2u);

    // The sparse accessor's bufferViews must be remapped to point at the
    // SAME underlying bufferView they did before removal, not left at their
    // stale pre-removal index (which would now be out of bounds or point at
    // the wrong surviving bufferView).
    ASSERT_LT(surviving_accessor.sparse.indices.bufferView, doc.bufferViews.size());
    ASSERT_LT(surviving_accessor.sparse.values.bufferView, doc.bufferViews.size());
    EXPECT_EQ(surviving_accessor.sparse.indices.bufferView,
              surviving_accessor.sparse.values.bufferView);

    // The image's bufferView must likewise be remapped, not left dangling.
    ASSERT_EQ(doc.images.size(), 1u);
    ASSERT_LT(static_cast<size_t>(doc.images[0].bufferView), doc.bufferViews.size());

    // The image and the sparse accessor referenced DIFFERENT bufferViews
    // before removal (1 and 2 respectively); they must still reference
    // different bufferViews after remapping -- if the bug were "remap
    // everything to the same new index" this would incorrectly pass, so
    // check they diverge rather than just that both are in range.
    EXPECT_NE(static_cast<uint32_t>(doc.images[0].bufferView),
              surviving_accessor.sparse.indices.bufferView);
}
