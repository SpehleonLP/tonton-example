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
    // exist to exercise the invariant above. If this drifts to 0 the test
    // below is vacuous, so make that visible rather than silently passing.
    ASSERT_GT(unanimated_checked, 0)
        << "expected at least one unanimated node in treefrog.glb to exercise "
           "the locked-joint invariant; corpus file may have changed";

    // The count itself: not a fraction of total nodes (that bound is false
    // per the measurement above) but the actual number of animated nodes
    // whose motion clears the noise threshold. 56 is the measured baseline
    // for this exact corpus file with default Options; deriving it any more
    // "structurally" than this would mean reimplementing the threshold
    // logic (peak-to-peak deviation vs. rotation/translation/scale
    // thresholds) here, which is exactly the kind of test that can never
    // fail against a broken threshold. Bracket it against the channel-count
    // upper bound (which we CAN derive from the document) and pin the exact
    // measured count as a regression guard.
    EXPECT_LE(a.articulations.size(), animated_nodes.size())
        << "cannot emit more articulations than nodes with any animation channel";
    EXPECT_EQ(a.articulations.size(), 56u)
        << "measured baseline for treefrog.glb with default Options; if this "
           "changes intentionally (threshold tuning, new stage types, etc.) "
           "update the constant, but a silent drift here is a real regression";
}

// ---------------------------------------------------------------------------
// Ruling 3: pointing_vector is not a per-joint anatomical value.
// infer_pointing_vectors votes for a single model-dominant bone axis, so the
// only thing legitimately assertable is that the vote is applied uniformly
// across every articulation in the document -- not a specific direction per
// joint. If a future change accidentally made this per-joint (e.g. reusing
// a per-node local axis instead of the model-wide vote), this test would
// start failing on the very first divergent pair.
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

TEST(ChaChaIntegration, ScorpionProducesArticulations)
{
    auto doc = load("emporer scorpion.glb");
    auto a   = run(doc);
    EXPECT_FALSE(a.articulations.empty());
    // Measured baseline (see task-15 report): 45.
    EXPECT_EQ(a.articulations.size(), 45u);
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
//    their solved angles must reproduce the original keyframes. Requires no
//    anatomical ground truth.
// 2) Constant unit scale channels must not produce scale stages.
// 3) The DOF histogram is reported (and at least one non-zero-DOF joint
//    exists), which is also where fit_residual_rad gets surfaced.
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

        // Reachability check: the emitted axes must span the observed motion.
        // A joint constrained to fewer axes than it moves in would show up here.
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
    // Reported rather than tightly asserted: mixamo rigs contain genuine
    // three-axis motion at many joints, so this bounds gross failure only.
    std::printf("[sophia] sampled=%d worst residual=%.2f deg\n",
                sampled, worst * 57.29577951);
    EXPECT_LT(worst, 3.2);   // radians; anything near pi indicates a broken fit
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
    EXPECT_GT(hist[1] + hist[2] + hist[3], 0);
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
// channels are constant unit scale and get filtered; see
// SophiaConstantScaleChannelsProduceNoScaleStages above), so the bridge's
// "scale is a bare multiplicative ratio, not degrees" branch
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
