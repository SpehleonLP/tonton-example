#include "chacha_fxgltf_bridge.h"
#include "fx/gltf.h"
#include <nlohmann/json.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <unordered_map>

// Lightweight accessor data navigator (same pattern as RTT::AccessorData
// but without rintintin dependency)
struct AccessorData {
    const fx::gltf::Accessor* accessor{};
    const fx::gltf::BufferView* bufferView{};
    const uint8_t* data{};

    AccessorData(const fx::gltf::Document& doc, int32_t accessorIdx) {
        if (accessorIdx < 0 || accessorIdx >= static_cast<int32_t>(doc.accessors.size()))
            return;
        accessor = &doc.accessors[accessorIdx];
        if (accessor->bufferView < 0 || accessor->bufferView >= static_cast<int32_t>(doc.bufferViews.size()))
            return;
        bufferView = &doc.bufferViews[accessor->bufferView];
        if (bufferView->buffer < 0 || bufferView->buffer >= static_cast<int32_t>(doc.buffers.size()))
            return;
        const auto& buffer = doc.buffers[bufferView->buffer];
        data = buffer.data.data() + bufferView->byteOffset + accessor->byteOffset;
    }

    bool isValid() const { return accessor && bufferView && data; }
};

namespace ChaChaFxGltf {

// ---------------------------------------------------------------------------
// Helper: case-insensitive prefix check
// ---------------------------------------------------------------------------
static bool starts_with_agi(const std::string& name)
{
    if (name.size() < 4) return false;
    auto prefix = name.substr(0, 4);
    for (auto& c : prefix) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return prefix == "agi_";
}

// ---------------------------------------------------------------------------
// extract_animation_channels
// ---------------------------------------------------------------------------
ExtractedAnimations extract_animation_channels(const fx::gltf::Document& doc)
{
    ExtractedAnimations result;

    for (uint32_t anim_idx = 0; anim_idx < doc.animations.size(); ++anim_idx) {
        const auto& anim = doc.animations[anim_idx];

        if (starts_with_agi(anim.name))
            result.agi_animation_indices.insert(anim_idx);

        for (const auto& channel : anim.channels) {
            if (channel.target.node < 0) continue;
            if (channel.sampler < 0 || channel.sampler >= static_cast<int32_t>(anim.samplers.size()))
                continue;

            const auto& sampler = anim.samplers[channel.sampler];

            // Map target.path to ChaCha::Property
            ChaCha::Property prop;
            int values_per_key;
            if (channel.target.path == "rotation") {
                prop = ChaCha::Property::Rotation;
                values_per_key = 4;
            } else if (channel.target.path == "translation") {
                prop = ChaCha::Property::Translation;
                values_per_key = 3;
            } else if (channel.target.path == "scale") {
                prop = ChaCha::Property::Scale;
                values_per_key = 3;
            } else {
                continue; // weights or unknown
            }

            // Map interpolation
            ChaCha::InterpolationType interp;
            switch (sampler.interpolation) {
            case fx::gltf::Animation::Sampler::Type::Step:
                interp = ChaCha::InterpolationType::Step;
                break;
            case fx::gltf::Animation::Sampler::Type::CubicSpline:
                interp = ChaCha::InterpolationType::CubicSpline;
                break;
            default:
                interp = ChaCha::InterpolationType::Linear;
                break;
            }

            // Read time accessor
            AccessorData time_data(doc, sampler.input);
            AccessorData value_data(doc, sampler.output);
            if (!time_data.isValid() || !value_data.isValid()) continue;

            const uint32_t key_count = time_data.accessor->count;
            if (key_count == 0) continue;

            // Copy times (always float scalars)
            const float* time_ptr = reinterpret_cast<const float*>(time_data.data);
            result.time_storage.emplace_back(time_ptr, time_ptr + key_count);

            // Copy values
            const float* value_ptr = reinterpret_cast<const float*>(value_data.data);
            uint32_t total_values;
            if (interp == ChaCha::InterpolationType::CubicSpline) {
                // CubicSpline has 3x values per keyframe (in-tangent, value, out-tangent)
                total_values = key_count * values_per_key * 3;
            } else {
                total_values = key_count * values_per_key;
            }
            result.value_storage.emplace_back(value_ptr, value_ptr + total_values);

            ChaCha::AnimationChannel ch;
            ch.node = channel.target.node;
            ch.property = prop;
            ch.interp = interp;
            ch.times = std::span<const float>(result.time_storage.back());
            ch.values = std::span<const float>(result.value_storage.back());
            result.channels.push_back(ch);
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// extract_skeleton
// ---------------------------------------------------------------------------
ExtractedSkeleton extract_skeleton(const fx::gltf::Document& doc, int skin_index)
{
    ExtractedSkeleton result;

    if (skin_index < 0 || skin_index >= static_cast<int>(doc.skins.size()))
        throw std::invalid_argument("Invalid skin index");

    const auto& skin = doc.skins[skin_index];
    const size_t joint_count = skin.joints.size();
    if (joint_count == 0)
        throw std::invalid_argument("Skin has no joints");

    result.joint_nodes.assign(skin.joints.begin(), skin.joints.end());
    result.parents.resize(joint_count, -1);
    result.rest_rotations.resize(joint_count, glm::quat(1, 0, 0, 0));
    result.rest_translations.resize(joint_count, glm::vec3(0));

    // Build node-to-joint map
    std::unordered_map<uint32_t, int> node_to_joint;
    for (size_t i = 0; i < joint_count; ++i)
        node_to_joint[skin.joints[i]] = static_cast<int>(i);

    // Build parent array from node.children
    for (size_t i = 0; i < joint_count; ++i) {
        const auto& node = doc.nodes[skin.joints[i]];
        for (auto child : node.children) {
            auto it = node_to_joint.find(child);
            if (it != node_to_joint.end())
                result.parents[it->second] = static_cast<int>(i);
        }
    }

    // Extract rest poses from node-local TRS.
    // Animation channels store local-space values, so rest poses must also
    // be local-space for correct rest-relative subtraction in ChaCha.
    // (Inverse bind matrices give mesh-space poses — wrong basis for this.)
    for (size_t i = 0; i < joint_count; ++i) {
        const auto& node = doc.nodes[skin.joints[i]];

        if (node.matrix != fx::gltf::defaults::IdentityMatrix) {
            glm::mat4 mat;
            std::memcpy(&mat[0].x, node.matrix.data(), sizeof(glm::mat4));
            glm::vec3 scale, translation, skew;
            glm::quat rotation;
            glm::vec4 perspective;
            glm::decompose(mat, scale, rotation, translation, skew, perspective);
            result.rest_rotations[i] = rotation;
            result.rest_translations[i] = translation;
        } else {
            // glTF rotation is [x,y,z,w], glm::quat constructor is (w,x,y,z)
            result.rest_rotations[i] = glm::quat(
                node.rotation[3], node.rotation[0],
                node.rotation[1], node.rotation[2]);
            result.rest_translations[i] = glm::vec3(
                node.translation[0], node.translation[1], node.translation[2]);
        }
    }

    return result;
}

ChaCha::Skeleton ExtractedSkeleton::as_skeleton() const
{
    return ChaCha::Skeleton{
        .parents = std::span<const int>(parents),
        .rest_rotations = std::span<const glm::quat>(rest_rotations),
        .rest_translations = std::span<const glm::vec3>(rest_translations),
    };
}

// ---------------------------------------------------------------------------
// write_agi_articulations
// ---------------------------------------------------------------------------

static const char* stage_type_to_agi_type(ChaCha::StageType type)
{
    switch (type) {
    case ChaCha::StageType::xTranslate: return "xTranslate";
    case ChaCha::StageType::yTranslate: return "yTranslate";
    case ChaCha::StageType::zTranslate: return "zTranslate";
    case ChaCha::StageType::xRotate:    return "xRotate";
    case ChaCha::StageType::yRotate:    return "yRotate";
    case ChaCha::StageType::zRotate:    return "zRotate";
    case ChaCha::StageType::xScale:     return "xScale";
    case ChaCha::StageType::yScale:     return "yScale";
    case ChaCha::StageType::zScale:     return "zScale";
    }
    return "unknown";
}

static bool is_rotation_stage(ChaCha::StageType type)
{
    return type == ChaCha::StageType::xRotate ||
           type == ChaCha::StageType::yRotate ||
           type == ChaCha::StageType::zRotate;
}

static constexpr float rad_to_deg = 180.0f / 3.14159265358979323846f;

void write_agi_articulations(
    fx::gltf::Document& doc,
    const std::vector<ChaCha::Articulation>& articulations,
    const std::vector<uint32_t>& joint_nodes)
{
    nlohmann::json agi_array = nlohmann::json::array();

    for (const auto& artic : articulations) {
        // Resolve node index
        uint32_t node_idx = (artic.node >= 0 && artic.node < static_cast<int>(joint_nodes.size()))
            ? joint_nodes[artic.node]
            : static_cast<uint32_t>(artic.node);

        // Determine name: prefer node name from document
        std::string name;
        if (node_idx < doc.nodes.size() && !doc.nodes[node_idx].name.empty())
            name = doc.nodes[node_idx].name;
        else if (!artic.name.empty())
            name = artic.name;
        else
            name = "joint_" + std::to_string(artic.node);

        // Build stages array
        nlohmann::json stages_json = nlohmann::json::array();
        for (const auto& stage : artic.stages) {
            float conv = is_rotation_stage(stage.type) ? rad_to_deg : 1.0f;
            float vel_conv = is_rotation_stage(stage.type) ? rad_to_deg : 1.0f;

            nlohmann::json sj;
            sj["type"] = stage_type_to_agi_type(stage.type);
            sj["minimumValue"] = stage.min_value * conv;
            sj["maximumValue"] = stage.max_value * conv;
            sj["initialValue"] = stage.initial_value * conv;
            if (stage.max_velocity > 0)
                sj["maximumSpeed"] = stage.max_velocity * vel_conv;
            if (stage.max_effort > 0)
                sj["maximumEffort"] = stage.max_effort;
            stages_json.push_back(sj);
        }

        nlohmann::json entry;
        entry["name"] = name;
        entry["stages"] = stages_json;
        if (artic.node >= 0 && artic.node < static_cast<int>(joint_nodes.size()))
            entry["pointingVector"] = {0, 0, 1}; // default forward

        agi_array.push_back(entry);

        // Set node-level extension
        if (node_idx < doc.nodes.size()) {
            doc.nodes[node_idx].extensionsAndExtras["extensions"]["AGI_articulations"]["articulationName"] = name;
        }
    }

    // Set document-level extension
    doc.extensionsAndExtras["extensions"]["AGI_articulations"]["articulations"] = agi_array;

    // Add to extensionsUsed
    auto& used = doc.extensionsUsed;
    if (std::find(used.begin(), used.end(), "AGI_articulations") == used.end())
        used.push_back("AGI_articulations");
}

// ---------------------------------------------------------------------------
// remove_agi_animations
// ---------------------------------------------------------------------------

void remove_agi_animations(fx::gltf::Document& doc)
{
    // 1. Identify AGI_ animation indices
    std::unordered_set<uint32_t> agi_indices;
    for (uint32_t i = 0; i < doc.animations.size(); ++i) {
        if (starts_with_agi(doc.animations[i].name))
            agi_indices.insert(i);
    }
    if (agi_indices.empty()) return;

    // 2. Build accessor refcount from ALL consumers
    std::unordered_map<int32_t, int> accessor_refcount;

    auto ref_accessor = [&](int32_t idx) {
        if (idx >= 0) accessor_refcount[idx]++;
    };
    auto ref_accessor_u = [&](uint32_t idx) {
        accessor_refcount[static_cast<int32_t>(idx)]++;
    };

    // All animation samplers
    for (const auto& anim : doc.animations) {
        for (const auto& sampler : anim.samplers) {
            ref_accessor(sampler.input);
            ref_accessor(sampler.output);
        }
    }
    // Mesh primitives
    for (const auto& mesh : doc.meshes) {
        for (const auto& prim : mesh.primitives) {
            ref_accessor(prim.indices);
            for (const auto& [key, val] : prim.attributes)
                ref_accessor_u(val);
            for (const auto& target : prim.targets)
                for (const auto& [key, val] : target)
                    ref_accessor_u(val);
        }
    }
    // Skins
    for (const auto& skin : doc.skins)
        ref_accessor(skin.inverseBindMatrices);

    // 3. Decrement refcounts for AGI_ animation samplers
    for (auto idx : agi_indices) {
        for (const auto& sampler : doc.animations[idx].samplers) {
            if (sampler.input >= 0) accessor_refcount[sampler.input]--;
            if (sampler.output >= 0) accessor_refcount[sampler.output]--;
        }
    }

    // 4. Collect zero-refcount accessors
    std::unordered_set<int32_t> removable_accessors;
    for (const auto& [idx, count] : accessor_refcount) {
        if (count <= 0) removable_accessors.insert(idx);
    }

    // 5. Build bufferView refcount from all accessors
    std::unordered_map<int32_t, int> bv_refcount;
    for (int32_t i = 0; i < static_cast<int32_t>(doc.accessors.size()); ++i) {
        if (doc.accessors[i].bufferView >= 0)
            bv_refcount[doc.accessors[i].bufferView]++;
    }
    // Decrement for removable accessors
    for (auto acc_idx : removable_accessors) {
        if (acc_idx >= 0 && acc_idx < static_cast<int32_t>(doc.accessors.size())) {
            auto bv = doc.accessors[acc_idx].bufferView;
            if (bv >= 0) bv_refcount[bv]--;
        }
    }

    std::unordered_set<int32_t> removable_bvs;
    for (const auto& [idx, count] : bv_refcount) {
        if (count <= 0) removable_bvs.insert(idx);
    }

    // 6. Build old->new remapping tables
    std::vector<int32_t> accessor_remap(doc.accessors.size(), -1);
    {
        int32_t new_idx = 0;
        for (int32_t i = 0; i < static_cast<int32_t>(doc.accessors.size()); ++i) {
            if (removable_accessors.count(i) == 0)
                accessor_remap[i] = new_idx++;
        }
    }

    std::vector<int32_t> bv_remap(doc.bufferViews.size(), -1);
    {
        int32_t new_idx = 0;
        for (int32_t i = 0; i < static_cast<int32_t>(doc.bufferViews.size()); ++i) {
            if (removable_bvs.count(i) == 0)
                bv_remap[i] = new_idx++;
        }
    }

    // 7. Remove entries (reverse order to keep indices stable)
    {
        std::vector<fx::gltf::Accessor> new_accessors;
        for (int32_t i = 0; i < static_cast<int32_t>(doc.accessors.size()); ++i) {
            if (removable_accessors.count(i) == 0)
                new_accessors.push_back(std::move(doc.accessors[i]));
        }
        doc.accessors = std::move(new_accessors);
    }
    {
        std::vector<fx::gltf::BufferView> new_bvs;
        for (int32_t i = 0; i < static_cast<int32_t>(doc.bufferViews.size()); ++i) {
            if (removable_bvs.count(i) == 0)
                new_bvs.push_back(std::move(doc.bufferViews[i]));
        }
        doc.bufferViews = std::move(new_bvs);
    }

    // 8. Remap all accessor indices throughout document
    auto remap_acc = [&](int32_t& idx) {
        if (idx >= 0 && idx < static_cast<int32_t>(accessor_remap.size()))
            idx = accessor_remap[idx];
    };
    auto remap_acc_u = [&](uint32_t& idx) {
        auto i = static_cast<int32_t>(idx);
        if (i >= 0 && i < static_cast<int32_t>(accessor_remap.size()) && accessor_remap[i] >= 0)
            idx = static_cast<uint32_t>(accessor_remap[i]);
    };
    auto remap_bv = [&](int32_t& idx) {
        if (idx >= 0 && idx < static_cast<int32_t>(bv_remap.size()))
            idx = bv_remap[idx];
    };

    // Remap accessor.bufferView
    for (auto& acc : doc.accessors)
        remap_bv(acc.bufferView);

    // Remap animation sampler accessors (non-AGI only, since AGI are being removed)
    for (uint32_t i = 0; i < doc.animations.size(); ++i) {
        if (agi_indices.count(i)) continue;
        for (auto& sampler : doc.animations[i].samplers) {
            remap_acc(sampler.input);
            remap_acc(sampler.output);
        }
    }

    // Remap mesh primitive accessors
    for (auto& mesh : doc.meshes) {
        for (auto& prim : mesh.primitives) {
            remap_acc(prim.indices);
            for (auto& [key, val] : prim.attributes)
                remap_acc_u(val);
            for (auto& target : prim.targets)
                for (auto& [key, val] : target)
                    remap_acc_u(val);
        }
    }

    // Remap skin accessors
    for (auto& skin : doc.skins)
        remap_acc(skin.inverseBindMatrices);

    // 9. Remove AGI_ animations (reverse order)
    std::vector<uint32_t> sorted_agi(agi_indices.begin(), agi_indices.end());
    std::sort(sorted_agi.rbegin(), sorted_agi.rend());
    for (auto idx : sorted_agi)
        doc.animations.erase(doc.animations.begin() + idx);
}

// ---------------------------------------------------------------------------
// has_agi_articulations
// ---------------------------------------------------------------------------

bool has_agi_articulations(const fx::gltf::Document& doc)
{
    if (!doc.extensionsAndExtras.is_object()) return false;
    auto ext_it = doc.extensionsAndExtras.find("extensions");
    if (ext_it == doc.extensionsAndExtras.end()) return false;
    return ext_it->find("AGI_articulations") != ext_it->end();
}

// ---------------------------------------------------------------------------
// print_articulations_json
// ---------------------------------------------------------------------------

void print_articulations_json(
    std::ostream& os,
    const fx::gltf::Document& doc,
    const std::vector<ChaCha::Articulation>& articulations,
    const std::vector<uint32_t>& joint_nodes)
{
    nlohmann::json root = nlohmann::json::array();

    for (const auto& artic : articulations) {
        nlohmann::json entry;

        // Resolve name from node if possible
        uint32_t node_idx = (artic.node >= 0 && artic.node < static_cast<int>(joint_nodes.size()))
            ? joint_nodes[artic.node]
            : static_cast<uint32_t>(artic.node);
        std::string name;
        if (node_idx < doc.nodes.size() && !doc.nodes[node_idx].name.empty())
            name = doc.nodes[node_idx].name;
        else if (!artic.name.empty())
            name = artic.name;
        else
            name = "joint_" + std::to_string(artic.node);
        entry["name"] = name;
        entry["node"] = artic.node;
        if (artic.node >= 0 && artic.node < static_cast<int>(joint_nodes.size()))
            entry["glTF_node"] = joint_nodes[artic.node];

        nlohmann::json stages_json = nlohmann::json::array();
        for (const auto& stage : artic.stages) {
            float conv = is_rotation_stage(stage.type) ? rad_to_deg : 1.0f;
            nlohmann::json sj;
            sj["type"] = stage_type_to_agi_type(stage.type);
            sj["min"] = stage.min_value * conv;
            sj["max"] = stage.max_value * conv;
            sj["initial"] = stage.initial_value * conv;
            if (stage.max_velocity > 0)
                sj["maxSpeed"] = stage.max_velocity * conv;
            stages_json.push_back(sj);
        }
        entry["stages"] = stages_json;
        root.push_back(entry);
    }

    os << root.dump(2) << "\n";
}

} // namespace ChaChaFxGltf
