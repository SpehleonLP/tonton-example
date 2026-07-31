#include "chacha_fxgltf_bridge.h"
#include "chacha_naming.h"
#include "fx/gltf.h"
#include <nlohmann/json.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <unordered_map>

// Component/element byte sizes, needed to bounds-check the byte EXTENT an
// accessor claims -- not just the index chain into bufferView/buffer.
static uint32_t accessor_component_size(fx::gltf::Accessor::ComponentType t)
{
    switch (t) {
    case fx::gltf::Accessor::ComponentType::Byte:
    case fx::gltf::Accessor::ComponentType::UnsignedByte:
        return 1;
    case fx::gltf::Accessor::ComponentType::Short:
    case fx::gltf::Accessor::ComponentType::UnsignedShort:
        return 2;
    case fx::gltf::Accessor::ComponentType::UnsignedInt:
    case fx::gltf::Accessor::ComponentType::Float:
        return 4;
    default:
        return 0;
    }
}

static uint32_t accessor_component_count(fx::gltf::Accessor::Type t)
{
    switch (t) {
    case fx::gltf::Accessor::Type::Scalar: return 1;
    case fx::gltf::Accessor::Type::Vec2:   return 2;
    case fx::gltf::Accessor::Type::Vec3:   return 3;
    case fx::gltf::Accessor::Type::Vec4:   return 4;
    case fx::gltf::Accessor::Type::Mat2:   return 4;
    case fx::gltf::Accessor::Type::Mat3:   return 9;
    case fx::gltf::Accessor::Type::Mat4:   return 16;
    default: return 0;
    }
}

// Lightweight accessor data navigator (same pattern as RTT::AccessorData
// but without rintintin dependency)
struct AccessorData {
    const fx::gltf::Accessor* accessor{};
    const fx::gltf::BufferView* bufferView{};
    const uint8_t* data{};

    AccessorData(const fx::gltf::Document& doc, int32_t accessorIdx) {
        if (accessorIdx < 0 || accessorIdx >= static_cast<int32_t>(doc.accessors.size()))
            return;
        const auto* acc = &doc.accessors[accessorIdx];
        if (acc->bufferView < 0 || acc->bufferView >= static_cast<int32_t>(doc.bufferViews.size()))
            return;
        const auto* bv = &doc.bufferViews[acc->bufferView];
        if (bv->buffer < 0 || bv->buffer >= static_cast<int32_t>(doc.buffers.size()))
            return;
        const auto& buffer = doc.buffers[bv->buffer];

        // This bridge only ever reads accessor data as raw float arrays (see the
        // reinterpret_cast<const float*> call sites below) -- a non-Float
        // accessor would silently be reinterpreted as float data, so reject it
        // here rather than trust the caller's assumption downstream.
        if (acc->componentType != fx::gltf::Accessor::ComponentType::Float)
            return;

        const uint32_t comp_size = accessor_component_size(acc->componentType);
        const uint32_t num_comp  = accessor_component_count(acc->type);
        if (comp_size == 0 || num_comp == 0)
            return;
        const uint32_t element_size = comp_size * num_comp;
        const uint32_t stride = bv->byteStride != 0 ? bv->byteStride : element_size;

        // Byte-extent check: verify byteOffset + count*stride (the last
        // element's end) actually fits within the buffer. The earlier checks
        // above only validate the accessor -> bufferView -> buffer INDEX
        // chain; a crafted or corrupt accessor.count/byteOffset can still
        // overrun the buffer's actual byte length, which the index chain
        // alone can't catch.
        const uint64_t base_offset =
            static_cast<uint64_t>(bv->byteOffset) + acc->byteOffset;
        uint64_t required = base_offset;
        if (acc->count > 0)
            required += static_cast<uint64_t>(acc->count - 1) * stride + element_size;
        if (required > buffer.data.size())
            return;

        accessor = acc;
        bufferView = bv;
        data = buffer.data.data() + base_offset;
    }

    bool isValid() const { return accessor && bufferView && data; }
};

namespace ChaChaFxGltf {

// ---------------------------------------------------------------------------
// Helper: case-insensitive prefix check
// ---------------------------------------------------------------------------
static bool starts_with_agi(const std::string& name)
{
    // Authored configuration animations are named things like
    // "AGI Configuration" -- note the space, not an underscore.
    if (name.size() < 4) return false;
    auto prefix = name.substr(0, 4);
    for (auto& c : prefix) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return prefix == "agi ";
}

// ---------------------------------------------------------------------------
// extract_animation_channels
// ---------------------------------------------------------------------------
ExtractedAnimations extract_animation_channels(const fx::gltf::Document& doc)
{
    ExtractedAnimations result;

    // Build owned name storage COMPLETELY before taking any string_view into
    // it -- reallocation would dangle every earlier view otherwise.
    result.name_storage.reserve(doc.animations.size());
    for (const auto& a : doc.animations)
        result.name_storage.push_back(a.name);
    for (const auto& s : result.name_storage)
        result.animations.push_back(ChaCha::Animation{s});

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

            // total_values is derived from the TIME accessor's count and the
            // channel's target path, not from the value accessor's own
            // metadata -- AccessorData already verified value_data.accessor's
            // OWN declared count/type fit the buffer, but a malformed file
            // could still declare a time accessor with a larger count than
            // the paired value accessor actually holds. Re-derive the value
            // accessor's own safe element budget and refuse to read past it.
            const uint32_t value_num_comp = accessor_component_count(value_data.accessor->type);
            const uint64_t value_budget =
                static_cast<uint64_t>(value_data.accessor->count) * value_num_comp;
            if (value_num_comp == 0 || total_values > value_budget)
                continue;

            result.value_storage.emplace_back(value_ptr, value_ptr + total_values);

            ChaCha::AnimationChannel ch;
            ch.node = channel.target.node;
            ch.animation = static_cast<int>(anim_idx);
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
ExtractedSkeleton extract_skeleton(const fx::gltf::Document& doc)
{
    ExtractedSkeleton result;
    const size_t n = doc.nodes.size();

    result.parents.assign(n, -1);
    result.rest_rotations.assign(n, glm::quat(1, 0, 0, 0));
    result.rest_translations.assign(n, glm::vec3(0));
    result.rest_scales.assign(n, glm::vec3(1));

    for (size_t i = 0; i < n; ++i)
        for (auto child : doc.nodes[i].children)
            if (child >= 0 && static_cast<size_t>(child) < n)
                result.parents[child] = static_cast<int>(i);

    // Extract rest poses from node-local TRS.
    // Animation channels store local-space values, so rest poses must also
    // be local-space for correct rest-relative subtraction in ChaCha.
    // (Inverse bind matrices give mesh-space poses — wrong basis for this.)
    for (size_t i = 0; i < n; ++i) {
        const auto& node = doc.nodes[i];

        if (node.matrix != fx::gltf::defaults::IdentityMatrix) {
            glm::mat4 mat;
            std::memcpy(&mat[0].x, node.matrix.data(), sizeof(glm::mat4));
            glm::vec3 scale, translation, skew;
            glm::quat rotation;
            glm::vec4 perspective;
            glm::decompose(mat, scale, rotation, translation, skew, perspective);
            result.rest_rotations[i]    = rotation;
            result.rest_translations[i] = translation;
            result.rest_scales[i]       = scale;
        } else {
            // glTF rotation is [x,y,z,w], glm::quat constructor is (w,x,y,z)
            result.rest_rotations[i] = glm::quat(
                node.rotation[3], node.rotation[0],
                node.rotation[1], node.rotation[2]);
            result.rest_translations[i] = glm::vec3(
                node.translation[0], node.translation[1], node.translation[2]);
            result.rest_scales[i] = glm::vec3(
                node.scale[0], node.scale[1], node.scale[2]);
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
        .rest_scales = std::span<const glm::vec3>(rest_scales),
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
    case ChaCha::StageType::Invalid:
        // Never a valid AGI type. Callers must not reach here -- both
        // write_agi_articulations and print_articulations_json skip any
        // stage whose type falls outside [0, kStageTypeCount) before calling
        // this function. Handled explicitly (rather than via fallthrough)
        // so -Wswitch stays exhaustive and documents the sentinel.
        break;
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

// Number of real (emittable) StageType values -- excludes the StageType::Invalid
// sentinel, which analyze() contractually never emits on a real Stage. `occurrence`
// arrays below are sized to this and slots outside [0, kStageTypeCount) are skipped
// rather than indexed, so a stray Invalid (or any future out-of-range value) cannot
// write past the array.
static constexpr int kStageTypeCount = 9;

void write_agi_articulations(
    fx::gltf::Document& doc,
    const std::vector<ChaCha::Articulation>& articulations)
{
    nlohmann::json agi_array = nlohmann::json::array();
    std::vector<std::string> taken_names;

    for (const auto& artic : articulations) {
        const uint32_t node_idx = static_cast<uint32_t>(artic.node);
        std::string source = (node_idx < doc.nodes.size() && !doc.nodes[node_idx].name.empty())
            ? doc.nodes[node_idx].name
            : ("joint_" + std::to_string(artic.node));
        const std::string name = ChaCha::sanitize_articulation_name(source, taken_names);

        // Build stages array. Stage types CAN repeat within one articulation
        // (proper-Euler charts with a repeated axis) -- track occurrence per
        // slot rather than deduplicating by type, and preserve slot order
        // exactly since AGI applies stages in order of appearance.
        nlohmann::json stages_json = nlohmann::json::array();
        int occurrence[kStageTypeCount] = {};

        for (const auto& stage : artic.stages) {
            const int slot = static_cast<int>(stage.type);
            if (slot < 0 || slot >= kStageTypeCount) {
                // StageType::Invalid (or any other out-of-range value) must never
                // reach AGI output -- it has no valid AGI type string and would
                // index past `occurrence`. analyze() contractually never emits
                // this on a real Stage; skip defensively rather than trust that.
                std::cerr << "warning: skipping stage with invalid type " << slot
                          << " on articulation \"" << name << "\"\n";
                continue;
            }
            const int   occur = occurrence[slot]++;
            const float conv  = is_rotation_stage(stage.type) ? rad_to_deg : 1.0f;

            nlohmann::json sj;
            sj["name"]         = ChaCha::stage_name_for(stage.type, occur);
            sj["type"]         = stage_type_to_agi_type(stage.type);
            sj["minimumValue"] = stage.min_value     * conv;
            sj["maximumValue"] = stage.max_value     * conv;
            sj["initialValue"] = stage.initial_value * conv;
            if (stage.max_velocity > 0)
                sj["extras"]["chachaMaximumSpeed"] = stage.max_velocity * conv;
            if (stage.max_acceleration > 0)
                sj["extras"]["chachaMaximumAcceleration"] = stage.max_acceleration * conv;
            stages_json.push_back(sj);
        }

        nlohmann::json entry;
        entry["name"]           = name;
        entry["stages"]         = stages_json;
        entry["pointingVector"] = {artic.pointing_vector.x,
                                   artic.pointing_vector.y,
                                   artic.pointing_vector.z};
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

    // 5. Build bufferView refcount from all consumers: accessors (including
    // their sparse indices/values, which reference bufferViews independently
    // of the accessor's own bufferView), and images.
    std::unordered_map<int32_t, int> bv_refcount;
    for (int32_t i = 0; i < static_cast<int32_t>(doc.accessors.size()); ++i) {
        const auto& acc = doc.accessors[i];
        if (acc.bufferView >= 0)
            bv_refcount[acc.bufferView]++;
        if (!acc.sparse.empty()) {
            bv_refcount[static_cast<int32_t>(acc.sparse.indices.bufferView)]++;
            bv_refcount[static_cast<int32_t>(acc.sparse.values.bufferView)]++;
        }
    }
    for (const auto& image : doc.images) {
        // Images may reference a bufferView (embedded) or a uri (external/
        // data-uri). By convention (see fx::gltf's Image::to_json), a
        // bufferView-backed image has an empty uri; when uri is set,
        // bufferView is not meaningful even though it defaults to 0.
        if (image.uri.empty())
            bv_refcount[image.bufferView]++;
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

    // Remap accessor.bufferView, plus sparse indices/values bufferViews
    for (auto& acc : doc.accessors) {
        remap_bv(acc.bufferView);
        if (!acc.sparse.empty()) {
            auto idx_bv = static_cast<int32_t>(acc.sparse.indices.bufferView);
            remap_bv(idx_bv);
            acc.sparse.indices.bufferView = static_cast<uint32_t>(idx_bv);

            auto val_bv = static_cast<int32_t>(acc.sparse.values.bufferView);
            remap_bv(val_bv);
            acc.sparse.values.bufferView = static_cast<uint32_t>(val_bv);
        }
    }

    // Remap image.bufferView (only meaningful when the image has no uri)
    for (auto& image : doc.images) {
        if (image.uri.empty())
            remap_bv(image.bufferView);
    }

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
    const std::vector<ChaCha::Articulation>& articulations)
{
    nlohmann::json root = nlohmann::json::array();
    std::vector<std::string> taken_names;

    for (const auto& artic : articulations) {
        nlohmann::json entry;

        const uint32_t node_idx = static_cast<uint32_t>(artic.node);
        std::string source = (node_idx < doc.nodes.size() && !doc.nodes[node_idx].name.empty())
            ? doc.nodes[node_idx].name
            : ("joint_" + std::to_string(artic.node));
        const std::string name = ChaCha::sanitize_articulation_name(source, taken_names);

        entry["name"] = name;
        entry["node"] = artic.node;

        nlohmann::json stages_json = nlohmann::json::array();
        int occurrence[kStageTypeCount] = {};

        for (const auto& stage : artic.stages) {
            const int slot = static_cast<int>(stage.type);
            if (slot < 0 || slot >= kStageTypeCount) {
                std::cerr << "warning: skipping stage with invalid type " << slot
                          << " on articulation \"" << name << "\"\n";
                continue;
            }
            const int   occur = occurrence[slot]++;
            const float conv  = is_rotation_stage(stage.type) ? rad_to_deg : 1.0f;

            nlohmann::json sj;
            sj["name"] = ChaCha::stage_name_for(stage.type, occur);
            sj["type"] = stage_type_to_agi_type(stage.type);
            sj["min"] = stage.min_value * conv;
            sj["max"] = stage.max_value * conv;
            sj["initial"] = stage.initial_value * conv;
            if (stage.max_velocity > 0)
                sj["maxSpeed"] = stage.max_velocity * conv;
            if (stage.max_acceleration > 0)
                sj["maxAcceleration"] = stage.max_acceleration * conv;
            stages_json.push_back(sj);
        }
        entry["stages"] = stages_json;
        root.push_back(entry);
    }

    os << root.dump(2) << "\n";
}

} // namespace ChaChaFxGltf
