#ifndef CHACHA_FXGLTF_BRIDGE_H
#define CHACHA_FXGLTF_BRIDGE_H

#include "chacha_types.h"
#include "chacha_stage.h"
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <unordered_set>
#include <vector>

namespace fx { namespace gltf { struct Document; }}

namespace ChaChaFxGltf {

struct ExtractedAnimations {
    // Owned storage that spans point into
    std::vector<std::vector<float>> time_storage;
    std::vector<std::vector<float>> value_storage;

    // Channels referencing the storage above
    std::vector<ChaCha::AnimationChannel> channels;

    // Indices of animations with AGI_ prefix (for later removal)
    std::unordered_set<uint32_t> agi_animation_indices;
};

struct ExtractedSkeleton {
    std::vector<int> parents;
    std::vector<glm::quat> rest_rotations;
    std::vector<glm::vec3> rest_translations;

    // Maps joint index (0..N-1) to glTF node index
    std::vector<uint32_t> joint_nodes;

    ChaCha::Skeleton as_skeleton() const;
};

// Extract animation channels from all animations in the document.
// Channels reference nodes by glTF node index (not joint index).
// Caller must remap to joint indices before calling ChaCha::analyze().
ExtractedAnimations extract_animation_channels(const fx::gltf::Document& doc);

// Extract skeleton from a skin. Returns joint hierarchy and rest poses.
ExtractedSkeleton extract_skeleton(const fx::gltf::Document& doc, int skin_index);

// Write articulation results as AGI_articulations extension on the document.
// Converts radians to degrees at the boundary.
void write_agi_articulations(
    fx::gltf::Document& doc,
    const std::vector<ChaCha::Articulation>& articulations,
    const std::vector<uint32_t>& joint_nodes);

// Remove animations whose name starts with "AGI_" (case-insensitive).
// Also removes orphaned accessors and bufferViews via reference counting,
// with full index remapping across the document.
void remove_agi_animations(fx::gltf::Document& doc);

// Check if the document already has AGI_articulations extension.
bool has_agi_articulations(const fx::gltf::Document& doc);

// Print articulations as JSON to an output stream (for --stdout mode).
// Resolves joint names from the document's node names.
void print_articulations_json(
    std::ostream& os,
    const fx::gltf::Document& doc,
    const std::vector<ChaCha::Articulation>& articulations,
    const std::vector<uint32_t>& joint_nodes);

} // namespace ChaChaFxGltf

#endif // CHACHA_FXGLTF_BRIDGE_H
