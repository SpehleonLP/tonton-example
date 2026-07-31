#ifndef CHACHA_FXGLTF_BRIDGE_H
#define CHACHA_FXGLTF_BRIDGE_H

#include "chacha_types.h"
#include "chacha_stage.h"
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace fx { namespace gltf { struct Document; }}

namespace ChaChaFxGltf {

struct ExtractedAnimations {
    // Owned storage that spans point into
    std::vector<std::vector<float>> time_storage;
    std::vector<std::vector<float>> value_storage;

    // Channels referencing the storage above
    std::vector<ChaCha::AnimationChannel> channels;

    // Owned animation-name storage. Must be fully populated before any
    // string_view into it is taken (see `animations` below), or
    // reallocation invalidates every earlier view.
    std::vector<std::string> name_storage;

    // Animation descriptors referencing name_storage. Parallel to
    // doc.animations (indexed by AnimationChannel::animation).
    std::vector<ChaCha::Animation> animations;

    // `channels` and `animations` hold views into `time_storage`/`value_storage`
    // and `name_storage` respectively, all siblings of this same struct.
    // Moving is safe (the moved-from vectors' storage pointers transfer along
    // with the struct, so every view stays valid). Copying is NOT: a copy's
    // views would keep pointing at the ORIGINAL's storage, not its own copied
    // vectors, which is silently correct until the original is destroyed and
    // then a use-after-free. Delete copy so that's a compile error, not a
    // latent bug waiting for someone to write `auto b = a;`.
    ExtractedAnimations() = default;
    ExtractedAnimations(const ExtractedAnimations&) = delete;
    ExtractedAnimations& operator=(const ExtractedAnimations&) = delete;
    ExtractedAnimations(ExtractedAnimations&&) = default;
    ExtractedAnimations& operator=(ExtractedAnimations&&) = default;
};

struct ExtractedSkeleton {
    std::vector<int> parents;
    std::vector<glm::quat> rest_rotations;
    std::vector<glm::vec3> rest_translations;
    std::vector<glm::vec3> rest_scales;

    ChaCha::Skeleton as_skeleton() const;
};

// Extract animation channels from all animations in the document.
// Channels reference nodes by glTF node index, and animations by index
// into the returned ExtractedAnimations::animations array.
ExtractedAnimations extract_animation_channels(const fx::gltf::Document& doc);

// Extract skeleton in glTF node space: arrays are sized to doc.nodes.size()
// and indexed directly by node index. AGI articulations are per node and
// animation channels target nodes, so this is node space, not skin-joint
// space.
ExtractedSkeleton extract_skeleton(const fx::gltf::Document& doc);

// Write articulation results as AGI_articulations extension on the document.
// Converts radians to degrees for rotation stages at the boundary.
void write_agi_articulations(
    fx::gltf::Document& doc,
    const std::vector<ChaCha::Articulation>& articulations);

// Check if the document already has AGI_articulations extension.
bool has_agi_articulations(const fx::gltf::Document& doc);

// Print articulations as JSON to an output stream (for --stdout mode).
// Resolves joint names from the document's node names.
void print_articulations_json(
    std::ostream& os,
    const fx::gltf::Document& doc,
    const std::vector<ChaCha::Articulation>& articulations);

} // namespace ChaChaFxGltf

#endif // CHACHA_FXGLTF_BRIDGE_H
