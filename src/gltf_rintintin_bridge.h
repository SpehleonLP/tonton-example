#ifndef GLTF_RINTINTIN_BRIDGE_H
#define GLTF_RINTINTIN_BRIDGE_H
#include "fx/gltf.h"
#include "rintintin.h"
#include "lf_rintintin.h"
#include <future>
#include <glm/fwd.hpp>
#include <memory>
#include <span>
#include <vector>

namespace fx { namespace gltf { struct Document; struct Primitive; }}
enum class RttErrorCode;

// Why a primitive was classified as a thin shell, so callers can report which
// rule fired -- an explicit marker and the alphaMode heuristic mean different
// things when a model's tensors come out wrong.
enum class ThinShellReason : uint8_t {
	None,
	MaterialExtras,      // material.extras.LF_THIN_SHELL == true
	MaterialName,        // material name ends in _shell / _card
	AlphaHeuristic,      // alphaMode != Opaque && doubleSided
	// Explicit "this is solid" -- vetoes the heuristic. alphaMode BLEND +
	// doubleSided is often an authoring workaround rather than a statement
	// about geometry, so an author must be able to say no.
	MaterialExtrasSolid, // material.extras.LF_THIN_SHELL == false
	MaterialNameSolid,   // material name ends in _solid
};

struct ThinShellInfo {
	bool             thin   = false;
	ThinShellReason  reason = ThinShellReason::None;
	double           thickness = 0.04 / 1000;  // metres; overridden by LF_SHELL_THICKNESS
};

const char * ToString(ThinShellReason reason);

// Create rintintin mesh from glTF primitive
struct RintintinMeshData {
    rintintin_mesh mesh;
    // mesh has raw pointers to these so they need to be on the heap so they don't move. 
    std::unique_ptr<std::array<rintintin_attrib, 3>> attributes;
};

// `unskinned` builds the mesh from POSITION alone, synthesising joint 0 at
// weight 1.0 for every vertex -- pairs with createSyntheticSingleJointSkin so a
// static mesh (a ball, a crystal) yields a single whole-body tensor.
RintintinMeshData createRintintinMeshFromPrimitive(
		const fx::gltf::Document& document, 
		const fx::gltf::Primitive& primitive,
		ThinShellInfo * thin_shell = nullptr,
		bool unskinned = false);
    
struct RintintinSkinData
{
	rintintin_skin skin;
	std::unique_ptr<const char*[]> names;
	std::unique_ptr<rintintin_vec3[]> origins;
	std::unique_ptr<int[]> parents;
	// Heap-stored so `names[0]` stays valid across a move (an SSO string would not).
	std::unique_ptr<std::string> synthetic_name;
};

// A one-joint skin for nodes with no glTF skin. `origin` is the joint position
// in mesh space; there is no artist placement to respect here, so the caller
// picks whatever seed best describes the geometry.
RintintinSkinData createSyntheticSingleJointSkin(std::string const& name,
                                                 rintintin_vec3 origin = {0, 0, 0});

RintintinSkinData createRintintinSkinFromSkin(
		const fx::gltf::Document& document, 
		int skinnedMeshIdx, std::span<glm::quat> rotations = {});
   
namespace RTT
{
// Helper structure to hold glTF vertex attribute data for rintintin processing
struct GLTFAttributeData {
    const fx::gltf::Document* document;
    const fx::gltf::Accessor* accessor;
    const fx::gltf::BufferView* bufferView;
    const uint8_t* data;
    
    GLTFAttributeData(const fx::gltf::Document* doc, int32_t accessorIdx);
    
    bool isValid() const;
};
}

struct RintintinCommand
{
using attrib_t = decltype(RintintinMeshData::attributes);
	static std::shared_ptr<RintintinCommand> Factory(fx::gltf::Document const& doc, size_t index, bool get_obb, std::span<glm::quat> rotations_out = {});
	
	std::vector<rintintin_metrics> const& GetMetrics() const;
	std::vector<rintintin_inertia_estimation> const& GetBoundingBoxes() const;
	LF::RinTinTin MakeExtension(bool eigenDecomposition) const;
	
	rintintin_process_command GetMeshCommand();
	rintintin_skin const& GetSkin() const { return skin.skin; };
	
	const std::string		 name;
	
private:
	RintintinCommand(fx::gltf::Document const& doc, size_t index, std::span<glm::quat> rotations_out);
	
	struct Result
	{
		std::vector<rintintin_metrics> metrics;
		std::vector<rintintin_inertia_estimation> bounds;	
		
		inline bool empty() const { return metrics.empty(); }
	};
	
	std::vector<rintintin_mesh> meshes;
	std::vector<attrib_t> attributes;
	RintintinSkinData     skin;
	mutable std::future<Result> _future;
	mutable Result _result;
	int error_code{0};
};

namespace LF
{
RinTinTin::Metrics Factory(rintintin_metrics const& it);
RinTinTin::Eigen MakeEigen(rintintin_metrics const& it);
Transform Factory(rintintin_inertia_estimation const& it);
}


#endif // GLTF_RINTINTIN_BRIDGE_H
