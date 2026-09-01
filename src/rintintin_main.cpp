#include <iostream>
#include <filesystem>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include "gltf_rintintin_bridge.h"
#include "primitives.h"
#include "lf_rintintin.h"
#include "../modules/fx-gltf/include/fx/gltf.h"
#include "../modules/rintintin/include/rintintin.h"
#include <span>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <limits>

bool OpenFile(fx::gltf::Document & doc, std::filesystem::path const& path);
void SaveFile(fx::gltf::Document & doc, std::filesystem::path const& path);
void ProcessFile(fx::gltf::Document & dst, fx::gltf::Document & src, bool compare_centroids);

enum class RttErrorCode;

void test_4d_vfunc();

namespace LF { 	void to_json(nlohmann::json & json, RinTinTin const& db); }

struct Arguments
{
	std::filesystem::path input;
	std::filesystem::path output;
	std::filesystem::path tensors;
	bool compare_centroids{};
};

std::vector<Arguments> GetArguments(int argc, const char * args[])
{
	Arguments read;
	std::vector<Arguments> r;
	
	int state = 0;

	for(int i = 1; i < argc; ++i)
	{
		if(args[i][0] == '-')
		{
			state = args[i][1];
			
			if(state == 'v')
			{
				read.tensors = (std::string(read.input) += ("-tensors.glb"));
			}
			
			if(state == 'c')
			{
				read.compare_centroids = true;
				state = 0;
			}
			
			continue;
		}
		
		switch(state)
		{
		case 'v':
			read.tensors = args[i];
			break;
		case 'o': 
			read.output = args[i];
			break;
		default:
			if(std::filesystem::exists(read.input))
			{
				r.push_back(read);
			}
			else 
			{				
				read.input = args[i];
				read.output = args[i] + std::string("-output.gltf");
				read.tensors = std::filesystem::path{};
			}
			break;
		}
		
		state = 0;
	}
		
	if(std::filesystem::exists(read.input))
	{
		r.push_back(read);
	}
	
	return r;
}

static void PrintUsage(const char * prog)
{
	std::fprintf(stderr,
		"rintintin-analyze — per-joint volumetric / second-moment analysis for skinned glTF\n"
		"\n"
		"Usage:\n"
		"  %s <input.glb|input.gltf> [options]\n"
		"\n"
		"Writes:\n"
		"  <input>-output.gltf   skeleton + tensor visualization with LF_RINTINTIN extension\n"
		"\n"
		"Options (must FOLLOW the input path they apply to):\n"
		"  -o <path>     override the output path\n"
		"  -v [<path>]   also emit a tensor-visualization .glb\n"
		"                no path = <input>-tensors.glb\n"
		"  -c            solve isolated joints' centroids as well as using their\n"
		"                joint origin, and report both against the oriented box\n"
		"\n"
		"Options are bound to the preceding input path, so they must come after it:\n"
		"  %s in.glb -v tensors.glb        writes tensors.glb\n"
		"  %s -v tensors.glb in.glb        -v is ignored, no tensor file is written\n"
		"\n"
		"One input per invocation. Additional positional arguments are not processed.\n"
		"\n"
		"Environment variables (debug instrumentation):\n"
		"  RTT_PROBE_JOINT=<joint_name>\n"
		"      For each skinned-mesh node, dumps to stderr:\n"
		"        [rtt-ibm]    every joint's parent, IBM-derived origin, and name\n"
		"        [rtt-vprobe] every vertex whose weights touch <joint_name>:\n"
		"                       position, joint slots, weight slots, total wt_to_target\n"
		"      Use to compare two files: redirect stderr and diff.\n"
		"      Example:\n"
		"        RTT_PROBE_JOINT=\"mixamorig:RightHandThumb2\" %s in.glb 2> probe.log\n"
		"\n"
		"Note: a tensor file is only written when the input contains a skinned mesh\n"
		"(a node with both a mesh and a skin). Files with no skin produce no tensors.\n"
		"\n"
		"Exit: 0 on success; errors are logged to stderr.\n",
		prog, prog, prog, prog);
}

int main(int argc,const char * args[])
{
	if(argc < 2
	|| std::strcmp(args[1], "-h") == 0
	|| std::strcmp(args[1], "--help") == 0)
	{
		PrintUsage(args[0]);
		return argc < 2 ? 1 : 0;
	}

	auto argv = GetArguments(argc, args);

	if(argv.empty())
	{
		std::fprintf(stderr, "%s: no valid input files. Use -h for usage.\n", args[0]);
		return 1;
	}

	for(auto & arg : argv)
	{
		try
		{
			fx::gltf::Document doc;
			if(!OpenFile(doc, arg.input))
				continue;

			fx::gltf::Document dst;
			
			auto _now = std::chrono::high_resolution_clock::now();
			ProcessFile(dst, doc, arg.compare_centroids);
			auto time_taken = std::chrono::high_resolution_clock::now() - _now;
			auto duration_ms = std::chrono::duration<double, std::milli>(time_taken);
			std::cout << "processed in: " << duration_ms.count() << "ms\n";
			
			SaveFile(doc, arg.output);
			SaveFile(dst, arg.tensors);
		}
		catch(std::exception & e)
		{
			std::cerr << arg.input << ": " << e.what() << ".\n";
		}
		catch(RttErrorCode ec)
		{
			std::cerr << arg.input << ": " << rintintin_get_error_string(int(ec)) << "\n";
		}
	}

	return 0;
}

bool OpenFile(fx::gltf::Document & doc, std::filesystem::path const& path)
{
	if(std::filesystem::exists(path) == false)
	{
		throw std::runtime_error("no such file or directory.");
		return false;
	}
	
	std::string extension = std::string(path.extension());
	std::transform(extension.begin(), extension.end(), extension.begin(),
		[](unsigned char c){ return std::tolower(c); });
		
	if(extension == ".gltf")
		doc = fx::gltf::LoadFromText(path);
	else if(extension == ".glb")
		doc = fx::gltf::LoadFromBinary(path, fx::gltf::ReadQuotas{~0u, ~0u, ~0u});
	else
	{
		throw std::runtime_error("file is not a gltf file");
	}
	
	return doc.buffers.size();
}

void SaveFile(fx::gltf::Document & doc, std::filesystem::path const& path)
{
	auto IsBinary = [](std::filesystem::path const& path)
	{
		auto extension = std::string(path.extension());
		
		for(auto & c : extension) c = tolower(c);
		
		if(extension == ".glb")
			return 1;
		if(extension == ".gltf")
			return 0;
			
		return -1;
	};

	int is_binary = IsBinary(path);
	if(is_binary == -1) return;
	
	if(doc.scenes.size())
	{
		for(auto & buf : doc.buffers)
		{
			buf.byteLength = buf.data.size();
				
			if(is_binary)
			{
				buf.uri.clear();
			}
			else
			{
				buf.uri = std::filesystem::path(path).filename().replace_extension(".bin");
			}
		}
					
		fx::gltf::Save(doc, path, is_binary);
		std::cout << "saved: " << path << "\n";
	}
};


void VisualizeInertia(fx::gltf::Document & doc, std::string const& name, rintintin_skin & skin, rintintin_metrics * metrics, glm::mat4 const& world);

namespace {

glm::mat4 LocalMatrix(fx::gltf::Node const& n)
{
	// fx-gltf defaults `matrix` to identity, so a non-identity matrix means the
	// file used the matrix form; otherwise compose TRS.
	if(n.matrix != fx::gltf::defaults::IdentityMatrix)
		return glm::make_mat4(n.matrix.data());

	glm::mat4 t = glm::translate(glm::mat4(1.0f),
		glm::vec3(n.translation[0], n.translation[1], n.translation[2]));
	glm::mat4 r = glm::mat4_cast(
		glm::quat(n.rotation[3], n.rotation[0], n.rotation[1], n.rotation[2]));
	glm::mat4 s = glm::scale(glm::mat4(1.0f),
		glm::vec3(n.scale[0], n.scale[1], n.scale[2]));

	return t * r * s;
}

// Tensors come out of rintintin in the mesh node's local space, but the object
// is drawn under that node's world transform -- without this the visualization
// sits at the origin, rotated by whatever the exporter's up-axis fix was, and
// can't be eyeballed against the source.
glm::mat4 WorldMatrix(fx::gltf::Document const& doc, uint32_t node)
{
	std::vector<int32_t> parent(doc.nodes.size(), -1);
	for(uint32_t i = 0; i < doc.nodes.size(); ++i)
		for(auto c : doc.nodes[i].children)
			if(uint32_t(c) < parent.size())
				parent[c] = int32_t(i);

	glm::mat4 m(1.0f);
	for(int32_t j = int32_t(node), guard = 0;
	    j >= 0 && guard <= int32_t(doc.nodes.size());
	    j = parent[j], ++guard)
	{
		m = LocalMatrix(doc.nodes[j]) * m;
	}

	return m;
}

// Seed for a synthetic joint. These meshes are non-manifold, so the volume
// integral is origin-dependent -- an arbitrary joint at the mesh origin makes
// the measurement depend on where the artist happened to place that origin, and
// identical geometry at different offsets then measures differently. The bounds
// centre travels with the geometry, so congruent parts agree.
rintintin_vec3 MeshBoundsCentre(fx::gltf::Document const& doc, fx::gltf::Mesh const& mesh)
{
	glm::dvec3 lo(std::numeric_limits<double>::max());
	glm::dvec3 hi(std::numeric_limits<double>::lowest());
	bool any = false;

	for(auto const& prim : mesh.primitives)
	{
		auto it = prim.attributes.find("POSITION");
		if(it == prim.attributes.end()) continue;
		if(uint32_t(it->second) >= doc.accessors.size()) continue;

		auto const& acc = doc.accessors[it->second];
		if(acc.min.size() < 3 || acc.max.size() < 3) continue;

		for(int k = 0; k < 3; ++k)
		{
			lo[k] = std::min(lo[k], double(acc.min[k]));
			hi[k] = std::max(hi[k], double(acc.max[k]));
		}
		any = true;
	}

	if(!any) return {0, 0, 0};

	glm::dvec3 c = (lo + hi) * 0.5;
	return {c.x, c.y, c.z};
}

// Score a candidate centroid against the joint's oriented bounding box, which is
// derived from vertex extents (argmax-cluster PCA) rather than from the mass
// distribution -- so it is an independent opinion about where the body is.
// Returns the largest per-axis overshoot in OBB half-extents: <= 1 is inside.
double ObbOvershoot(rintintin_inertia_estimation const& obb, rintintin_vec3 const& c)
{
	glm::dquat q(obb.rotation.w, obb.rotation.x, obb.rotation.y, obb.rotation.z);
	glm::dvec3 d(c.x - obb.translation.x, c.y - obb.translation.y, c.z - obb.translation.z);
	glm::dvec3 local = glm::inverse(q) * d;
	glm::dvec3 half(obb.scale.x, obb.scale.y, obb.scale.z);

	double worst = 0;
	for(int k = 0; k < 3; ++k)
	{
		double h = std::abs(half[k]);
		if(h <= 0) continue;
		worst = std::max(worst, std::abs(local[k]) / h);
	}
	return worst;
}

} // anonymous namespace

void ProcessFile(fx::gltf::Document & dst, fx::gltf::Document & src, bool compare_centroids)
{
	using attrib_t = decltype(RintintinMeshData::attributes);	
		
	for(auto i = 0u; i < src.nodes.size(); ++i)
	{
		if(uint32_t(src.nodes[i].mesh) >= src.meshes.size())
			continue;
			
		// A node with a mesh but no skin still has a tensor worth measuring --
		// give it a synthetic one-joint skin so a static prop (a ball, a crystal)
		// reports one whole-body tensor instead of being skipped silently.
		const bool has_skin = uint32_t(src.nodes[i].skin) < src.skins.size();
			
		auto & mesh = src.meshes[src.nodes[i].mesh];
		std::vector<rintintin_mesh> meshes;
		std::vector<attrib_t> attributes;
		
		meshes.reserve(mesh.primitives.size());
		
		for(auto & primitive : mesh.primitives)
		{
			ThinShellInfo thin_shell{};
			auto m = createRintintinMeshFromPrimitive(src, primitive, &thin_shell, !has_skin);
			
			if(thin_shell.reason != ThinShellReason::None)
				std::cerr << "[rtt-shell] " << src.nodes[i].name
					<< " prim " << meshes.size() << ": "
					<< (thin_shell.thin ? "thin shell" : "solid") << " ("
					<< ToString(thin_shell.reason) << ")"
					<< (thin_shell.thin ? ", thickness " + std::to_string(thin_shell.thickness) + "m" : "")
					<< "\n";
			
			meshes.push_back(std::move(m.mesh));
			attributes.push_back(std::move(m.attributes));
		}
				
		auto skin = has_skin
			? createRintintinSkinFromSkin(src, src.nodes[i].skin)
			: createSyntheticSingleJointSkin(src.nodes[i].name, MeshBoundsCentre(src, mesh));
		std::vector<rintintin_metrics> metrics(skin.skin.no_joints);

		// [rtt-vprobe] Dump per-vertex weights touching a target joint.
		// Enabled by env RTT_PROBE_JOINT="mixamorig:RightShoulder" (or any joint name).
		if (const char * tgt = std::getenv("RTT_PROBE_JOINT"))
		{
			int target = -1;
			for (uint32_t j = 0; j < skin.skin.no_joints; ++j)
				if (skin.skin.bone_names[j] && std::strcmp(skin.skin.bone_names[j], tgt) == 0)
					{ target = int(j); break; }
			std::fprintf(stderr, "[rtt-vprobe] node=%s target='%s' idx=%d (mesh=%d, prims=%zu)\n",
				src.nodes[i].name.c_str(), tgt, target,
				src.nodes[i].mesh, meshes.size());
			if (target >= 0)
			{
				for (size_t mi = 0; mi < meshes.size(); ++mi)
				{
					auto const& M  = meshes[mi];
					auto const* JA = (rintintin_attrib const*)M.joints_user_data;
					auto const* WA = (rintintin_attrib const*)M.weights_user_data;
					auto const* PA = (rintintin_attrib const*)M.position_user_data;
					size_t hits = 0;
					for (uint32_t v = 0; v < M.no_verts; ++v)
					{
						int32_t J[4] = {0,0,0,0};
						double  W[4] = {0,0,0,0};
						double  P[4] = {0,0,0,0};
						rintintin_read_attrib_generic_i(J, v, JA);
						rintintin_read_attrib_generic_f(W, v, WA);
						rintintin_read_attrib_generic_f(P, v, PA);
						double wt = 0;
						for (int s = 0; s < 4; ++s)
							if (J[s] == target) wt += W[s];
						if (wt > 0)
						{
							++hits;
							std::fprintf(stderr,
								"[rtt-vprobe] prim=%zu v=%u P=(%.4f,%.4f,%.4f) J=(%d,%d,%d,%d) W=(%.4f,%.4f,%.4f,%.4f) wt_to_target=%.4f\n",
								mi, v, P[0], P[1], P[2],
								J[0], J[1], J[2], J[3],
								W[0], W[1], W[2], W[3], wt);
						}
					}
					std::fprintf(stderr, "[rtt-vprobe] prim=%zu vertices_touching_target=%zu / %zu\n",
						mi, hits, (size_t)M.no_verts);
				}
			}
		}
		
		std::vector<uint8_t> scratch_space;
		rintintin_process_command cmd{};
		
		// single threaded
		cmd.meshes = meshes.data();
		cmd.no_meshes = meshes.size();
		cmd.skin = skin.skin;
		cmd.results = metrics.data();  
		cmd.name = src.nodes[i].name.c_str();        
		
		
		int no_threads = 1;
		cmd.max_threads = no_threads;
		cmd.scratch_space_byte_length = rintintin_get_scratch_space_size(skin.skin.no_joints, no_threads);
		scratch_space.resize(cmd.scratch_space_byte_length);
		cmd.scratch_space = scratch_space.data();
		
		
		auto ec = rintintin_begin(&cmd);
		if(ec < 0) throw RttErrorCode(ec);
		
		ec = rintintin_read_mesh(&cmd, 0, no_threads);
		if(ec < 0) throw RttErrorCode(ec);
			
		ec = rintintin_end(&cmd);
		if(ec < 0) throw RttErrorCode(ec);

		VisualizeInertia(dst, src.nodes[i].name, cmd.skin, cmd.results, WorldMatrix(src, i));
		
		std::vector<rintintin_inertia_estimation> bounds;	
	
		bounds.resize(cmd.skin.no_joints);
		
		rintintin_bounding_box_command b_cmd{};
		b_cmd.meshes = cmd.meshes;
		b_cmd.metrics = cmd.results;
		
		b_cmd.result = bounds.data();
		
		b_cmd.no_joints = cmd.skin.no_joints;
		b_cmd.no_meshes = cmd.no_meshes;
		b_cmd.result_byte_length = sizeof(bounds[0]) * bounds.size();
		
		// Scratch enables argmax-cluster PCA; without it the OBB rotation comes
		// from second_moment, which is skinning-weighted and so disagrees with
		// the argmax extents pass. Matches gltfRepackager's bridge.
		std::vector<uint8_t> obb_scratch(rintintin_oriented_bounding_boxes_scratch_size(cmd.skin.no_joints));
		b_cmd.scratch_space = obb_scratch.data();
		b_cmd.scratch_space_byte_length = uint32_t(obb_scratch.size());
		
		ec = rintintin_oriented_bounding_boxes(&b_cmd);
		if(ec < 0) throw RttErrorCode(ec);
		
		if(compare_centroids)
		{
			// The mesh has already been read; only the solve is repeated. end()
			// accumulates into results, so they must be re-zeroed first.
			std::vector<rintintin_metrics> solved(skin.skin.no_joints);
			cmd.results = solved.data();
			cmd.flags   = RINTINTIN_SOLVE_CENTROIDS;
			
			auto ec2 = rintintin_end(&cmd);
			cmd.results = metrics.data();
			cmd.flags   = 0;
			
			if(ec2 < 0)
			{
				std::cerr << "[rtt-solve] " << src.nodes[i].name << ": second solve failed: "
					<< rintintin_get_error_string(int(ec2)) << "\n";
			}
			else for(auto j = 0u; j < skin.skin.no_joints; ++j)
			{
				bool isolated = skin.skin.parents[j] < 0;
				for(auto k = 0u; k < skin.skin.no_joints && isolated; ++k)
					if(skin.skin.parents[k] == int(j)) isolated = false;
				if(!isolated) continue;
				
				auto const& a = metrics[j];
				auto const& b = solved[j];
				std::cerr << "[rtt-solve] " << src.nodes[i].name << " joint "
					<< (skin.skin.bone_names ? skin.skin.bone_names[j] : "?") << "\n"
					<< "    joint-origin  vol " << a.volume
					<< "  centroid (" << a.centroid.x << ", " << a.centroid.y << ", " << a.centroid.z << ")"
					<< "  obb " << ObbOvershoot(bounds[j], a.centroid) << "\n"
					<< "    solved-centre vol " << b.volume
					<< "  centroid (" << b.centroid.x << ", " << b.centroid.y << ", " << b.centroid.z << ")"
					<< "  obb " << ObbOvershoot(bounds[j], b.centroid) << "\n";
			}
		}
		
		LF::RinTinTin extension;
		
		extension.metrics.resize(cmd.skin.no_joints);
		extension.eigenDecompositions.resize(cmd.skin.no_joints);
		extension.orientedBoundedBoxes.resize(cmd.skin.no_joints);
		
		for(auto i = 0u; i < cmd.skin.no_joints; ++i)
		{
			extension.metrics[i] = LF::Factory(cmd.results[i]);
			extension.eigenDecompositions[i] = LF::MakeEigen(cmd.results[i]);
			extension.orientedBoundedBoxes[i] = LF::Factory(bounds[i]); 
		}
		
		src.nodes[i].extensionsAndExtras["extras"]["LF_RINTINTIN"] = extension;
	}
}

void VisualizeInertia(fx::gltf::Document & doc, std::string const& name, rintintin_skin & skin,  rintintin_metrics * metrics, glm::mat4 const& world)
{
	if(doc.buffers.empty())
	{
		// gen icosphere is scaled such that the volume of the icosphere given subdivisions equals the volume of the sphere of the radius
		// rather than being a straightforward scale
		// so we need the proper scales to save the GLTF file.
		constexpr double sphere_max_scale = 1.0461329221725464;
		constexpr double cube_max_scale = 0.5;
		constexpr double cylinder_max_scale = 1.023326754570007*0.5;
		
		constexpr vec3 sphere_max = {sphere_max_scale, sphere_max_scale, sphere_max_scale};
		constexpr vec3 cube_max = {cube_max_scale, cube_max_scale, cube_max_scale};
		constexpr vec3 cylinder_max = {cylinder_max_scale, cylinder_max_scale, 0.5};
	
		AddPrimitive(doc, "icosphere", generateIcosphere(1, 1), sphere_max);
		AddPrimitive(doc, "cylinder", generateCylinder(0.5, 1, 12), cylinder_max);
		AddPrimitive(doc, "cube", generateBox(1, 1, 1), cube_max);
		AddPrimitive(doc, "cone", generateCone(0.5, 1, 12), cylinder_max);
	}
	
	// Ensure metrics span size matches expected joint count
	
	std::vector<rintintin_inertia_estimation> volumes(skin.no_joints);
	rintintin_estimate_shapes(volumes.data(), metrics, skin.no_joints);
    
	// Make sure we didn't get any NaN values
	for(const auto& volume : volumes)
	{
		assert(volume.rotation.x == volume.rotation.x);
	}
    
	// Create nodes
	auto begin = doc.nodes.size();
	doc.nodes.resize(doc.nodes.size() + skin.no_joints * 2 + 1);
	auto container = doc.nodes.size() - 1;
	doc.nodes[container].name = name;
	std::memcpy(doc.nodes[container].matrix.data(), glm::value_ptr(world), sizeof(float) * 16);
    
	// Create aligned spans starting from the skeleton nodes
	auto skeleton_nodes = std::span{doc.nodes}.subspan(begin, skin.no_joints);
	auto mesh_nodes = std::span{doc.nodes}.subspan(begin + skin.no_joints, skin.no_joints);
	auto joint_translations = std::span{skin.joint_translation_mesh_space, skin.no_joints};
	auto parents = std::span{skin.parents, skin.no_joints};
	auto bone_names = skin.bone_names ? std::span{skin.bone_names, skin.no_joints} : std::span<const char* const>{};
    
	// Add skeleton to debug skin creation
	for(auto i = 0u; i < skeleton_nodes.size(); ++i) {
		const auto parent_idx = parents[i];
		const auto& joint_pos = joint_translations[i];
		
		// Set bone name if available
		if(!bone_names.empty() && bone_names[i])
			skeleton_nodes[i].name = bone_names[i];
			
		skeleton_nodes[i].translation = {
			static_cast<float>(joint_pos.x),
			static_cast<float>(joint_pos.y),
			static_cast<float>(joint_pos.z)
		};
			
		// Handle parent-child relationships
		if(parent_idx < 0) {
			doc.nodes[container].children.push_back(begin + i);
		} else {
			const auto& parent_pos = joint_translations[parent_idx];
			skeleton_nodes[parent_idx].children.push_back(begin + i);
			
			skeleton_nodes[i].translation[0] -= static_cast<float>(parent_pos.x);
			skeleton_nodes[i].translation[1] -= static_cast<float>(parent_pos.y);
			skeleton_nodes[i].translation[2] -= static_cast<float>(parent_pos.z);
		}
      
		// Create mesh node for inertia tensor visualization
		skeleton_nodes[i].children.push_back(begin + skin.no_joints + i);
        
		mesh_nodes[i].mesh = 0;//volumes[i].type;
		mesh_nodes[i].name = skeleton_nodes[i].name + ".tensor";
		
		const auto& centroid = metrics[i].centroid;
		const auto& rotation = volumes[i].rotation;
		const auto& scale = volumes[i].scale;
		
		if(centroid.x == 0 || std::isnormal(float(centroid.x)))
			mesh_nodes[i].translation = {
				static_cast<float>(centroid.x - joint_pos.x), 
				static_cast<float>(centroid.y - joint_pos.y), 
				static_cast<float>(centroid.z - joint_pos.z)
			};
		
		if(rotation.w == rotation.w)
			mesh_nodes[i].rotation = {
				static_cast<float>(rotation.x), 
				static_cast<float>(rotation.y), 
				static_cast<float>(rotation.z), 
				static_cast<float>(rotation.w)
			};
		
		if(scale.x == scale.x)
			mesh_nodes[i].scale = {
				static_cast<float>(scale.x), 
				static_cast<float>(scale.y), 
				static_cast<float>(scale.z)
			};
	}
    
	// One scene holding every mesh's container. A scene per call left a
	// multi-mesh model showing only whichever one `doc.scene` pointed at.
	if(doc.scenes.empty())
	{
		doc.scenes.push_back({});
		doc.scenes.back().name = "tensors";
	}
	doc.scene = 0;
    
	doc.scenes.front().nodes.push_back(container);
}
