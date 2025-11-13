#include <iostream>
#include <filesystem>
#include "gltf_rintintin_bridge.h"
#include "primitives.h"
#include "lf_rintintin.h"
#include "../modules/fx-gltf/include/fx/gltf.h"
#include "../modules/rintintin/include/rintintin.h"
#include <span>

#include <glm/gtc/quaternion.hpp>

bool OpenFile(fx::gltf::Document & doc, std::filesystem::path const& path);
void SaveFile(fx::gltf::Document & doc, std::filesystem::path const& path);
void ProcessFile(fx::gltf::Document & dst, fx::gltf::Document & src);

enum class RttErrorCode;

void test_4d_vfunc();

namespace LF { 	void to_json(nlohmann::json & json, RinTinTin const& db); }

struct Arguments
{
	std::filesystem::path input;
	std::filesystem::path output;
	std::filesystem::path tensors;
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

int main(int argc,const char * args[])
{
	auto argv = GetArguments(argc, args);
	
	for(auto & arg : argv)
	{
		try
		{
			fx::gltf::Document doc;
			if(!OpenFile(doc, arg.input))
				continue;

			fx::gltf::Document dst;
			
			auto _now = std::chrono::high_resolution_clock::now();
			ProcessFile(dst, doc);
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
		doc = fx::gltf::LoadFromBinary(path);
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
				buf.uri = std::filesystem::path(path).replace_extension(".bin");
			}
		}
					
		fx::gltf::Save(doc, path, is_binary);
		std::cout << "saved: " << path << "\n";
	}
};


void VisualizeInertia(fx::gltf::Document & doc, std::string const& name, rintintin_skin & skin, rintintin_metrics * metrics);

void ProcessFile(fx::gltf::Document & dst, fx::gltf::Document & src)
{
	using attrib_t = decltype(RintintinMeshData::attributes);	
		
	for(auto i = 0u; i < src.nodes.size(); ++i)
	{
		if(uint32_t(src.nodes[i].skin) >= src.skins.size()
		|| uint32_t(src.nodes[i].mesh) >= src.meshes.size())
			continue;
			
	//	if(src.nodes[i].mesh != 3) continue; 
			
		auto & mesh = src.meshes[src.nodes[i].mesh];
		std::vector<rintintin_mesh> meshes;
		std::vector<attrib_t> attributes;
		
		meshes.reserve(mesh.primitives.size());
		
		for(auto & primitive : mesh.primitives)
		{
			bool is_alpha_card{};
			auto m = createRintintinMeshFromPrimitive(src, primitive, &is_alpha_card);
			
		//	if(!is_alpha_card)
			{
				meshes.push_back(std::move(m.mesh));
				attributes.push_back(std::move(m.attributes));
			}
		}
				
		auto skin = createRintintinSkinFromSkin(src, src.nodes[i].skin);
		std::vector<rintintin_metrics> metrics(skin.skin.no_joints);
		
		std::vector<uint8_t> scratch_space;
		rintintin_process_command cmd;
		
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

		VisualizeInertia(dst, src.nodes[i].name, cmd.skin, cmd.results);
		
		std::vector<rintintin_inertia_estimation> bounds;	
	
		bounds.resize(cmd.skin.no_joints);
		
		rintintin_bounding_box_command b_cmd;
		b_cmd.meshes = cmd.meshes;
		b_cmd.metrics = cmd.results;
		
		b_cmd.result = bounds.data();
		
		b_cmd.no_joints = cmd.skin.no_joints;
		b_cmd.no_meshes = cmd.no_meshes;
		b_cmd.result_byte_length = sizeof(bounds[0]) * bounds.size();
		
		ec = rintintin_oriented_bounding_boxes(&b_cmd);
		if(ec < 0) throw RttErrorCode(ec);
		
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

void VisualizeInertia(fx::gltf::Document & doc, std::string const& name, rintintin_skin & skin,  rintintin_metrics * metrics)
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
	doc.nodes.back().name = name;
    
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
			doc.nodes.back().children.push_back(begin + i);
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
    
	// Create scene and add root nodes
	doc.scenes.push_back({});
	doc.scenes.back().name = name;
	doc.scene = 0;
    
	for(auto i = 0u; i < skeleton_nodes.size(); ++i) {
		doc.scenes.back().nodes.push_back(begin + i);
	}
}
