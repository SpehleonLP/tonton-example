#include "gltf_rintintin_bridge.h"
#include "primitives.h"
#include "fx/gltf.h"

void VisualizeTensors(RintintinCommand const& command, fx::gltf::Document& doc)
{
	auto & skin = command.GetSkin();
	auto & metrics = command.GetMetrics();
	
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
	rintintin_estimate_shapes(volumes.data(), metrics.data(), skin.no_joints);
    
	// Make sure we didn't get any NaN values
	for(const auto& volume : volumes)
	{
		assert(volume.rotation.x == volume.rotation.x);
	}
    
	// Create nodes
	auto begin = doc.nodes.size();
	doc.nodes.resize(doc.nodes.size() + skin.no_joints * 2 + 1);
	doc.nodes.back().name = command.name;
    
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
			
		mesh_nodes[i].extensionsAndExtras["extras"]["volume"] = metrics[i].volume;	
		mesh_nodes[i].extensionsAndExtras["extras"]["covariance"] = (std::array<double, 3>&)metrics[i].covariance;
	}
    
	// Create scene and add root nodes
	doc.scenes.push_back({});
	doc.scenes.back().name = command.name;
	doc.scene = 0;
    
	for(auto i = 0u; i < skeleton_nodes.size(); ++i) {
		doc.scenes.back().nodes.push_back(begin + i);
	}
}
