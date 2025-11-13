#include "gltf_rintintin_bridge.h"
#include "fx/gltf.h"
#include <glm/mat4x4.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <span>

using GLTFAttributeData = RTT::GLTFAttributeData;
// Helper structure to hold glTF vertex attribute data for rintintin processing

GLTFAttributeData::GLTFAttributeData(const fx::gltf::Document* doc, int32_t accessorIdx) 
	: document(doc), accessor(nullptr), bufferView(nullptr), data(nullptr) {
	if (accessorIdx >= 0 && accessorIdx < static_cast<int32_t>(doc->accessors.size())) {
		accessor = &doc->accessors[accessorIdx];
		if (accessor->bufferView >= 0 && accessor->bufferView < static_cast<int32_t>(doc->bufferViews.size())) {
			bufferView = &doc->bufferViews[accessor->bufferView];
			if (bufferView->buffer >= 0 && bufferView->buffer < static_cast<int32_t>(doc->buffers.size())) {
				const auto& buffer = doc->buffers[bufferView->buffer];
				data = buffer.data.data() + bufferView->byteOffset + accessor->byteOffset;
			}
		}
	}
}

bool GLTFAttributeData::isValid() const {
	return accessor && bufferView && data;
}

// Convert glTF component type to rintintin type
rintintin_type gltfToRinTinTinType(fx::gltf::Accessor::ComponentType componentType) {
    switch (componentType) {
        case fx::gltf::Accessor::ComponentType::Byte: return RINTINTIN_TYPE_BYTE;
        case fx::gltf::Accessor::ComponentType::UnsignedByte: return RINTINTIN_TYPE_UNSIGNED_BYTE;
        case fx::gltf::Accessor::ComponentType::Short: return RINTINTIN_TYPE_SHORT;
        case fx::gltf::Accessor::ComponentType::UnsignedShort: return RINTINTIN_TYPE_UNSIGNED_SHORT;
       // case fx::gltf::Accessor::ComponentType::Int: return RINTINTIN_TYPE_INT;
        case fx::gltf::Accessor::ComponentType::UnsignedInt: return RINTINTIN_TYPE_UNSIGNED_INT;
        case fx::gltf::Accessor::ComponentType::Float: return RINTINTIN_TYPE_FLOAT;
        default: throw std::runtime_error("Unsupported glTF component type");
    }
}

// Get number of components for glTF accessor type
unsigned char getComponentSizeInBytes(fx::gltf::Accessor::ComponentType type) {
    switch (type) {
        case fx::gltf::Accessor::ComponentType::Byte: return 1;
        case fx::gltf::Accessor::ComponentType::UnsignedByte: return 1;
        case fx::gltf::Accessor::ComponentType::Short: return 2;
        case fx::gltf::Accessor::ComponentType::UnsignedShort: return 2;
       // case fx::gltf::Accessor::ComponentType::Int: return RINTINTIN_TYPE_INT;
        case fx::gltf::Accessor::ComponentType::UnsignedInt: return 4;
        case fx::gltf::Accessor::ComponentType::Float: return 4;
        default: return 0;
    }
}

// Get number of components for glTF accessor type
unsigned char getComponentCount(fx::gltf::Accessor::Type type) {
    switch (type) {
        case fx::gltf::Accessor::Type::Scalar: return 1;
        case fx::gltf::Accessor::Type::Vec2: return 2;
        case fx::gltf::Accessor::Type::Vec3: return 3;
        case fx::gltf::Accessor::Type::Vec4: return 4;
        case fx::gltf::Accessor::Type::Mat2: return 4;
        case fx::gltf::Accessor::Type::Mat3: return 9;
        case fx::gltf::Accessor::Type::Mat4: return 16;
        default: return 0;
    }
}

// Helper to create rintintin_attrib from glTF accessor
rintintin_attrib createRintintinAttrib(const GLTFAttributeData& gltfData) {
    if (!gltfData.isValid()) {
        throw std::runtime_error("Invalid glTF attribute data");
    }
    
    rintintin_attrib attrib = {};
    attrib.src = const_cast<void*>(static_cast<const void*>(gltfData.data));
    attrib.byte_length = gltfData.bufferView->byteLength;
    attrib.type = gltfToRinTinTinType(gltfData.accessor->componentType);
    attrib.size = getComponentCount(gltfData.accessor->type);
    attrib.normalized = gltfData.accessor->normalized ? 1 : 0;
    attrib.stride = static_cast<unsigned short>(gltfData.bufferView->byteStride);
    if (attrib.stride == 0) {
        // If stride is 0, it means tightly packed
        attrib.stride = static_cast<unsigned short>(
            getComponentSizeInBytes(gltfData.accessor->componentType) * attrib.size
        );
    }
    attrib.offset = 0; // Already accounted for in the data pointer
    
    return attrib;
}

// Convert glTF primitive mode to rintintin geometry type
rintintin_geometry_type gltfToRinTinTinGeometry(fx::gltf::Primitive::Mode mode) {
    switch (mode) {
        case fx::gltf::Primitive::Mode::Triangles: return RINTINTIN_TRIANGLES;
        case fx::gltf::Primitive::Mode::TriangleStrip: return RINTINTIN_TRIANGLE_STRIP;
        case fx::gltf::Primitive::Mode::TriangleFan: return RINTINTIN_TRIANGLE_FAN;
        default: throw std::runtime_error("Unsupported primitive mode for rintintin");
    }
}

RintintinMeshData createRintintinMeshFromPrimitive(
    const fx::gltf::Document& document, 
    const fx::gltf::Primitive& primitive, bool * is_alpha_card) {
    
    RintintinMeshData meshData = {};
    
    // Get required vertex attributes
    auto positionIter = primitive.attributes.find("POSITION");
    auto jointsIter = primitive.attributes.find("JOINTS_0");
    auto weightsIter = primitive.attributes.find("WEIGHTS_0");
    
    if (positionIter == primitive.attributes.end() ||
        jointsIter == primitive.attributes.end() ||
        weightsIter == primitive.attributes.end()) {
        throw std::runtime_error("Required vertex attributes (POSITION, JOINTS_0, WEIGHTS_0) not found");
    }
    
    // Create attribute data structures
    GLTFAttributeData positionData(&document, positionIter->second);
    GLTFAttributeData jointsData(&document, jointsIter->second);
    GLTFAttributeData weightsData(&document, weightsIter->second);
    
    if (!positionData.isValid() || !jointsData.isValid() || !weightsData.isValid()) {
        throw std::runtime_error("Invalid vertex attribute data");
    }
    
    // Create rintintin attribute descriptors
    meshData.attributes = std::make_unique<std::array<rintintin_attrib, 3>>();
    (*meshData.attributes)[0] = createRintintinAttrib(positionData);
	(*meshData.attributes)[1] = createRintintinAttrib(jointsData);
	(*meshData.attributes)[2] = createRintintinAttrib(weightsData);
	
    // Set up rintintin mesh structure
    meshData.mesh = {};
    meshData.mesh.position = rintintin_read_attrib_generic_f;
    meshData.mesh.joints = rintintin_read_attrib_generic_i;
    meshData.mesh.weights = rintintin_read_attrib_generic_f;
    
    meshData.mesh.position_user_data = &(*meshData.attributes)[0];
    meshData.mesh.joints_user_data = &(*meshData.attributes)[1];
    meshData.mesh.weights_user_data = &(*meshData.attributes)[2];
    
    meshData.mesh.no_verts = positionData.accessor->count;
    meshData.mesh.geometry_type = gltfToRinTinTinGeometry(primitive.mode);
    
    // Handle indices
    if (primitive.indices >= 0) {
        auto indexData = GLTFAttributeData(&document, primitive.indices);
        if (indexData.isValid()) {
            meshData.mesh.index_array_buffer = indexData.data;
            meshData.mesh.index_type = gltfToRinTinTinType(indexData.accessor->componentType);
            meshData.mesh.no_indices = indexData.accessor->count;
        }
    } else {
        meshData.mesh.index_array_buffer = nullptr;
        meshData.mesh.no_indices = 0;
    }
    
    meshData.mesh.surface_mode = RINTINTIN_SURFACE_NORMAL; // Assume front-facing (CCW winding)
    meshData.mesh.thickness = 0.04 / 1000; // thickness of paper..?
    bool maybe_alpha_card = false;
    
    if(uint32_t(primitive.material) < document.materials.size())
    {
		auto & mat = document.materials[primitive.material];
		
		maybe_alpha_card =
			mat.alphaMode != fx::gltf::Material::AlphaMode::Opaque
		&&  mat.doubleSided;
    }
    
    if(is_alpha_card) *is_alpha_card = maybe_alpha_card;
    
	if(maybe_alpha_card)
		meshData.mesh.surface_mode = RINTINTIN_SURFACE_THIN_SHELL; // Assume front-facing (CCW winding)
    
    return meshData;
}

RintintinSkinData createRintintinSkinFromSkin(
    const fx::gltf::Document& document, 
    int skinIndex,
    std::span<glm::quat> rotations) 
{
	// terrible but works well enough for the demo.
	static std::vector<std::string> nameStorage; // Keep strings alive
    nameStorage.clear();
    
    RintintinSkinData result = {};
        
    if (skinIndex == -1 || skinIndex >= static_cast<int>(document.skins.size())) {
        throw std::invalid_argument("No valid skin found for the specified mesh");
    }
    
    const auto& gltfSkin = document.skins[skinIndex];
    const size_t jointCount = gltfSkin.joints.size();
    
    if (jointCount == 0) {
        throw std::invalid_argument("Skin has no joints");
    }
    
    // Allocate memory for the arrays
    result.names = std::make_unique<const char*[]>(jointCount);
    result.origins = std::make_unique<rintintin_vec3[]>(jointCount);
    result.parents = std::make_unique<int[]>(jointCount);
    
    memset(result.parents.get(), 0xFF, sizeof(int)*jointCount);
    
    // Build a map from node index to joint index for parent lookup
    std::unordered_map<uint32_t, int> nodeToJointMap;
    for (size_t i = 0; i < jointCount; ++i) {
        nodeToJointMap[gltfSkin.joints[i]] = static_cast<int>(i);
    }
    
    for (size_t i = 0; i < jointCount; ++i) {
        const uint32_t nodeIndex = gltfSkin.joints[i];
        
        if (nodeIndex >= document.nodes.size()) {
            throw std::invalid_argument("Invalid joint node index in skin");
        }
            
        const auto& jointNode = document.nodes[nodeIndex];
        nameStorage.push_back(jointNode.name.empty() ? 
            ("Joint_" + std::to_string(i)) : jointNode.name);
    }
    
    for(auto & r : rotations)
    {
		r = glm::quat(1, 0, 0, 0);
    }
		
    // Process each joint
    for (size_t i = 0; i < jointCount; ++i) {
        const uint32_t nodeIndex = gltfSkin.joints[i];
        
        if (nodeIndex >= document.nodes.size()) {
            throw std::invalid_argument("Invalid joint node index in skin");
        }
        
        const auto& jointNode = document.nodes[nodeIndex];
        
        result.names[i] = nameStorage[i].c_str();
        
        // Extract translation from the node
        // If matrix is not identity, decompose it; otherwise use translation directly
        rintintin_vec3 translation;
        if (jointNode.matrix != fx::gltf::defaults::IdentityMatrix) {
            // Extract translation from the 4x4 matrix (last column, first 3 elements)
            translation.x = jointNode.matrix[12];
            translation.y = jointNode.matrix[13];
            translation.z = jointNode.matrix[14];
        } else {
            // Use the translation vector directly
            translation.x = jointNode.translation[0];
            translation.y = jointNode.translation[1];
            translation.z = jointNode.translation[2];
        }
        
        result.origins[i] = translation;
        
        // Find parent joint index
        
        for(auto child : jointNode.children)
        {
			int childNode = nodeToJointMap[child];
			result.parents[childNode] = i;
        }
        
    }
    
//    int * parent = result.parents.get();
    
	GLTFAttributeData inverseBindMatrices(&document, gltfSkin.inverseBindMatrices);

    // Alternative approach for joint positions using inverse bind matrices if available
    if (inverseBindMatrices.isValid()) {
        
        const auto& accessor = *inverseBindMatrices.accessor;
        
        auto getInverseTranslation = [](const float * _matrix, glm::quat * q) -> rintintin_vec3 {
			glm::mat4 matrix;
			memcpy(&(matrix[0].x), _matrix, sizeof(matrix));
			
			matrix = glm::inverse(matrix);
			
			if(q)
			{
				glm::vec3 scale;
				glm::quat rotation;
				glm::vec3 translation;
				glm::vec3 skew;
				glm::vec4 perspective;
				if(glm::decompose(matrix, scale, rotation, translation, skew, perspective) == 0)
					throw std::invalid_argument("Inverse bind pose invalid (cannot be decomposed).");
				
				*q = rotation;
				return rintintin_vec3{translation.x, translation.y, translation.z};
			}
			
			return rintintin_vec3{matrix[3].x, matrix[3].y, matrix[3].z};
		};
		
        // Verify the accessor has the right format for matrices
        if (accessor.type == fx::gltf::Accessor::Type::Mat4 && 
            accessor.componentType == fx::gltf::Accessor::ComponentType::Float &&
            accessor.count == jointCount) {
            
            const float* matrixData = reinterpret_cast<const float*>(inverseBindMatrices.data);
            
            // Extract positions from inverse bind matrices
            for (size_t i = 0; i < jointCount; ++i) {
                // Each matrix is 16 floats, we want the inverse of the translation part
                const float* matrix = matrixData + (i * 16);
                
                result.origins[i] = getInverseTranslation(matrix, (i < rotations.size())? &rotations[i] : nullptr);
            }
        }
    }
    
    // Setup the rintintin_skin structure
    result.skin.bone_names = result.names.get();
    result.skin.joint_translation_mesh_space = result.origins.get();
    result.skin.parents = result.parents.get();
    result.skin.no_joints = static_cast<unsigned int>(jointCount);
    
    return result;
}

std::shared_ptr<RintintinCommand> RintintinCommand::Factory(fx::gltf::Document const& doc, size_t index, bool get_obb, std::span<glm::quat> rotations_out)
{
	auto retn = std::shared_ptr<RintintinCommand>(new RintintinCommand(doc, index, rotations_out));
	
	auto function = [retn, get_obb]() -> Result
	{
		std::vector<rintintin_metrics> metrics(retn->skin.skin.no_joints);	
		std::vector<rintintin_inertia_estimation> bounds;	
		
		std::vector<uint8_t> scratch_space;
		rintintin_process_command cmd;
		
		// single threaded
		cmd.meshes = retn->meshes.data();
		cmd.no_meshes = retn->meshes.size();
		cmd.skin = retn->skin.skin;
		cmd.results = metrics.data();  
		cmd.name = retn->name.c_str();        
		
		int no_threads = 1;
		cmd.max_threads = no_threads;
		cmd.scratch_space_byte_length = rintintin_get_scratch_space_size(retn->skin.skin.no_joints, no_threads);
		scratch_space.resize(cmd.scratch_space_byte_length);
		cmd.scratch_space = scratch_space.data();
		
		auto & ec = retn->error_code;
		ec = rintintin_begin(&cmd);
		if(ec < 0) goto error;
		
		ec = rintintin_read_mesh(&cmd, 0, no_threads);
		if(ec < 0) goto error;
				
		ec = rintintin_end(&cmd);
		if(ec < 0) goto error;
		
		if(get_obb)
		{
			bounds.resize(retn->skin.skin.no_joints);
			
			rintintin_bounding_box_command b_cmd;
			b_cmd.meshes = cmd.meshes;
			b_cmd.metrics = cmd.results;
			
			b_cmd.result = bounds.data();
			
			b_cmd.no_joints = retn->skin.skin.no_joints;
			b_cmd.no_meshes = cmd.no_meshes;
			b_cmd.result_byte_length = sizeof(bounds[0]) * bounds.size();
			
			ec = rintintin_oriented_bounding_boxes(&b_cmd);
			if(ec < 0) goto error;
		}
		
		return 
		{
			.metrics=std::move(metrics),
			.bounds=std::move(bounds)
		};
		
error:
		metrics.clear();
		return {};
	};
	
	retn->_future = std::async(std::launch::async, function);
	return retn;
}

RintintinCommand::RintintinCommand(fx::gltf::Document const& src, size_t i, std::span<glm::quat> rotations_out) :
	name(src.nodes[i].name)
{
	auto & mesh = src.meshes[src.nodes[i].mesh];
	
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
			
	skin = createRintintinSkinFromSkin(src, src.nodes[i].skin, rotations_out);
	
}

rintintin_process_command RintintinCommand::GetMeshCommand()
{
	rintintin_process_command cmd;
	
	cmd.meshes = meshes.data();
	cmd.no_meshes = meshes.size();
	cmd.skin = skin.skin;
	cmd.results = nullptr;  
	cmd.name = name.c_str();        
	
	int no_threads = 1;
	cmd.max_threads = no_threads;
	cmd.scratch_space_byte_length = 0;
	cmd.scratch_space = 0;
	
	return cmd;
}

std::vector<rintintin_metrics> const& RintintinCommand::GetMetrics() const
{
	if(_result.empty() && error_code == 0)
	{
		_future.wait();
		_result = _future.get();
    }
		
	if(error_code)
		throw RttErrorCode{error_code};

	return _result.metrics;
}

std::vector<rintintin_inertia_estimation> const&  RintintinCommand::GetBoundingBoxes() const
{
	if(_result.empty() && error_code == 0)
	{
		_future.wait();
		_result = _future.get();
    }
		
	if(error_code)
		throw RttErrorCode{error_code};

	return _result.bounds;
}
	


std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::dmat3 const& I)
{
	rintintin_symmetric_mat3 sm = 
	{
		.xx = (I[0][0]),
		.yy = (I[1][1]),
		.zz = (I[2][2]),
		.xy = (I[0][1]),
		.xz = (I[0][2]),
		.yz = (I[1][2]),
	};
	
	auto eigen = rintintin_compute_eigen(&sm);
	auto q = rintintin_compute_rotation_quat(&eigen);
	
	return std::make_pair<glm::quat, glm::vec3>(
		(glm::dquat&)q,
		(glm::dvec3&)eigen.values
	);
}

std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::mat3 const& I)
{
	rintintin_symmetric_mat3 sm = 
	{
		.xx = (I[0][0]),
		.yy = (I[1][1]),
		.zz = (I[2][2]),
		.xy = (I[0][1]),
		.xz = (I[0][2]),
		.yz = (I[1][2]),
	};
	
	auto eigen = rintintin_compute_eigen(&sm);
	auto q = rintintin_compute_rotation_quat(&eigen);
	
	return std::make_pair<glm::quat, glm::vec3>(
		(glm::dquat&)q,
		(glm::dvec3&)eigen.values
	);
}

LF::RinTinTin  RintintinCommand::MakeExtension(bool eigenDecomposition) const
{
	if(_result.empty() && error_code == 0)
	{
		_future.wait();
		_result = _future.get();
    }
		
	if(error_code)
		throw RttErrorCode{error_code};

	LF::RinTinTin r;
	
	r.metrics.resize(_result.metrics.size());
	
	if(eigenDecomposition)
		r.eigenDecompositions.resize(_result.metrics.size());
		
	r.orientedBoundedBoxes.resize(_result.bounds.size());
	
	for(auto i = 0u; i < r.metrics.size(); ++i)
	{
		r.metrics[i] = LF::Factory(_result.metrics[i]);
		
		if(eigenDecomposition)
			r.eigenDecompositions[i] = LF::MakeEigen(_result.metrics[i]);
	}
	
	for(auto i = 0u; i < _result.bounds.size(); ++i)
	{
		r.orientedBoundedBoxes[i] = LF::Factory(_result.bounds[i]); 
	}
	
	return r;
}
	
LF::RinTinTin::Metrics LF::Factory(rintintin_metrics const& it)
{
	LF::RinTinTin::Metrics r;
	
	r.volume = it.volume;
	r.surfaceArea = it.surfaceArea;
	
	(glm::vec3&)r.centroid = (glm::dvec3&)it.centroid;
	r.inertia = {
		(float)it.inertia.xx,
		(float)it.inertia.yy,
		(float)it.inertia.zz,
		(float)it.inertia.xy,
		(float)it.inertia.xz,
		(float)it.inertia.yz,
	};	
	
	(glm::vec3&)r.min = (glm::dvec3&)it.aabb_min;
	(glm::vec3&)r.max = (glm::dvec3&)it.aabb_max;
	(glm::vec3&)r.covariance = (glm::dvec3&)it.covariance;

	return r;
}

LF::RinTinTin::Eigen LF::MakeEigen(const rintintin_metrics &it)
{
	auto eigen = rintintin_compute_eigen(&it.inertia);
	auto quat = rintintin_compute_rotation_quat(&eigen);
		
	return LF::RinTinTin::Eigen{
		.rotation={float(quat.w), float(quat.x), float(quat.y), float(quat.z)},
		.lambda={float(eigen.values[0]), float(eigen.values[1]), float(eigen.values[2])}
	};
}

LF::Transform LF::Factory(rintintin_inertia_estimation const& it)
{
	glm::vec4 r = (glm::dvec4&)it.rotation;
	glm::vec3 t = (glm::dvec3&)it.translation;
	glm::vec3 s = (glm::dvec3&)it.scale;

	return
	{
		.rotation=(std::array<float, 4>&)r,
		.translation=(std::array<float, 3>&)t,
		.scaling=(std::array<float, 3>&)s,
	};
}
