#include "primitives.h"
#include "fx/gltf.h"

// for an icosphere so instead of using catmul clark to make a smooth subsurf you can just use normalize * radius
void Subdivide_Icosphere(std::vector<std::array<vec3, 3>> & dst, std::array<vec3, 3> const& tri, float radius, int subdivisions)
{
// Base case: no more subdivisions needed
    if (subdivisions <= 0) {
        dst.push_back(tri);
        return;
    }
    
    // Get the three vertices of the triangle
    vec3 v0 = tri[0];
    vec3 v1 = tri[1];
    vec3 v2 = tri[2];
    
    // Calculate midpoints of each edge
    vec3 m01 = (v0 + v1) * 0.5f;
    vec3 m12 = (v1 + v2) * 0.5f;
    vec3 m20 = (v2 + v0) * 0.5f;
    
    // Normalize midpoints to sphere surface and scale by radius
    m01 = normalize(m01) * radius;
    m12 = normalize(m12) * radius;
    m20 = normalize(m20) * radius;
    
    // Create four new triangles from the subdivided triangle
    std::array<vec3, 3> tri0 = {v0, m01, m20};
    std::array<vec3, 3> tri1 = {m01, v1, m12};
    std::array<vec3, 3> tri2 = {m20, m12, v2};
    std::array<vec3, 3> tri3 = {m01, m12, m20}; // Center triangle
    
    // Recursively subdivide each of the four triangles
    Subdivide_Icosphere(dst, tri0, radius, subdivisions - 1);
    Subdivide_Icosphere(dst, tri1, radius, subdivisions - 1);
    Subdivide_Icosphere(dst, tri2, radius, subdivisions - 1);
    Subdivide_Icosphere(dst, tri3, radius, subdivisions - 1);
}

// Generate PartialTensor for an icosphere
std::vector<std::array<vec3, 3>> generateIcosphere(float sphere_radius, int subdivisions) {  
    // Volume efficiency factors for different subdivision levels
    // These are approximate values based on the geometric relationship
    // where face centroids lie below the sphere surface
    static constexpr double sphere_volume = 4 * M_PI / 3;
    static const float icosphere_volume[] = {
        2.53615,
		3.65871,
		4.04704,
		4.15274,
		4.17974,
		4.18652,
		4.18822,
		4.18865
    };
    
    enum
    {
		N = sizeof(icosphere_volume) / sizeof(*icosphere_volume)
    };
    
    // Get efficiency factor for this subdivision level
    auto ico_vol = (subdivisions < N) ? 
		icosphere_volume[subdivisions] : 
		icosphere_volume[N-1];
    
    // Calculate required icosphere radius to achieve target volume
    // V_target = (4/3) * π * sphere_radius³
    // V_icosphere = efficiency * (4/3) * π * icosphere_radius³
    // Therefore: icosphere_radius = sphere_radius / cbrt(efficiency)
    float icosphere_radius = sphere_radius / cbrt(ico_vol / sphere_volume);
    
	const float φ = (1.0f + std::sqrt(5.0f)) * 0.5f;
	const float invφ = 1.0f / φ;
	std::array<vec3,12> icoV = {{
		{ -1,  invφ,  0}, {  1,  invφ,  0}, { -1, -invφ,  0}, {  1, -invφ,  0},
		{  0,  -1,    invφ}, {  0,   1,   invφ}, {  0,  -1,   -invφ}, {  0,   1,  -invφ},
		{  invφ,  0, -1}, {  invφ,  0,  1}, { -invφ,  0, -1}, { -invφ,  0,  1}
	}};
	// 20 triangular faces
	static const std::array<std::array<uint8_t,3>, 20> icoF = {{
		{{0,11,5}}, {{7,0,5}}, {{7,5,1}}, {{0,7,10}},  {{0,2,11}},
		{{1,5,9}}, {{9,5,11}}, {{0,10,2}}, {{10,8,6}}, {{7,1,8}},
		{{3,9,4}}, {{4,6,3}},  {{4,2,6}}, {{3,6,8}}, {{3,1,9}},
		{{9,11,4}}, {{2,4,11}}, {{6,2,10}}, {{10,7,8}}, {{3,8,1}}
	}};

	for (auto& v : icoV)
		v = normalize(v) * icosphere_radius;
            
    
    // Subdivide faces (simplified for this example)
    // In a real implementation, you'd recursively subdivide each triangle
    
    std::vector<std::array<vec3, 3>> triangles;
    for (const auto& face : icoF) {
		std::array<vec3, 3> tri = {icoV[face[0]], icoV[face[1]], icoV[face[2]]};
		Subdivide_Icosphere(triangles, tri, icosphere_radius, subdivisions);
    }
    
    
    return triangles;
}

std::vector<std::array<vec3, 3>> generateCylinder(float radius, float height, int radial_segments)
{
	auto polygon_area = (radial_segments/2) * std::sin(2 * M_PI / radial_segments);
	radius = radius / std::sqrt(polygon_area / M_PI);

	std::vector<std::array<vec3, 3>> triangles;
      
    // Top and bottom caps
    vec3 top_center{0, 0, height / 2.0f};
    vec3 bottom_center{0, 0, -height / 2.0f};
    
    float c_angle1 = std::cos(0);
    float s_angle1 = std::sin(0);
    
    auto PushRadialSegment = [&](float angle2)
    {
		float c_angle2 = std::cos(angle2);
		float s_angle2 = std::sin(angle2);
			
        vec3 top1{radius * c_angle1, radius * s_angle1, height / 2.0f};
        vec3 top2{radius * c_angle2, radius * s_angle2, height / 2.0f};
        vec3 bottom1{radius * c_angle1, radius * s_angle1, -height / 2.0f};
        vec3 bottom2{radius * c_angle2, radius * s_angle2, -height / 2.0f};
        
        c_angle1 = c_angle2;
        s_angle1 = s_angle2;
        
        // Top cap
        triangles.push_back({top_center, top1, top2});				
        // Bottom cap
        triangles.push_back({bottom_center, bottom2, bottom1});	
        
		// sides
		triangles.push_back({bottom1, top2, top1});				
		triangles.push_back({top2, bottom1, bottom2});				
    };
    
    // Generate cylinder mesh
    float angle_step = 2.0f * M_PI / radial_segments;
    for (int i = 0; i < radial_segments; ++i) {
		float angle2 = (i + 1) * angle_step;
        
		if (i == radial_segments - 1) {
			angle2 = 0;  // Ensure perfect closure
		}
		
		PushRadialSegment(angle2);
    }
    
 //   std::ofstream fstream("/home/anyuser/test_cylinder.obj");
//    writeObjObject(fstream, "cylinder", triangles);
       
	return triangles;
}

std::vector<std::array<vec3, 3>> generateCone(float radius, float height, int radial_segments)
{
    auto polygon_area = (radial_segments/2) * std::sin(2 * M_PI / radial_segments);
    radius = radius / std::sqrt(polygon_area / M_PI);

    std::vector<std::array<vec3, 3>> triangles;
      
    // Cone tip and bottom center
    vec3 tip{0, 0, height * 0.75f};
    vec3 bottom_center{0, 0, -height * 0.25f};
    
    float c_angle1 = std::cos(0);
    float s_angle1 = std::sin(0);
    
    auto PushRadialSegment = [&](float angle2)
    {
        float c_angle2 = std::cos(angle2);
        float s_angle2 = std::sin(angle2);
            
        vec3 bottom1{radius * c_angle1, radius * s_angle1, -height / 2.0f};
        vec3 bottom2{radius * c_angle2, radius * s_angle2, -height / 2.0f};
        
        c_angle1 = c_angle2;
        s_angle1 = s_angle2;
        
        // Bottom cap triangle
        triangles.push_back({bottom_center, bottom2, bottom1});
        
        // Side triangle (from bottom edge to tip)
        triangles.push_back({bottom1, bottom2, tip});
    };
    
    // Generate cone mesh
    float angle_step = 2.0f * M_PI / radial_segments;
    for (int i = 0; i < radial_segments; ++i) {
        float angle2 = (i + 1) * angle_step;
        
        if (i == radial_segments - 1) {
            angle2 = 0;  // Ensure perfect closure
        }
        
        PushRadialSegment(angle2);
    }
    
    return triangles;
}

// Generate a rectangular box
std::vector<std::array<vec3, 3>> generateBox(float width, float height, float depth) {
    std::vector<std::array<vec3, 3>> triangles;
    
    float w = width / 2.0f;
    float h = height / 2.0f;
    float d = depth / 2.0f;
    
    // Define the 8 vertices of the box
    vec3 vertices[8] = {
        vec3{-w, -h, -d}, vec3{w, -h, -d}, vec3{w, h, -d}, vec3{-w, h, -d},
        vec3{-w, -h, d}, vec3{w, -h, d}, vec3{w, h, d}, vec3{-w, h, d}
    };
    
    // Define the 12 triangles (2 per face)
    int indices[36] = {
        // Front face
        0, 1, 2, 0, 2, 3,
        // Back face
        4, 6, 5, 4, 7, 6,
        // Left face
        0, 3, 7, 0, 7, 4,
        // Right face
        1, 5, 6, 1, 6, 2,
        // Top face
        3, 2, 6, 3, 6, 7,
        // Bottom face
        0, 4, 5, 0, 5, 1
    };
    
    for (int i = 0; i < 36; i += 3) {
    // winding is wrong in indices order... 
        triangles.push_back({vertices[indices[i]], vertices[indices[i+2]], vertices[indices[i+1]]});
    }
    
    return triangles;
}


int AddPrimitive(fx::gltf::Document & doc, const char * name, std::vector<std::array<vec3, 3>> const& triangles, vec3 limit)
{        
	// Flatten triangles into vertices and indices
	std::vector<uint16_t> indices;
	size_t counter = 0;
	
	for(const auto& tri : triangles) {
		for(const auto& vertex : tri) {
			indices.push_back(counter++);
		}
	}
	
	// Create buffer for vertex and index data
	size_t vertexDataSize = triangles.size() * sizeof(triangles[0]);
	size_t indexDataSize = indices.size() * sizeof(uint16_t);
	size_t totalSize = vertexDataSize + indexDataSize;
	
	if(doc.buffers.empty())
	{
		doc.buffers.resize(1);
		doc.buffers[0].uri = "primitives.bin";
	}
	
	auto vertexDataOffset = doc.buffers.front().data.size();
	auto indexDataOffset = vertexDataOffset + vertexDataSize;
	doc.buffers.front().data.resize(doc.buffers.front().data.size() + totalSize);
	
	// Copy data
	memcpy(doc.buffers[0].data.data() + vertexDataOffset, triangles.data(), vertexDataSize);
	memcpy(doc.buffers[0].data.data() + indexDataOffset, indices.data(), indexDataSize);
	
	// Create buffer views
	auto verticesBufferViewIdx = doc.bufferViews.size();
	auto indiciesBufferViewIdx = verticesBufferViewIdx+1;
// since we're only adding 2 items at a time resize() is actually worse.	
	doc.bufferViews.push_back({});
	doc.bufferViews.push_back({});
	
	doc.bufferViews[verticesBufferViewIdx].buffer = 0;
	doc.bufferViews[verticesBufferViewIdx].byteOffset = vertexDataOffset;
	doc.bufferViews[verticesBufferViewIdx].byteLength = vertexDataSize;
	doc.bufferViews[verticesBufferViewIdx].target = fx::gltf::BufferView::TargetType::ArrayBuffer;
	
	doc.bufferViews[indiciesBufferViewIdx].buffer = 0;
	doc.bufferViews[indiciesBufferViewIdx].byteOffset = indexDataOffset;
	doc.bufferViews[indiciesBufferViewIdx].byteLength = indexDataSize;
	doc.bufferViews[indiciesBufferViewIdx].target = fx::gltf::BufferView::TargetType::ElementArrayBuffer;
	
	// Create accessors
	
	auto verticesIdx = doc.accessors.size();
	auto indicesIdx = verticesIdx+1;
	doc.accessors.push_back({});
	doc.accessors.push_back({});
	
	doc.accessors[verticesIdx].bufferView = verticesBufferViewIdx;
	doc.accessors[verticesIdx].byteOffset = 0;
	doc.accessors[verticesIdx].componentType = fx::gltf::Accessor::ComponentType::Float;
	doc.accessors[verticesIdx].count = triangles.size()*3;
	doc.accessors[verticesIdx].type = fx::gltf::Accessor::Type::Vec3;
	
	doc.accessors[verticesIdx].min = {-limit.x, -limit.y, -limit.z};
	doc.accessors[verticesIdx].max = {limit.x, limit.y, limit.z};
	
	doc.accessors[indicesIdx].bufferView = indiciesBufferViewIdx;
	doc.accessors[indicesIdx].byteOffset = 0;
	doc.accessors[indicesIdx].componentType = fx::gltf::Accessor::ComponentType::UnsignedShort;
	doc.accessors[indicesIdx].count = indices.size();
	doc.accessors[indicesIdx].type = fx::gltf::Accessor::Type::Scalar;
	
	// Create mesh
	auto i = doc.meshes.size();
	doc.meshes.push_back({});
	doc.meshes[i].name = name;
	doc.meshes[i].primitives.resize(1);
	doc.meshes[i].primitives[0].attributes["POSITION"] = verticesIdx;
	doc.meshes[i].primitives[0].indices = indicesIdx;
	doc.meshes[i].primitives[0].mode = fx::gltf::Primitive::Mode::Triangles;
	
	return i;
}

float dot(vec3 const& a, vec3 const& b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

vec3 normalize(vec3 const& in)
{
	float l = std::sqrt(dot(in, in));
	l = l == 0? 0 : 1.0 / l;
	return {in.x * l, in.y * l, in.z * l};
}

vec3 operator*(vec3 const& in, float b)
{
	return vec3{in.x*b, in.y*b, in.z*b};
}

vec3 operator+(vec3 const& a, vec3 const& b)
{
	return {a.x+b.x, a.y+b.y, a.z+b.z};
}

vec3 operator-(vec3 const& a, vec3 const& b)
{
	return {a.x-b.x, a.y-b.y, a.z-b.z};
}

vec3 cross(vec3 const& a, vec3 const& b)
{
	return {
		a.y*b.z - a.z*b.y,
		a.z*b.x - a.x*b.z,
		a.x*b.y - a.y*b.x
	};
}

#if 0

void test_4d_vfunc()
{
	std::vector<std::array<vec3, 3>> triangles;
    
    float w = 1.0 / 2.0f;
    float h = 1.0 / 2.0f;
    float d = 1.0 / 2.0f;
    
    #define v4(a, b, c, dens) vec4{.x=a,.y=b,.z=c,.d=dens}
    
    // Define the 8 vertices of the box
    vec4 vertices[8] = {
        v4(-w, -h, -d, .5f), v4(w, -h, -d, .8f), v4(w, h, -d, 1.2f), v4(-w, h, -d, 2.0f),
        v4(-w, -h, d, 1.0f), v4(w, -h, d, 0.33f), v4(w, h, d, 3.0f), v4(-w, h, d, 1.2f)
    };
    
    // Define the 12 triangles (2 per face)
    int indices[36] = {
        // Front face
        0, 1, 2, 0, 2, 3,
        // Back face
        4, 6, 5, 4, 7, 6,
        // Left face
        0, 3, 7, 0, 7, 4,
        // Right face
        1, 5, 6, 1, 6, 2,
        // Top face
        3, 2, 6, 3, 6, 7,
        // Bottom face
        0, 4, 5, 0, 5, 1
    };
    
    // volume
    vec3 v_func = {0};
    // denstiy
    vec3 d_func = {0};
    
    auto process_tri = [&v_func, &d_func](vec4 const& p0, vec4 const& p1, vec4 const& p2)
    {
		float d0 = p0.d;
		float d1 = p1.d;
		float d2 = p2.d;
		
		vec3 temp;
		temp.x = (-d0*p1.y*p2.z + d0*p1.z*p2.y + d1*p0.y*p2.z - d1*p0.z*p2.y - d2*p0.y*p1.z + d2*p0.z*p1.y);
		temp.y = (d0*p1.x*p2.z - d0*p1.z*p2.x - d1*p0.x*p2.z + d1*p0.z*p2.x + d2*p0.x*p1.z - d2*p0.z*p1.x);
		temp.z = (-d0*p1.x*p2.y + d0*p1.y*p2.x + d1*p0.x*p2.y - d1*p0.y*p2.x - d2*p0.x*p1.y + d2*p0.y*p1.x);
    
		v_func = v_func + cross(p2.xyz - p0.xyz, p1.xyz - p0.xyz);
		d_func = d_func + temp;
    };
    
    
    for (int i = 0; i < 36; i += 3) {
    
    // winding is wrong in indices order... 
        process_tri(vertices[indices[i]], vertices[indices[i+2]], vertices[indices[i+1]]);
    }
    
    int break_point = 0;
    ++break_point;
    

}
#endif
