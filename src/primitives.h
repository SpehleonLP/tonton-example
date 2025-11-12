#ifndef PRIMITIVES_H
#define PRIMITIVES_H
#include <vector>
#include <array>

struct vec3 { float x,y,z; };
union vec4 { vec3 xyz; struct { float x,y,z,d; }; };

float dot(vec3 const& a, vec3 const& b);
vec3 cross(vec3 const& a, vec3 const& b);
vec3 normalize(vec3 const& in);
vec3 operator*(vec3 const& in, float b);
vec3 operator-(vec3 const& a, vec3 const& b);
vec3 operator+(vec3 const& a, vec3 const& b);

std::vector<std::array<vec3, 3>> generateIcosphere(float sphere_radius, int subdivisions = 2);
std::vector<std::array<vec3, 3>> generateCylinder(float radius, float height, int radial_segments = 16);
std::vector<std::array<vec3, 3>> generateBox(float width, float height, float depth);
std::vector<std::array<vec3, 3>> generateCone(float radius, float height, int radial_segments=16);

namespace fx { namespace gltf { struct Document; }}

int AddPrimitive(fx::gltf::Document & doc, const char * name, std::vector<std::array<vec3, 3>> const& triangles, vec3 limit);

#endif // PRIMITIVES_H
