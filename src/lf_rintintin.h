#ifndef LF_RINTINTIN_H
#define LF_RINTINTIN_H
#include <array>
#include <vector>

namespace LF
{

struct Transform
{
	std::array<float, 4> rotation;
	std::array<float, 3> translation;
	std::array<float, 3> scaling;
	
	inline bool empty() const 
	{ 
		return scaling == std::array<float, 3>{1.f, 1.f, 1.f} 
		&& translation == std::array<float, 3>{0.f, 0.f, 0.f} 
		&& rotation == std::array<float, 4>{1.f, 0.f, 0.f, 0.f}; 
	} 
};

struct Rintintin
{
	struct Eigen
	{
		std::array<float, 4> rotation;
		std::array<float, 3> lambda;
	
		inline bool empty() const { return false; } 
	};
	
	struct Metrics
	{
		float volume{};
		float surfaceArea{}; 
		
		std::array<float, 3> centroid{0};
		std::array<float, 6> inertia{0};
		
		std::array<float, 3> min, max{0};		
		std::array<float, 3> covariance{0};
	
		inline bool empty() const { return false; } 
	};

	std::vector<Metrics> metrics;
	std::vector<Eigen> eigenDecompositions;
	std::vector<Transform> orientedBoundedBoxes;
	
	inline bool empty() const { return metrics.empty() && eigenDecompositions.empty() && orientedBoundedBoxes.empty(); } 
};

}

#endif // LF_RINTINTIN_H
