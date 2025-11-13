#include "lf_rintintin.h"
#include "fx/gltf.h"
        
using namespace fx::gltf::detail;
#define ReadField(x) ReadRequiredField(#x, json, db.x)
#define ReadOptField(x) ReadOptionalField(#x, json, db.x)

#define WriteReqField(x) (json[#x] = db.x)
#define Write_Field(x) WriteField(#x, json, db.x)
#define WriteOptField(x, y) WriteField(#x, json, db.x, y)

namespace LF
{
	inline void from_json(nlohmann::json const& json, RinTinTin::Eigen & db)
	{
		ReadField(rotation);
		ReadField(lambda);
	}
	
	inline void from_json(nlohmann::json const& json, Transform & db)
	{
		ReadOptField(rotation);
		ReadOptField(translation);
		ReadOptField(scaling);
	}
	
	inline void from_json(nlohmann::json const& json, RinTinTin::Metrics & db)
	{
		ReadField(volume);
		ReadField(centroid);
		ReadField(inertia);
		
		ReadOptField(surfaceArea);
		ReadOptField(min);
		ReadOptField(max);
		ReadOptField(covariance);
	}
	
	inline void to_json(nlohmann::json & json, Transform const& db)
	{
		WriteOptField(rotation, fx::gltf::defaults::IdentityRotation);
		WriteOptField(translation, fx::gltf::defaults::NullVec3);
		WriteOptField(scaling, fx::gltf::defaults::IdentityVec3);
	}
	
	inline void to_json(nlohmann::json & json, RinTinTin::Eigen const& db)
	{
		WriteReqField(rotation);
		WriteReqField(lambda);
	}
	
	inline void to_json(nlohmann::json & json, RinTinTin::Metrics const& db)
	{
		WriteReqField(volume);
		WriteReqField(centroid);
		WriteReqField(inertia);
		
		WriteOptField(surfaceArea, 0.f);
		WriteOptField(min, fx::gltf::defaults::NullVec3);
		WriteOptField(max, fx::gltf::defaults::NullVec3);
		WriteOptField(covariance, fx::gltf::defaults::NullVec3);
	}
	
	
	void from_json(nlohmann::json const& json, RinTinTin & db)
	{
		ReadOptField(metrics);
		ReadOptField(eigenDecompositions);
		ReadOptField(orientedBoundedBoxes);
	}
	
	void to_json(nlohmann::json & json, RinTinTin const& db)
	{
		Write_Field(metrics);
		Write_Field(eigenDecompositions);
		Write_Field(orientedBoundedBoxes);
	}
}
