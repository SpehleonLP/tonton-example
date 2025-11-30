#include "tonton_fxgltf_memo.h"
#include "../modules/dodeedum/extensions/dodeedum_fxgltf_bridge.h"
#include "fx/gltf.h"
#include "../include/tonton_wordlist.h"
#include "gltf_rintintin_bridge.h"
#include "lf_rintintin.h"
#include "tonton_skinnedmesh.h"
#include <glm/gtx/matrix_decompose.hpp>

using GLTFAttributeData = RTT::GLTFAttributeData;

namespace LF { void from_json(nlohmann::json const& json, RinTinTin & db); }
	
static std::string GetCapability(fx::gltf::Document const* doc, uint32_t i)
{	
	auto * json = &doc->nodes[i].extensionsAndExtras;
	auto itr = json->find("extras");
	if(itr == json->end())
		return {};
		
	auto itr2 = itr->find("capability");
	if(itr2 == itr->end())
		return {};
		
	try
	{
		return itr2->get<std::string>();
	}
	catch(...)
	{
		return {};
	}
};
	
	
TonTon::GltfMemo::GltfMemo(std::shared_ptr<const fx::gltf::Document> document_file) :
	_doc(std::move(document_file)) 
{
	if(!_doc) return;

	std::vector<counted_ptr<const TonTon::Mesh>> meshes(_doc->meshes.size());
	std::vector<counted_ptr<const TonTon::Armature>> skins(_doc->skins.size());
	
	for(auto i = 0u; i < meshes.size(); ++i)
	{
		meshes[i] = TonTon::Mesh::Factory(DoDeeDum::GetMesh(*_doc, i));
	}
	
	LF::RinTinTin rtt;
	std::string capabilities;
	
	auto GetRTTExtension = [&](uint32_t i) -> bool
	{
		auto * json = &_doc->nodes[i].extensionsAndExtras;
		auto itr = json->find("extensions");
		if(itr == json->end())
			return false;
			
		auto itr2 = itr->find("LF_RINTINTIN");
		if(itr2 == itr->end())
			return false;
			
		try
		{
			rtt = itr2->get<LF::RinTinTin>();
		}
		catch(...)
		{
			return false;
		}
		
		return true;
	};
	
	for(auto i = 0u; i < _doc->nodes.size(); ++i)
	{
		if(uint32_t(_doc->nodes[i].skin) < skins.size()
		&& uint32_t(_doc->nodes[i].mesh) < meshes.size())
		{
			auto & sk = _doc->skins[_doc->nodes[i].skin];
			auto N =  sk.joints.size();
			
			if(skins[_doc->nodes[i].skin] == nullptr)
			{
				if(GetRTTExtension(i))
					BuildRaw(_doc.get(), i);
				else
				{
					std::vector<glm::quat> q(sk.joints.size());
					
					auto command = RintintinCommand::Factory(*_doc, i, false, q);
				
					if(skins[_doc->nodes[i].skin] == nullptr)
					{
						auto cmd = command->GetMeshCommand();
						std::vector<Word> words;
						GLTFAttributeData inverseBindPoseMatrics(_doc.get(), sk.inverseBindMatrices);
						
						skins[_doc->nodes[i].skin] = TonTon::Armature::Factory(
							shared_array<std::string>::FromArray(cmd.skin.bone_names,  N),
							shared_array<int>::FromArray(cmd.skin.parents, N),
							std::span<const glm::mat4>{(const glm::mat4*)(inverseBindPoseMatrics.data), inverseBindPoseMatrics.accessor->count},
							shared_array<immutable_array<Word>>::Build( N, 
							[&](uint32_t j)
							{
								words.clear();
								StringToWords(words, cmd.skin.bone_names[j]);
								
								if(j < sk.joints.size())
									StringToWords(words, GetCapability(_doc.get(), sk.joints[j]));
								
								return shared_array<Word>::FromArray(words);
							})					
						);
					}
				
					_nodes.push_back({
						.idx=i,
						.skin=skins[_doc->nodes[i].skin],
						.mesh=meshes[_doc->nodes[i].mesh],
						.skinnedMesh=nullptr,
						.cmd = std::move(command)
					});
				}
			}
			
			if(_nodes.back().cmd == nullptr)
			{
				auto idx = _nodes.size()-1;
				
				auto & metrics = rtt.metrics;
				
				_nodes[idx].skinnedMesh = SkinnedMesh::Factory(
					_nodes[idx].mesh,
					_nodes[idx].skin,
				
					//aabb
					shared_array<TonTon::Cube>::Build(N, [&metrics](int j) -> TonTon::Cube 
					{
						return {
						.min= (glm::dvec3&)metrics[j].min,
						.max= (glm::dvec3&)metrics[j].max
						}; 
					}),
					//surface area
					shared_array<float>::Build(N, [&metrics](int j) -> float { return metrics[j].surfaceArea; }),
					// volume
					shared_array<float>::Build(N, [&metrics](int j) -> float { return metrics[j].volume; }),
					//centroids
					shared_array<glm::vec3>::Build(N, [&metrics](int j) -> glm::vec3 { return (glm::dvec3&)metrics[j].centroid; }),
					//intertia
					shared_array<std::array<float, 6>>::Build(N, [&metrics](int j) -> std::array<float, 6> { 
							auto &I = metrics[j].inertia;
							auto &C = metrics[j].covariance;
						
							return {
								 C[0], C[1], C[2],
								-I[3],-I[4],-I[5]
							};
						})		
					);
			
			}
			
			
		}
	}
}

TonTon::GltfMemo::~GltfMemo() = default;

counted_ptr<const TonTon::SkinnedMesh> TonTon::GltfMemo::operator[](int idx) const
{
	if(_nodes[idx].skinnedMesh != nullptr)
		return _nodes[idx].skinnedMesh;
	
	auto & metrics = _nodes[idx].cmd->GetMetrics();	
	auto N = metrics.size();
	
	_nodes[idx].skinnedMesh = SkinnedMesh::Factory(
		_nodes[idx].mesh,
		_nodes[idx].skin,
	
		//aabb
		shared_array<TonTon::Cube>::Build(N, [&metrics](int i) -> TonTon::Cube 
		{
			return {
			.min= (glm::dvec3&)metrics[i].aabb_min,
			.max= (glm::dvec3&)metrics[i].aabb_max
			}; 
		}),
		//surface area
		shared_array<float>::Build(N, [&metrics](int i) -> float { return metrics[i].surfaceArea; }),
		// volume
		shared_array<float>::Build(N, [&metrics](int i) -> float { return metrics[i].volume; }),
		//centroids
		shared_array<glm::vec3>::Build(N, [&metrics](int i) -> glm::vec3 { return (glm::dvec3&)metrics[i].centroid; }),
		//covariance
		shared_array<std::array<float, 6>>::Build(N, [&metrics](int i) -> std::array<float, 6> { 
				auto &I = metrics[i].inertia;
				auto &C = metrics[i].covariance;
			// intertia computed with unit density
				return {
					float( C.x ),float( C.y ),float( C.z ),
					float(-I.xy),float(-I.xz),float(-I.yz)
				};
			})		
	);
		
	return _nodes[idx].skinnedMesh;
}

counted_ptr<const TonTon::SkinnedMesh> TonTon::GltfMemo::at(int nodeIdx) const { return operator[](nodeIdx); }

counted_ptr<const TonTon::Armature> TonTon::GltfMemo::BuildRaw(fx::gltf::Document const* _doc, size_t i)
{
	auto & sk = _doc->skins[_doc->nodes[i].skin];	
	auto N =  sk.joints.size();		
	
	std::vector<Word> words;
	
	auto bone_names = shared_array<std::string>::Build(N, [&](uint32_t j) -> std::string
	{
		auto node = sk.joints[j];
		if(node >= _doc->nodes.size()) return {};
		return _doc->nodes[node].name;
	});
	
	// Build a map from node index to joint index for parent lookup
	std::unordered_map<uint32_t, int> nodeToJointMap;
	for (size_t j = 0; j < N; ++j) {
		nodeToJointMap[sk.joints[j]] = static_cast<int>(j);
	}

	auto parents = shared_array<int>(N, -1);
	
	GLTFAttributeData inverseBindPoseMatrics(_doc, sk.inverseBindMatrices);
	/*
	if(inverseBindPoseMatrics.isValid())
	{
		auto array = (glm::mat4*)(inverseBindPoseMatrics.data);
		
		auto M = std::min<size_t>(N, inverseBindPoseMatrics.accessor->count);
		for(auto j = 0u; j < M; ++j)
		{
			auto node = sk.joints[j];
			if(node < _doc->nodes.size()) 
			{
				for(auto child : _doc->nodes[node].children)
				{
					int childNode = nodeToJointMap[child];
					parents[childNode] = i;
				}
			}

			glm::mat4 matrix = glm::inverse(array[j]);
			
			glm::vec3 scale;
			glm::quat rotation;
			glm::vec3 translation;
			glm::vec3 skew;
			glm::vec4 perspective;
			if(glm::decompose(matrix, scale, rotation, translation, skew, perspective) == 0)
				continue;
				
			rotations[j] = rotation;
			positions[j] = translation;
		}
	}	*/		
	
	return TonTon::Armature::Factory(
		bone_names,
		parents,
		std::span<const glm::mat4>{(const glm::mat4*)(inverseBindPoseMatrics.data), inverseBindPoseMatrics.accessor->count},
		shared_array<immutable_array<Word>>::Build( N, 
		[&](uint32_t j) -> shared_array<Word>
		{
			words.clear();
			StringToWords(words, bone_names[j]);					
			StringToWords(words, GetCapability(_doc, j));
			
			return shared_array<Word>::FromArray(words);
		})			
	);
}
	
