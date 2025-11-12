#ifndef TONTON_FXGLTF_MEMO_H
#define TONTON_FXGLTF_MEMO_H
#include "tonton_shared_array.hpp"
#include "tonton_counted_ptr.hpp"
#include <memory>
#include <unordered_map>
#include <vector>

namespace fx { namespace gltf { struct Document; } }
namespace TonTon { struct Armature; struct Mesh; struct SkinnedMesh;  }

namespace DoDeeDum 
{
struct Primitive;
using mesh = std::vector<Primitive>; 
}

struct RintintinCommand;

namespace TonTon
{

struct GltfMemo 
{	
	static counted_ptr<GltfMemo> Factory(std::shared_ptr<const fx::gltf::Document> in)
	{ return UncountedWrap(new GltfMemo(std::move(in))); }

	void AddRef() const { ++_refCount; };
	void Release() const { if(--_refCount == 0) delete this; }
	
	bool empty() const { return _nodes.empty(); }
	size_t size() const { return _nodes.size(); }
	
	counted_ptr<const TonTon::SkinnedMesh> operator[](int nodeIdx) const;
	counted_ptr<const TonTon::SkinnedMesh> at(int nodeIdx) const;
	
private:
	GltfMemo(std::shared_ptr<const fx::gltf::Document>);
	GltfMemo(GltfMemo const&) = delete;
	~GltfMemo();
	
	void Destroy() const;
	
	struct Node
	{
		uint32_t idx;
		counted_ptr<const Armature> skin;
		counted_ptr<const Mesh> mesh;
		counted_ptr<const SkinnedMesh> skinnedMesh;
		std::shared_ptr<RintintinCommand> cmd;
	};
	
	mutable std::atomic<int> _refCount{1};
	std::shared_ptr<const fx::gltf::Document> _doc;
	mutable std::vector<Node> _nodes;
	
	static counted_ptr<const Armature> BuildRaw(fx::gltf::Document const* _doc, size_t i);
};

}

#endif // TONTON_FXGLTF_MEMO_H
