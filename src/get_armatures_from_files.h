#ifndef GET_ARMATURES_FROM_FILES_H
#define GET_ARMATURES_FROM_FILES_H
#include "tonton_fxgltf_memo.h"
#include <span>

namespace TonTon
{
class InputBuilder;
};

struct InputFile
{
	int index;
	std::string path;
	std::string filename;
	
	counted_ptr<TonTon::GltfMemo> memo;
};

std::vector<InputFile> GetArmaturesFromFiles(std::span<const char*> args);

#endif // GET_ARMATURES_FROM_FILES_H
