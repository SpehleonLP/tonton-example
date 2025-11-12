#include "get_armatures_from_files.h"
#include "fx/gltf.h"
#include <filesystem>
#include <iostream>

#include "rintintin.h"

enum class RttErrorCode;
static bool OpenFile(fx::gltf::Document & doc, const char * argument);

std::vector<InputFile> GetArmaturesFromFiles(std::span<const char*> args)
{
	std::vector<InputFile> result;
	result.reserve(8);

	for(auto i = 0u; i < args.size(); ++i)
	{
		auto path = args[i];
			
		try
		{
			fx::gltf::Document mesh_file;
			if(!OpenFile(mesh_file, path))
				return {};

			result.push_back(InputFile{
				.path=path,
				.filename=std::filesystem::path(path).stem().string(),
				.memo = TonTon::GltfMemo::Factory(std::make_shared<fx::gltf::Document>(std::move(mesh_file)))						
			});
		}
		catch(std::exception & e)
		{
			std::cerr << path << ": " << e.what() << ".\n";
		}
		catch(RttErrorCode ec)
		{
			std::cerr << path << ": " << rintintin_get_error_string(int(ec)) << "\n";
		}
	}
	/*
	for(auto i = 0u, j = 0u; i < result.size(); i=j)
	{
		fx::gltf::Document dst;
		auto path = result[i].path;
		
		for(j = i; j < result.size() && result[j].path == path; ++j)
		{
			auto ptr = dynamic_cast<TonTon::RintintinBuilder*>(result[j].builder.get());
			
			if(ptr)
				ptr->VisualizeTensors(dst);
		}
		
		if(dst.scenes.size())
		{			
		// change to save in binary format instead of text format.
			for(auto & buf : dst.buffers)
			{
				buf.byteLength = buf.data.size();
				buf.uri.clear();
			}
				
			std::string save_path= (std::string(path) += "-debug_tensors.glb");
			fx::gltf::Save(dst, save_path, true);
			std::cout << "saved: " << save_path << "\n";
		}
	}	*/		
			
	return result;
}

static bool OpenFile(fx::gltf::Document & doc, const char * argument)
{
	std::filesystem::path path(argument);
	
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

