#include <iostream>
#include "get_armatures_from_files.h"
#include "tonton_formatter.h"
#include "tonton_input.h"
#include "tonton_output.h"

const char banner[] = 
"\n"
"TTTTTTTTTTTTTTTTTTTTTTT                                                 TTTTTTTTTTTTTTTTTTTTTTT                                \n"
"T:::::::::::::::::::::T                                                 T:::::::::::::::::::::T                                \n"
"T:::::::::::::::::::::T                                                 T:::::::::::::::::::::T                                \n"
"T:::::TT:::::::TT:::::T                                                 T:::::TT:::::::TT:::::T                                \n"
"TTTTTT  T:::::T  TTTTTTooooooooooo   nnnn  nnnnnnnn                     TTTTTT  T:::::T  TTTTTTooooooooooo   nnnn  nnnnnnnn    \n"
"        T:::::T      oo:::::::::::oo n:::nn::::::::nn                           T:::::T      oo:::::::::::oo n:::nn::::::::nn  \n"
"        T:::::T     o:::::::::::::::on::::::::::::::nn                          T:::::T     o:::::::::::::::on::::::::::::::nn \n"
"        T:::::T     o:::::ooooo:::::onn:::::::::::::::n ---------------         T:::::T     o:::::ooooo:::::onn:::::::::::::::n\n"
"        T:::::T     o::::o     o::::o  n:::::nnnn:::::n -:::::::::::::-         T:::::T     o::::o     o::::o  n:::::nnnn:::::n\n"
"        T:::::T     o::::o     o::::o  n::::n    n::::n ---------------         T:::::T     o::::o     o::::o  n::::n    n::::n\n"
"        T:::::T     o::::o     o::::o  n::::n    n::::n                         T:::::T     o::::o     o::::o  n::::n    n::::n\n"
"        T:::::T     o::::o     o::::o  n::::n    n::::n                         T:::::T     o::::o     o::::o  n::::n    n::::n\n"
"      TT:::::::TT   o:::::ooooo:::::o  n::::n    n::::n                       TT:::::::TT   o:::::ooooo:::::o  n::::n    n::::n\n"
"      T:::::::::T   o:::::::::::::::o  n::::n    n::::n                       T:::::::::T   o:::::::::::::::o  n::::n    n::::n\n"
"      T:::::::::T    oo:::::::::::oo   n::::n    n::::n                       T:::::::::T    oo:::::::::::oo   n::::n    n::::n\n"
"      TTTTTTTTTTT      ooooooooooo     nnnnnn    nnnnnn                       TTTTTTTTTTT      ooooooooooo     nnnnnn    nnnnnn\n"
"\n"
"====================================================================================================================================\n\n";

enum class RttErrorCode;


int main(int argc, const char * args[])
{
	if(argc > 1)
		std::cout << "running rintintin...\n";
		
	auto armatures = GetArmaturesFromFiles({args+1, args+argc});
	
	if(armatures.size())
		std::cout << banner;
	
	for(auto & armature : armatures) 
	{
		TonTon::Input input;
	//	input.structure_vs_weight = 1.0;
	//	input.muscle_quality = 1.0;
		input.behavior.scale = glm::vec3(1.0);
		for(auto i = 0u; i < armature.memo->size(); ++i)
		{
			input.skinnedMesh = armature.memo->at(i);
			auto output = TonTon::Output::Factory(input);
			std::cout << *output;
		}
		
		//solver << TonTon::BasicMorphology{};
		//solver();
	}
	
	(void)banner;
	
	return 0;
}
