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


namespace TonTon
{
Environment createEarthAir();
Environment createEarthOcean();
Environment createTitan();
Environment createProximaCentauriB();
Environment createTrappist1e();
Environment createKepler442b();
Environment createLHS1140b();
Environment createKepler62f();
Environment createK2_18b();
Environment createWolf1061c();
Environment createGliese667Cc();
Environment createIcyMoonOcean();
}

int main(int argc, const char * args[])
{
//	if(argc > 1) std::cout << "running rintintin...\n";
		
	auto armatures = GetArmaturesFromFiles({args+1, args+argc});
	
//	if(armatures.size()) std::cout << banner;
	
	auto to_lower = [&](std::string && s)
	{
		for(auto & c : s)
			c = tolower(c);
		return s;
	};
	
	int index = 1;
	TonTon::Environment environment;
	for(auto & armature : armatures)
	{
		// armature.index is relative to the span passed to GetArmaturesFromFiles (args+1)
		// so we need to add 1 to get the absolute index in the original args array
		int absolute_index = armature.index + 1;
		for(; index < absolute_index; ++index)
		{
			if(strncmp("--env=", args[index], 6) != 0)
				continue;

			std::string env = to_lower(std::string(args[index]+6));

			if(env.find("air") != std::string::npos) environment = TonTon::createEarthAir();
			else if(env.find("ocean") != std::string::npos) environment = TonTon::createEarthOcean();
			else if(env.find("titan") != std::string::npos) environment = TonTon::createTitan();
			else if(env.find("centauri") != std::string::npos) environment = TonTon::createProximaCentauriB();
			else if(env.find("trappist") != std::string::npos) environment = TonTon::createTrappist1e();
			else if(env.find("422b") != std::string::npos) environment = TonTon::createKepler442b();
			else if(env.find("lhs") != std::string::npos) environment = TonTon::createLHS1140b();
			else if(env.find("62f") != std::string::npos) environment = TonTon::createKepler62f();
			else if(env.find("18b") != std::string::npos) environment = TonTon::createK2_18b();
			else if(env.find("wolf") != std::string::npos) environment = TonTon::createWolf1061c();
			else if(env.find("gliese") != std::string::npos) environment = TonTon::createGliese667Cc();
			else if(env.find("europa") != std::string::npos) environment = TonTon::createIcyMoonOcean();		
		}
	
		TonTon::Input input;
		input.environment = environment;
	//	input.structure_vs_weight = 1.0;
	//	input.muscle_quality = 1.0;
		input.average_density=0.5;
		input.behavior.social_tendency=0.0;
		
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

namespace TonTon
{

// Earth baseline 
Environment createEarthAir()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.225f;
    env.fluidViscosity_Pa_s = 1.81e-5f;
    env.gravity_m_s2 = 9.81f;
    env.pressure_Pa = 101325.0f + (1025.0f * 9.81f * 30.0f); // Surface + 30m depth
    env.temperature_K = 293.15f;          // 20°C
    return env;
}

// Earth baseline - temperate ocean at 30m depth
Environment createEarthOcean()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1025.0f;     // Seawater
    env.fluidViscosity_Pa_s = 0.00107f;   // Seawater at 20°C
    env.gravity_m_s2 = 9.81f;
    env.pressure_Pa = 101325.0f + (1025.0f * 9.81f * 30.0f); // Surface + 30m depth
    env.temperature_K = 293.15f;          // 20°C
    return env;
}

// Proxima Centauri b - potential subsurface ocean world
// Higher gravity (1.27g), tidally locked, possibly water under ice
Environment createProximaCentauriB()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.4f;          // Denser atmosphere assumed
    env.fluidViscosity_Pa_s = 1.85e-5f;
    env.gravity_m_s2 = 12.46f;            // 1.27 Earth gravity
    env.pressure_Pa = 150000.0f + (1030.0f * 12.46f * 50.0f); // Ice pressure + 50m depth
    env.temperature_K = 278.15f;          // 5°C - cold but liquid
    return env;
}

// TRAPPIST-1e - habitable zone, Earth-like mass
// Likely ocean world with moderate conditions
Environment createTrappist1e()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.3f;
    env.fluidViscosity_Pa_s = 1.82e-5f;
    env.gravity_m_s2 = 9.12f;             // 0.93 Earth gravity
    env.pressure_Pa = 105000.0f + (1015.0f * 9.12f * 40.0f); // 40m depth
    env.temperature_K = 288.15f;          // 15°C
    return env;
}

// Kepler-442b - super-Earth in habitable zone
// Higher gravity, potentially deep oceans
Environment createKepler442b()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.8f;          // Thicker atmosphere
    env.fluidViscosity_Pa_s = 1.95e-5f;
    env.gravity_m_s2 = 13.25f;            // 1.35 Earth gravity (super-Earth)
    env.pressure_Pa = 135000.0f + (1050.0f * 13.25f * 60.0f); // Deep ocean, 60m
    env.temperature_K = 283.15f;          // 10°C
    return env;
}

// LHS 1140 b - super-Earth, potentially water world
// Dense atmosphere, high pressure ocean
Environment createLHS1140b()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 2.1f;          // Very thick atmosphere
    env.fluidViscosity_Pa_s = 2.0e-5f;
    env.gravity_m_s2 = 17.15f;            // 1.75 Earth gravity
    env.pressure_Pa = 180000.0f + (1045.0f * 17.15f * 45.0f); // High pressure world
    env.temperature_K = 280.15f;          // 7°C
    return env;
}

// Kepler-62f - in habitable zone, low gravity
// Possibly ocean world with lighter conditions
Environment createKepler62f()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.0f;          // Thinner atmosphere
    env.fluidViscosity_Pa_s = 1.75e-5f;
    env.gravity_m_s2 = 7.35f;             // 0.75 Earth gravity
    env.pressure_Pa = 95000.0f + (1010.0f * 7.35f * 35.0f); // Lower pressure, 35m depth
    env.temperature_K = 298.15f;          // 25°C - warm world
    return env;
}

// K2-18b - mini-Neptune with possible water vapor atmosphere
// Exotic "hycean" world with potential liquid water layer
Environment createK2_18b()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 3.5f;          // H2-rich atmosphere, very dense
    env.fluidViscosity_Pa_s = 2.2e-5f;
    env.gravity_m_s2 = 23.54f;            // 2.4 Earth gravity (mini-Neptune)
    env.pressure_Pa = 250000.0f + (980.0f * 23.54f * 25.0f); // Extreme pressure
    env.temperature_K = 348.15f;          // 75°C - hot hycean world
    return env;
}

// Wolf 1061c - rocky planet in habitable zone
// Potential thin atmosphere with cold oceans
Environment createWolf1061c()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 0.9f;          // Thin atmosphere
    env.fluidViscosity_Pa_s = 1.7e-5f;
    env.gravity_m_s2 = 11.27f;            // 1.15 Earth gravity
    env.pressure_Pa = 88000.0f + (1035.0f * 11.27f * 55.0f); // Moderate depth
    env.temperature_K = 271.15f;          // -2°C - near freezing
    return env;
}

// Gliese 667 Cc - in habitable zone of red dwarf
// Tidally locked, warm dayside ocean
Environment createGliese667Cc()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.5f;
    env.fluidViscosity_Pa_s = 1.88e-5f;
    env.gravity_m_s2 = 13.73f;            // 1.4 Earth gravity
    env.pressure_Pa = 115000.0f + (995.0f * 13.73f * 38.0f); // 38m depth
    env.temperature_K = 308.15f;          // 35°C - tropical
    return env;
}

// Extreme cold environment - icy moon ocean
Environment createIcyMoonOcean()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1040.0f;     // Salty prevents freezing
    env.fluidViscosity_Pa_s = 0.0018f;    // Very viscous when cold
    env.gravity_m_s2 = 7.84f;             // 0.8 Earth gravity
    env.pressure_Pa = 120000.0f + (1040.0f * 7.84f * 100.0f); // Deep under ice
    env.temperature_K = 268.15f;          // -5°C
    return env;
}

// Titan's surface atmosphere (not in lakes)
Environment createTitan() {
    Environment env;
    env.fluidDensity_Kg_m3 = 5.3f;          // Very dense atmosphere at surface
    env.fluidViscosity_Pa_s = 0.0000063f;   // Nitrogen gas viscosity at ~94K
    env.gravity_m_s2 = 1.352f;              // 0.138 Earth gravity
    env.pressure_Pa = 146500.0f;            // 1.45 atm surface pressure
    env.temperature_K = 93.7f;              // -179.45°C (Titan surface temp)
    return env;
}

}
