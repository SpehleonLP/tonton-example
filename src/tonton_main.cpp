#include <iostream>
#include <cxxopts.hpp>
#include "get_armatures_from_files.h"
#include "tonton_builder.h"
#include "tonton_formatter.h"
#include "tonton_input.h"
#include "tonton_skinnedmesh.h"
#include "tonton_analysis.h"

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

TonTon::Environment parseEnvironment(const std::string& env_str)
{
    std::string env_lower = env_str;
    for (auto& c : env_lower)
        c = tolower(c);

    if (env_lower == "air") return TonTon::createEarthAir();
    else if (env_lower == "ocean") return TonTon::createEarthOcean();
    else if (env_lower == "titan") return TonTon::createTitan();
    else if (env_lower == "centauri" || env_lower == "proxima") return TonTon::createProximaCentauriB();
    else if (env_lower == "trappist") return TonTon::createTrappist1e();
    else if (env_lower == "422b" || env_lower == "kepler422b") return TonTon::createKepler442b();
    else if (env_lower == "lhs" || env_lower == "lhs1140b") return TonTon::createLHS1140b();
    else if (env_lower == "62f" || env_lower == "kepler62f") return TonTon::createKepler62f();
    else if (env_lower == "18b" || env_lower == "k2-18b") return TonTon::createK2_18b();
    else if (env_lower == "wolf" || env_lower == "wolf1061c") return TonTon::createWolf1061c();
    else if (env_lower == "gliese" || env_lower == "gliese667cc") return TonTon::createGliese667Cc();
    else if (env_lower == "europa" || env_lower == "icy") return TonTon::createIcyMoonOcean();
    else if (env_lower == "carboniferous" || env_lower == "carb") {
    
		auto env = TonTon::createEarthAir();
		env.temperature_K = TonTon::temp_K(303);
		env.fluidDensity_Kg_m3 *= 1.6;
		return env;
    }
    else if (env_lower == "warm-titan") 
    {
		auto env = TonTon::createTitan();
		env.temperature_K=TonTon::createEarthAir().temperature_K;
		return env;
    }
    else {
        std::cerr << "Unknown environment: " << env_str << ", defaulting to air\n";
        return TonTon::createEarthAir();
    }
}

int main(int argc, char* argv[])
{
    try {
        cxxopts::Options options("tonton-analyze", "Biomechanical creature analysis from GLTF files");

        options.add_options()
            ("files", "GLTF files to analyze", cxxopts::value<std::vector<std::string>>())
            ("h,help", "Print usage")
            ;

        options.add_options("Environment")
            ("env", "Environment preset (air, ocean, titan, centauri, trappist, 422b, lhs, 62f, 18b, wolf, gliese, europa, warm-titan)",
             cxxopts::value<std::string>()->default_value("air"))
            ;

        options.add_options("Physical Parameters")
            ("scale", "Scale multiplier (1.0 = original size)",
             cxxopts::value<float>()->default_value("1.0"))
            ("density", "Average body density (0.0=700kg/m³ → 1.0=1050kg/m³)",
             cxxopts::value<float>()->default_value("0.5"))
            ("structure", "Structure vs weight (0=lightweight/fragile → 1=robust/heavy)",
             cxxopts::value<float>()->default_value("0.5"))
            ("muscle", "Muscle quality (0=weak → 1=peak performance)",
             cxxopts::value<float>()->default_value("0.5"))
            ("feathers", "Feather quality (0=poor aerodynamics → 1=optimal)",
             cxxopts::value<float>()->default_value("0.5"))
            ("metabolism", "Metabolic efficiency (0=poor → 1=optimal)",
             cxxopts::value<float>()->default_value("0.5"))
            ("stability", "Flight mode (0=speed optimized → 1=hovering optimized)",
             cxxopts::value<float>()->default_value("0.5"))
            ("activity", "Activity level (0=flapping → 1=soaring)",
             cxxopts::value<float>()->default_value("0.5"))
            ("scaling", "Scaling strategy (0=normal → 1=aggressive size compensation)",
             cxxopts::value<float>()->default_value("0.5"))
            ("climbing", "Climbing ability (0=none → 1=vertical surfaces)",
             cxxopts::value<float>()->default_value("0.5"))
            ;

        options.add_options("Behavior")
            ("coloration", "Coloration strategy (-1=camouflage → +1=aposematism)",
             cxxopts::value<float>()->default_value("0.0"))
            ("aggression", "Aggression adjustment (0.0-1.0)",
             cxxopts::value<float>()->default_value("0.5"))
            ("activity-adjust", "Activity adjustment (0.0-1.0)",
             cxxopts::value<float>()->default_value("0.5"))
            ("endurance", "Endurance vs power (0=sprint → 1=endurance)",
             cxxopts::value<float>()->default_value("0.5"))
            ("risk", "Risk tolerance (0.0-1.0)",
             cxxopts::value<float>()->default_value("0.5"))
            ("social", "Social tendency (0=solitary → 1=highly social)",
             cxxopts::value<float>()->default_value("0.5"))
            ("seasonal", "Seasonal behavior (0=resident → 1=migratory)",
             cxxopts::value<float>()->default_value("0.5"))
            ("circadian", "Activity pattern (0=diurnal → 1=nocturnal)",
             cxxopts::value<float>()->default_value("0.5"))
            ("adaptability", "Adaptability (0=none → 1=advanced tool use)",
             cxxopts::value<float>()->default_value("0.0"))
            ;

        options.add_options("Mana")
            ("water", "Aristotle: takes shape of container, flows and changes",
             cxxopts::value<float>()->default_value("0.0"))
            ("fire", "Aristotle: source of motion and life",
             cxxopts::value<float>()->default_value("0.0"))
            ("earth", "Aristotle: heavy and stable",
             cxxopts::value<float>()->default_value("0.0"))
            ("air", "Aristotle: medium actively pushes objects along their path",
             cxxopts::value<float>()->default_value("0.0"))
            ("aether", "Aristotle: falls upward",
             cxxopts::value<float>()->default_value("0.5"))
            ("shadow", "stick to surfaces, conform to terrain",
             cxxopts::value<float>()->default_value("0.0"))
            ;
            
        options.add_options("Display")
            ("banner", "Show ASCII banner")
            ("q,quiet", "Suppress banner and verbose output")
            ("v,verbose", "Show intermediary builder struct")
            ;

        options.parse_positional({"files"});
        options.positional_help("FILE [FILE...]");

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help() << std::endl;
            return 0;
        }

        if (!result.count("files")) {
            std::cerr << "Error: No input files specified\n\n";
            std::cout << options.help() << std::endl;
            return 1;
        }

        // Show banner unless quiet
        if (result.count("banner") && !result.count("quiet")) {
            std::cout << banner;
        }

        // Parse environment
        TonTon::Environment environment = parseEnvironment(result["env"].as<std::string>());

        // Build input configuration
        TonTon::Input input;
        input.environment = environment;

        // Physical parameters
        input.scale = TonTon::length_b_to_m(result["scale"].as<float>());
        input.average_density = result["density"].as<float>();
        input.structure_vs_weight = result["structure"].as<float>();
        input.muscle_quality = result["muscle"].as<float>();
        input.feather_quality = result["feathers"].as<float>();
        input.metabolic_efficiency = result["metabolism"].as<float>();
        input.stability_vs_speed = result["stability"].as<float>();
        input.activity_level = result["activity"].as<float>();
        input.scaling_strategy = result["scaling"].as<float>();
        input.climbing_ability = result["climbing"].as<float>();

        // Behavior parameters
        input.behavior.coloration = result["coloration"].as<float>();
        input.behavior.aggression_adjustment = result["aggression"].as<float>();
        input.behavior.activity_adjustment = result["activity-adjust"].as<float>();
        input.behavior.endurance_vs_power = result["endurance"].as<float>();
        input.behavior.risk_tolerance = result["risk"].as<float>();
        input.behavior.social_tendency = result["social"].as<float>();
        input.behavior.seasonal_behavior = result["seasonal"].as<float>();
        input.behavior.activity_pattern = result["circadian"].as<float>();
        input.behavior.adaptability = result["adaptability"].as<float>();

        input.mana.water = result["water"].as<float>();
        input.mana.fire = result["fire"].as<float>();
        input.mana.earth = result["earth"].as<float>();
        input.mana.air = result["air"].as<float>();
        input.mana.aether = result["aether"].as<float>();
        input.mana.shadow = result["shadow"].as<float>();
        
        // Get file list
        auto files = result["files"].as<std::vector<std::string>>();

        // Convert to const char* array for GetArmaturesFromFiles
        std::vector<const char*> file_ptrs;
        for (const auto& f : files) {
            file_ptrs.push_back(f.c_str());
        }

        // Load and process files
        std::vector<InputFile> armatures = GetArmaturesFromFiles({file_ptrs.data(), file_ptrs.data() + file_ptrs.size()});

        if (armatures.empty()) {
            std::cerr << "Error: No valid armatures found in input files\n";
            return 1;
        }

        // Process each armature
        for (auto& armature : armatures) {
            for (auto i = 0u; i < armature.memo->size(); ++i) {
                input.builder = TonTon::Builder::Factory(armature.memo->at(i));
                
                if(result.count("verbose"))
					std::cout << *input.builder;
                
                auto output = TonTon::Output::Factory(input);
                std::cout << *output;
            }
        }

    } catch (const cxxopts::exceptions::exception& e) {
        std::cerr << "Error parsing arguments: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

// ============================================================================
// Environment Presets
// ============================================================================

namespace TonTon
{

// Earth baseline
Environment createEarthAir()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.225f;
    env.fluidViscosity_Pa_s = 1.81e-5f;
    env.gravity_m_s2 = 9.81f;
    env.fluidPressure_Pa = 101325.0f + (1025.0f * 9.81f * 30.0f); // Surface + 30m depth
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
    env.fluidPressure_Pa = 101325.0f + (1025.0f * 9.81f * 30.0f); // Surface + 30m depth
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
    env.fluidPressure_Pa = 150000.0f + (1030.0f * 12.46f * 50.0f); // Ice pressure + 50m depth
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
    env.fluidPressure_Pa = 105000.0f + (1015.0f * 9.12f * 40.0f); // 40m depth
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
    env.fluidPressure_Pa = 135000.0f + (1050.0f * 13.25f * 60.0f); // Deep ocean, 60m
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
    env.fluidPressure_Pa = 180000.0f + (1045.0f * 17.15f * 45.0f); // High pressure world
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
    env.fluidPressure_Pa = 95000.0f + (1010.0f * 7.35f * 35.0f); // Lower pressure, 35m depth
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
    env.fluidPressure_Pa = 250000.0f + (980.0f * 23.54f * 25.0f); // Extreme pressure
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
    env.fluidPressure_Pa = 88000.0f + (1035.0f * 11.27f * 55.0f); // Moderate depth
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
    env.fluidPressure_Pa = 115000.0f + (995.0f * 13.73f * 38.0f); // 38m depth
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
    env.fluidPressure_Pa = 120000.0f + (1040.0f * 7.84f * 100.0f); // Deep under ice
    env.temperature_K = 268.15f;          // -5°C
    return env;
}

// Titan's surface atmosphere (not in lakes)
Environment createTitan() {
    Environment env;
    env.fluidDensity_Kg_m3 = 5.3f;          // Very dense atmosphere at surface
    env.fluidViscosity_Pa_s = 0.0000063f;   // Nitrogen gas viscosity at ~94K
    env.gravity_m_s2 = 1.352f;              // 0.138 Earth gravity
    env.fluidPressure_Pa = 146500.0f;            // 1.45 atm surface pressure
    env.temperature_K = 93.7f;              // -179.45°C (Titan surface temp)
    return env;
}

}
