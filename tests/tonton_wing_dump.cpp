// THROWAWAY diagnostic: dump per-appendage geometry across all sample models so
// we can calibrate a geometry-based "flat spanning surface (wing/fin)" detector
// without guessing thresholds. Delete once the metric is chosen.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "get_armatures_from_files.h"
#include "tonton_builder.h"
#include "tonton_input.h"
#include "tonton_analysis.h"
#include "tonton_skinnedmesh.h"
#include "tonton_wordlist.h"

#ifndef TONTON_SAMPLE_MODELS_DIR
#define TONTON_SAMPLE_MODELS_DIR "sample models"
#endif

using namespace TonTon;

namespace {

Environment EarthAir()
{
    Environment env;
    env.fluidDensity_Kg_m3 = 1.225f;
    env.fluidViscosity_Pa_s = 1.81e-5f;
    env.gravity_m_s2 = 9.81f;
    env.fluidPressure_Pa = 101325.0f;
    env.temperature_K = 293.15f;
    return env;
}

std::string FlagStr(SemanticFlags f)
{
    std::string s;
    auto add = [&](SemanticFlags bit, const char* name) {
        if (HasFlag(f, bit)) { if (!s.empty()) s += "|"; s += name; }
    };
    add(SemanticFlags::WING,     "WING");
    add(SemanticFlags::FIN,      "FIN");
    add(SemanticFlags::FORELIMB, "FORE");
    add(SemanticFlags::HINDLIMB, "HIND");
    add(SemanticFlags::LIMB,     "LIMB");
    add(SemanticFlags::DIGIT,    "DIGIT");
    add(SemanticFlags::TENTACLE, "TENT");
    add(SemanticFlags::LEFT,     "L");
    add(SemanticFlags::RIGHT,    "R");
    if (s.empty()) s = "-";
    return s;
}

void DumpModel(const char* label, const char* filename)
{
    Input input;
    input.environment = EarthAir();
    input.scale = length_b_to_m(1.0f);
    input.average_density = 0.5f;

    std::string path = std::string(TONTON_SAMPLE_MODELS_DIR) + "/" + filename;
    std::vector<const char*> args = { path.c_str() };
    auto files = GetArmaturesFromFiles({ args.data(), args.data() + args.size() });
    if (files.empty() || !files[0].memo || files[0].memo->size() == 0) {
        printf("[%s] FAILED TO LOAD %s\n", label, filename);
        return;
    }

    auto skinned = files[0].memo->at(0);
    auto builder = Builder::Factory(skinned);
    auto const& names = skinned->skin->names;

    printf("\n===== %s (%s) : %u appendages =====\n", label, filename,
           (unsigned)builder->appendages.size());
    printf("%-22s %-14s %8s %10s %10s %9s %9s %9s %9s %9s\n",
           "root->tip", "flags", "len", "faceArea", "volume", "thick", "chord",
           "t/chord", "area/cs", "AR");

    for (auto const& ap : builder->appendages) {
        std::string rt = (ap.root < names.size() ? names[ap.root] : "?")
                       + "->" + (ap.tip < names.size() ? names[ap.tip] : "?");
        float len   = float(ap.stretched_length);
        float area  = float(ap.surface.area);     // flat-face projected silhouette
        float vol   = float(ap.volume);
        float chord = float(ap.surface.chord);
        float maxcs = float(ap.maxCrossSection);

        float thick   = (area  > 1e-12f) ? vol / area     : 0.f;   // mean plate thickness
        float t_chord = (chord > 1e-9f)  ? thick / chord  : 0.f;   // thickness-to-chord
        float a_cs    = (maxcs > 1e-12f) ? area / maxcs   : 0.f;   // face area / limb cross-section
        float AR      = (area  > 1e-12f) ? len * len / area : 0.f; // aspect ratio (span^2/area)

        printf("%-22s %-14s %8.4f %10.5f %10.6f %9.5f %9.4f %9.4f %9.2f %9.3f\n",
               rt.c_str(), FlagStr(ap.semantic_flags).c_str(),
               len, area, vol, thick, chord, t_chord, a_cs, AR);
    }
}

} // namespace

void DumpFlight(const char* label, const char* filename)
{
    Input input;
    input.environment = EarthAir();
    input.scale = length_b_to_m(1.0f);
    input.average_density = 0.5f;
    input.muscle_quality = 0.5f;
    input.stability_vs_speed = 0.5f;

    std::string path = std::string(TONTON_SAMPLE_MODELS_DIR) + "/" + filename;
    std::vector<const char*> args = { path.c_str() };
    auto files = GetArmaturesFromFiles({ args.data(), args.data() + args.size() });
    if (files.empty() || !files[0].memo || files[0].memo->size() == 0) return;
    input.builder = Builder::Factory(files[0].memo->at(0));
    auto out = Output::Factory(input);

    printf("\n----- %s flight/metabolism -----\n", label);
    printf("  body_mass_kg      = %.4f\n", float(out->physical.body_mass_kg));
    printf("  is_endotherm      = %d\n", (int)out->metabolic.is_endotherm());
    printf("  body_temp_K       = %.2f\n", float(out->metabolic.body_temperature_K));
    printf("  basal_rate_W      = %.4f\n", float(out->metabolic.basal_rate_W));
    printf("  max_rate_W        = %.4f\n", float(out->metabolic.max_rate_W));
    printf("  muscle_mass_kg    = %.4f\n", float(out->metabolic.muscle_mass_kg));
    printf("  aerial?           = %d\n", (int)out->aerial.has_value());
    if (out->aerial.has_value()) {
        auto const& a = *out->aerial;
        printf("  wingbeat_Hz       = %.2f\n", float(a.wingbeat_frequency_Hz));
        printf("  cruise_speed_m_s  = %.2f\n", float(a.cruise_speed_m_s));
        printf("  max_flight_m_s    = %.2f\n", float(a.max_flight_speed_m_s));
        printf("  wing_loading_N_m2 = %.2f\n", float(a.wing_loading_N_m2));
        printf("  hovering_power_W  = %.3f\n", float(a.hovering_power_W));
    }
}

TEST(WingDump, BatFlight)
{
    DumpFlight("bat", "batto.glb");
}

TEST(WingDump, AllModels)
{
    DumpModel("dragonfly", "dragonfly.glb");
    DumpModel("cat",       "cat.glb");
    DumpModel("shark",     "shark.glb");
    DumpModel("eel",       "eel.glb");
    DumpModel("penguin",   "penguin.glb");
    DumpModel("treefrog",  "treefrog.glb");
    DumpModel("bat",       "batto.glb");
}
