#include <iostream>
#include <filesystem>
#include <algorithm>
#include <unordered_map>
#include <cxxopts.hpp>
#include "fx/gltf.h"
#include "chacha.h"
#include "chacha_fxgltf_bridge.h"

// ---------------------------------------------------------------------------
// File I/O (adapted from rintintin_main.cpp)
// ---------------------------------------------------------------------------

static bool OpenFile(fx::gltf::Document& doc, const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path))
        throw std::runtime_error("No such file: " + path.string());

    std::string ext = path.extension().string();
    for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if (ext == ".gltf")
        doc = fx::gltf::LoadFromText(path);
    else if (ext == ".glb")
        doc = fx::gltf::LoadFromBinary(path);
    else
        throw std::runtime_error("Not a glTF file: " + path.string());

    return true;
}

static void SaveFile(fx::gltf::Document& doc, const std::filesystem::path& path)
{
    std::string ext = path.extension().string();
    for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    bool binary;
    if (ext == ".glb")
        binary = true;
    else if (ext == ".gltf")
        binary = false;
    else
        throw std::runtime_error("Unknown output extension: " + ext);

    for (auto& buf : doc.buffers) {
        buf.byteLength = static_cast<uint32_t>(buf.data.size());
        if (binary)
            buf.uri.clear();
        else
            buf.uri = std::filesystem::path(path).filename().replace_extension(".bin").string();
    }

    fx::gltf::Save(doc, path, binary);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    try {
        cxxopts::Options options("chacha-example",
            "Deduce joint articulation constraints from glTF animation data.\n"
            "Writes AGI_articulations extension to output file.");

        options.add_options()
            ("h,help", "Print usage")
            ("f,force", "Overwrite existing AGI_articulations")
            ("s,stdout", "Print articulations as JSON to stdout (no file output)")
            ("strip-animations", "Remove all animations from output file")
            ("rotation-threshold",
             "Noise threshold for rotation in radians",
             cxxopts::value<float>()->default_value("0.01"))
            ("translation-threshold",
             "Noise threshold for translation in meters",
             cxxopts::value<float>()->default_value("0.001"))
            ("input", "Input glTF/GLB file", cxxopts::value<std::string>())
            ("output", "Output glTF/GLB file (default: <input>-chacha.<ext>)",
             cxxopts::value<std::string>())
            ;

        options.parse_positional({"input", "output"});
        options.positional_help("INPUT [OUTPUT]");

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help() << std::endl;
            return 0;
        }

        if (!result.count("input")) {
            std::cerr << "Error: No input file specified\n\n";
            std::cout << options.help() << std::endl;
            return 4;
        }

        // Resolve paths
        std::filesystem::path input_path = result["input"].as<std::string>();
        std::filesystem::path output_path;
        bool stdout_mode = result.count("stdout") > 0;

        if (result.count("output")) {
            output_path = result["output"].as<std::string>();
        } else if (!stdout_mode) {
            auto stem = input_path.stem().string();
            auto ext = input_path.extension().string();
            output_path = input_path.parent_path() / (stem + "-chacha" + ext);
        }

        // Load
        fx::gltf::Document doc;
        try {
            OpenFile(doc, input_path);
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
            return 4;
        }

        // Check preconditions
        if (ChaChaFxGltf::has_agi_articulations(doc) && !result.count("force")) {
            std::cerr << "Error: File already has AGI_articulations extension.\n"
                      << "Use --force to overwrite.\n";
            return 1;
        }

        if (doc.animations.empty()) {
            std::cerr << "Error: No animations found in " << input_path << "\n"
                      << "ChaCha requires animation data to deduce joint constraints.\n";
            return 2;
        }

        // Find first skin
        int skin_index = -1;
        for (size_t i = 0; i < doc.nodes.size(); ++i) {
            if (doc.nodes[i].skin >= 0 && doc.nodes[i].skin < static_cast<int>(doc.skins.size())) {
                skin_index = doc.nodes[i].skin;
                break;
            }
        }
        if (skin_index < 0 && !doc.skins.empty())
            skin_index = 0;

        if (skin_index < 0) {
            std::cerr << "Error: No skin/skeleton found in " << input_path << "\n"
                      << "ChaCha requires a skeleton to analyze animation data.\n";
            return 2;
        }

        // Extract data
        auto extracted_anims = ChaChaFxGltf::extract_animation_channels(doc);
        auto extracted_skel = ChaChaFxGltf::extract_skeleton(doc, skin_index);

        if (extracted_anims.channels.empty()) {
            std::cerr << "Error: No usable animation channels found.\n"
                      << "Channels must target rotation, translation, or scale.\n";
            return 2;
        }

        // Remap channel node indices from glTF global to skeleton-local joint indices
        std::unordered_map<uint32_t, int> node_to_joint;
        for (size_t i = 0; i < extracted_skel.joint_nodes.size(); ++i)
            node_to_joint[extracted_skel.joint_nodes[i]] = static_cast<int>(i);

        std::vector<ChaCha::AnimationChannel> remapped_channels;
        remapped_channels.reserve(extracted_anims.channels.size());
        for (auto ch : extracted_anims.channels) {
            auto it = node_to_joint.find(static_cast<uint32_t>(ch.node));
            if (it == node_to_joint.end()) continue; // channel targets a non-skeleton node
            ch.node = it->second;
            remapped_channels.push_back(ch);
        }

        if (remapped_channels.empty()) {
            std::cerr << "Error: No animation channels target skeleton joints.\n";
            return 2;
        }

        // Run analysis
        ChaCha::Options chacha_opts;
        chacha_opts.rotation_threshold_rad = result["rotation-threshold"].as<float>();
        chacha_opts.translation_threshold_m = result["translation-threshold"].as<float>();

        auto articulations = ChaCha::analyze(
            std::span<const ChaCha::AnimationChannel>(remapped_channels),
            extracted_skel.as_skeleton(),
            chacha_opts);

        if (articulations.empty()) {
            std::cerr << "Error: No articulations produced.\n"
                      << "All observed motion was below the noise threshold.\n"
                      << "Try lowering --rotation-threshold (current: "
                      << chacha_opts.rotation_threshold_rad << " rad) or\n"
                      << "--translation-threshold (current: "
                      << chacha_opts.translation_threshold_m << " m).\n";
            return 3;
        }

        std::cerr << "Found " << articulations.size() << " articulated joints.\n";

        // Output
        if (stdout_mode) {
            ChaChaFxGltf::print_articulations_json(
                std::cout, doc, articulations, extracted_skel.joint_nodes);
            return 0;
        }

        // Write extension to document
        ChaChaFxGltf::write_agi_articulations(doc, articulations, extracted_skel.joint_nodes);

        // Remove AGI_ prefix animations
        ChaChaFxGltf::remove_agi_animations(doc);

        // Optionally strip all animations
        if (result.count("strip-animations")) {
            doc.animations.clear();
            // Note: we don't clean up orphaned accessors/bufferViews here
            // as that would require a full document walk similar to remove_agi_animations
        }

        // Save
        try {
            SaveFile(doc, output_path);
            std::cerr << "Wrote: " << output_path << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Error writing output: " << e.what() << "\n";
            return 4;
        }

    } catch (const cxxopts::exceptions::exception& e) {
        std::cerr << "Error parsing arguments: " << e.what() << "\n";
        return 4;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 4;
    }

    return 0;
}
