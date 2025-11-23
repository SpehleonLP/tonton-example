# GEMINI.md

This file provides guidance to the Gemini agent when working with code in this repository.

## Project Overview

This repository, `tonton-example`, is a C++ command-line pipeline that demonstrates procedural creature analysis from rigged 3D GLTF files. It integrates two main libraries, `rintintin` and `tonton`, to perform a comprehensive analysis of a creature's physical and behavioral characteristics based on its 3D model.

The core workflow is a two-step process:

1.  **Volumetric Analysis (`rintintin`)**: The `rintintin-analyze` executable processes a GLTF file, calculating volumetric properties for each joint in the model's skeleton (e.g., volume, inertia tensors, centers of mass). It enriches the input GLTF file with this data using a custom `LF_RINTINTIN` extension.
2.  **Biomechanical Analysis (`tonton`)**: The `tonton-analyze` executable reads the enriched GLTF file and uses the volumetric data, along with semantic analysis of bone names, to apply physics-based biomechanical models. It outputs a detailed report on the creature's physical properties, locomotion capabilities (e.g., flight speed, gait), metabolic rates, and inferred AI behavior archetypes.

This pipeline is designed for applications like game development and procedural animation, allowing developers to automatically generate character stats and movement parameters from 3D art assets.

## Build and Run Commands

The project uses CMake and requires a C++17 compatible compiler.

### Build Steps

```bash
# 1. Clone the repository with its submodules
git clone --recursive https://github.com/SpehleonLP/tonton-example.git
cd tonton-example

# 2. Create a build directory
mkdir build
cd build

# 3. Configure and build the project
cmake ..
make
```

The executables `rintintin-analyze` and `tonton-analyze` will be located in the `build/` directory.

### Example Workflow

```bash
# Assuming you are in the build/ directory

# Step 1: Run volumetric analysis on a sample model
# This creates 'batto.glb' in the same directory, now with the LF_RINTINTIN extension.
./rintintin-analyze ../sample\ models/batto.glb

# Step 2: Run biomechanical analysis on the enriched model
# The output is printed to standard output.
./tonton-analyze ../sample\ models/batto.glb

# You can redirect the output to a file
./tonton-analyze ../sample\ models/batto.glb > batto_analysis.txt
```

## Architecture

The primary role of this repository is to act as a **bridge** between the `fx-gltf` library and the `rintintin` and `tonton` analysis libraries.

### Key Components

*   **`src/`**: Contains the main entry points (`rintintin_main.cpp`, `tonton_main.cpp`) and the "bridge" code that translates data between the libraries.
    *   `gltf_rintintin_bridge.h/.cpp`: Adapts `fx-gltf` data structures to the C-style API of the `rintintin` library.
    *   `tonton_fxgltf_memo.h/.cpp`: Adapts `fx-gltf` data, including the custom `LF_RINTINTIN` extension data, to the C++ API of the `tonton` library.
    *   `lf_rintintin.h/.cpp`: Defines the structure and serialization/deserialization logic for the custom `LF_RINTINTIN` GLTF extension.
*   **`modules/`**: Contains the core logic as Git submodules.
    *   **`rintintin/`** (GPL-2.0): A C library for 3D volumetric analysis of meshes.
    *   **`tonton/`** (Apache-2.0): A C++ library for biomechanical analysis. It relies on semantic bone names (`tonton_wordlist.h`) and physics-based rules (`src/Rules/`).
    *   **`fx-gltf/`** (MIT): A header-only C++ library for loading GLTF 2.0 files.
    *   **`dodeedum/`** (MIT): A C++ library for 2D projection and silhouette analysis, used by `tonton`.
*   **`LF_RINTINTIN` Custom GLTF Extension**: This is the data pipeline that connects the two executables. It's a JSON object embedded in the GLTF file that stores per-joint volumetric data computed by `rintintin`, ready to be consumed by `tonton`.

### Data Flow

1.  `rintintin-analyze` uses `fx-gltf` to load a model.
2.  `gltf_rintintin_bridge` converts the GLTF mesh and skin data into a format `rintintin` can process.
3.  `rintintin` computes metrics like volume, inertia, and covariance.
4.  The results are converted into the `LF_RINTINTIN` extension format and saved to the GLTF file.
5.  `tonton-analyze` uses `fx-gltf` to load the enriched model.
6.  `tonton_fxgltf_memo` parses the model, including the `LF_RINTINTIN` data and semantic bone names.
7.  `tonton`'s rules engine uses this input to calculate and output the final analysis.

## Development Conventions

*   **Code Style**: The codebase primarily uses `snake_case` for file naming and `PascalCase` for types. C++17 features are used, with `tonton` specifically requiring C++20.
*   **Memory Management**: The project uses a mix of raw pointers (for C APIs), `std::unique_ptr`, `std::shared_ptr`, and a custom `counted_ptr` within the `tonton` library.
*   **Semantic Bone Naming**: For the `tonton` analysis to be effective, input GLTF models must use anatomical terms in their bone names (e.g., "left_wing", "tail_tip", "head"). A list of recognized terms can be found in `modules/tonton/include/tonton_wordlist.h`.
*   **Units**: The entire pipeline assumes SI units (meters, kilograms, seconds).
*   **Licensing**: Due to the `rintintin` dependency, the combined pipeline in this repository is licensed under **GPL-2.0**. The individual submodules have their own licenses (Apache-2.0 for `tonton`, MIT for `fx-gltf` and `dodeedum`).

This `GEMINI.md` file should provide a solid foundation for understanding the project's structure and goals.
