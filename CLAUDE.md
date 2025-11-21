# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TonTon Example is a C++ command-line pipeline that demonstrates procedural creature analysis from GLTF files. It combines **rintintin** (3D volumetric analysis) with **tonton** (biomechanical analysis) to extract physically-grounded locomotion parameters, behavioral profiles, and sensory capabilities from rigged 3D models.

**Core workflow:**
```
GLTF file (rigged character)
    ↓
rintintin-analyze (computes volumes, inertia, covariance)
    ↓
GLTF + LF_RINTINTIN extension
    ↓
tonton-analyze (biomechanical analysis)
    ↓
Physical parameters, locomotion capabilities, AI behavior hints
```

## Build Commands

```bash
# Configure and build
mkdir build && cd build
cmake ..
make

# Run volumetric analysis (outputs debug visualization + adds extension)
./rintintin-analyze ../path/to/model.gltf

# Run creature analysis (reads LF_RINTINTIN extension)
./tonton-analyze ../path/to/model.gltf

# Specify environment for analysis
./tonton-analyze model.gltf --env=ocean
./tonton-analyze model.gltf --env=proxima
```

**Build requirements:**
- CMake 3.15+
- C++20 compiler (C++17 minimum for some components)
- C99 compiler
- GLM (OpenGL Mathematics library)
- Threads support

**Executables built:**
- `rintintin-analyze`: Volumetric analysis tool
- `tonton-analyze`: Biomechanical analysis tool

## Architecture

### Two-Executable Pipeline

**1. rintintin-analyze** (`src/rintintin_main.cpp`):
- Entry point for volumetric analysis
- Reads GLTF files via fx-gltf library
- Creates `RintintinCommand` objects for each skinned mesh node
- Computes per-joint metrics: volume, centroid, inertia tensor, covariance, bounding boxes
- Outputs debug visualization (`*-balls.glb`) showing inertia ellipsoids
- Optionally saves enriched GLTF with `LF_RINTINTIN` extension via `-o` flag
- Processes files in parallel using futures

**2. tonton-analyze** (`src/tonton_main.cpp`):
- Entry point for biomechanical analysis
- Reads GLTF files with `LF_RINTINTIN` extension data
- Creates `TonTon::Input` from GLTF via `GltfMemo` wrapper
- Runs physics-based analysis via `TonTon::Output::Factory()`
- Prints formatted results to stdout (flight speeds, metabolic rates, behavior archetypes, etc.)
- Supports environment selection via `--env=` flag for different gravity/atmosphere conditions

### Bridge Layer (Core Architecture Pattern)

The critical architectural pattern is the **bridge layer** that adapts between GLTF data structures and analysis library APIs. This is the main code in this repository:

**GLTF → Rintintin Bridge** (`src/gltf_rintintin_bridge.h/cpp`):
- `RintintinMeshData`: Converts fx-gltf primitives to rintintin mesh format
- `RintintinSkinData`: Converts GLTF skin data to rintintin skeleton format
- `RintintinCommand`: High-level wrapper that orchestrates volumetric analysis
  - Manages async computation via `std::future`
  - Converts rintintin results to `LF::RinTinTin` extension format
  - Handles multiple primitives per node
- `RTT::GLTFAttributeData`: Helper for accessing GLTF buffer data safely

**GLTF → TonTon Bridge** (`src/tonton_fxgltf_memo.h/cpp`):
- `TonTon::GltfMemo`: Factory class that creates TonTon data structures from GLTF documents
  - Converts GLTF skins to `TonTon::Armature` (semantic bone hierarchy)
  - Converts GLTF meshes to `TonTon::Mesh` (DoDeeDum wrapper)
  - Combines into `TonTon::SkinnedMesh` with volumetric data from `LF_RINTINTIN` extension
  - Uses reference-counted smart pointers (`counted_ptr<T>`)
  - Lazily builds and caches data per node
- `GetArmaturesFromFiles()` (`src/get_armatures_from_files.h/cpp`): File loading utility

**Common Utilities:**
- `src/primitives.h/cpp`: GLTF primitive access helpers
- `src/lf_rintintin.h/cpp`: Extension data structure and JSON serialization
- `src/visualize_tensors.h/cpp`: Creates debug visualization GLB with inertia ellipsoids

### The LF_RINTINTIN Extension

This is a **custom GLTF 2.0 vendor extension** (namespace `LF_`) that stores precomputed volumetric data. Defined in:
- Data structures: `src/lf_rintintin.h`
- Serialization: `src/lf_rintintin.cpp`
- Specification: `LF_RINTINTIN.md`
- JSON schema: `node.LF_RINTINTIN.schema.json`

**Placement:** Attached to nodes with both `mesh` and `skin` properties.

**Data stored per joint** (all arrays same length as joint count):
- `metrics[]`: Volume, surface area, centroid, inertia tensor (6 values: Ixx, Iyy, Izz, Ixy, Ixz, Iyz), AABB, covariance
- `eigenDecompositions[]`: Principal axes (quat) and eigenvalues for inertia tensor
- `orientedBoundingBoxes[]`: Tight-fitting OBBs per joint (rotation, translation, scaling)

**Critical details:**
- All metrics computed in **bind pose** in **node-local coordinates**
- Inertia tensors assume **unit density** (multiply by material density for actual physics)
- Designed for non-manifold geometry (real game assets often have topology issues)
- Reference frame uses glTF conventions: right-handed, +Y up, +Z forward

### Submodule Dependencies

This repository integrates three submodules:

**1. rintintin** (GPL-2.0) - `modules/rintintin/`:
- 3D volumetric mesh analysis engine
- Computes inertia tensors even for non-manifold meshes
- C API: `rintintin.h` with types like `rintintin_mesh`, `rintintin_skin`, `rintintin_metrics`
- **License note:** This makes the entire pipeline GPL-2.0

**2. tonton** (Apache-2.0) - `modules/tonton/`:
- Biomechanical creature analysis library
- Semantic bone parsing via anatomical wordlist
- Physics-based locomotion rules in `src/Rules/`
- See `modules/tonton/CLAUDE.md` for detailed architecture
- Key feature: Does NOT compute volumes itself, requires external data (from rintintin)

**3. fx-gltf** (MIT) - `modules/fx-gltf/`:
- GLTF 2.0 loading library
- Provides `fx::gltf::Document`, `fx::gltf::Primitive`, etc.
- Uses nlohmann::json internally

**4. dodeedum** (MIT) - nested in `modules/tonton/modules/dodeedum/`:
- 2D projection and silhouette analysis
- Used internally by tonton for projected area calculations
- Bridge code at `modules/tonton/modules/dodeedum/extensions/dodeedum_fxgltf_bridge.cpp` (included in tonton executable build)

## Common Development Tasks

### Adding Support for New GLTF Extensions

If you need to read additional GLTF extension data:

1. Define C++ structure in appropriate bridge header
2. Add JSON deserialization in corresponding `.cpp` file using nlohmann::json
3. Update `RintintinCommand` or `GltfMemo` to extract and store the data
4. Pass to analysis libraries via their Input structures

### Modifying Volumetric Analysis Pipeline

The rintintin analysis flow:
1. `rintintin_main.cpp` loads GLTF → `fx::gltf::Document`
2. `ProcessFile()` iterates nodes with mesh+skin
3. `RintintinCommand::Factory()` creates analysis command
4. `RintintinCommand::GetMeshCommand()` returns C API command struct
5. Results converted to `LF::RinTinTin` via `MakeExtension()`
6. Extension serialized to JSON and attached to node

**Key function:** `createRintintinMeshFromPrimitive()` in `gltf_rintintin_bridge.cpp` - handles GLTF accessor/buffer/bufferView access patterns.

### Modifying Biomechanical Analysis

The tonton analysis flow:
1. `tonton_main.cpp` loads GLTF → `GetArmaturesFromFiles()`
2. `GltfMemo::operator[]()` lazily builds `SkinnedMesh` per node
3. `BuildRaw()` converts GLTF skin to `TonTon::Armature` with semantic parsing
4. `TonTon::Input` populated with environment, scale, density
5. `TonTon::Output::Factory()` runs all analysis rules
6. Results printed via `tonton_formatter.h` overloaded `operator<<`

**To add new creature parameters:** Modify `modules/tonton/src/Rules/` files and corresponding output structures in `modules/tonton/include/tonton_output.h`.

### Working with Semantic Bone Names

TonTon extracts meaning from bone names like "left_wing_tip" → `{left, wing, tip}`. This is critical for analysis quality.

**Recognized terms:** See `modules/tonton/include/tonton_wordlist.h` (~260 anatomical terms)
- Location: left, right, front, rear, upper, lower
- Body parts: wing, leg, tail, head, neck, spine
- Specializations: finger, talon, fin, tentacle, eye

**Bone naming best practices for input models:**
- Use snake_case or camelCase: "leftWing" or "left_wing"
- Include laterality: left/right for paired structures
- Use anatomical terms: "humerus" not "upper_arm_bone_1"
- Be specific: "wing_primary_feather" not just "feather"

## Code Style and Patterns

**File naming:**
- Snake_case: `gltf_rintintin_bridge.cpp`
- Headers include guards: `#ifndef GLTF_RINTINTIN_BRIDGE_H`

**Memory management:**
- Raw pointers in C API structs (rintintin)
- `std::unique_ptr` for owned heap data that can't move (mesh attributes)
- `std::shared_ptr` for shared ownership (GLTF documents)
- `counted_ptr<T>` for tonton reference counting (see tonton CLAUDE.md)
- `std::future` for async computation

**Bridge pattern structure:**
```cpp
// Create data on stack
RintintinMeshData createRintintinMeshFromPrimitive(...) {
    RintintinMeshData result;
    result.attributes = std::make_unique<...>();  // Heap allocation
    result.mesh.attributes = result.attributes.get();  // Point to heap
    // Fill in mesh data...
    return result;  // Move semantics preserve pointer validity
}
```

**Error handling:**
- Return bools for success/failure in low-level functions
- Use `RttErrorCode` enum for rintintin errors
- Print error messages to stderr, continue processing other files

## Important Implementation Details

**Coordinate systems:**
- glTF: Right-handed, +Y up, +Z forward (towards viewer), +X right
- Quaternions: GLM format (x, y, z, w)
- Matrices: GLM column-major

**Units (SI throughout):**
- Distance: meters
- Mass: kilograms
- Time: seconds
- Angles: radians (except output display)
- Density: kg/m³
- Inertia: kg⋅m² (but stored as unit-density m^5 in extension)

**GLTF accessor patterns:**
```cpp
// Standard pattern for reading GLTF buffers
auto& accessor = doc.accessors[accessor_idx];
auto& bufferView = doc.bufferViews[accessor.bufferView];
auto& buffer = doc.buffers[bufferView.buffer];
const uint8_t* data = buffer.data.data() + bufferView.byteOffset + accessor.byteOffset;
```

**Eigen decomposition:**
- Required by tonton: `std::pair<glm::quat, glm::vec3> EigenDecomposition(glm::dmat3 const&)`
- Declared in `gltf_rintintin_bridge.h`
- Implementation in `gltf_rintintin_bridge.cpp`
- Returns: quaternion (principal axes) + vec3 (eigenvalues sorted min→max)

## Environment Presets

The `tonton-analyze` tool supports different planetary/environmental conditions:

```bash
--env=air           # Earth atmosphere (default)
--env=ocean         # Earth ocean
--env=centauri      # Proxima Centauri b
--env=trappist      # TRAPPIST-1e
--env=422b          # Kepler-442b
--env=lhs           # LHS 1140b
--env=62f           # Kepler-62f
--env=18b           # K2-18b
--env=wolf          # Wolf 1061c
--env=gliese        # Gliese 667Cc
--env=europa        # Icy moon ocean
```

Each preset adjusts gravity, atmospheric density, fluid density, temperature - affecting metabolic rates, flight speeds, swimming capabilities, etc.

Defined in `modules/tonton/src/tonton_environments.cpp`.

## Input Model Requirements

For best analysis results, GLTF models should have:

1. **Rigged with skinning:**
   - Joint hierarchy (armature/skeleton)
   - Vertex weights binding mesh to joints
   - Rest pose transforms

2. **Semantic bone names:**
   - Use anatomical terms: wing, leg, tail, head, etc.
   - Include laterality: left/right for paired structures
   - See `modules/tonton/include/tonton_wordlist.h` for recognized vocabulary

3. **Closed meshes:**
   - Required for accurate volumetric analysis
   - Non-manifold geometry is tolerated but reduces accuracy

4. **Consistent units:**
   - Assume meters for all measurements
   - Models at wrong scale will produce nonsensical results (e.g., 1 unit = 1 foot breaks physics calculations)

## Debug Visualization

The `rintintin-analyze` tool creates `*-balls.glb` files showing inertia tensors as ellipsoids:
- Position: Joint centroid in bind pose
- Orientation: Principal axes of inertia (from eigen decomposition)
- Scale: Proportional to eigenvalues (larger = more rotational inertia)

Generated by `VisualizeTensors()` in `src/visualize_tensors.cpp`.

**Use cases:**
- Verify volumetric analysis is correct
- Debug skinning weight issues (isolated joints = wrong weights)
- Check mesh topology problems (collapsed ellipsoids = degenerate geometry)
- Understand mass distribution visually

## Licensing Note

This repository is **GPL-2.0** due to rintintin dependency. Individual components have different licenses:
- tonton-example bridge code: GPL-2.0 (this repository)
- rintintin: GPL-2.0
- tonton: Apache-2.0 (can be used independently)
- dodeedum: MIT
- fx-gltf: MIT

If GPL is problematic, you can use tonton independently with a different volumetric analysis solution.
