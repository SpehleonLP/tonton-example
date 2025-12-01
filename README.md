# TonTon Example Pipeline

This repository provides command-line tools that demonstrate the complete pipeline for procedural creature analysis from GLTF files. It combines [rintintin](https://github.com/SpehleonLP/rintintin) (3D volumetric analysis) and [tonton](https://github.com/SpehleonLP/tonton) (biomechanical analysis) to extract physically-grounded locomotion parameters from rigged 3D models.

## What This Does

**Complete workflow:**
```
GLTF file (rigged character)
    ↓
rintintin (computes volumes, inertia, covariance matrices)
    ↓
GLTF + LF_RINTINTIN extension (enriched with volumetric data)
    ↓
tonton Builder (processes raw geometric measurements)
    ↓
tonton Analysis (applies biomechanical models with dimensional analysis)
    ↓
Physical parameters, locomotion capabilities, behavioral profile
```

**Key features:**
- **Compile-time dimensional analysis**: Uses C++ template metaprogramming (`Quantity<M,L,T>`) to catch unit errors at compile time
- **Three-tier architecture**: SkinnedMesh → Builder (raw measurements) → Output (physics-based analysis)
- **Configurable parameters**: Tune muscle quality, feather quality, behavior, environment, and more via command-line
- **Multiple environments**: Test creatures in Earth air/ocean, Titan's atmosphere, or exoplanet conditions

## Executables

This repository builds two command-line tools:

### 1. `rintintin-analyze` - Volumetric Analysis

Processes GLTF files to compute volumetric properties for each joint in skinned meshes.

**Usage:**
```bash
rintintin-analyze <input.gltf>
```

**Output:**
- Creates `<input.gltf>-balls.glb` - Debug visualization with inertia tensors rendered as ellipsoids
- Adds `LF_RINTINTIN` extension to the original GLTF (when used with `-o` flag)

**What it computes:**
- Volume and surface area per joint
- 3D centroid (center of mass)
- Inertia tensor (6 values: Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
- Covariance matrix (3 eigenvalues)
- Eigendecomposition (rotation + lambda values)
- Oriented bounding boxes
- Axis-aligned bounding boxes (min, max)

**The LF_RINTINTIN Extension:**

This is a **custom GLTF 2.0 extension** (not approved by Khronos) defined in [src/lf_rintintin.h](src/lf_rintintin.h). It stores volumetric analysis data on nodes that bind skins to meshes.
Note: rintintin does not check that the tensors are positive-determinent, though they will be for well formed input. 

Structure:
```json
{
  "extensions": {
    "LF_RINTINTIN": {
      "metrics": [
        {
          "volume": 0.0234,
          "surfaceArea": 0.456,
          "centroid": [0.0, 0.5, 0.0],
          "inertia": [0.001, 0.002, 0.0015, 0.0, 0.0, 0.0],
          "min": [-0.1, 0.0, -0.1],
          "max": [0.1, 1.0, 0.1],
          "covariance": [0.5, 0.3, 0.2]
        }
      ],
      "eigenDecompositions": [
        {
          "rotation": [1.0, 0.0, 0.0, 0.0],
          "lambda": [0.5, 0.3, 0.2]
        }
      ],
      "orientedBoundedBoxes": [
        {
          "rotation": [1.0, 0.0, 0.0, 0.0],
          "translation": [0.0, 0.5, 0.0],
          "scaling": [0.2, 1.0, 0.2]
        }
      ]
    }
  }
}
```

Each array has one entry per joint in the skeleton, indexed by joint order.

### 2. `tonton-analyze` - Biomechanical Analysis

Reads GLTF files and performs creature analysis with configurable parameters. if LF_RINTINTIN is provided it will use those values, otherwise it will run rintintin itself. 

**Usage:**
```bash
tonton-analyze [options] <file1.gltf> [file2.gltf ...]

Options:
  -h, --help                Show help message

  Environment:
  --env <preset>            Environment preset (default: air)
                            Options: air, ocean, titan, centauri, trappist,
                                     422b, lhs, 62f, 18b, wolf, gliese, europa

  Physical Parameters (0.0-1.0 normalized):
  --scale <float>           Scale multiplier (default: 1.0)
  --density <float>         Body density (0.0=700kg/m³, 1.0=1050kg/m³, default: 0.5)
  --structure <float>       Structural quality (default: 0.5)
  --muscle <float>          Muscle quality (default: 0.5)
  --feathers <float>        Feather/membrane quality (default: 0.5)
  --metabolism <float>      Metabolic efficiency (default: 0.5)
  --stability <float>       Stability vs speed (0=speed, 1=hovering, default: 0.5)
  --activity <float>        Activity multiplier (default: 1.0)
  --scaling <float>         Wing scaling factor (default: 1.0)
  --climbing <float>        Climbing capability multiplier (default: 1.0)

  Behavior Parameters (0.0-1.0 normalized):
  --coloration <float>      Coloration conspicuousness (default: 0.5)
  --aggression <float>      Aggression adjustment (default: 0.0)
  --activity-adjust <float> Activity level adjustment (default: 0.0)
  --endurance <float>       Endurance adjustment (default: 0.0)
  --risk <float>            Risk tolerance adjustment (default: 0.0)
  --social <float>          Social tendency adjustment (default: 0.0)
  --seasonal <float>        Seasonal behavior adjustment (default: 0.0)
  --circadian <float>       Circadian rhythm adjustment (default: 0.0)
  --adaptability <float>    Behavioral adaptability (default: 0.5)
  
  Mana Parameters :
  --water <float>           Aristotle: takes shape of container, flows and changes. improves ability to change direction.
  --fire <float>            Aristotle: source of motion and life
  --earth <float>           Aristotle: heavy and stable. behaves as if cross section of limbs is increased.
  --air <float>             Aristotle: medium actively pushes objects along their path. behaves as if propulsion surfaces are bigger.
  --aether <float>          Aristotle: celestial element, falls upward. adds bouyancy force making things floatier. 
  --shadow <float>          stick to surfaces, conform to terrain. increases traction and grip.
```

**Understanding Input Parameters: Natural Variation vs. Mystical Corrections**

TonTon analyzes creatures using **allometric scaling laws** - empirical relationships observed across real animals. For example, "bird wing area typically scales with body mass^0.67" based on studies of hundreds of species.

These biological relationships aren't perfect. When research papers report R² = 0.7, that means:
- **70% of variation is explained** by the formula (mass → wing area relationship)
- **30% is unexplained** - real animals naturally vary around the trend line

A hummingbird and an eagle of the same mass don't have *exactly* the same wing area predicted by the formula. One might have 15% more, the other 15% less, depending on ecological niche, flight style, and evolutionary history.

**Standard Parameters: Positioning Within Natural Biological Variance**

Parameters like `--muscle`, `--feathers`, `--structure` let you position a creature within that observed biological variation:

- **0.0** = bottom of the natural range (viable but suboptimal)
- **0.5** = average (exactly on the regression line from research papers)
- **1.0** = top of the natural range (exceptional but still biologically plausible)

These don't break physics - they pick where in the spectrum of *real observed animals* your creature falls. Think of it as tuning whether you want a "below average" bird or an "Olympic athlete" bird.

**Mana Parameters: Breaking Physics for Impossible Creatures**

Sometimes you need to analyze creatures that **violate the laws of physics** - like dragons with wings far too small to generate enough lift, or pegasi that are too heavy to fly with horse-sized bodies.

Mana parameters are **mystical corrections** that bend the rules:

- `--aether <value>` - Reduces effective weight by 2^value (things weigh less, fall slower)
- `--air <value>` - Increases wing/propulsion thrust by 2^value (air actively pushes the creature)
- `--fire <value>` - Boosts metabolic power by 2^value (more energy available)
- `--earth <value>` - Increases structural strength (as if bones/limbs were thicker)
- `--water <value>` - Improves maneuverability and directional changes
- `--shadow <value>` - Increases traction and grip on surfaces

**Examples:**
- **Dragons** have a lot of wind force, so you'd use `--air=2` to increase their wing thrust by 4×
- **Pegasus** is much more graceful and light, so you'd use `--aether=2` to treat it as if it weighs 4× less

Without these corrections, the analysis would correctly report "this creature cannot physically fly" - mana parameters let you model *how* the magic makes it work.

**Command-Line Examples:**
```bash
# Basic analysis with defaults
tonton-analyze dragon.gltf

# High-quality bird with strong muscles
tonton-analyze --muscle 0.9 --feathers 0.95 --metabolism 0.8 bird.gltf

# Aquatic creature analysis
tonton-analyze --env ocean --density 0.3 fish.gltf

# Titan atmosphere (low gravity, dense atmosphere)
tonton-analyze --env titan --scale 10.0 dragonfly.gltf

# Aggressive predator with high endurance
tonton-analyze --aggression 0.8 --endurance 0.7 predator.gltf

# Analyze multiple files with same parameters
tonton-analyze --env ocean *.gltf
```

**Environment Presets:**
- `air` - Earth atmosphere (default)
- `ocean` - Earth ocean
- `titan` - Saturn's moon Titan (low gravity, dense atmosphere, very cold)
- `centauri` - Proxima Centauri b
- `trappist` - TRAPPIST-1e
- `422b` - Kepler-442b
- `lhs` - LHS 1140b
- `62f` - Kepler-62f
- `18b` - K2-18b
- `wolf` - Wolf 1061c
- `gliese` - Gliese 667Cc
- `europa` - Europa ocean

**Output:**
Prints analysis results to stdout using the tonton std::format formatter. Example output:
```
====================================================================
PHYSICAL PROPERTIES
  Body Mass: 2.45 kg
  Body Length: 0.85 m
  Body Volume: 0.00234 m³
  Surface Area: 1.23 m²
  Cross-sectional Area: 0.045 m²
  Fineness Ratio: 5.2

AERIAL LOCOMOTION
  Wing Span: 1.8 m
  Wing Area: 0.34 m²
  Wing Loading: 70.5 N/m²
  Wingbeat Frequency: 12.3 Hz
  Min Flight Speed (stall): 8.5 m/s
  Cruise Speed: 15.2 m/s
  Max Flight Speed: 22.8 m/s
  Hovering Efficiency: 0.0 (not viable)
  Flapping Efficiency: 0.85

METABOLIC
  Basal Rate: 18.5 W
  Max Rate: 277.5 W
  Aerobic Scope: 15.0x
  Muscle Mass: 1.10 kg (45% of body)
  Available Power: 440 W

BEHAVIOR
  AI Archetype: AERIAL_PREDATOR
  Aggression: 0.72
  Social Tendency: 0.25 (solitary)
  Activity Level: 0.68
  Diurnal Preference: 0.80 (diurnal)
  Is Migratory: false

SENSORY SYSTEMS
  Vision Acuity: 0.85
  Binocular Overlap: 0.65
  Detection Range: 450 m
  Has Color Vision: true
  Has Night Vision: false
...
```

## Example Workflow

```bash
# Step 1: Compute volumetric data
./rintintin-analyze dragon.gltf

# Step 2: View debug visualization (optional)
# Open dragon.gltf-balls.glb in a GLTF viewer
# You'll see ellipsoids representing inertia tensors for each joint

# Step 3: Run creature analysis with default parameters
./tonton-analyze dragon.gltf > dragon-analysis.txt

# Step 4: Review the physical parameters
cat dragon-analysis.txt

# Advanced: Customize analysis parameters
./tonton-analyze \
  --env air \
  --muscle 0.8 \
  --feathers 0.9 \
  --metabolism 0.7 \
  --aggression 0.6 \
  dragon.gltf > dragon-custom.txt

# Analyze for different environments
./tonton-analyze --env ocean --density 0.3 sea_creature.gltf
./tonton-analyze --env titan --scale 5.0 giant_dragonfly.gltf
```

## Building

This project uses CMake and requires C++20.

### Prerequisites
- CMake 3.15+
- **C++20 compiler** (GCC 13+, Clang 14+, or MSVC 19.29+)
  - Required for `std::format` in tonton formatter
  - Some components may work with C++17, but C++20 is recommended
- GLM (OpenGL Mathematics library)

### Build Steps
```bash
# Clone with submodules
git clone --recursive https://github.com/SpehleonLP/tonton-example.git
cd tonton-example

# Build
mkdir build && cd build
cmake ..
make

# Executables will be in build/
./rintintin-analyze ../path/to/model.gltf
./tonton-analyze ../path/to/model.gltf
```

## Repository Structure

```
tonton-example/
├── src/
│   ├── rintintin_main.cpp          # rintintin executable entry point
│   ├── tonton_main.cpp             # tonton executable entry point
│   ├── lf_rintintin.h/.cpp         # Custom GLTF extension definition
│   ├── gltf_rintintin_bridge.h/.cpp # Bridge between GLTF and rintintin
│   ├── tonton_fxgltf_memo.h/.cpp   # Bridge between GLTF and tonton
│   ├── visualize_tensors.h/.cpp    # Debug visualization generator
│   ├── get_armatures_from_files.h/.cpp # GLTF loading utilities
│   └── primitives.h/.cpp           # Mesh primitive handling
├── modules/
│   ├── rintintin/                  # GPL-2.0: Volumetric analysis
│   ├── tonton/                     # Apache-2.0: Biomechanical analysis
│   │   └── dodeedum/              # MIT: 2D projection/silhouette
│   └── fx-gltf/                    # MIT: GLTF loading library
├── sample models/                  # Example GLTF files (if any)
├── CMakeLists.txt
├── LICENSE                         # GPL-2.0
└── README.md
```

## Submodules

This repository includes three submodules:

- **[rintintin](https://github.com/SpehleonLP/rintintin)** (GPL-2.0) - 3D volumetric mesh analysis
- **[tonton](https://github.com/SpehleonLP/tonton)** (Apache-2.0) - Biomechanical creature analysis
  - Includes **[dodeedum](https://github.com/SpehleonLP/dodeedum)** (MIT) as nested submodule
- **[fx-gltf](https://github.com/jessey-git/fx-gltf)** (MIT) - GLTF 2.0 loading library

## Requirements for Input Models

For best results, your GLTF models should:

1. **Have semantic bone names**: Use anatomical terms like "left_wing", "right_hind_leg", "tail_tip", "jaw", "eye", etc.
   - See [tonton/include/tonton_wordlist.h](modules/tonton/include/tonton_wordlist.h) for recognized terms

2. **Be rigged with skinning**: The mesh should have:
   - Joint hierarchy (armature)
   - Vertex weights binding mesh to joints
   - Rest pose positions and rotations

3. **Have proper topology**:
   - Closed meshes (for volumetric analysis)
   - Reasonable triangle density
   - Proper winding order

4. **Use consistent units**: Meters are assumed for all measurements

## Debug Visualization

The `rintintin` tool creates a debug visualization file (`*-balls.glb`) that shows:
- **Ellipsoids** representing the inertia tensor for each joint
- Oriented to match the principal axes of rotation
- Scaled according to the eigenvalues (larger = more rotational inertia)

This is useful for:
- Verifying the volumetric analysis is correct
- Understanding mass distribution visually
- Debugging rigging issues
- Tuning mesh topology for better results

Example: A wing joint should show an ellipsoid elongated along the wing's length, indicating high rotational inertia about that axis.

## Limitations & Known Issues

1. **GLTF Extension Not Standard**: The `LF_RINTINTIN` extension is custom and won't be recognized by standard GLTF tools. This is intentional - it's a data pipeline extension, not meant for rendering.

2. **Memory Usage**: Large models with many joints can consume significant memory during volumetric analysis.

3. **Processing Time**: Volumetric analysis is computationally expensive. Complex characters may take several seconds to process.

4. **Semantic Parsing**: TonTon relies on bone names containing recognizable anatomical terms. Non-standard naming conventions may reduce analysis quality.

5. **GPL Licensing**: Because this integrates rintintin (GPL-2.0), the complete pipeline is GPL-licensed even though tonton itself is Apache-2.0.

## Use Cases

- **Game Development**: Auto-generate creature stats and movement parameters from art assets
- **Procedural Animation**: Compute IK chain parameters, gait cycles, wingbeat frequencies
- **AI Behavior**: Generate behavioral profiles from morphology (predator/prey, diurnal/nocturnal, social/solitary)
- **Physics Simulation**: Extract mass properties and inertia tensors for realistic physics
- **Research**: Study relationships between anatomy and locomotion capabilities
- **Prototyping**: Rapidly iterate on creature designs with instant physical feedback

## Related Projects

- **[rintintin](https://github.com/SpehleonLP/rintintin)** - The volumetric analysis engine
- **[tonton](https://github.com/SpehleonLP/tonton)** - The biomechanical analysis library
- **[dodeedum](https://github.com/SpehleonLP/dodeedum)** - 2D silhouette and projection analysis

Each can be used independently, but this repository shows them working together as a complete pipeline.

## Contributing

Contributions welcome! Areas of interest:
- Support for additional GLTF extensions
- Performance optimizations
- Better semantic parsing
- Additional locomotion modes
- Improved debug visualizations
- Example models

If you have expertise in biomechanics, comparative physiology, or animal locomotion, please review the [tonton rules engine](modules/tonton/src/Rules/) and contribute improvements!

## License

This repository is licensed under **GPL-2.0** due to its dependency on rintintin.

Individual submodules have their own licenses:
- rintintin: GPL-2.0
- tonton: Apache-2.0
- dodeedum: MIT
- fx-gltf: MIT

See [LICENSE](LICENSE) for full GPL-2.0 terms.

## References

For the scientific basis of the biomechanical models, see [modules/tonton/bibliography.md](modules/tonton/bibliography.md).
