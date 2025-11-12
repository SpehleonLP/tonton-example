# Sample Models

This directory contains test models for the tonton/rintintin pipeline.

## Source

These models are derived from **David O'Reilly's Everything Library - Animals**:
- Original source: https://davidoreilly.itch.io/everything-library-animals
- Original creator: [David O'Reilly](https://www.davidoreilly.com/)
- Original license: MIT License

## Modifications

The models in this directory have been modified from the originals:

1. **Scaling**: Each animal has been scaled to realistic proportions based on typical adult heights for the species
2. **Rigging**: Skeletal armatures have been added with semantic bone names for tonton analysis
3. **Skinning**: Vertex weights have been painted to bind the mesh to the skeleton

These modifications make the models suitable for testing the volumetric and biomechanical analysis pipeline.

## Why These Models?

David O'Reilly's Everything Library models are excellent test cases because:
- **Diverse anatomy**: Wide variety of body plans (quadrupeds, birds, fish, insects, etc.)
- **Clean topology**: Well-constructed meshes suitable for volumetric analysis
- **Low-poly aesthetic**: Fast processing while still anatomically representative
- **Recognizable species**: Easy to validate if analysis results make biological sense
- **Permissive license**: MIT allows modification and redistribution

## Usage

These models are intended as:
- **Test cases** for the rintintin/tonton pipeline
- **Reference examples** showing proper bone naming conventions
- **Benchmarks** for validating analysis results against known biology
- **Learning resources** for understanding how anatomy affects computed parameters

To process a model:
```bash
# Compute volumetric properties
../build/rintintin "animal_name.gltf"

# View debug visualization
# Open animal_name.gltf-balls.glb in a GLTF viewer

# Run biomechanical analysis
../build/tonton "animal_name.gltf"
```

## Semantic Bone Naming

The rigs use anatomical terminology recognized by tonton's semantic parser:
- Limbs: `left_front_leg`, `right_hind_leg`, `left_wing`
- Spine: `spine_root`, `spine_thorax`, `spine_pelvis`, `neck`, `head`
- Extremities: `tail_root`, `tail_tip`, `jaw`, `eye`, `ear`
- Digits: `left_front_foot_toe_1`, `finger`, `thumb`
- Aquatic: `fin_dorsal`, `fin_pectoral`, `fin_caudal`

See [../modules/tonton/include/tonton_wordlist.h](../modules/tonton/include/tonton_wordlist.h) for the complete list of recognized terms.

## Expected Results

After processing, you should see analysis results that match biological expectations:

**Example - Bird:**
- Lightweight body (low density)
- High wing loading for the size
- Endothermic metabolism
- High aerobic scope (15x)
- Forward-facing eyes (binocular vision)
- Aerial predator or forager behavior

**Example - Fish:**
- Neutral buoyancy density
- Streamlined body (high fineness ratio)
- Lateral eyes (prey species) or forward eyes (predator)
- BCF or MPF propulsion mode
- Ectothermic metabolism

**Example - Quadruped:**
- Erect or semi-erect posture
- Multiple gait patterns (walk, trot, gallop)
- Sprint vs sustainable speed ratios
- Eye position correlates with predator/prey status

## License

These modified models are released under the same **MIT License** as the originals.

Original models Copyright (c) David O'Reilly
Modified versions Copyright (c) 2025 Spehleon LP

Permission is hereby granted, free of charge, to any person obtaining a copy of these models and associated files, to deal in them without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies, and to permit persons to whom the models are furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the models.

THE MODELS ARE PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE MODELS OR THE USE OR OTHER DEALINGS IN THE MODELS.

## Attribution

When using these models, please credit:
- Original models: David O'Reilly (https://davidoreilly.itch.io/everything-library-animals)
- Rigging/skinning modifications: Spehleon LP

## Contributing Additional Models

If you'd like to contribute additional test models:
1. Use the anatomical naming conventions from tonton_wordlist.h
2. Ensure meshes are closed and manifold for volumetric analysis
3. Scale to realistic proportions (meters as unit)
4. Test with the pipeline and verify results make biological sense
5. Include attribution for the original mesh source
6. Ensure licensing is compatible (MIT, Apache, CC0, or similar)
