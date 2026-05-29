# LF_RINTINTIN

## Contributors

* Spehleon LP

## Status

Vendor extension

## Dependencies

Written against the glTF 2.0 spec.

## Overview

This extension provides per-joint geometric and physical properties for skinned meshes. It enables accurate physics simulation, procedural collider generation, and biomechanical analysis of rigged characters by storing precomputed bounding volumes and the second moment tensor (from which an inertia tensor is trivially derived) for each joint's influenced geometry.

Traditional approaches require runtime computation of these properties or rely on artist-authored approximations. This extension solves the problem by providing mathematically rigorous metrics computed from the actual mesh geometry, even for topologically complex or non-manifold meshes common in game assets.

## Motivation

Skinned mesh characters in games and simulations often require physical properties for:

- **Physics simulation**: Ragdoll dynamics, procedural animation
- **Collision detection**: Automatic collider generation from skeleton
- **Biomechanical analysis**: Performance estimation for creatures (flight, locomotion)
- **Animation retargeting**: Understanding mass distribution for IK solvers
- **Optimization**: Per-joint culling and LOD decisions

Computing second moment / inertia tensors and geometric properties at runtime is expensive. Artist-authored colliders are time-consuming and error-prone. This extension provides these properties as preprocessed data.

## Placement

This extension is added to nodes that have both a `mesh` and a `skin` property. The metrics are computed per-joint based on the vertices influenced by each joint in the bind pose.

```json
{
  "nodes": [
    {
      "mesh": 0,
      "skin": 0,
      "extensions": {
        "LF_RINTINTIN": {
          "metrics": [...],
          "eigenDecompositions": [...],
          "orientedBoundingBoxes": [...]
        }
      }
    }
  ]
}
```

## Properties

All arrays in this extension must have the same length as the number of joints in the referenced skin. Each element corresponds to the joint at that index.

All three property arrays are optional, but at least one must be present if the extension is used.

### Metrics

Per-joint geometric properties. All metrics are computed at unit density in the local space of the node.

|   |Type|Unit|Description|Required|
|---|----|----|-----------|--------|
|**volume**|`number`| m^3 |Volume of geometry influenced by this joint|No, default: `0.0`|
|**surfaceArea**|`number`| m^2 |Surface area of geometry influenced by this joint|No, default: `0.0`|
|**centroid**|`number[3]`| m |Center of mass in node-local coordinates|No, default: `[0.0, 0.0, 0.0]`|
|**min**|`number[3]` | m |Minimum corner of axis-aligned bounding box in node-local coordinates|No, default: `[0.0, 0.0, 0.0]`|
|**max**|`number[3]`| m | Maximum corner of axis-aligned bounding box in node-local coordinates|No, default: `[0.0, 0.0, 0.0]`|
|**secondMoment**|`number[6]`| m^5 |Second moment tensor `[Mxx, Myy, Mzz, Mxy, Mxz, Myz]` about the centroid, where `M_ij = integral (x_i - c_i)(x_j - c_j) dV`. Pre-multiplied by volume (assumes unit density); has units m^5. See "Deriving an inertia tensor" below.|No, default: `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`|

**Example:**

```json
"metrics": [
  {
    "volume": 0.00024,
    "surfaceArea": 0.045,
    "centroid": [0.0, 0.15, 0.0],
    "min": [-0.05, 0.1, -0.05],
    "max": [0.05, 0.2, 0.05],
    "secondMoment": [0.0008, 0.0012, 0.0008, 0.0, 0.0, 0.0]
  }
]
```

#### Deriving an inertia tensor

This is the main thing you will want to do with the second moment tensor. Note the
sign flip on off-diagonal terms — these go in opposite directions in the two
tensors:

```
Ixx = M.yy + M.zz       Ixy = -M.xy
Iyy = M.xx + M.zz       Ixz = -M.xz
Izz = M.xx + M.yy       Iyz = -M.yz
```

equivalently, `I = trace(M) * Identity - M` (with the standard physics sign
convention on off-diagonals). Both `M` and the resulting `I` are already volume-
weighted; multiply by material density to obtain a physical inertia tensor in
kg⋅m².

### Eigen Decompositions

Principal axes and eigenvalues of the second moment tensor, useful for oriented collider generation. (The second moment and inertia tensors share eigenvectors — they are simultaneously diagonalizable — so the rotation is equally valid as a frame for either.)

|   |Type|Description|Required|
|---|----|-----------|--------|
|**rotation**|`number[4]`|Quaternion (x, y, z, w) representing the principal axes orientation|Yes|
|**lambda**|`number[3]`|Eigenvalues `[λ_min, λ_mid, λ_max]` in ascending order|Yes|

The rotation quaternion transforms from node-local space to the principal axes frame where the second moment tensor (and therefore the inertia tensor) is diagonal.

**Example:**

```json
"eigenDecompositions": [
  {
    "rotation": [0.0, 0.0, 0.0, 1.0],
    "lambda": [0.00008, 0.00012, 0.00015]
  }
]
```

### Oriented Bounding Boxes

Tight-fitting oriented bounding boxes for each joint's influenced geometry.

|   |Type|Description|Required|
|---|----|-----------|--------|
|**rotation**|`number[4]`|Quaternion (x, y, z, w) representing OBB orientation|Yes|
|**translation**|`number[3]`|Center position of OBB in node-local coordinates|Yes|
|**scaling**|`number[3]`|Half-extents of the box along each axis|Yes|

The OBB is defined as a unit cube `[-1, 1]³` transformed by the given rotation, translation, and scaling.

**Example:**

```json
"orientedBoundingBoxes": [
  {
    "rotation": [0.0, 0.0, 0.0, 1.0],
    "translation": [0.0, 0.15, 0.0],
    "scaling": [0.05, 0.1, 0.05]
  }
]
```

## Reference Frame

All metrics are computed in the **bind pose** and expressed in the **local coordinate system of the node** to which the extension is attached. This allows metrics to be transformed into world space or joint-local space by applying the appropriate transformations from the glTF scene graph.

For runtime physics simulation:
1. Transform each joint's metrics by its current animated transformation matrix
2. Accumulate contributions from all joints to obtain total body properties
3. Apply standard rigid body dynamics

## Implementation Notes

### Non-Manifold Geometry

This extension is designed to work with real-world game assets that may have non-manifold topology, open meshes, self-intersections, or inconsistent winding. Implementations should use robust geometric algorithms that can handle degenerate cases.

### Mass and Density

The extension stores geometric properties (volume, second moment) but not mass directly. Applications should apply material density to convert volume to mass and to scale the derived inertia tensor:

```
mass = density × volume
inertia_kgm2 = density × (trace(M) * Id - M)
```

For biomechanical analysis, typical densities:
- Soft tissue: ~1000 kg/m³
- Bone: ~1800 kg/m³  
- Feathers: ~800 kg/m³
- Muscle: ~1060 kg/m³

### Coordinate System Conventions

glTF uses a right-handed coordinate system with +Y up, +Z forward (towards viewer), and +X right.

The `secondMoment` array stores `[Mxx, Myy, Mzz, Mxy, Mxz, Myz]` as the symmetric matrix:

```
     | Mxx  Mxy  Mxz |
M =  | Mxy  Myy  Myz |
     | Mxz  Myz  Mzz |
```

All entries are non-negated integrals. The corresponding inertia tensor follows the standard physics sign convention with negated off-diagonals (see "Deriving an inertia tensor" above).

## Schema

* [node.LF_RINTINTIN.schema.json](schema/node.LF_RINTINTIN.schema.json)

## Example Usage

### Procedural Capsule Collider Generation

Using eigen decompositions to create oriented capsule colliders:

```python
def generate_capsule_collider(eigen, metrics):
    # Principal axis with largest eigenvalue = capsule axis
    capsule_axis = eigen.rotation * [0, 0, 1]  
    
    # Use middle eigenvalue to estimate radius
    radius = sqrt(eigen.lambda[1] / metrics.volume)
    
    # Estimate height from bounding box
    height = distance(metrics.min, metrics.max)
    
    return Capsule(
        center=metrics.centroid,
        axis=capsule_axis,
        radius=radius,
        height=height
    )
```

### Runtime Physics Accumulation

```python
def compute_body_properties(animated_joints, rintintin_data):
    total_inertia = zeros(3, 3)
    total_mass = 0
    weighted_centroid = zeros(3)
    
    for joint_idx, joint_transform in enumerate(animated_joints):
        metrics = rintintin_data.metrics[joint_idx]
        mass = DENSITY * metrics.volume
        
        # Transform centroid to world space
        world_centroid = joint_transform * metrics.centroid
        weighted_centroid += mass * world_centroid
        total_mass += mass
        
        # Build inertia tensor from second moment (flip signs on off-diagonals!)
        M = metrics.secondMoment   # [Mxx, Myy, Mzz, Mxy, Mxz, Myz]
        I_local = matrix([
            [ M[1]+M[2], -M[3],     -M[4]    ],
            [-M[3],       M[0]+M[2], -M[5]    ],
            [-M[4],      -M[5],      M[0]+M[1]],
        ])
        I_world = joint_transform.rotation * I_local * joint_transform.rotation.T
        total_inertia += I_world + parallel_axis_term(mass, world_centroid)
    
    return PhysicsBody(
        mass=total_mass,
        center_of_mass=weighted_centroid / total_mass,
        inertia_tensor=total_inertia
    )
```

## Tools

* **rintintin** - C++ library and command-line tool for computing and adding this extension to glTF files
* **tonton** - Biomechanical analysis tool that uses LF_RINTINTIN data for creature performance estimation

