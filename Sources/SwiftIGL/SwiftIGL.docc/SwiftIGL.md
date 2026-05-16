# ``SwiftIGL``

Swift bindings for libigl — a simple C++ geometry processing library.

## Overview

SwiftIGL wraps [libigl](https://libigl.github.io) and Eigen through
Swift's C++ interop.

SwiftIGL exposes a narrow, hand-curated slice of libigl through Swift's C++
interop. The current surface is sized to support voxelization-style
pipelines: mesh I/O, normals, signed distance fields, marching cubes, and
a few common cleanup operations.

All buffers are **flat row-major**:

- vertices: `3 * V` doubles (`[x0,y0,z0, x1,y1,z1, …]`)
- faces:    `3 * F` int32 vertex indices

This matches typical GPU layouts (Metal, MLX, nvdiffrast) — no transpose
or per-row allocation at the boundary.

### Typical pipeline

```swift
import SwiftIGL

// 1. Load a mesh
let mesh = try readTriangleMesh(at: url)

// 2. Build a voxel grid covering the mesh
let bb = try boundingBox(mesh, pad: 0.05)
let lo = bb[vertex: 0], hi = bb[vertex: 6]   // 2 opposite corners
let grid = try voxelGrid(
    bboxMin: lo, bboxMax: hi,
    largestSide: 128
)

// 3. Sample the SDF on the grid
let sdf = try signedDistance(
    of: grid.points, on: mesh, type: .fastWindingNumber
)

// 4. Re-extract as a clean mesh
let recon = try marchingCubes(scalars: sdf.distances, grid: grid)
```

## Topics

### Meshes and I/O

- ``TriangleMesh``
- ``readTriangleMesh(at:)``
- ``writeTriangleMesh(_:to:)``

### Geometric quantities

- ``perVertexNormals(_:)``
- ``perFaceNormals(_:)``

### Signed distance and winding numbers

- ``signedDistance(of:on:type:)``
- ``SignType``
- ``SignedDistanceResult``
- ``windingNumber(of:on:)``

### Voxelization and surface reconstruction

- ``VoxelGrid``
- ``voxelGrid(bboxMin:bboxMax:largestSide:pad:)``
- ``marchingCubes(scalars:grid:isovalue:)``

### Mesh processing

- ``boundingBox(_:pad:)``
- ``decimate(_:maxFaces:)``
- ``DecimationResult``
- ``removeDuplicateVertices(_:epsilon:)``
- ``DeduplicationResult``
- ``uniqueSimplices(_:)``

### Errors

- ``IGLError``

## Design notes

### No Eigen types in Swift

Eigen's expression templates and >3 generic parameters fall outside
Swift's C++ interop surface. The C++ bridge wraps caller buffers with
`Eigen::Map<>` internally and copies results back into flat
`std::vector`s on return. Swift never sees an Eigen type.

### No exceptions across the boundary

C++ exceptions hitting Swift terminate the program. Every bridge entry
point returns `bool`; on failure it writes a message to a trailing
`std::string&`. ``IGLError`` carries that message into Swift's
throwing-function machinery.

### Cxx interop is contagious

A Swift target that enables C++ interop forces all its downstream
dependents to also enable it. ``SwiftIGL`` is kept deliberately small to
minimise this propagation; consumer apps need only
`.interoperabilityMode(.Cxx)` if they touch the SwiftIGL API directly.

## Acknowledgements

Built on [libigl](https://libigl.github.io) (MPL2, © Alec Jacobson and
contributors) and [Eigen](https://eigen.tuxfamily.org) (MPL2).
