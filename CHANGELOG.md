# Changelog

All notable changes to SwiftIGL are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this
project aims to adhere to [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
once it reaches `1.0.0`.

## [Unreleased]

## [0.3.0]

### Added

- ``MeshAABB`` — persistent axis-aligned bounding-box hierarchy. Build
  once per mesh, run thousands of `O(log F)` queries against it. Wraps
  `igl::AABB<MatrixXd, 3>` via an opaque-handle pattern so the
  template-heavy state stays inside the C++ bridge.
  - ``MeshAABB/closestPoint(of:)`` — batch closest-point query.
  - ``MeshAABB/rayHits(origins:directions:)`` — batch ray/mesh
    intersection.
- One-shot ``rayMeshIntersect(origins:directions:on:)`` — simpler API
  for low-throughput cases (one-off pickings, etc.).
- ``RayHitsResult`` value type: `faceIds` (`-1` for miss), `ts`, and
  `barycentrics`.
- Topology queries:
  - ``edges(_:)`` — unique undirected edges.
  - ``triangleTriangleAdjacency(_:)`` — face-face adjacency
    (`-1` for boundary edges).
- 7 new tests across `MeshAABB`, `Topology`, and `Ray casting (one-shot)`
  suites (29 total).

## [0.2.0]

### Added (closest-point / barycentric / curvature — o-voxel-aligned)
- ``pointMeshSquaredDistance(of:on:)`` — squared distance + closest
  face id + closest point; cheaper than ``signedDistance`` (no sign
  computation).
- ``barycentricCoordinates(of:in:)`` — point-in-triangle barycentrics.
- ``closestPointBarycentrics(of:on:)`` — convenience combo matching
  o-voxel's `bvh.unsigned_distance(..., return_uvw=True)` output.
- ``faceBarycenters(_:)`` — per-face centroids.
- ``gaussianCurvature(_:)`` — per-vertex discrete angle deficit
  (verified on cube via Gauss-Bonnet sum = 4π).

### Added (tooling and infrastructure)
- `MeshToSDF` executable target — end-to-end demo of the
  mesh → SDF → marching-cubes pipeline (`swift run MeshToSDF in.obj
  out.obj [grid-side] [pad]`).
- Error-path tests covering invalid buffer shapes and missing files
  (5 new tests; 22 total).
- CHANGELOG.
- GitHub Actions CI (`swift build` + `swift test` on macOS 15 arm64).
- DocC GitHub Pages deployment workflow.
- README quick-start with badges and SPM install snippet.

## [0.1.0]

Initial release.

### Added
- Swift bindings for libigl 2.5.0 + Eigen 3.4.0 via a header-only
  xcframework (`igl.xcframework`).
- Sibling `libigl-xcframework-builder` repo producing the xcframework
  with one-command release via `make LIBIGL_VERSION=x.y.z release`.
- Initial Swift surface:
  - Mesh I/O: ``readTriangleMesh(at:)``, ``writeTriangleMesh(_:to:)``
    for `.obj`, `.off`, `.ply`, `.stl`, `.mesh`, `.wrl`.
  - Normals: ``perVertexNormals(_:)``, ``perFaceNormals(_:)``.
  - Signed distance: ``signedDistance(of:on:type:)`` with 5 sign-type
    modes (pseudonormal, winding number, default, unsigned, fast
    winding number).
  - Winding numbers: ``windingNumber(of:on:)``.
  - Voxelization: ``voxelGrid(bboxMin:bboxMax:largestSide:pad:)``,
    ``marchingCubes(scalars:grid:isovalue:)``.
  - Mesh processing: ``boundingBox(_:pad:)``, ``decimate(_:maxFaces:)``,
    ``removeDuplicateVertices(_:epsilon:)``, ``uniqueSimplices(_:)``.
- ``TriangleMesh`` value type with flat row-major storage.
- ``IGLError`` for failed libigl operations.
- DocC reference documentation with Topics groups.
- Tests covering mesh I/O round-trip, normals, signed distance,
  winding numbers, voxel grids, marching cubes on a sphere, bounding
  box, deduplication, and unique simplices.

[Unreleased]: https://github.com/mnmly/SwiftIGL/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/mnmly/SwiftIGL/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/mnmly/SwiftIGL/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/mnmly/SwiftIGL/releases/tag/v0.1.0
