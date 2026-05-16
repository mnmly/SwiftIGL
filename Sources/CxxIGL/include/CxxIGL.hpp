// CxxIGL.hpp — Swift-visible C++ surface for libigl.
//
// Design rules (set by Swift Cxx interop constraints):
//   * No Eigen types in any function signature — Eigen is template-heavy
//     and not safe to expose across the Cxx interop boundary. Internally
//     we wrap the caller's flat vectors with Eigen::Map<...> before
//     handing them to libigl, and copy results back into flat vectors on
//     return.
//   * No exceptions across the boundary. C++ exceptions hitting Swift
//     terminate the program. Every entry point returns bool; failure is
//     signalled via a trailing std::string* errorOut.
//   * Only std::string + std::vector<double|int32_t> are exchanged at the
//     boundary. Those have first-class CxxStdlib support in Swift.
//   * Meshes are flattened row-major:
//       vertices: 3 * V doubles  ([x0,y0,z0, x1,y1,z1, ...])
//       faces:    3 * F int32_t  ([i0,j0,k0, i1,j1,k1, ...])
//     Swift wraps these in idiomatic types (see SwiftIGL.swift).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace swiftigl {

// Concrete vector specializations exposed to Swift. Swift's C++ interop
// requires fully-instantiated template types; bare `std::vector<T>` is
// rejected because it leaves the Allocator parameter unspecialised. These
// aliases materialise the specializations so Swift sees plain named types.
using DoubleVector = std::vector<double>;
using Int32Vector  = std::vector<int32_t>;

// ---------------------------------------------------------------------------
// Mesh I/O
// ---------------------------------------------------------------------------

/// Read a triangle mesh from `path` (.obj, .off, .ply, .stl, .mesh, .wrl).
/// On success returns true and fills `verticesOut` / `facesOut` (cleared
/// first). On failure returns false and writes a message to `errorOut`.
bool readTriangleMesh(const std::string& path,
                      DoubleVector& verticesOut,
                      Int32Vector& facesOut,
                      std::string& errorOut);

/// Write a triangle mesh to `path`. Format chosen by extension.
/// `vertices.size()` must be a multiple of 3; same for `faces`.
bool writeTriangleMesh(const std::string& path,
                       const DoubleVector& vertices,
                       const Int32Vector& faces,
                       std::string& errorOut);

// ---------------------------------------------------------------------------
// Geometric quantities
// ---------------------------------------------------------------------------

/// Per-vertex normals via libigl's area-weighted averaging (default
/// weighting type). Returns a flat 3 * V buffer.
///
/// Validates input shapes; on shape mismatch sets errorOut and returns false.
bool perVertexNormals(const DoubleVector& vertices,
                      const Int32Vector& faces,
                      DoubleVector& normalsOut,
                      std::string& errorOut);

/// Per-face normals (unit length). Returns a flat 3 * F buffer.
bool perFaceNormals(const DoubleVector& vertices,
                    const Int32Vector& faces,
                    DoubleVector& normalsOut,
                    std::string& errorOut);

// ---------------------------------------------------------------------------
// Voxelization-aligned operations
// ---------------------------------------------------------------------------

/// Sign convention for ``signedDistance``. Mirrors `igl::SignedDistanceType`.
enum SignType : int32_t {
    kPseudonormal        = 0,
    kWindingNumber       = 1,
    kSignedDistanceDefault = 2,
    kUnsigned            = 3,
    kFastWindingNumber   = 4,
};

/// Signed distance from each query point to the mesh (V, F).
///
/// Outputs three flat buffers, all of size #P (or #P * 3 for closest points):
///   - `distancesOut`        — signed distance (or unsigned, see signType)
///   - `faceIdsOut`          — index of the closest face
///   - `closestPointsOut`    — closest point on the mesh (3 * #P)
bool signedDistance(const DoubleVector& queryPoints,
                    const DoubleVector& vertices,
                    const Int32Vector&  faces,
                    int32_t signType,
                    DoubleVector& distancesOut,
                    Int32Vector&  faceIdsOut,
                    DoubleVector& closestPointsOut,
                    std::string&  errorOut);

/// Generalised winding number at each query point.
/// Robust inside/outside test for non-watertight meshes
/// (|w| > 0.5 ⇒ inside, for sane orientation).
bool windingNumber(const DoubleVector& vertices,
                   const Int32Vector&  faces,
                   const DoubleVector& queryPoints,
                   DoubleVector& windingNumbersOut,
                   std::string&  errorOut);

/// Construct a regular voxel grid covering `[bboxMin, bboxMax]`.
///
/// `largestSideCount` is the number of cell centers along the longest axis
/// (the other two are derived to keep cells cubic). `padCount` extends the
/// box by N cells in every direction.
///
/// Outputs:
///   - `gridPointsOut` — (nx * ny * nz) * 3 cell-center positions
///   - `sideOut`       — 3-element {nx, ny, nz}
bool voxelGrid(const DoubleVector& bboxMin,
               const DoubleVector& bboxMax,
               int32_t largestSideCount,
               int32_t padCount,
               DoubleVector& gridPointsOut,
               Int32Vector&  sideOut,
               std::string&  errorOut);

/// Marching cubes on a dense scalar field defined at every grid corner.
///
/// `scalars` is `nx*ny*nz` long, indexed as `S(x + y*nx + z*nx*ny)`.
/// `gridPoints` is the matching (nx*ny*nz)*3 buffer of corner positions
/// (typically the output of ``voxelGrid``).
bool marchingCubes(const DoubleVector& scalars,
                   const DoubleVector& gridPoints,
                   int32_t nx, int32_t ny, int32_t nz,
                   double  isovalue,
                   DoubleVector& verticesOut,
                   Int32Vector&  facesOut,
                   std::string&  errorOut);

/// Bounding-box mesh (8 corners + 12 triangle faces) of `vertices`.
/// `pad` inflates the AABB on every side.
bool boundingBox(const DoubleVector& vertices,
                 double pad,
                 DoubleVector& cornersOut,
                 Int32Vector&  facesOut,
                 std::string&  errorOut);

/// Greedy edge-collapse decimation to a target face count.
///
/// Outputs:
///   - `verticesOut`, `facesOut` — decimated mesh
///   - `birthFacesOut`           — for each output face, index of the
///                                  original face it descends from (#FF)
bool decimate(const DoubleVector& vertices,
              const Int32Vector&  faces,
              int32_t maxFaces,
              DoubleVector& verticesOut,
              Int32Vector&  facesOut,
              Int32Vector&  birthFacesOut,
              std::string&  errorOut);

/// Merge vertices within `epsilon` of each other and remap face indices.
///
/// Outputs:
///   - `verticesOut`, `facesOut` — deduplicated mesh
///   - `uniqueIndicesOut`        — #SV indices so SV = V(uniqueIndices,:)
///   - `inverseIndicesOut`       — #V indices so V = SV(inverseIndices,:)
bool removeDuplicateVertices(const DoubleVector& vertices,
                             const Int32Vector&  faces,
                             double epsilon,
                             DoubleVector& verticesOut,
                             Int32Vector&  facesOut,
                             Int32Vector&  uniqueIndicesOut,
                             Int32Vector&  inverseIndicesOut,
                             std::string&  errorOut);

/// Combinatorially-unique simplices in `faces` (order-independent).
/// Useful to dedupe faces after a marching-cubes extraction.
bool uniqueSimplices(const Int32Vector& faces,
                     Int32Vector& facesOut,
                     std::string& errorOut);

// ---------------------------------------------------------------------------
// Closest-point / barycentric / curvature queries (o-voxel-aligned)
// ---------------------------------------------------------------------------

/// Squared distance from each query point to the closest face of (V, F).
///
/// Cheaper than ``signedDistance`` (no sign computation). Outputs:
///   - `sqrDistancesOut`  — `#P` squared distances
///   - `faceIdsOut`       — `#P` closest face indices
///   - `closestPointsOut` — `3 * #P` closest-point coordinates
bool pointMeshSquaredDistance(const DoubleVector& queryPoints,
                              const DoubleVector& vertices,
                              const Int32Vector&  faces,
                              DoubleVector& sqrDistancesOut,
                              Int32Vector&  faceIdsOut,
                              DoubleVector& closestPointsOut,
                              std::string&  errorOut);

/// Barycentric coordinates of `#P` query points, each w.r.t. a triangle
/// `(A_i, B_i, C_i)`. Inputs are flat 3*P buffers, output is flat 3*P
/// `(u, v, w)` per query.
bool barycentricCoordinates(const DoubleVector& queryPoints,
                            const DoubleVector& triangleA,
                            const DoubleVector& triangleB,
                            const DoubleVector& triangleC,
                            DoubleVector& barycentricsOut,
                            std::string&  errorOut);

/// Per-face barycenters (centroids). Returns a flat 3 * F buffer.
bool faceBarycenters(const DoubleVector& vertices,
                     const Int32Vector&  faces,
                     DoubleVector& barycentersOut,
                     std::string&  errorOut);

/// Per-vertex discrete Gaussian curvature (2π minus sum of interior
/// angles at each vertex). Returns a `#V` buffer.
///
/// Note: meaningful for interior vertices of a manifold mesh; boundary
/// vertices receive a related but less-physical value.
bool gaussianCurvature(const DoubleVector& vertices,
                       const Int32Vector&  faces,
                       DoubleVector& curvatureOut,
                       std::string&  errorOut);

// ---------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------

/// Unique undirected edges of `(V, F)`. Returns a flat `2 * E` buffer of
/// vertex-index pairs `(v0, v1)`.
bool edges(const Int32Vector& faces,
           Int32Vector& edgesOut,
           std::string& errorOut);

/// Triangle-triangle adjacency: for each face `i` and edge `j ∈ {0,1,2}`,
/// returns the index of the adjacent face, or `-1` for a boundary edge.
///
/// Output is a flat `3 * F` buffer. Edge ordering: edge 0 is `(v0, v1)`,
/// edge 1 is `(v1, v2)`, edge 2 is `(v2, v0)`.
bool triangleTriangleAdjacency(const Int32Vector& faces,
                               Int32Vector& adjacencyOut,
                               std::string& errorOut);

// ---------------------------------------------------------------------------
// Ray casting (single-shot — for many rays, prefer MeshAABB)
// ---------------------------------------------------------------------------

/// One-shot ray/mesh intersection. For repeated queries against the
/// same mesh, build a ``MeshAABBHandle`` once and use ``meshAABBRayHits``.
///
/// `origins` and `directions` are both flat `3 * R` buffers. Outputs:
///   - `faceIdsOut`     — `R` indices of the first-hit face (`-1` if miss)
///   - `tsOut`          — `R` parametric distances along each ray
///   - `barycentricsOut`— `2 * R` `(u, v)` coords on the hit face
bool rayMeshIntersect(const DoubleVector& origins,
                      const DoubleVector& directions,
                      const DoubleVector& vertices,
                      const Int32Vector&  faces,
                      Int32Vector& faceIdsOut,
                      DoubleVector& tsOut,
                      DoubleVector& barycentricsOut,
                      std::string&  errorOut);

// ---------------------------------------------------------------------------
// MeshAABB — persistent acceleration structure
// ---------------------------------------------------------------------------
//
// The `igl::AABB<MatrixXd, 3>` class plus its referenced V and F are
// kept inside the bridge as an opaque handle. Swift wraps it via a
// final class with deinit, never seeing the stateful template types.

struct MeshAABBHandle;

/// Build an AABB tree for `(vertices, faces)`. Returns nullptr on
/// failure (message in `errorOut`). Caller owns the returned pointer
/// and must release it via ``destroyMeshAABB``.
MeshAABBHandle* makeMeshAABB(const DoubleVector& vertices,
                             const Int32Vector&  faces,
                             std::string& errorOut);

/// Release a tree created by ``makeMeshAABB``. Safe to call on nullptr.
void destroyMeshAABB(MeshAABBHandle* handle);

/// Batch closest-point query against the cached mesh. Same outputs as
/// ``pointMeshSquaredDistance`` (squared distance + face id + closest
/// point), but ~100× faster on large meshes when amortised across
/// many query points because the AABB is reused.
bool meshAABBClosestPoint(const MeshAABBHandle* handle,
                          const DoubleVector& queryPoints,
                          DoubleVector& sqrDistancesOut,
                          Int32Vector&  faceIdsOut,
                          DoubleVector& closestPointsOut,
                          std::string&  errorOut);

/// Batch ray/mesh intersection against the cached mesh. Inputs and
/// outputs match ``rayMeshIntersect``.
bool meshAABBRayHits(const MeshAABBHandle* handle,
                     const DoubleVector& origins,
                     const DoubleVector& directions,
                     Int32Vector&  faceIdsOut,
                     DoubleVector& tsOut,
                     DoubleVector& barycentricsOut,
                     std::string&  errorOut);

}  // namespace swiftigl
