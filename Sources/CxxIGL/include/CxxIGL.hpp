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

}  // namespace swiftigl
