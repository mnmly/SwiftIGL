import CxxIGL
import CxxStdlib
import Foundation

// MARK: - SDF / inside-outside

/// Sign convention for ``signedDistance(of:on:type:)``.
public enum SignType: Int32, Sendable {
    /// Fast pseudo-normal test [Bærentzen & Aanæs 2005]. Requires watertight meshes.
    case pseudonormal = 0
    /// Generalised winding number [Jacobson et al. 2013]. Robust to non-watertight meshes.
    case windingNumber = 1
    /// libigl's default (currently equivalent to pseudonormal).
    case `default` = 2
    /// Unsigned distance only.
    case unsigned = 3
    /// Fast winding number [Barill et al. 2018]. Recommended for large query sets.
    case fastWindingNumber = 4
}

/// Result of a signed-distance query.
///
/// All buffers are aligned with the input query points:
///   - `distances[i]` is the (signed) distance to the mesh
///   - `faceIds[i]` is the index of the closest face
///   - `closestPoints[3*i..<3*i+3]` is the closest surface point
public struct SignedDistanceResult: Sendable {
    /// `#P` signed (or unsigned, depending on ``SignType``) distances.
    public var distances: [Double]
    /// `#P` indices of the closest face for each query point.
    public var faceIds: [Int32]
    /// `3 * #P` closest-point coordinates on the mesh.
    public var closestPoints: [Double]
}

/// Signed distance from each query point to `mesh`.
///
/// `queryPoints` is a flat 3 * P buffer matching ``TriangleMesh/vertices``
/// layout. Pass ``SignType/fastWindingNumber`` for large point sets and
/// ``SignType/pseudonormal`` for small watertight meshes.
public func signedDistance(
    of queryPoints: [Double],
    on mesh: TriangleMesh,
    type: SignType = .default
) throws -> SignedDistanceResult {
    let points   = makeDoubleVector(queryPoints)
    let vertices = makeDoubleVector(mesh.vertices)
    let faces    = makeInt32Vector(mesh.faces)
    var distancesOut     = swiftigl.DoubleVector()
    var faceIdsOut       = swiftigl.Int32Vector()
    var closestPointsOut = swiftigl.DoubleVector()
    var errorOut         = std.string()

    let ok = swiftigl.signedDistance(
        points, vertices, faces, type.rawValue,
        &distancesOut, &faceIdsOut, &closestPointsOut, &errorOut
    )
    if !ok { throw IGLError.operationFailed(String(errorOut)) }
    return SignedDistanceResult(
        distances:     Array(distancesOut),
        faceIds:       Array(faceIdsOut),
        closestPoints: Array(closestPointsOut)
    )
}

/// Generalised winding number of `queryPoints` w.r.t. `mesh`.
///
/// Use as a robust inside/outside test: `|w| > 0.5` ⇒ inside for a
/// consistently oriented mesh. Slower than ``signedDistance(of:on:type:)``
/// for large point sets — prefer the latter with
/// ``SignType/fastWindingNumber``.
public func windingNumber(
    of queryPoints: [Double],
    on mesh: TriangleMesh
) throws -> [Double] {
    let vertices = makeDoubleVector(mesh.vertices)
    let faces    = makeInt32Vector(mesh.faces)
    let points   = makeDoubleVector(queryPoints)
    var out      = swiftigl.DoubleVector()
    var errorOut = std.string()

    let ok = swiftigl.windingNumber(vertices, faces, points, &out, &errorOut)
    if !ok { throw IGLError.operationFailed(String(errorOut)) }
    return Array(out)
}

// MARK: - Voxel grid & marching cubes

/// A regular lattice of cell-center points covering an AABB.
public struct VoxelGrid: Sendable {
    /// Flat (nx * ny * nz) * 3 buffer of cell-center positions.
    public var points: [Double]
    /// `(nx, ny, nz)` — number of cells along each axis.
    public var side: SIMD3<Int32>

    public var cellCount: Int { Int(side.x) * Int(side.y) * Int(side.z) }
}

/// Construct a voxel grid covering `[bboxMin, bboxMax]`.
///
/// - Parameters:
///   - bboxMin: minimum AABB corner.
///   - bboxMax: maximum AABB corner.
///   - largestSide: number of cells along the longest axis (others
///     derived to keep cells cubic).
///   - pad: extend the AABB by `pad` cells in every direction (default 0).
public func voxelGrid(
    bboxMin: SIMD3<Double>,
    bboxMax: SIMD3<Double>,
    largestSide: Int32,
    pad: Int32 = 0
) throws -> VoxelGrid {
    let lo = makeDoubleVector([bboxMin.x, bboxMin.y, bboxMin.z])
    let hi = makeDoubleVector([bboxMax.x, bboxMax.y, bboxMax.z])
    var pointsOut = swiftigl.DoubleVector()
    var sideOut   = swiftigl.Int32Vector()
    var errorOut  = std.string()

    let ok = swiftigl.voxelGrid(lo, hi, largestSide, pad, &pointsOut, &sideOut, &errorOut)
    if !ok { throw IGLError.operationFailed(String(errorOut)) }
    let sideArr = Array(sideOut)
    return VoxelGrid(
        points: Array(pointsOut),
        side:   SIMD3(sideArr[0], sideArr[1], sideArr[2])
    )
}

/// Marching-cubes mesh from a dense scalar field on a regular grid.
///
/// `scalars` is laid out as `S(x + y*nx + z*nx*ny)` (matching libigl's
/// convention and the output of ``voxelGrid(bboxMin:bboxMax:largestSide:pad:)``).
public func marchingCubes(
    scalars: [Double],
    grid: VoxelGrid,
    isovalue: Double = 0.0
) throws -> TriangleMesh {
    let s    = makeDoubleVector(scalars)
    let gv   = makeDoubleVector(grid.points)
    var vOut = swiftigl.DoubleVector()
    var fOut = swiftigl.Int32Vector()
    var err  = std.string()

    let ok = swiftigl.marchingCubes(
        s, gv, grid.side.x, grid.side.y, grid.side.z,
        isovalue, &vOut, &fOut, &err
    )
    if !ok { throw IGLError.operationFailed(String(err)) }
    return TriangleMesh(vertices: Array(vOut), faces: Array(fOut))
}

// MARK: - Bounding box

/// AABB triangle mesh of `mesh` (8 corners, 12 triangles).
///
/// `pad` inflates the box on every side.
public func boundingBox(_ mesh: TriangleMesh, pad: Double = 0.0) throws -> TriangleMesh {
    let v = makeDoubleVector(mesh.vertices)
    var cornersOut = swiftigl.DoubleVector()
    var facesOut   = swiftigl.Int32Vector()
    var err        = std.string()
    let ok = swiftigl.boundingBox(v, pad, &cornersOut, &facesOut, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return TriangleMesh(vertices: Array(cornersOut), faces: Array(facesOut))
}

// MARK: - Decimation & cleanup

/// Result of a ``decimate(_:maxFaces:)`` call.
public struct DecimationResult: Sendable {
    public var mesh: TriangleMesh
    /// For each output face, the index of the original face it descends
    /// from. Aligned with `mesh.faces` (length = `mesh.faceCount`).
    public var birthFaces: [Int32]
}

/// Greedy edge-collapse decimation to a target face count.
///
/// Note: libigl's `decimate` assumes a **closed manifold mesh** and may
/// return `false` (thrown here) on non-manifold input.
public func decimate(_ mesh: TriangleMesh, maxFaces: Int32) throws -> DecimationResult {
    let v = makeDoubleVector(mesh.vertices)
    let f = makeInt32Vector(mesh.faces)
    var vOut = swiftigl.DoubleVector()
    var fOut = swiftigl.Int32Vector()
    var jOut = swiftigl.Int32Vector()
    var err  = std.string()
    let ok = swiftigl.decimate(v, f, maxFaces, &vOut, &fOut, &jOut, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return DecimationResult(
        mesh: TriangleMesh(vertices: Array(vOut), faces: Array(fOut)),
        birthFaces: Array(jOut)
    )
}

/// Result of a ``removeDuplicateVertices(_:epsilon:)`` call.
public struct DeduplicationResult: Sendable {
    public var mesh: TriangleMesh
    /// `mesh.vertices[uniqueIndices[i]]` is the `i`-th retained vertex
    /// (in terms of the *original* indexing).
    public var uniqueIndices: [Int32]
    /// `inverseIndices[i]` is the new index of original vertex `i`.
    public var inverseIndices: [Int32]
}

/// Merge vertices within `epsilon` of each other and remap face indices.
///
/// `epsilon = 0` means exact match; `1e-7` is a reasonable floating-point
/// tolerance for cleaning marching-cubes output.
public func removeDuplicateVertices(
    _ mesh: TriangleMesh,
    epsilon: Double
) throws -> DeduplicationResult {
    let v = makeDoubleVector(mesh.vertices)
    let f = makeInt32Vector(mesh.faces)
    var vOut = swiftigl.DoubleVector()
    var fOut = swiftigl.Int32Vector()
    var uniqueOut  = swiftigl.Int32Vector()
    var inverseOut = swiftigl.Int32Vector()
    var err = std.string()
    let ok = swiftigl.removeDuplicateVertices(
        v, f, epsilon, &vOut, &fOut, &uniqueOut, &inverseOut, &err
    )
    if !ok { throw IGLError.operationFailed(String(err)) }
    return DeduplicationResult(
        mesh: TriangleMesh(vertices: Array(vOut), faces: Array(fOut)),
        uniqueIndices: Array(uniqueOut),
        inverseIndices: Array(inverseOut)
    )
}

// MARK: - Closest-point / barycentric / curvature

/// Result of a ``pointMeshSquaredDistance(of:on:)`` query.
public struct PointMeshDistanceResult: Sendable {
    /// `#P` squared distances to the closest face.
    public var sqrDistances: [Double]
    /// `#P` indices of the closest face for each query point.
    public var faceIds: [Int32]
    /// `3 * #P` closest-point coordinates on the mesh.
    public var closestPoints: [Double]
}

/// Squared distance from each query point to the closest face of `mesh`.
///
/// Cheaper than ``signedDistance(of:on:type:)`` — no sign computation.
/// Use this when you only need raw proximity (e.g. nearest-face lookup
/// for texture transfer); use ``signedDistance(of:on:type:)`` when you
/// need inside/outside information.
public func pointMeshSquaredDistance(
    of queryPoints: [Double],
    on mesh: TriangleMesh
) throws -> PointMeshDistanceResult {
    let p = makeDoubleVector(queryPoints)
    let v = makeDoubleVector(mesh.vertices)
    let f = makeInt32Vector(mesh.faces)
    var sqrOut = swiftigl.DoubleVector()
    var idsOut = swiftigl.Int32Vector()
    var cpOut  = swiftigl.DoubleVector()
    var err    = std.string()
    let ok = swiftigl.pointMeshSquaredDistance(p, v, f, &sqrOut, &idsOut, &cpOut, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return PointMeshDistanceResult(
        sqrDistances:  Array(sqrOut),
        faceIds:       Array(idsOut),
        closestPoints: Array(cpOut)
    )
}

/// Barycentric coordinates of each query point w.r.t. the matching
/// triangle `(A_i, B_i, C_i)`.
///
/// All four inputs must be flat 3 * P buffers. Returns a flat 3 * P
/// `(u, v, w)` buffer where `u + v + w = 1` (within fp tolerance).
public func barycentricCoordinates(
    of queryPoints: [Double],
    in triangles: (a: [Double], b: [Double], c: [Double])
) throws -> [Double] {
    let p = makeDoubleVector(queryPoints)
    let a = makeDoubleVector(triangles.a)
    let b = makeDoubleVector(triangles.b)
    let c = makeDoubleVector(triangles.c)
    var out = swiftigl.DoubleVector()
    var err = std.string()
    let ok = swiftigl.barycentricCoordinates(p, a, b, c, &out, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return Array(out)
}

/// Closest-point query result enriched with barycentric coordinates on
/// the closest face — the natural output of an o-voxel-style BVH probe
/// (`bvh.unsigned_distance(..., return_uvw=True)`).
public struct ClosestPointBarycentricsResult: Sendable {
    /// `#P` squared distances.
    public var sqrDistances: [Double]
    /// `#P` closest face indices.
    public var faceIds: [Int32]
    /// `3 * #P` closest-point coordinates.
    public var closestPoints: [Double]
    /// `3 * #P` barycentric coordinates `(u, v, w)` on the closest face.
    public var barycentrics: [Double]
}

/// One-shot equivalent of `bvh.unsigned_distance(..., return_uvw=True)`
/// in o-voxel: returns the closest face per query, plus the closest
/// point and its barycentric coordinates on that face.
///
/// Useful for texture transfer, attribute baking, and any "find the
/// closest spot on the surface and read something there" workflow.
public func closestPointBarycentrics(
    of queryPoints: [Double],
    on mesh: TriangleMesh
) throws -> ClosestPointBarycentricsResult {
    let pmd = try pointMeshSquaredDistance(of: queryPoints, on: mesh)

    // Build per-query triangle corner buffers from the closest face ids.
    let pCount = pmd.faceIds.count
    var aBuf = [Double](repeating: 0, count: pCount * 3)
    var bBuf = [Double](repeating: 0, count: pCount * 3)
    var cBuf = [Double](repeating: 0, count: pCount * 3)
    for i in 0..<pCount {
        let fid = Int(pmd.faceIds[i])
        let face = mesh[face: fid]
        for k in 0..<3 {
            aBuf[3*i + k] = mesh.vertices[3 * Int(face.x) + k]
            bBuf[3*i + k] = mesh.vertices[3 * Int(face.y) + k]
            cBuf[3*i + k] = mesh.vertices[3 * Int(face.z) + k]
        }
    }
    let bary = try barycentricCoordinates(
        of: pmd.closestPoints,
        in: (a: aBuf, b: bBuf, c: cBuf)
    )
    return ClosestPointBarycentricsResult(
        sqrDistances:  pmd.sqrDistances,
        faceIds:       pmd.faceIds,
        closestPoints: pmd.closestPoints,
        barycentrics:  bary
    )
}

/// Per-face barycenters (centroids). Returns a flat 3 * F buffer.
public func faceBarycenters(_ mesh: TriangleMesh) throws -> [Double] {
    let v = makeDoubleVector(mesh.vertices)
    let f = makeInt32Vector(mesh.faces)
    var out = swiftigl.DoubleVector()
    var err = std.string()
    let ok = swiftigl.faceBarycenters(v, f, &out, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return Array(out)
}

/// Per-vertex discrete Gaussian curvature (2π minus sum of interior
/// angles). Returns a `#V` buffer.
///
/// Sum should equal `2π · χ(M)` for a closed manifold by Gauss-Bonnet —
/// e.g. `4π` for a topological sphere.
public func gaussianCurvature(_ mesh: TriangleMesh) throws -> [Double] {
    let v = makeDoubleVector(mesh.vertices)
    let f = makeInt32Vector(mesh.faces)
    var out = swiftigl.DoubleVector()
    var err = std.string()
    let ok = swiftigl.gaussianCurvature(v, f, &out, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return Array(out)
}

// MARK: - Topology

/// Unique undirected edges of `mesh`. Returns a flat `2 * E` buffer of
/// `(v0, v1)` index pairs.
public func edges(_ mesh: TriangleMesh) throws -> [Int32] {
    let f = makeInt32Vector(mesh.faces)
    var out = swiftigl.Int32Vector()
    var err = std.string()
    let ok = swiftigl.edges(f, &out, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return Array(out)
}

/// Triangle-triangle adjacency: for each face `i` and edge `j ∈ {0,1,2}`,
/// the index of the adjacent face, or `-1` for boundary edges.
///
/// Returned as a flat `3 * F` buffer. Edge convention: edge 0 is
/// `(v0, v1)`, edge 1 is `(v1, v2)`, edge 2 is `(v2, v0)`.
public func triangleTriangleAdjacency(_ mesh: TriangleMesh) throws -> [Int32] {
    let f = makeInt32Vector(mesh.faces)
    var out = swiftigl.Int32Vector()
    var err = std.string()
    let ok = swiftigl.triangleTriangleAdjacency(f, &out, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return Array(out)
}

// MARK: - Ray casting

/// Result of a batch ray/mesh intersection.
///
/// All buffers have one entry per ray (or two for `barycentrics`).
/// Misses are encoded as `faceIds[i] == -1` and `NaN` in `ts` and
/// `barycentrics`.
public struct RayHitsResult: Sendable {
    /// `#R` indices of the first-hit face (`-1` if the ray missed).
    public var faceIds: [Int32]
    /// `#R` parametric distances along each ray (`NaN` on miss).
    public var ts: [Double]
    /// `2 * #R` `(u, v)` barycentric coords on the hit face; the third
    /// coordinate is `1 - u - v`. `NaN` on miss.
    public var barycentrics: [Double]
}

/// One-shot ray/mesh intersection.
///
/// For repeated queries against the same mesh, prefer
/// ``MeshAABB/rayHits(origins:directions:)`` — building the AABB once
/// and querying it many times is asymptotically faster.
///
/// - Parameters:
///   - origins: flat `3 * R` buffer of ray origins.
///   - directions: flat `3 * R` buffer of ray directions (need not be
///     unit-length; `t` is reported in these units).
///   - mesh: the triangle mesh to intersect.
public func rayMeshIntersect(
    origins: [Double],
    directions: [Double],
    on mesh: TriangleMesh
) throws -> RayHitsResult {
    let o = makeDoubleVector(origins)
    let d = makeDoubleVector(directions)
    let v = makeDoubleVector(mesh.vertices)
    let f = makeInt32Vector(mesh.faces)
    var fidsOut = swiftigl.Int32Vector()
    var tsOut   = swiftigl.DoubleVector()
    var uvsOut  = swiftigl.DoubleVector()
    var err     = std.string()
    let ok = swiftigl.rayMeshIntersect(o, d, v, f, &fidsOut, &tsOut, &uvsOut, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return RayHitsResult(
        faceIds:      Array(fidsOut),
        ts:           Array(tsOut),
        barycentrics: Array(uvsOut)
    )
}

/// Combinatorially-unique faces (order-independent).
///
/// Useful to remove degenerate-duplicate faces from a marching-cubes
/// extraction. Returns only the unique faces; for the index-remap pair,
/// use libigl directly via the C++ surface.
public func uniqueSimplices(_ faces: [Int32]) throws -> [Int32] {
    let f = makeInt32Vector(faces)
    var fOut = swiftigl.Int32Vector()
    var err  = std.string()
    let ok = swiftigl.uniqueSimplices(f, &fOut, &err)
    if !ok { throw IGLError.operationFailed(String(err)) }
    return Array(fOut)
}
