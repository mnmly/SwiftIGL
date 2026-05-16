import CxxIGL
import CxxStdlib
import Foundation

/// Persistent axis-aligned bounding-box hierarchy over a triangle mesh.
///
/// Build one of these once per mesh, then run thousands of closest-point
/// or ray queries against it. Each query is `O(log F)` amortised, vs.
/// the `O(F)` work that ``signedDistance(of:on:type:)`` and
/// ``rayMeshIntersect(origins:directions:on:)`` do per call.
///
/// The cached `vertices` and `faces` are copied into the C++ side at
/// construction time, so mutating the original ``TriangleMesh`` afterwards
/// has no effect on the AABB. Build a new ``MeshAABB`` if the mesh changes.
///
/// ## Example
///
/// ```swift
/// let aabb = try MeshAABB(mesh)
/// let r = try aabb.closestPoint(of: queryPoints)   // many queries, one tree
/// ```
public final class MeshAABB: @unchecked Sendable {
    // C++ owns the AABB tree + cached V/F via a raw heap pointer.
    // We hold an UnsafeMutableRawPointer simply because Swift can't
    // express an `OpaquePointer to MeshAABBHandle` cleanly here.
    @usableFromInline internal var handle: OpaquePointer

    /// Build the tree over `mesh`. Throws ``IGLError`` if construction
    /// fails (e.g. malformed input).
    public init(_ mesh: TriangleMesh) throws {
        let v = makeDoubleVector(mesh.vertices)
        let f = makeInt32Vector(mesh.faces)
        var err = std.string()

        // Cxx interop imports `MeshAABBHandle*` as `OpaquePointer?`.
        // nullptr ⇒ nil ⇒ throw.
        guard let h = swiftigl.makeMeshAABB(v, f, &err) else {
            throw IGLError.operationFailed(String(err))
        }
        self.handle = h
    }

    deinit {
        swiftigl.destroyMeshAABB(handle)
    }

    /// Batch closest-point query. Returns the same shape as
    /// ``pointMeshSquaredDistance(of:on:)`` but reuses the cached tree.
    public func closestPoint(of queryPoints: [Double]) throws -> PointMeshDistanceResult {
        let p = makeDoubleVector(queryPoints)
        var sqrOut = swiftigl.DoubleVector()
        var idsOut = swiftigl.Int32Vector()
        var cpOut  = swiftigl.DoubleVector()
        var err    = std.string()
        let ok = swiftigl.meshAABBClosestPoint(
            handle, p, &sqrOut, &idsOut, &cpOut, &err
        )
        if !ok { throw IGLError.operationFailed(String(err)) }
        return PointMeshDistanceResult(
            sqrDistances:  Array(sqrOut),
            faceIds:       Array(idsOut),
            closestPoints: Array(cpOut)
        )
    }

    /// Batch ray/mesh intersection against the cached mesh.
    public func rayHits(
        origins: [Double],
        directions: [Double]
    ) throws -> RayHitsResult {
        let o = makeDoubleVector(origins)
        let d = makeDoubleVector(directions)
        var fidsOut = swiftigl.Int32Vector()
        var tsOut   = swiftigl.DoubleVector()
        var uvsOut  = swiftigl.DoubleVector()
        var err     = std.string()
        let ok = swiftigl.meshAABBRayHits(
            handle, o, d, &fidsOut, &tsOut, &uvsOut, &err
        )
        if !ok { throw IGLError.operationFailed(String(err)) }
        return RayHitsResult(
            faceIds:      Array(fidsOut),
            ts:           Array(tsOut),
            barycentrics: Array(uvsOut)
        )
    }
}
