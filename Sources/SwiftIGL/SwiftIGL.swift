import CxxIGL
import CxxStdlib
import Foundation

/// Errors thrown by ``SwiftIGL`` APIs. The associated string carries the
/// C++-side message captured from libigl / Eigen.
public enum IGLError: Error, CustomStringConvertible {
    case operationFailed(String)

    public var description: String {
        switch self {
        case .operationFailed(let msg): return msg
        }
    }
}

/// An immutable triangle mesh with flat row-major storage.
///
/// `vertices` is a 3 * V buffer (`[x0,y0,z0, x1,y1,z1, …]`) and `faces` is
/// a 3 * F buffer of vertex indices. This layout matches the C++ bridge so
/// no transposition or per-row allocation is needed on either side.
public struct TriangleMesh: Sendable {
    /// Flat row-major `3 * V` buffer of vertex positions.
    public var vertices: [Double]
    /// Flat row-major `3 * F` buffer of vertex-index triples.
    public var faces: [Int32]

    /// Build a mesh from flat vertex and face buffers.
    ///
    /// - Parameters:
    ///   - vertices: a 3 * V buffer of positions.
    ///   - faces: a 3 * F buffer of vertex indices.
    public init(vertices: [Double], faces: [Int32]) {
        precondition(vertices.count % 3 == 0, "vertices count must be a multiple of 3")
        precondition(faces.count    % 3 == 0, "faces count must be a multiple of 3")
        self.vertices = vertices
        self.faces = faces
    }

    /// Number of vertices (`vertices.count / 3`).
    public var vertexCount: Int { vertices.count / 3 }
    /// Number of triangles (`faces.count / 3`).
    public var faceCount: Int { faces.count / 3 }

    /// The `i`-th vertex as an `SIMD3<Double>`.
    public subscript(vertex i: Int) -> SIMD3<Double> {
        SIMD3(vertices[3*i], vertices[3*i + 1], vertices[3*i + 2])
    }

    /// The `i`-th face as an `SIMD3<Int32>` of vertex indices.
    public subscript(face i: Int) -> SIMD3<Int32> {
        SIMD3(faces[3*i], faces[3*i + 1], faces[3*i + 2])
    }
}

// MARK: - Mesh I/O

/// Read a triangle mesh from disk. Format inferred by file extension
/// (.obj, .off, .ply, .stl, .mesh, .wrl).
public func readTriangleMesh(at url: URL) throws -> TriangleMesh {
    var verticesOut = swiftigl.DoubleVector()
    var facesOut    = swiftigl.Int32Vector()
    var errorOut    = std.string()

    let ok = swiftigl.readTriangleMesh(
        std.string(url.path),
        &verticesOut,
        &facesOut,
        &errorOut
    )
    if !ok {
        throw IGLError.operationFailed(String(errorOut))
    }
    return TriangleMesh(
        vertices: Array(verticesOut),
        faces:    Array(facesOut)
    )
}

/// Write a triangle mesh to disk. Format inferred by extension.
public func writeTriangleMesh(_ mesh: TriangleMesh, to url: URL) throws {
    let vertices = makeDoubleVector(mesh.vertices)
    let faces    = makeInt32Vector(mesh.faces)

    var errorOut = std.string()
    let ok = swiftigl.writeTriangleMesh(
        std.string(url.path),
        vertices,
        faces,
        &errorOut
    )
    if !ok {
        throw IGLError.operationFailed(String(errorOut))
    }
}

// MARK: - Geometric quantities

/// Per-vertex normals (area-weighted average of incident face normals).
/// Returns a flat 3 * V buffer.
public func perVertexNormals(_ mesh: TriangleMesh) throws -> [Double] {
    try compute(mesh: mesh, op: swiftigl.perVertexNormals)
}

/// Per-face unit normals. Returns a flat 3 * F buffer. Degenerate faces
/// receive (0, 0, 0) rather than NaN.
public func perFaceNormals(_ mesh: TriangleMesh) throws -> [Double] {
    try compute(mesh: mesh, op: swiftigl.perFaceNormals)
}

// MARK: - Internal

private func compute(
    mesh: TriangleMesh,
    op: (swiftigl.DoubleVector,
         swiftigl.Int32Vector,
         inout swiftigl.DoubleVector,
         inout std.string) -> Bool
) throws -> [Double] {
    let vertices = makeDoubleVector(mesh.vertices)
    let faces    = makeInt32Vector(mesh.faces)

    var out      = swiftigl.DoubleVector()
    var errorOut = std.string()
    let ok = op(vertices, faces, &out, &errorOut)
    if !ok {
        throw IGLError.operationFailed(String(errorOut))
    }
    return Array(out)
}

internal func makeDoubleVector(_ source: [Double]) -> swiftigl.DoubleVector {
    var v = swiftigl.DoubleVector()
    v.reserve(source.count)
    for x in source { v.push_back(x) }
    return v
}

internal func makeInt32Vector(_ source: [Int32]) -> swiftigl.Int32Vector {
    var v = swiftigl.Int32Vector()
    v.reserve(source.count)
    for x in source { v.push_back(x) }
    return v
}
