import Foundation
import Testing
@testable import SwiftIGL

@Suite("Mesh I/O round-trip")
struct MeshIOTests {

    @Test("Read cube.obj returns 8 verts, 12 tris")
    func readCube() throws {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        let mesh = try readTriangleMesh(at: url)
        #expect(mesh.vertexCount == 8)
        #expect(mesh.faceCount == 12)
        // Centroid of the cube should be at origin.
        var cx = 0.0, cy = 0.0, cz = 0.0
        for i in 0..<mesh.vertexCount {
            let v = mesh[vertex: i]
            cx += v.x; cy += v.y; cz += v.z
        }
        #expect(Swift.abs(cx / Double(mesh.vertexCount)) < 1e-9)
        #expect(Swift.abs(cy / Double(mesh.vertexCount)) < 1e-9)
        #expect(Swift.abs(cz / Double(mesh.vertexCount)) < 1e-9)
    }

    @Test("Round-trip write→read preserves topology")
    func roundTrip() throws {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        let original = try readTriangleMesh(at: url)

        let tmp = URL(fileURLWithPath: NSTemporaryDirectory())
            .appendingPathComponent("swiftigl-roundtrip-\(UUID().uuidString).obj")
        defer { try? FileManager.default.removeItem(at: tmp) }
        try writeTriangleMesh(original, to: tmp)

        let reloaded = try readTriangleMesh(at: tmp)
        #expect(reloaded.vertexCount == original.vertexCount)
        #expect(reloaded.faceCount == original.faceCount)
    }
}

@Suite("Voxelization")
struct VoxelizationTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    @Test("voxelGrid side matches longest-axis request")
    func voxelGridShape() throws {
        let grid = try SwiftIGL.voxelGrid(
            bboxMin: SIMD3(-1, -1, -1),
            bboxMax: SIMD3(1, 1, 1),
            largestSide: 16
        )
        // For a cubic box, all three sides should equal `largestSide`.
        #expect(grid.side.x == 16 && grid.side.y == 16 && grid.side.z == 16)
        #expect(grid.points.count == grid.cellCount * 3)
    }

    @Test("signedDistance: cube interior negative, exterior positive")
    func signedDistanceSign() throws {
        let mesh = try loadCube()  // [-0.5, 0.5]^3
        let queries: [Double] = [
            0, 0, 0,     // interior
            1, 0, 0,     // exterior, +x
            0, 2, 0,     // exterior, +y
        ]
        let r = try SwiftIGL.signedDistance(of: queries, on: mesh, type: .fastWindingNumber)
        #expect(r.distances[0] < 0, "origin should be inside (got \(r.distances[0]))")
        #expect(r.distances[1] > 0)
        #expect(r.distances[2] > 0)
        #expect(r.closestPoints.count == queries.count)
        #expect(r.faceIds.count == queries.count / 3)
    }

    @Test("windingNumber: 1 inside, 0 outside")
    func windingNumberInsideOutside() throws {
        let mesh = try loadCube()
        let w = try SwiftIGL.windingNumber(
            of: [0, 0, 0,
                 5, 5, 5],
            on: mesh
        )
        #expect(Swift.abs(w[0] - 1.0) < 1e-3, "origin winding number should be 1, got \(w[0])")
        #expect(Swift.abs(w[1]) < 1e-3,       "far point winding number should be 0, got \(w[1])")
    }

    @Test("marchingCubes on sphere SDF recovers a closed surface")
    func marchingCubesSphere() throws {
        let grid = try SwiftIGL.voxelGrid(
            bboxMin: SIMD3(-1.5, -1.5, -1.5),
            bboxMax: SIMD3( 1.5,  1.5,  1.5),
            largestSide: 24
        )
        // Sample the unit-sphere SDF at every grid point.
        var scalars = [Double](repeating: 0, count: grid.cellCount)
        for i in 0..<grid.cellCount {
            let x = grid.points[3*i + 0]
            let y = grid.points[3*i + 1]
            let z = grid.points[3*i + 2]
            scalars[i] = (x*x + y*y + z*z).squareRoot() - 1.0
        }
        let sphere = try SwiftIGL.marchingCubes(scalars: scalars, grid: grid)
        #expect(sphere.vertexCount > 0)
        #expect(sphere.faceCount > 0)
        // Every extracted vertex should be on the unit sphere within a
        // voxel-sized tolerance.
        let voxelSize: Double = 3.0 / 24.0
        for i in 0..<sphere.vertexCount {
            let v = sphere[vertex: i]
            let r = (v.x*v.x + v.y*v.y + v.z*v.z).squareRoot()
            #expect(Swift.abs(r - 1.0) < voxelSize, "vertex \(i) off-surface: r=\(r)")
        }
    }
}

@Suite("Closest-point & barycentric")
struct ClosestPointTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    @Test("pointMeshSquaredDistance returns zero for on-surface points")
    func pmsdOnSurface() throws {
        let mesh = try loadCube()
        // Sample 3 vertices of the cube directly.
        let queries: [Double] = [
            mesh.vertices[0], mesh.vertices[1], mesh.vertices[2],
            mesh.vertices[3], mesh.vertices[4], mesh.vertices[5],
            mesh.vertices[6], mesh.vertices[7], mesh.vertices[8],
        ]
        let r = try SwiftIGL.pointMeshSquaredDistance(of: queries, on: mesh)
        #expect(r.sqrDistances.count == 3)
        for d in r.sqrDistances {
            #expect(d < 1e-12, "vertex-on-mesh sqrD should be 0, got \(d)")
        }
    }

    @Test("pointMeshSquaredDistance: exterior point projects to face")
    func pmsdProjection() throws {
        let mesh = try loadCube()
        // Point at (2, 0, 0): the cube spans [-0.5, 0.5]^3, so closest
        // point is on the +x face at (0.5, 0, 0), squared distance 2.25.
        let r = try SwiftIGL.pointMeshSquaredDistance(of: [2, 0, 0], on: mesh)
        #expect(Swift.abs(r.sqrDistances[0] - 2.25) < 1e-9)
        #expect(Swift.abs(r.closestPoints[0] - 0.5) < 1e-9)
        #expect(Swift.abs(r.closestPoints[1]) < 1e-9)
        #expect(Swift.abs(r.closestPoints[2]) < 1e-9)
    }

    @Test("barycentricCoordinates: centroid of canonical triangle is (1/3, 1/3, 1/3)")
    func barycentricCentroid() throws {
        let p: [Double] = [1.0/3.0, 1.0/3.0, 0]   // centroid of (0,0)-(1,0)-(0,1)
        let a: [Double] = [0, 0, 0]
        let b: [Double] = [1, 0, 0]
        let c: [Double] = [0, 1, 0]
        let bary = try SwiftIGL.barycentricCoordinates(of: p, in: (a: a, b: b, c: c))
        #expect(bary.count == 3)
        for v in bary {
            #expect(Swift.abs(v - 1.0/3.0) < 1e-9, "expected 1/3, got \(v)")
        }
    }

    @Test("closestPointBarycentrics: exterior point gets valid barycentrics")
    func combinedClosestBarycentric() throws {
        let mesh = try loadCube()
        let r = try SwiftIGL.closestPointBarycentrics(of: [2, 0.1, 0.1], on: mesh)
        // u + v + w should sum to 1 (within fp tolerance).
        let sum = r.barycentrics[0] + r.barycentrics[1] + r.barycentrics[2]
        #expect(Swift.abs(sum - 1.0) < 1e-9, "barycentric sum should be 1, got \(sum)")
        // All three should be non-negative for a point projecting strictly
        // *inside* the closest triangle.
        for w in r.barycentrics {
            #expect(w >= -1e-9, "barycentric component should be non-negative, got \(w)")
        }
    }
}

@Suite("Geometric analysis")
struct GeometricAnalysisTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    @Test("faceBarycenters: cube face centroids lie on ±0.5 planes")
    func centroidShape() throws {
        let mesh = try loadCube()
        let bc = try SwiftIGL.faceBarycenters(mesh)
        #expect(bc.count == 3 * mesh.faceCount)
        // Every face centroid of the unit cube has exactly one coordinate
        // at ±0.5 (the face it lives on) — i.e. max |coord| == 0.5.
        for i in 0..<mesh.faceCount {
            let mx = max(Swift.abs(bc[3*i]), Swift.abs(bc[3*i + 1]), Swift.abs(bc[3*i + 2]))
            #expect(Swift.abs(mx - 0.5) < 1e-9, "face \(i) centroid not on ±0.5 plane: \(mx)")
        }
    }

    @Test("gaussianCurvature: cube vertex angle deficit is π/2")
    func cubeGaussianCurvature() throws {
        let mesh = try loadCube()
        let K = try SwiftIGL.gaussianCurvature(mesh)
        #expect(K.count == mesh.vertexCount)
        // Each cube corner has three 90° angles meeting → angle deficit
        // is 2π − 3·(π/2) = π/2. Gauss-Bonnet check: sum = 8 · π/2 = 4π.
        for (i, k) in K.enumerated() {
            #expect(Swift.abs(k - .pi / 2) < 1e-9, "vertex \(i) angle deficit ≠ π/2: \(k)")
        }
        let total = K.reduce(0, +)
        #expect(Swift.abs(total - 4 * .pi) < 1e-9, "Gauss-Bonnet sum should be 4π, got \(total)")
    }
}

@Suite("Mesh processing")
struct MeshProcessingTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    @Test("boundingBox of cube returns 8 corners, 12 tris")
    func boundingBoxShape() throws {
        let mesh = try loadCube()
        let bb = try SwiftIGL.boundingBox(mesh)
        #expect(bb.vertexCount == 8)
        #expect(bb.faceCount == 12)
    }

    @Test("removeDuplicateVertices is a no-op on a clean cube")
    func removeDuplicatesIdempotent() throws {
        let mesh = try loadCube()
        let r = try SwiftIGL.removeDuplicateVertices(mesh, epsilon: 1e-12)
        #expect(r.mesh.vertexCount == mesh.vertexCount)
        #expect(r.mesh.faceCount == mesh.faceCount)
    }

    @Test("uniqueSimplices removes a duplicated face")
    func uniqueSimplicesDedup() throws {
        let faces: [Int32] = [
            0, 1, 2,
            0, 2, 3,
            2, 1, 0,   // same simplex as the first, reordered
        ]
        let unique = try SwiftIGL.uniqueSimplices(faces)
        #expect(unique.count == 6)  // 2 unique simplices × 3
    }
}

@Suite("Geometric quantities")
struct GeometryTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    @Test("perVertexNormals shape matches V")
    func vertexNormalsShape() throws {
        let mesh = try loadCube()
        let N = try perVertexNormals(mesh)
        #expect(N.count == 3 * mesh.vertexCount)
        // Each normal should be ~unit length for a cube.
        for i in 0..<mesh.vertexCount {
            let n = SIMD3(N[3*i], N[3*i + 1], N[3*i + 2])
            let len = (n.x*n.x + n.y*n.y + n.z*n.z).squareRoot()
            #expect(Swift.abs(len - 1.0) < 1e-6, "vertex normal \(i) not unit length: \(len)")
        }
    }

    @Test("perFaceNormals shape and unit length")
    func faceNormalsShape() throws {
        let mesh = try loadCube()
        let N = try perFaceNormals(mesh)
        #expect(N.count == 3 * mesh.faceCount)
        for i in 0..<mesh.faceCount {
            let n = SIMD3(N[3*i], N[3*i + 1], N[3*i + 2])
            let len = (n.x*n.x + n.y*n.y + n.z*n.z).squareRoot()
            #expect(Swift.abs(len - 1.0) < 1e-6, "face normal \(i) not unit length: \(len)")
        }
    }
}
