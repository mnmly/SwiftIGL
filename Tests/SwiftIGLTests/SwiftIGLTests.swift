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
