import Foundation
import Testing
@testable import SwiftIGL

@Suite("MeshAABB")
struct MeshAABBTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    @Test("closestPoint via AABB matches the one-shot result")
    func aabbMatchesOneShot() throws {
        let mesh = try loadCube()
        let queries: [Double] = [
            0, 0, 0,
            2, 0, 0,
            0, 2, 0,
            -1, -1, 1,
        ]
        let oneShot = try pointMeshSquaredDistance(of: queries, on: mesh)
        let aabb = try MeshAABB(mesh)
        let cached = try aabb.closestPoint(of: queries)

        #expect(cached.sqrDistances.count == oneShot.sqrDistances.count)
        for i in 0..<cached.sqrDistances.count {
            #expect(
                Swift.abs(cached.sqrDistances[i] - oneShot.sqrDistances[i]) < 1e-12,
                "sqrD differs at \(i): aabb=\(cached.sqrDistances[i]) oneShot=\(oneShot.sqrDistances[i])"
            )
            #expect(cached.faceIds[i] == oneShot.faceIds[i])
        }
    }

    @Test("AABB is reusable across many calls")
    func reuseAcrossCalls() throws {
        let mesh = try loadCube()
        let aabb = try MeshAABB(mesh)
        // 10 separate queries should all succeed without rebuilding.
        for i in 0..<10 {
            let x = Double(i) * 0.1
            let r = try aabb.closestPoint(of: [x, 0, 0])
            #expect(r.sqrDistances.count == 1)
        }
    }

    @Test("rayHits: ray through origin in +x direction hits +x face at t=0.5")
    func rayHitsThroughOrigin() throws {
        let mesh = try loadCube()
        let aabb = try MeshAABB(mesh)
        let r = try aabb.rayHits(
            origins:    [0, 0, 0],
            directions: [1, 0, 0]
        )
        // Cube spans [-0.5, 0.5]^3; ray from origin in +x hits +x face at x=0.5.
        #expect(r.faceIds[0] != -1, "expected a hit, got miss")
        #expect(Swift.abs(r.ts[0] - 0.5) < 1e-9, "expected t=0.5, got \(r.ts[0])")
    }

    @Test("rayHits: ray pointing away from mesh misses")
    func rayHitsMiss() throws {
        let mesh = try loadCube()
        let aabb = try MeshAABB(mesh)
        let r = try aabb.rayHits(
            origins:    [10, 10, 10],
            directions: [1,  1,  1]
        )
        #expect(r.faceIds[0] == -1, "expected miss, got faceId=\(r.faceIds[0])")
        #expect(r.ts[0].isNaN)
    }
}

@Suite("Topology")
struct TopologyTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    @Test("edges of a cube triangulation: 18 unique edges (12 face + 6 diagonal)")
    func cubeEdges() throws {
        let mesh = try loadCube()
        let e = try SwiftIGL.edges(mesh)
        #expect(e.count % 2 == 0)
        let edgeCount = e.count / 2
        // Cube has 12 surface edges + 6 face-diagonals (one per face) = 18.
        #expect(edgeCount == 18, "expected 18 edges, got \(edgeCount)")
    }

    @Test("triangleTriangleAdjacency: closed cube has no boundaries")
    func cubeAdjacency() throws {
        let mesh = try loadCube()
        let tt = try SwiftIGL.triangleTriangleAdjacency(mesh)
        #expect(tt.count == 3 * mesh.faceCount)
        // Every entry should reference a real face (no -1 boundary markers
        // on a closed mesh).
        for (i, neighbor) in tt.enumerated() {
            #expect(neighbor >= 0,
                "edge \(i % 3) of face \(i / 3) has no adjacent face (neighbor=\(neighbor))")
            #expect(Int(neighbor) < mesh.faceCount)
        }
    }
}

@Suite("Ray casting (one-shot)")
struct RayCastTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    @Test("rayMeshIntersect agrees with the AABB-batched version")
    func oneShotMatchesAABB() throws {
        let mesh = try loadCube()
        let origins:    [Double] = [0, 0, 0,   10, 0, 0]
        let directions: [Double] = [1, 0, 0,   1, 0, 0]
        let one  = try SwiftIGL.rayMeshIntersect(origins: origins, directions: directions, on: mesh)
        let aabb = try MeshAABB(mesh)
        let cached = try aabb.rayHits(origins: origins, directions: directions)
        for i in 0..<2 {
            #expect(one.faceIds[i] == cached.faceIds[i])
            if one.faceIds[i] != -1 {
                #expect(Swift.abs(one.ts[i] - cached.ts[i]) < 1e-9)
            }
        }
    }
}
