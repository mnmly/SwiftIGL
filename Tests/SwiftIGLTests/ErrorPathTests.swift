import Foundation
import Testing
@testable import SwiftIGL

@Suite("Error paths")
struct ErrorPathTests {

    private func loadCube() throws -> TriangleMesh {
        let url = try #require(Bundle.module.url(forResource: "cube", withExtension: "obj"))
        return try readTriangleMesh(at: url)
    }

    // ------------------------------------------------------------------
    // I/O failures
    // ------------------------------------------------------------------

    @Test("readTriangleMesh throws on missing file")
    func missingFile() {
        let url = URL(fileURLWithPath: "/nonexistent/path/to/missing-mesh.obj")
        do {
            _ = try readTriangleMesh(at: url)
            Issue.record("expected IGLError; readTriangleMesh succeeded")
        } catch let err as IGLError {
            #expect(err.description.contains("read_triangle_mesh"))
        } catch {
            Issue.record("expected IGLError, got \(type(of: error)): \(error)")
        }
    }

    // ------------------------------------------------------------------
    // Buffer-shape validation
    // (Mesh-shape errors trap via TriangleMesh preconditions — those are
    //  programmer errors, not recoverable. Bridge-level shape checks
    //  produce IGLError, which is what we test below.)
    // ------------------------------------------------------------------

    @Test("barycentricCoordinates rejects mismatched corner buffers")
    func mismatchedBarycentricBuffers() {
        let p: [Double] = [0, 0, 0,   1, 1, 1]   // 2 query points = 6 doubles
        let a: [Double] = [0, 0, 0,   0, 0, 0]
        let b: [Double] = [1, 0, 0,   1, 0, 0]
        let c: [Double] = [0, 1, 0]              // ← short by one triangle
        do {
            _ = try barycentricCoordinates(of: p, in: (a: a, b: b, c: c))
            Issue.record("expected IGLError; barycentricCoordinates succeeded")
        } catch let err as IGLError {
            #expect(err.description.contains("must all be"))
        } catch {
            Issue.record("expected IGLError, got \(type(of: error)): \(error)")
        }
    }

    @Test("marchingCubes rejects scalar buffer of wrong size")
    func badMarchingCubesShape() throws {
        let grid = try voxelGrid(
            bboxMin: SIMD3(-1, -1, -1),
            bboxMax: SIMD3(1, 1, 1),
            largestSide: 8
        )
        // Pass a too-short scalar buffer.
        let truncated = [Double](repeating: 0, count: grid.cellCount - 1)
        do {
            _ = try marchingCubes(scalars: truncated, grid: grid)
            Issue.record("expected IGLError; marchingCubes succeeded")
        } catch let err as IGLError {
            #expect(err.description.contains("nx*ny*nz"))
        } catch {
            Issue.record("expected IGLError, got \(type(of: error)): \(error)")
        }
    }

    // (voxelGrid intentionally has no bridge-level shape guard beyond
    // bbox vector lengths; the SIMD3<Double> Swift signature makes that
    // unreachable. `largestSide` validity is enforced by libigl itself
    // via assert(), which is a programmer-error trap, not IGLError.)

    // ------------------------------------------------------------------
    // signedDistance: shape mismatch via odd-length query buffer
    // ------------------------------------------------------------------

    @Test("signedDistance rejects non-multiple-of-3 query buffer")
    func badSignedDistanceQuery() throws {
        let mesh = try loadCube()
        let queries: [Double] = [0, 0, 0, 1]   // 4 doubles → not a multiple of 3
        do {
            _ = try signedDistance(of: queries, on: mesh)
            Issue.record("expected IGLError; signedDistance succeeded")
        } catch let err as IGLError {
            #expect(err.description.contains("multiple of 3"))
        } catch {
            Issue.record("expected IGLError, got \(type(of: error)): \(error)")
        }
    }

    @Test("pointMeshSquaredDistance rejects non-multiple-of-3 query buffer")
    func badPointMeshQuery() throws {
        let mesh = try loadCube()
        do {
            _ = try pointMeshSquaredDistance(of: [0, 0], on: mesh)
            Issue.record("expected IGLError; pointMeshSquaredDistance succeeded")
        } catch let err as IGLError {
            #expect(err.description.contains("multiple of 3"))
        } catch {
            Issue.record("expected IGLError, got \(type(of: error)): \(error)")
        }
    }
}
