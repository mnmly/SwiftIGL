// MeshToSDF — end-to-end demo of the SwiftIGL voxelization pipeline.
//
// Usage:
//   swift run MeshToSDF <input-mesh> <output-mesh> [grid-side] [pad]
//
// Example:
//   swift run MeshToSDF bunny.obj bunny-recon.obj 128 0.05
//
// Pipeline:
//   1. read input mesh
//   2. compute its padded bounding box
//   3. build a voxel grid covering the box
//   4. sample the signed-distance field on the grid
//      (fast winding number — robust to non-watertight inputs)
//   5. extract iso-surface via marching cubes
//   6. write the reconstructed mesh

import Foundation
import SwiftIGL

let args = CommandLine.arguments
guard args.count >= 3 else {
    FileHandle.standardError.write(Data(
        "Usage: \(args[0]) <input-mesh> <output-mesh> [grid-side=64] [pad=0.05]\n".utf8
    ))
    exit(64)
}

let inputPath  = args[1]
let outputPath = args[2]
let gridSide   = Int32(args.count >= 4 ? Int(args[3]) ?? 64 : 64)
let padFrac    = args.count >= 5 ? Double(args[4]) ?? 0.05 : 0.05

let inputURL  = URL(fileURLWithPath: inputPath)
let outputURL = URL(fileURLWithPath: outputPath)

do {
    print("→ reading \(inputPath)")
    let mesh = try readTriangleMesh(at: inputURL)
    print("   V=\(mesh.vertexCount)  F=\(mesh.faceCount)")

    // Bounding box of the mesh, then pad it by `padFrac` of its diagonal
    // so the marching cubes iso-surface doesn't get clipped.
    var lo: SIMD3<Double> = mesh[vertex: 0]
    var hi: SIMD3<Double> = lo
    for i in 1..<mesh.vertexCount {
        let v = mesh[vertex: i]
        lo = SIMD3(min(lo.x, v.x), min(lo.y, v.y), min(lo.z, v.z))
        hi = SIMD3(max(hi.x, v.x), max(hi.y, v.y), max(hi.z, v.z))
    }
    let d = hi - lo
    let diag = (d.x * d.x + d.y * d.y + d.z * d.z).squareRoot()
    let pad  = diag * padFrac
    let bboxMin = SIMD3(lo.x - pad, lo.y - pad, lo.z - pad)
    let bboxMax = SIMD3(hi.x + pad, hi.y + pad, hi.z + pad)
    print("→ bbox \(bboxMin) … \(bboxMax)  (pad=\(pad))")

    print("→ building voxel grid (largestSide=\(gridSide))")
    let grid = try voxelGrid(
        bboxMin: bboxMin,
        bboxMax: bboxMax,
        largestSide: gridSide
    )
    print("   side=\(grid.side)  cells=\(grid.cellCount)")

    print("→ sampling SDF (fast winding number)")
    let sdfStart = Date()
    let sdf = try signedDistance(
        of: grid.points, on: mesh, type: .fastWindingNumber
    )
    print("   sampled \(sdf.distances.count) points in \(String(format: "%.2f", -sdfStart.timeIntervalSinceNow))s")

    print("→ extracting iso-surface (marching cubes, isovalue=0)")
    let mcStart = Date()
    let recon = try marchingCubes(scalars: sdf.distances, grid: grid)
    print("   V=\(recon.vertexCount)  F=\(recon.faceCount)  in \(String(format: "%.2f", -mcStart.timeIntervalSinceNow))s")

    print("→ writing \(outputPath)")
    try writeTriangleMesh(recon, to: outputURL)

    print("✓ done")
} catch let error as IGLError {
    FileHandle.standardError.write(Data("error: \(error.description)\n".utf8))
    exit(1)
} catch {
    FileHandle.standardError.write(Data("error: \(error)\n".utf8))
    exit(1)
}
