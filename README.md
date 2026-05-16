# SwiftIGL

[![CI](https://github.com/mnmly/SwiftIGL/actions/workflows/ci.yml/badge.svg)](https://github.com/mnmly/SwiftIGL/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-DocC-blue)](https://mnmly.github.io/SwiftIGL/documentation/swiftigl/)
[![Swift 6.0](https://img.shields.io/badge/Swift-6.0-orange.svg)](https://swift.org)
[![Platforms](https://img.shields.io/badge/platforms-macOS%2013%2B-lightgrey.svg)](https://github.com/mnmly/SwiftIGL)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Swift bindings for [libigl](https://libigl.github.io) — a small C++
geometry-processing library — via a header-only xcframework and Swift's
C++ interop.

## Install

Add to `Package.swift`:

```swift
dependencies: [
    .package(url: "https://github.com/mnmly/SwiftIGL", from: "0.1.0"),
],
targets: [
    .target(
        name: "YourTarget",
        dependencies: [.product(name: "SwiftIGL", package: "SwiftIGL")],
        swiftSettings: [.interoperabilityMode(.Cxx)]  // contagious from SwiftIGL
    ),
]
```

> **Note:** C++ interop is contagious — any Swift target that imports
> `SwiftIGL` must also set `.interoperabilityMode(.Cxx)`.

## Quick start

```swift
import SwiftIGL

// Load a mesh
let mesh = try readTriangleMesh(at: url)

// Voxelize it
let bb   = try boundingBox(mesh, pad: 0.05)
let grid = try voxelGrid(
    bboxMin: bb[vertex: 0], bboxMax: bb[vertex: 6],
    largestSide: 128
)

// Sample SDF on the grid
let sdf = try signedDistance(
    of: grid.points, on: mesh, type: .fastWindingNumber
)

// Re-extract as a clean mesh
let recon = try marchingCubes(scalars: sdf.distances, grid: grid)
try writeTriangleMesh(recon, to: outURL)
```

Full API: see the [DocC reference](https://mnmly.github.io/SwiftIGL/documentation/swiftigl/).

## Current surface (v0.1)

- **Mesh I/O** — `readTriangleMesh`, `writeTriangleMesh` (`.obj`, `.off`, `.ply`, `.stl`, `.mesh`, `.wrl`)
- **Normals** — `perVertexNormals`, `perFaceNormals`
- **Signed distance / inside-outside** — `signedDistance(of:on:type:)`, `windingNumber(of:on:)`
- **Voxelization** — `voxelGrid`, `marchingCubes`
- **Mesh processing** — `boundingBox`, `decimate`, `removeDuplicateVertices`, `uniqueSimplices`

Modules currently **out of scope**: `copyleft/cgal` (booleans), `tetgen`,
`embree`, `predicates`, `restricted/*`. Open an issue if you need any.

## Requirements

- macOS 13+
- Swift 6.0 / Xcode 16+
- C++17

## How it's built

```
libigl-xcframework-builder  →  igl.xcframework  →  SwiftIGL
  (sibling repo, shell)         (header-only         (Swift API
  produces stub-lib +            xcframework,         + thin C++
  headers xcframework)           released via         bridge)
                                 GH Releases)
```

- **`igl` binaryTarget** — library-form xcframework: a stub `.a` plus
  `Headers/{igl, Eigen, unsupported}/`. Library-form (not framework-form)
  so both `<igl/...>` and `<Eigen/...>` resolve in C++ TUs.
- **`CxxIGL` target** — hand-written C++ bridge. Owns every `#include`
  of libigl/Eigen. Narrow surface (no Eigen types, no exceptions, only
  `std::string` + `std::vector<double|int32_t>` across the boundary).
- **`SwiftIGL` target** — idiomatic Swift API. C++ interop is enabled
  here only; consumers' Swift code uses the Swift API and never sees a
  C++ type.

Why **header-only library-form** xcframework: libigl consumers need
both `<igl/...>` and `<Eigen/...>` resolvable in C++ TUs. A
single-prefix framework module cannot offer both; a library+headers
xcframework can.

## Local development

By default the package resolves `igl.xcframework` from a GitHub Release.
To iterate against a locally-built xcframework, swap the binaryTarget
in `Package.swift`:

```swift
.binaryTarget(name: "igl", path: "Frameworks/igl.xcframework")
```

Rebuild with:

```sh
cd ../../cpp/libigl-xcframework-builder
make LIBIGL_VERSION=2.5.0
```

The builder mirrors into `SwiftIGL/Frameworks/` when
`SWIFT_PACKAGE_FRAMEWORKS_DIR` is set in its `config.sh`. `Frameworks/`
is gitignored.

## Build & test

```sh
swift build
swift test
```

## Design notes

- **No Eigen types cross the Swift boundary.** Eigen's expression
  templates and >3 generic parameters fall outside Swift's C++ interop
  surface. The bridge wraps caller buffers with `Eigen::Map<>`
  internally and copies results back into flat `std::vector`s.
- **No exceptions cross the boundary.** Every bridge entry point
  returns `bool`; on failure it writes a message to a trailing
  `std::string&`. Swift rethrows as `IGLError`.
- **Flat row-major storage** (`3 * V` doubles, `3 * F` int32_t).
  Matches typical GPU/raw-buffer layouts (Metal, MLX, nvdiffrast) —
  no transpose at the boundary; no per-row allocation.

## License

[MIT](LICENSE) for the SwiftIGL bindings. The bundled libigl + Eigen
headers remain under their original [MPL2](THIRD_PARTY_LICENSES.md)
licenses.
