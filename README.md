# SwiftIGL

Swift bindings for [libigl](https://libigl.github.io) — a simple C++ geometry processing library.

## Status

**v0, very small surface.** Proving the Swift ↔ C++ ↔ libigl ↔ Eigen pipeline:

- Mesh I/O: `readTriangleMesh`, `writeTriangleMesh` (`.obj`, `.off`, `.ply`, `.stl`, `.mesh`, `.wrl`)
- Normals: `perVertexNormals`, `perFaceNormals`

Roadmap: voxelization, decimation, signed-distance, harmonic / biharmonic deformations, AABB queries — added module-by-module as needed to match the operations available in [TRELLIS.2/o-voxel](https://github.com/microsoft/TRELLIS.2).

## How it's built

```
libigl + Eigen sources
        │
        ▼  libigl-xcframework-builder (sibling repo on disk)
        │      stages headers + stub static archive
        ▼
   igl.xcframework            (header-only, library-form xcframework)
        │
        │  consumed via path-based binaryTarget during local dev
        ▼
   SwiftIGL (this repo)
   ├── CxxIGL    — C++ bridge target. Owns all #includes of igl/Eigen.
   │               Exposes a narrow surface (no Eigen types, no exceptions,
   │               only std::string + std::vector<double|int32_t>).
   └── SwiftIGL  — Idiomatic Swift API. Cxx interop is contagious to
                   dependents, so this target is kept small.
```

Why header-only library-form (not framework-form) xcframework: libigl
consumers need both `<igl/...>` and `<Eigen/...>` to resolve in C++ TUs.
A single-prefix framework module cannot offer both; a library+headers
xcframework can.

## Local development

The xcframework is currently consumed by **path** from `Frameworks/igl.xcframework`. To regenerate it:

```sh
cd ../../cpp/libigl-xcframework-builder
cp config.sh.example config.sh   # one-time; sets SWIFT_PACKAGE_FRAMEWORKS_DIR
make LIBIGL_VERSION=2.5.0
```

This shallow-clones libigl + Eigen, stages headers, builds a stub static archive, runs `xcodebuild -create-xcframework`, and mirrors the result into `SwiftIGL/Frameworks/igl.xcframework`.

## Build & test

```sh
swift build
swift test
```

## Requirements

- macOS 13+
- Swift 6.0 / Xcode 16+
- C++17

## Design notes

- **No Eigen types cross the Swift boundary.** Eigen's expression templates and >3 generic parameters are outside the supported C++ interop surface. The bridge wraps caller buffers with `Eigen::Map<>` internally, calls libigl, then flattens results back into `std::vector`s.
- **No exceptions cross the boundary.** Every bridge entry point returns `bool` and writes a message to a trailing `std::string&` out-param on failure. The Swift side rethrows as `IGLError`.
- **Mesh storage is flat row-major** (`3 * V` doubles, `3 * F` int32_t). Matches typical GPU/raw-buffer layout — no per-row allocation, easy to forward to Metal/MLX, no transpose at the boundary.
