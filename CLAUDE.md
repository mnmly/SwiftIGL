# CLAUDE.md — SwiftIGL

## Documentation invariants

This package ships DocC-generated reference docs. Keep them buildable:

- Every **public** symbol gets a `///` doc comment. One sentence for
  properties; methods get a brief paragraph plus `- Parameters:`,
  `- Returns:`, `- Throws:` as relevant.
- Parameter docs use the **internal** label. For `func foo(of points: [Double])`
  document `- points:`, not `- of:`.
- Cross-references use signature-sensitive double backticks:
  `` ``signedDistance(of:on:type:)`` ``. Wrong signatures silently fail
  to render as links.
- New top-level public types belong in a `## Topics` group of
  `Sources/SwiftIGL/SwiftIGL.docc/SwiftIGL.md`. Forgetting this leaves
  the symbol unreachable from the landing page.
- The landing-page **first paragraph** (summary) cannot contain links —
  DocC drops the whole summary. Put links in the Overview body.
- `## Acknowledgements` etc. that contain prose should be paragraphs,
  not bullet lists with non-link text — DocC parses any `## Heading`
  followed by a list as a Topics-style group and rejects non-links.

## Verifying

```sh
scripts/build_docs.sh           # static export to ./docs
scripts/build_docs.sh preview   # live preview server
```

Expect exit 0 with no warnings on user-authored prose.

## Architecture

```
libigl-xcframework-builder  →  Frameworks/igl.xcframework  →  CxxIGL  →  SwiftIGL
   (sibling repo, shell)         (binaryTarget, 19 MB         (C++          (Swift API,
   produces stub-lib +            header tree)                 bridge)       Cxx interop)
   headers xcframework)
```

- **`igl` binaryTarget** — library-form xcframework: a stub `.a` plus
  `Headers/{igl,Eigen,unsupported}/`. Library-form (not framework-form)
  so both `<igl/...>` and `<Eigen/...>` resolve in C++ TUs.
- **`CxxIGL` target** — owns every `#include` of libigl / Eigen.
  Hand-written narrow surface in `include/CxxIGL.hpp`. No Eigen types,
  no exceptions across the boundary.
- **`SwiftIGL` target** — public Swift API. C++ interop is enabled here
  only; downstream consumers do **not** need to enable it.

## Interop rules (set by Swift C++ interop constraints)

1. **No Eigen types in bridge signatures.** Eigen's expression templates
   and >3 generic parameters fall outside the supported surface. Bridge
   wraps caller buffers with `Eigen::Map<>` internally, copies results
   back to flat `std::vector`s on return.
2. **No exceptions across the boundary.** Every bridge entry point
   returns `bool`; on failure writes a message to a trailing
   `std::string& errorOut`. Swift rethrows as ``IGLError``.
3. **`std::vector<T>` must be a fully-instantiated type.** Swift rejects
   `std::vector<T>` because it leaves `Allocator` unspecialised. The
   bridge declares `using DoubleVector = std::vector<double>;` and
   `using Int32Vector = std::vector<int32_t>;` so Swift sees plain
   named types.
4. **Flat row-major storage** (`3 * V` doubles, `3 * F` int32_t). No
   transpose at the boundary; Eigen `Map<MatrixXdR>` views the flat
   buffer directly via the row-major specialisation.
5. **`Swift.abs(_:)` must be qualified** in test/consumer code that
   enables Cxx interop — `std_math_h` shadows the global `abs`.

## Common gotchas

- **`SourceKit: No such module 'CxxIGL'`** from FleetView/clangd is
  spurious — `swift build` resolves the module correctly. The IDE just
  doesn't see the xcframework header path until SPM activates the build.
- **`std::vector` Swift-side methods are `reserve(_:)`, `push_back(_:)`,
  `size() -> UInt`** — not Swift's `reserveCapacity` / `append` / `count`.
  Use `Array(cxxVector)` to convert.
- **xcframework rebuild after a libigl version bump** must come from the
  sibling builder: `cd ../../cpp/libigl-xcframework-builder && make
  LIBIGL_VERSION=2.x.y`. The builder mirrors into `Frameworks/`
  automatically (controlled by `SWIFT_PACKAGE_FRAMEWORKS_DIR` in its
  `config.sh`).
- **Eigen pin must match libigl's `FetchContent` pin.** As of libigl
  2.5.0 → Eigen 3.4.0. Check `cmake/recipes/external/eigen.cmake`
  upstream when bumping.

## Adding a new libigl function

1. Add a narrow bridge in `Sources/CxxIGL/include/CxxIGL.hpp`:
   - Inputs: `const DoubleVector&`, `const Int32Vector&`, primitives,
     `const std::string&`.
   - Outputs: `DoubleVector&` / `Int32Vector&` out-params.
   - No Eigen, no throw — `bool` return + `std::string& errorOut`.
2. Implement in `Sources/CxxIGL/CxxIGL.cpp`. Use `Eigen::Map<>` to view
   caller buffers; copy back into the out-vector on success.
3. Expose idiomatic Swift in `Sources/SwiftIGL/SwiftIGL.swift`. Wrap
   `swiftigl.<bridgeFn>(...)`, throw ``IGLError`` on `false`, convert
   output `swiftigl.DoubleVector` to `[Double]` with `Array(_:)`.
4. Add a test in `Tests/SwiftIGLTests/`. Use `Swift.abs` for floating
   tolerance checks.

## Tests

```sh
swift build
swift test
```

Test fixtures live in `Tests/SwiftIGLTests/Resources/`. `cube.obj` is
the smallest mesh that exercises both vertex and face normals.

## Out of scope (for v1)

- `copyleft/cgal`, `copyleft/tetgen`, `embree`, `predicates`,
  `restricted/*` modules — add to the builder + Swift surface when a
  consuming feature actually needs them.
- iOS / Catalyst — builder is macOS arm64 only.
- DocC docs — copy SwiftPDAL's `scripts/build_docs.sh` pattern when the
  API surface is large enough to warrant it.
