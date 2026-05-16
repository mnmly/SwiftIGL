// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "SwiftIGL",
    platforms: [
        .macOS(.v13)
    ],
    products: [
        .library(name: "SwiftIGL", targets: ["SwiftIGL"]),
        .library(name: "SwiftIGL Dynamic", type: .dynamic, targets: ["SwiftIGL"]),
    ],
    dependencies: [
        .package(url: "https://github.com/swiftlang/swift-docc-plugin", from: "1.4.3"),
    ],
    targets: [
        // Header-only libigl + Eigen, packaged as a library-form xcframework
        // by libigl-xcframework-builder. Released as a GitHub asset on this
        // repo; the builder's `make release` publishes new tags.
        //
        // For local iteration against an unreleased version of the
        // xcframework, swap this for:
        //   .binaryTarget(name: "igl", path: "Frameworks/igl.xcframework")
        // (the builder mirrors into Frameworks/ when SWIFT_PACKAGE_FRAMEWORKS_DIR
        // is set in its config.sh). Frameworks/ is gitignored.
        .binaryTarget(
            name: "igl",
            url: "https://github.com/mnmly/SwiftIGL/releases/download/libigl-2.5.0/igl.xcframework.zip",
            checksum: "34b02652c1b07ad4fc19b176ff379ff9c09cedfbba0831eba3ac903d51e405f5"
        ),

        // C++ bridge target. Owns all #includes of libigl / Eigen and
        // exposes a small, interop-safe surface to Swift:
        //   - no Eigen types in signatures (template-heavy → opaque to Swift)
        //   - no exceptions (bool return + out-param convention)
        //   - only std::string + std::vector<double|int32_t> at the boundary
        .target(
            name: "CxxIGL",
            dependencies: ["igl"],
            publicHeadersPath: "include",
            cxxSettings: [
                .headerSearchPath("include"),
                // Eigen's MPL2-only mode keeps us off the LGPL portions.
                .define("EIGEN_MPL2_ONLY", to: "1"),
            ]
        ),

        // Idiomatic Swift surface. Cxx interop is contagious to dependents,
        // so we keep this target small to limit propagation.
        .target(
            name: "SwiftIGL",
            dependencies: ["CxxIGL"],
            swiftSettings: [
                .interoperabilityMode(.Cxx)
            ]
        ),

        .testTarget(
            name: "SwiftIGLTests",
            dependencies: ["SwiftIGL"],
            resources: [
                .copy("Resources/cube.obj")
            ],
            swiftSettings: [
                .interoperabilityMode(.Cxx)
            ]
        ),
    ],
    cxxLanguageStandard: .cxx17
)
