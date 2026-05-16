#include "CxxIGL.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/signed_distance.h>
#include <igl/winding_number.h>
#include <igl/voxel_grid.h>
#include <igl/marching_cubes.h>
#include <igl/bounding_box.h>
#include <igl/decimate.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/unique_simplices.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/barycenter.h>
#include <igl/gaussian_curvature.h>

#include <exception>
#include <limits>

namespace swiftigl {

namespace {

// Row-major Eigen matrix types that match our flat std::vector layout
// (`{x0,y0,z0,x1,y1,z1,...}`). Eigen's default storage order is column-major
// — using RowMajor here lets us Eigen::Map<>() the std::vector directly
// without a transpose copy.
using MatrixXdR = Eigen::Matrix<double,  Eigen::Dynamic, 3, Eigen::RowMajor>;
using MatrixXiR = Eigen::Matrix<int32_t, Eigen::Dynamic, 3, Eigen::RowMajor>;

bool validateMesh(const DoubleVector& V,
                  const Int32Vector& F,
                  std::string& errorOut) {
    if (V.size() % 3 != 0) {
        errorOut = "vertices buffer size must be a multiple of 3";
        return false;
    }
    if (F.size() % 3 != 0) {
        errorOut = "faces buffer size must be a multiple of 3";
        return false;
    }
    return true;
}

// Copy an Eigen matrix back into a flat std::vector in row-major order.
template <typename Derived>
void flatten(const Eigen::MatrixBase<Derived>& M, DoubleVector& out) {
    out.resize(static_cast<size_t>(M.rows()) * 3);
    for (Eigen::Index i = 0; i < M.rows(); ++i) {
        out[3 * i + 0] = M(i, 0);
        out[3 * i + 1] = M(i, 1);
        out[3 * i + 2] = M(i, 2);
    }
}

// Variant for integer matrices.
template <typename Derived>
void flattenI(const Eigen::MatrixBase<Derived>& M, Int32Vector& out, int cols) {
    out.resize(static_cast<size_t>(M.rows()) * cols);
    for (Eigen::Index i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < cols; ++j) {
            out[cols * i + j] = static_cast<int32_t>(M(i, j));
        }
    }
}

// Materialise an Eigen::MatrixXd / Xi from a flat vector of triples.
Eigen::MatrixXd toMatrixXd3(const DoubleVector& src) {
    Eigen::MatrixXd M(static_cast<Eigen::Index>(src.size() / 3), 3);
    for (Eigen::Index i = 0; i < M.rows(); ++i) {
        M(i, 0) = src[3 * i + 0];
        M(i, 1) = src[3 * i + 1];
        M(i, 2) = src[3 * i + 2];
    }
    return M;
}

Eigen::MatrixXi toMatrixXi3(const Int32Vector& src) {
    Eigen::MatrixXi M(static_cast<Eigen::Index>(src.size() / 3), 3);
    for (Eigen::Index i = 0; i < M.rows(); ++i) {
        M(i, 0) = static_cast<int>(src[3 * i + 0]);
        M(i, 1) = static_cast<int>(src[3 * i + 1]);
        M(i, 2) = static_cast<int>(src[3 * i + 2]);
    }
    return M;
}

}  // namespace

bool readTriangleMesh(const std::string& path,
                      DoubleVector& verticesOut,
                      Int32Vector& facesOut,
                      std::string& errorOut) {
    verticesOut.clear();
    facesOut.clear();
    try {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        if (!igl::read_triangle_mesh(path, V, F)) {
            errorOut = "igl::read_triangle_mesh failed for '" + path + "'";
            return false;
        }
        verticesOut.resize(static_cast<size_t>(V.rows()) * 3);
        for (Eigen::Index i = 0; i < V.rows(); ++i) {
            verticesOut[3 * i + 0] = V(i, 0);
            verticesOut[3 * i + 1] = V(i, 1);
            verticesOut[3 * i + 2] = V(i, 2);
        }
        facesOut.resize(static_cast<size_t>(F.rows()) * 3);
        for (Eigen::Index i = 0; i < F.rows(); ++i) {
            facesOut[3 * i + 0] = static_cast<int32_t>(F(i, 0));
            facesOut[3 * i + 1] = static_cast<int32_t>(F(i, 1));
            facesOut[3 * i + 2] = static_cast<int32_t>(F(i, 2));
        }
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("readTriangleMesh threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "readTriangleMesh threw an unknown exception";
        return false;
    }
}

bool writeTriangleMesh(const std::string& path,
                       const DoubleVector& vertices,
                       const Int32Vector& faces,
                       std::string& errorOut) {
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::Map<const MatrixXdR> V(vertices.data(),
                                      static_cast<Eigen::Index>(vertices.size() / 3),
                                      3);
        Eigen::Map<const MatrixXiR> F(faces.data(),
                                      static_cast<Eigen::Index>(faces.size() / 3),
                                      3);
        if (!igl::write_triangle_mesh(path, V, F)) {
            errorOut = "igl::write_triangle_mesh failed for '" + path + "'";
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("writeTriangleMesh threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "writeTriangleMesh threw an unknown exception";
        return false;
    }
}

bool perVertexNormals(const DoubleVector& vertices,
                      const Int32Vector& faces,
                      DoubleVector& normalsOut,
                      std::string& errorOut) {
    normalsOut.clear();
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::Map<const MatrixXdR> Vmap(vertices.data(),
                                         static_cast<Eigen::Index>(vertices.size() / 3),
                                         3);
        Eigen::Map<const MatrixXiR> Fmap(faces.data(),
                                         static_cast<Eigen::Index>(faces.size() / 3),
                                         3);
        Eigen::MatrixXd V = Vmap;
        Eigen::MatrixXi F = Fmap.cast<int>();
        Eigen::MatrixXd N;
        igl::per_vertex_normals(V, F, N);
        flatten(N, normalsOut);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("perVertexNormals threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "perVertexNormals threw an unknown exception";
        return false;
    }
}

bool perFaceNormals(const DoubleVector& vertices,
                    const Int32Vector& faces,
                    DoubleVector& normalsOut,
                    std::string& errorOut) {
    normalsOut.clear();
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::Map<const MatrixXdR> Vmap(vertices.data(),
                                         static_cast<Eigen::Index>(vertices.size() / 3),
                                         3);
        Eigen::Map<const MatrixXiR> Fmap(faces.data(),
                                         static_cast<Eigen::Index>(faces.size() / 3),
                                         3);
        Eigen::MatrixXd V = Vmap;
        Eigen::MatrixXi F = Fmap.cast<int>();
        Eigen::MatrixXd N;
        // The libigl signature is per_face_normals(V, F, Z, N) where Z is a
        // fallback for degenerate faces. Use zero vector — keeps degenerate
        // face normals at (0,0,0) instead of NaN.
        Eigen::Vector3d Z(0.0, 0.0, 0.0);
        igl::per_face_normals(V, F, Z, N);
        flatten(N, normalsOut);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("perFaceNormals threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "perFaceNormals threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// signedDistance
// ---------------------------------------------------------------------------

bool signedDistance(const DoubleVector& queryPoints,
                    const DoubleVector& vertices,
                    const Int32Vector&  faces,
                    int32_t signType,
                    DoubleVector& distancesOut,
                    Int32Vector&  faceIdsOut,
                    DoubleVector& closestPointsOut,
                    std::string&  errorOut) {
    distancesOut.clear();
    faceIdsOut.clear();
    closestPointsOut.clear();
    if (queryPoints.size() % 3 != 0) { errorOut = "queryPoints size must be multiple of 3"; return false; }
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::MatrixXd P = toMatrixXd3(queryPoints);
        Eigen::MatrixXd V = toMatrixXd3(vertices);
        Eigen::MatrixXi F = toMatrixXi3(faces);

        Eigen::VectorXd S;
        Eigen::VectorXi I;
        Eigen::MatrixXd C;
        Eigen::MatrixXd N;
        const auto t = static_cast<igl::SignedDistanceType>(signType);
        igl::signed_distance(P, V, F, t, S, I, C, N);

        distancesOut.resize(static_cast<size_t>(S.size()));
        for (Eigen::Index i = 0; i < S.size(); ++i) distancesOut[i] = S(i);
        faceIdsOut.resize(static_cast<size_t>(I.size()));
        for (Eigen::Index i = 0; i < I.size(); ++i) faceIdsOut[i] = static_cast<int32_t>(I(i));
        flatten(C, closestPointsOut);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("signedDistance threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "signedDistance threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// windingNumber
// ---------------------------------------------------------------------------

bool windingNumber(const DoubleVector& vertices,
                   const Int32Vector&  faces,
                   const DoubleVector& queryPoints,
                   DoubleVector& windingNumbersOut,
                   std::string&  errorOut) {
    windingNumbersOut.clear();
    if (queryPoints.size() % 3 != 0) { errorOut = "queryPoints size must be multiple of 3"; return false; }
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::MatrixXd V = toMatrixXd3(vertices);
        Eigen::MatrixXi F = toMatrixXi3(faces);
        Eigen::MatrixXd O = toMatrixXd3(queryPoints);
        Eigen::VectorXd W;
        igl::winding_number(V, F, O, W);
        windingNumbersOut.resize(static_cast<size_t>(W.size()));
        for (Eigen::Index i = 0; i < W.size(); ++i) windingNumbersOut[i] = W(i);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("windingNumber threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "windingNumber threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// voxelGrid
// ---------------------------------------------------------------------------

bool voxelGrid(const DoubleVector& bboxMin,
               const DoubleVector& bboxMax,
               int32_t largestSideCount,
               int32_t padCount,
               DoubleVector& gridPointsOut,
               Int32Vector&  sideOut,
               std::string&  errorOut) {
    gridPointsOut.clear();
    sideOut.clear();
    if (bboxMin.size() != 3 || bboxMax.size() != 3) {
        errorOut = "bboxMin and bboxMax must each have 3 elements";
        return false;
    }
    try {
        Eigen::AlignedBox<double, 3> box(
            Eigen::Vector3d(bboxMin[0], bboxMin[1], bboxMin[2]),
            Eigen::Vector3d(bboxMax[0], bboxMax[1], bboxMax[2]));
        Eigen::MatrixXd GV;
        Eigen::RowVector3i side;
        igl::voxel_grid(box, static_cast<int>(largestSideCount),
                        static_cast<int>(padCount), GV, side);
        flatten(GV, gridPointsOut);
        sideOut.resize(3);
        sideOut[0] = side(0);
        sideOut[1] = side(1);
        sideOut[2] = side(2);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("voxelGrid threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "voxelGrid threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// marchingCubes
// ---------------------------------------------------------------------------

bool marchingCubes(const DoubleVector& scalars,
                   const DoubleVector& gridPoints,
                   int32_t nx, int32_t ny, int32_t nz,
                   double  isovalue,
                   DoubleVector& verticesOut,
                   Int32Vector&  facesOut,
                   std::string&  errorOut) {
    verticesOut.clear();
    facesOut.clear();
    const size_t expectedScalar = static_cast<size_t>(nx) * static_cast<size_t>(ny) * static_cast<size_t>(nz);
    if (scalars.size() != expectedScalar) {
        errorOut = "scalars size must equal nx*ny*nz";
        return false;
    }
    if (gridPoints.size() != expectedScalar * 3) {
        errorOut = "gridPoints size must equal nx*ny*nz*3";
        return false;
    }
    try {
        Eigen::VectorXd S(static_cast<Eigen::Index>(scalars.size()));
        for (Eigen::Index i = 0; i < S.size(); ++i) S(i) = scalars[i];
        Eigen::MatrixXd GV = toMatrixXd3(gridPoints);
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::marching_cubes(S, GV,
                            static_cast<unsigned>(nx),
                            static_cast<unsigned>(ny),
                            static_cast<unsigned>(nz),
                            isovalue, V, F);
        flatten(V, verticesOut);
        flattenI(F, facesOut, 3);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("marchingCubes threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "marchingCubes threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// boundingBox
// ---------------------------------------------------------------------------

bool boundingBox(const DoubleVector& vertices,
                 double pad,
                 DoubleVector& cornersOut,
                 Int32Vector&  facesOut,
                 std::string&  errorOut) {
    cornersOut.clear();
    facesOut.clear();
    if (vertices.size() % 3 != 0) {
        errorOut = "vertices buffer size must be a multiple of 3";
        return false;
    }
    try {
        Eigen::MatrixXd V = toMatrixXd3(vertices);
        Eigen::MatrixXd BV;
        Eigen::MatrixXi BF;
        igl::bounding_box(V, pad, BV, BF);
        flatten(BV, cornersOut);
        flattenI(BF, facesOut, 3);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("boundingBox threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "boundingBox threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// decimate
// ---------------------------------------------------------------------------

bool decimate(const DoubleVector& vertices,
              const Int32Vector&  faces,
              int32_t maxFaces,
              DoubleVector& verticesOut,
              Int32Vector&  facesOut,
              Int32Vector&  birthFacesOut,
              std::string&  errorOut) {
    verticesOut.clear();
    facesOut.clear();
    birthFacesOut.clear();
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::MatrixXd V = toMatrixXd3(vertices);
        Eigen::MatrixXi F = toMatrixXi3(faces);
        Eigen::MatrixXd U;
        Eigen::MatrixXi G;
        Eigen::VectorXi J;
        if (!igl::decimate(V, F, static_cast<size_t>(maxFaces), U, G, J)) {
            errorOut = "igl::decimate returned false (often: input not a closed manifold)";
            return false;
        }
        flatten(U, verticesOut);
        flattenI(G, facesOut, 3);
        birthFacesOut.resize(static_cast<size_t>(J.size()));
        for (Eigen::Index i = 0; i < J.size(); ++i) birthFacesOut[i] = static_cast<int32_t>(J(i));
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("decimate threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "decimate threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// removeDuplicateVertices
// ---------------------------------------------------------------------------

bool removeDuplicateVertices(const DoubleVector& vertices,
                             const Int32Vector&  faces,
                             double epsilon,
                             DoubleVector& verticesOut,
                             Int32Vector&  facesOut,
                             Int32Vector&  uniqueIndicesOut,
                             Int32Vector&  inverseIndicesOut,
                             std::string&  errorOut) {
    verticesOut.clear();
    facesOut.clear();
    uniqueIndicesOut.clear();
    inverseIndicesOut.clear();
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::MatrixXd V = toMatrixXd3(vertices);
        Eigen::MatrixXi F = toMatrixXi3(faces);
        Eigen::MatrixXd SV;
        Eigen::MatrixXi SF;
        Eigen::VectorXi SVI, SVJ;
        igl::remove_duplicate_vertices(V, F, epsilon, SV, SVI, SVJ, SF);
        flatten(SV, verticesOut);
        flattenI(SF, facesOut, 3);
        uniqueIndicesOut.resize(static_cast<size_t>(SVI.size()));
        for (Eigen::Index i = 0; i < SVI.size(); ++i) uniqueIndicesOut[i] = static_cast<int32_t>(SVI(i));
        inverseIndicesOut.resize(static_cast<size_t>(SVJ.size()));
        for (Eigen::Index i = 0; i < SVJ.size(); ++i) inverseIndicesOut[i] = static_cast<int32_t>(SVJ(i));
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("removeDuplicateVertices threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "removeDuplicateVertices threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// uniqueSimplices
// ---------------------------------------------------------------------------

bool uniqueSimplices(const Int32Vector& faces,
                     Int32Vector& facesOut,
                     std::string& errorOut) {
    facesOut.clear();
    if (faces.size() % 3 != 0) {
        errorOut = "faces buffer size must be a multiple of 3";
        return false;
    }
    try {
        Eigen::MatrixXi F = toMatrixXi3(faces);
        Eigen::MatrixXi FF;
        igl::unique_simplices(F, FF);
        flattenI(FF, facesOut, 3);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("uniqueSimplices threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "uniqueSimplices threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// pointMeshSquaredDistance
// ---------------------------------------------------------------------------

bool pointMeshSquaredDistance(const DoubleVector& queryPoints,
                              const DoubleVector& vertices,
                              const Int32Vector&  faces,
                              DoubleVector& sqrDistancesOut,
                              Int32Vector&  faceIdsOut,
                              DoubleVector& closestPointsOut,
                              std::string&  errorOut) {
    sqrDistancesOut.clear();
    faceIdsOut.clear();
    closestPointsOut.clear();
    if (queryPoints.size() % 3 != 0) { errorOut = "queryPoints size must be multiple of 3"; return false; }
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::MatrixXd P = toMatrixXd3(queryPoints);
        Eigen::MatrixXd V = toMatrixXd3(vertices);
        Eigen::MatrixXi F = toMatrixXi3(faces);
        Eigen::VectorXd sqrD;
        Eigen::VectorXi I;
        Eigen::MatrixXd C;
        igl::point_mesh_squared_distance(P, V, F, sqrD, I, C);
        sqrDistancesOut.resize(static_cast<size_t>(sqrD.size()));
        for (Eigen::Index i = 0; i < sqrD.size(); ++i) sqrDistancesOut[i] = sqrD(i);
        faceIdsOut.resize(static_cast<size_t>(I.size()));
        for (Eigen::Index i = 0; i < I.size(); ++i) faceIdsOut[i] = static_cast<int32_t>(I(i));
        flatten(C, closestPointsOut);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("pointMeshSquaredDistance threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "pointMeshSquaredDistance threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// barycentricCoordinates
// ---------------------------------------------------------------------------

bool barycentricCoordinates(const DoubleVector& queryPoints,
                            const DoubleVector& triangleA,
                            const DoubleVector& triangleB,
                            const DoubleVector& triangleC,
                            DoubleVector& barycentricsOut,
                            std::string&  errorOut) {
    barycentricsOut.clear();
    if (queryPoints.size() % 3 != 0 ||
        triangleA.size() != queryPoints.size() ||
        triangleB.size() != queryPoints.size() ||
        triangleC.size() != queryPoints.size()) {
        errorOut = "queryPoints, A, B, C must all be 3*P-sized";
        return false;
    }
    try {
        Eigen::MatrixXd P = toMatrixXd3(queryPoints);
        Eigen::MatrixXd A = toMatrixXd3(triangleA);
        Eigen::MatrixXd B = toMatrixXd3(triangleB);
        Eigen::MatrixXd C = toMatrixXd3(triangleC);
        Eigen::MatrixXd L;
        igl::barycentric_coordinates(P, A, B, C, L);
        flatten(L, barycentricsOut);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("barycentricCoordinates threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "barycentricCoordinates threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// faceBarycenters
// ---------------------------------------------------------------------------

bool faceBarycenters(const DoubleVector& vertices,
                     const Int32Vector&  faces,
                     DoubleVector& barycentersOut,
                     std::string&  errorOut) {
    barycentersOut.clear();
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::MatrixXd V = toMatrixXd3(vertices);
        Eigen::MatrixXi F = toMatrixXi3(faces);
        Eigen::MatrixXd BC;
        igl::barycenter(V, F, BC);
        flatten(BC, barycentersOut);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("faceBarycenters threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "faceBarycenters threw an unknown exception";
        return false;
    }
}

// ---------------------------------------------------------------------------
// gaussianCurvature
// ---------------------------------------------------------------------------

bool gaussianCurvature(const DoubleVector& vertices,
                       const Int32Vector&  faces,
                       DoubleVector& curvatureOut,
                       std::string&  errorOut) {
    curvatureOut.clear();
    if (!validateMesh(vertices, faces, errorOut)) return false;
    try {
        Eigen::MatrixXd V = toMatrixXd3(vertices);
        Eigen::MatrixXi F = toMatrixXi3(faces);
        Eigen::VectorXd K;
        igl::gaussian_curvature(V, F, K);
        curvatureOut.resize(static_cast<size_t>(K.size()));
        for (Eigen::Index i = 0; i < K.size(); ++i) curvatureOut[i] = K(i);
        return true;
    } catch (const std::exception& e) {
        errorOut = std::string("gaussianCurvature threw: ") + e.what();
        return false;
    } catch (...) {
        errorOut = "gaussianCurvature threw an unknown exception";
        return false;
    }
}

}  // namespace swiftigl
