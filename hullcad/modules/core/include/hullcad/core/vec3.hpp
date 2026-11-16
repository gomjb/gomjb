#pragma once
// Minimal 3D vector usable from both host C++ and CUDA device code.
#include <cmath>

#if defined(__CUDACC__)
#define HC_HD __host__ __device__
#else
#define HC_HD
#endif

namespace hullcad {

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

HC_HD inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
HC_HD inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
HC_HD inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
HC_HD inline Vec3 operator*(double s, Vec3 a) { return {a.x * s, a.y * s, a.z * s}; }
HC_HD inline Vec3& operator+=(Vec3& a, Vec3 b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
HC_HD inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
HC_HD inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
HC_HD inline double norm(Vec3 a) { return sqrt(dot(a, a)); }
HC_HD inline Vec3 normalize(Vec3 a) { return a * (1.0 / norm(a)); }

}  // namespace hullcad
