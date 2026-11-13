#include "hullcad/mesh.hpp"

#include <fstream>
#include <map>
#include <stdexcept>

namespace hullcad {

TriMesh tessellate_hull(const BSplineSurface& S, int nu, int nv) {
    TriMesh m;
    const double u0 = S.knots_u[S.pu], u1 = S.knots_u[S.knots_u.size() - S.pu - 1];
    const double v0 = S.knots_v[S.pv], v1 = S.knots_v[S.knots_v.size() - S.pv - 1];

    // Port grid P(i,j), i along length (aft -> fwd), j along girth (keel -> deck edge).
    std::vector<uint32_t> port((nu + 1) * (nv + 1)), stbd((nu + 1) * (nv + 1));
    for (int i = 0; i <= nu; ++i) {
        double u = u0 + (u1 - u0) * i / nu;
        for (int j = 0; j <= nv; ++j) {
            double v = v0 + (v1 - v0) * j / nv;
            Vec3 p = S.eval(u, v);
            if (j == 0) p.y = 0.0;  // keel lies exactly on the centreline
            port[i * (nv + 1) + j] = static_cast<uint32_t>(m.verts.size());
            m.verts.push_back(p);
        }
    }
    for (int i = 0; i <= nu; ++i) {
        for (int j = 0; j <= nv; ++j) {
            if (j == 0) { stbd[i * (nv + 1)] = port[i * (nv + 1)]; continue; }
            Vec3 p = m.verts[port[i * (nv + 1) + j]];
            p.y = -p.y;
            stbd[i * (nv + 1) + j] = static_cast<uint32_t>(m.verts.size());
            m.verts.push_back(p);
        }
    }
    auto P = [&](int i, int j) { return port[i * (nv + 1) + j]; };
    auto Q = [&](int i, int j) { return stbd[i * (nv + 1) + j]; };

    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
            uint32_t a = P(i, j), b = P(i + 1, j), c = P(i + 1, j + 1), d = P(i, j + 1);
            m.tris.push_back({a, d, c});
            m.tris.push_back({a, c, b});
            a = Q(i, j); b = Q(i + 1, j); c = Q(i + 1, j + 1); d = Q(i, j + 1);
            m.tris.push_back({a, c, d});
            m.tris.push_back({a, b, c});
        }
        // Deck strip between port and starboard deck edges.
        uint32_t e0 = P(i, nv), e1 = P(i + 1, nv), s0 = Q(i, nv), s1 = Q(i + 1, nv);
        m.tris.push_back({e0, s0, e1});
        m.tris.push_back({e1, s0, s1});
    }

    // End caps: fan from the section centroid. Loop = keel -> port deck edge -> stbd deck edge -> keel.
    for (int end = 0; end < 2; ++end) {
        int i = end == 0 ? 0 : nu;
        std::vector<uint32_t> loop;
        for (int j = 0; j <= nv; ++j) loop.push_back(P(i, j));
        for (int j = nv; j >= 1; --j) loop.push_back(Q(i, j));
        Vec3 c;
        for (auto id : loop) c += m.verts[id];
        c = c * (1.0 / loop.size());
        uint32_t ci = static_cast<uint32_t>(m.verts.size());
        m.verts.push_back(c);
        for (size_t k = 0; k < loop.size(); ++k) {
            uint32_t a = loop[k], b = loop[(k + 1) % loop.size()];
            if (end == 0) m.tris.push_back({ci, b, a});  // aft cap faces -x
            else m.tris.push_back({ci, a, b});           // fwd cap faces +x
        }
    }
    return m;
}

TriMesh make_box(double L, double B, double D, int nx, int ny, int nz) {
    // Six faces, each a grid; vertices are welded through a coordinate map.
    TriMesh m;
    std::map<std::array<int, 3>, uint32_t> idx;
    auto vid = [&](int i, int j, int k) {
        std::array<int, 3> key{i, j, k};
        auto it = idx.find(key);
        if (it != idx.end()) return it->second;
        uint32_t id = static_cast<uint32_t>(m.verts.size());
        m.verts.push_back({L * i / nx, -0.5 * B + B * j / ny, D * k / nz});
        idx[key] = id;
        return id;
    };
    auto quad = [&](std::array<int, 3> a, std::array<int, 3> b, std::array<int, 3> c, std::array<int, 3> d) {
        uint32_t A = vid(a[0], a[1], a[2]), Bv = vid(b[0], b[1], b[2]);
        uint32_t C = vid(c[0], c[1], c[2]), Dv = vid(d[0], d[1], d[2]);
        m.tris.push_back({A, Bv, C});
        m.tris.push_back({A, C, Dv});
    };
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j) {
            quad({i, j, 0}, {i, j + 1, 0}, {i + 1, j + 1, 0}, {i + 1, j, 0});      // bottom (-z)
            quad({i, j, nz}, {i + 1, j, nz}, {i + 1, j + 1, nz}, {i, j + 1, nz});  // deck (+z)
        }
    for (int i = 0; i < nx; ++i)
        for (int k = 0; k < nz; ++k) {
            quad({i, 0, k}, {i + 1, 0, k}, {i + 1, 0, k + 1}, {i, 0, k + 1});      // stbd (-y)
            quad({i, ny, k}, {i, ny, k + 1}, {i + 1, ny, k + 1}, {i + 1, ny, k});  // port (+y)
        }
    for (int j = 0; j < ny; ++j)
        for (int k = 0; k < nz; ++k) {
            quad({0, j, k}, {0, j, k + 1}, {0, j + 1, k + 1}, {0, j + 1, k});      // aft (-x)
            quad({nx, j, k}, {nx, j + 1, k}, {nx, j + 1, k + 1}, {nx, j, k + 1});  // fwd (+x)
        }
    return m;
}

bool is_closed_and_consistent(const TriMesh& m) {
    std::map<std::pair<uint32_t, uint32_t>, int> edges;
    for (auto& t : m.tris)
        for (int k = 0; k < 3; ++k) {
            uint32_t a = t[k], b = t[(k + 1) % 3];
            if (a == b) continue;
            edges[{a, b}] += 1;
        }
    for (auto& [e, cnt] : edges) {
        if (cnt != 1) return false;
        auto it = edges.find({e.second, e.first});
        if (it == edges.end() || it->second != 1) return false;
    }
    return true;
}

double signed_volume(const TriMesh& m) {
    double v = 0.0;
    for (auto& t : m.tris) v += dot(m.verts[t[0]], cross(m.verts[t[1]], m.verts[t[2]]));
    return v / 6.0;
}

void write_obj(const TriMesh& m, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "# hullcad mesh\n";
    for (auto& v : m.verts) f << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';
    for (auto& t : m.tris) f << "f " << t[0] + 1 << ' ' << t[1] + 1 << ' ' << t[2] + 1 << '\n';
}

}  // namespace hullcad
