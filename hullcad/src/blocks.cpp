#include "hullcad/blocks.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "hullcad/lines_plan.hpp"

namespace hullcad {

namespace {

constexpr double kSteel = 7.85;  // t/m^3
constexpr double kBig = 1e6;

double plate_mass_per_area(const PlateSpec& s) { return s.t_mm * 1e-3 * kSteel * (1.0 + s.stiff_factor); }

// Liang-Barsky clip of a segment to a zone rectangle; returns false if nothing remains.
bool clip(Seg2& s, const Zone& r) {
    double t0 = 0.0, t1 = 1.0;
    const double dy = s.y1 - s.y0, dz = s.z1 - s.z0;
    auto edge = [&](double pp, double q) {
        if (pp == 0.0) return q >= 0.0;
        double t = q / pp;
        if (pp < 0.0) { if (t > t1) return false; if (t > t0) t0 = t; }
        else { if (t < t0) return false; if (t < t1) t1 = t; }
        return true;
    };
    if (!edge(-dy, s.y0 - r.ymin) || !edge(dy, r.ymax - s.y0) || !edge(-dz, s.z0 - r.zmin) ||
        !edge(dz, r.zmax - s.z0))
        return false;
    Seg2 c{s.y0 + t0 * dy, s.z0 + t0 * dz, s.y0 + t1 * dy, s.z0 + t1 * dz};
    s = c;
    return t1 > t0;
}

bool is_cargo(const TankerStructure& st, double xa, double xb) {
    double m = 0.5 * (xa + xb);
    return m > st.cargo_x0() && m < st.cargo_x1();
}

}  // namespace

std::vector<Zone> default_cargo_zones(const TankerParams& p, const TankerStructure& st, double off,
                                      double deck_depth) {
    const double a = 0.5 * p.B - st.double_side - off;  // side seam, just inboard of the inner hull
    const double zdb = st.double_bottom + off;           // seam just above the inner bottom
    const double zdk = p.D - deck_depth;                 // deck block seam
    return {
        {"DB-S", -a, off, -kBig, zdb},  {"DB-P", off, a, -kBig, zdb},
        {"SIDE-S", -kBig, -a, -kBig, kBig}, {"SIDE-P", a, kBig, -kBig, kBig},
        {"BHD", -a, a, zdb, zdk},
        {"DECK-S", -a, off, zdk, kBig}, {"DECK-P", off, a, zdk, kBig},
    };
}

Mass WeightModel::prefix_at(int z, double x) const {
    double s = std::clamp((x - x0) / dx, 0.0, static_cast<double>(nslice));
    int k = std::min(static_cast<int>(s), nslice - 1);
    double f = s - k;
    const Mass& a = prefix[k * nzone + z];
    const Mass& b = prefix[(k + 1) * nzone + z];
    return {a.w + f * (b.w - a.w), a.mx + f * (b.mx - a.mx), a.my + f * (b.my - a.my), a.mz + f * (b.mz - a.mz)};
}

Mass WeightModel::zone_mass(int z, double xa, double xb) const {
    Mass a = prefix_at(z, xa), b = prefix_at(z, xb);
    Mass m{b.w - a.w, b.mx - a.mx, b.my - a.my, b.mz - a.mz};
    for (auto& t : transverse)
        if (t.x >= xa && t.x < xb) m += t.zone[z];
    return m;
}

Mass WeightModel::ring_mass(double xa, double xb) const {
    Mass m;
    for (int z = 0; z < nzone; ++z) m += zone_mass(z, xa, xb);
    return m;
}

WeightModel build_weight_model(const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                               const std::vector<Zone>& zones, double dx_target, ThreadPool& pool) {
    WeightModel wm;
    wm.nzone = static_cast<int>(zones.size());
    wm.nslice = static_cast<int>(std::ceil(p.Lpp / dx_target));
    wm.dx = p.Lpp / wm.nslice;
    wm.x0 = 0.0;

    // Longitudinal members: one independent task per slice.
    std::vector<Mass> slice_mass(static_cast<size_t>(wm.nslice) * wm.nzone);
    pool.parallel_for(wm.nslice, [&](size_t k) {
        const double x = wm.x0 + (k + 0.5) * wm.dx;
        HullSection hs = hull_section(hull, x);
        for (auto& m : section_members(hs, x, p, st)) {
            const double mpa = plate_mass_per_area(*m.spec) * wm.dx;  // t per metre of girth
            for (int z = 0; z < wm.nzone; ++z) {
                Seg2 s = m.seg;
                if (!clip(s, zones[z])) continue;
                double len = std::hypot(s.y1 - s.y0, s.z1 - s.z0);
                if (len <= 0) continue;
                slice_mass[k * wm.nzone + z].add(len * mpa, x, 0.5 * (s.y0 + s.y1), 0.5 * (s.z0 + s.z1));
            }
        }
    });
    wm.prefix.assign(static_cast<size_t>(wm.nslice + 1) * wm.nzone, Mass{});
    for (int k = 0; k < wm.nslice; ++k)
        for (int z = 0; z < wm.nzone; ++z) {
            Mass m = wm.prefix[k * wm.nzone + z];
            m += slice_mass[k * wm.nzone + z];
            wm.prefix[(k + 1) * wm.nzone + z] = m;
        }

    // Transverse members: integrate the plate area in z strips, clipped to each zone.
    for (double x : st.bulkheads_x()) wm.transverse.push_back({x, "bulkhead", {}});
    for (double x : st.webs_x()) wm.transverse.push_back({x, "web", {}});
    std::sort(wm.transverse.begin(), wm.transverse.end(),
              [](const auto& a, const auto& b) { return a.x < b.x; });
    const double dz = 0.05;
    const int nz = static_cast<int>(std::ceil(p.D / dz));
    pool.parallel_for(wm.transverse.size(), [&](size_t i) {
        auto& item = wm.transverse[i];
        item.zone.assign(wm.nzone, Mass{});
        const bool web = item.kind == "web";
        const PlateSpec& spec = web ? st.web : st.trans_bulkhead;
        const double mpa = plate_mass_per_area(spec) * (web ? 1.0 - st.web_opening_ratio : 1.0);
        HullSection hs = hull_section(hull, item.x);
        for (int k = 0; k < nz; ++k) {
            const double z = (k + 0.5) * p.D / nz, h = p.D / nz;
            const double yh = hs.half_breadth(z);
            if (yh <= 0) continue;
            double iv[2][2];
            int n = 0;
            if (!web || z < st.double_bottom) {
                iv[n][0] = -yh; iv[n][1] = yh; ++n;
            } else {
                double yi = inner_hull_y(hs, p, st, z);
                if (yi <= 0.5) continue;
                iv[n][0] = yi; iv[n][1] = yh; ++n;
                iv[n][0] = -yh; iv[n][1] = -yi; ++n;
            }
            for (int zi = 0; zi < wm.nzone; ++zi) {
                const Zone& r = zones[zi];
                if (z < r.zmin || z >= r.zmax) continue;
                for (int j = 0; j < n; ++j) {
                    double a = std::max(iv[j][0], r.ymin), b = std::min(iv[j][1], r.ymax);
                    if (b > a) item.zone[zi].add((b - a) * h * mpa, item.x, 0.5 * (a + b), z);
                }
            }
        }
    });
    return wm;
}

std::vector<double> butt_candidates(const TankerParams& p, const TankerStructure& st, const DivisionRules& r) {
    std::vector<double> members = st.bulkheads_x();
    for (double x : st.webs_x()) members.push_back(x);
    std::vector<double> c{0.0};
    const int f_end = static_cast<int>(std::floor(st.frames.frame(p.Lpp)));
    for (int f = 0; f <= f_end; ++f) {
        double xm = 0.5 * (st.frames.x(f) + st.frames.x(f + 1));
        if (xm <= 0.0 || xm >= p.Lpp) continue;
        bool clear = std::all_of(members.begin(), members.end(),
                                 [&](double m) { return std::fabs(m - xm) >= r.min_butt_clearance; });
        if (clear) c.push_back(xm);
    }
    c.push_back(p.Lpp);
    return c;
}

Division evaluate_division(const WeightModel& wm, const std::vector<Zone>& zones, const TankerParams& p,
                           const TankerStructure& st, std::vector<double> butts, const DivisionRules& r) {
    butts.push_back(0.0);
    butts.push_back(p.Lpp);
    std::sort(butts.begin(), butts.end());
    butts.erase(std::unique(butts.begin(), butts.end(), [](double a, double b) { return std::fabs(a - b) < 1e-6; }),
                butts.end());

    Division d;
    d.butts = butts;
    char buf[256];
    std::vector<double> members = st.bulkheads_x();
    for (double x : st.webs_x()) members.push_back(x);
    for (size_t i = 1; i + 1 < butts.size(); ++i)
        for (double m : members)
            if (std::fabs(butts[i] - m) < r.min_butt_clearance) {
                std::snprintf(buf, sizeof buf, "butt at FR%.2f is %.0f mm from a transverse member (min %.0f mm)",
                              st.frames.frame(butts[i]), std::fabs(butts[i] - m) * 1000, r.min_butt_clearance * 1000);
                d.issues.push_back(buf);
            }

    for (size_t i = 0; i + 1 < butts.size(); ++i) {
        const int ring = static_cast<int>(i) + 1;
        const double xa = butts[i], xb = butts[i + 1], len = xb - xa;
        if (len > r.max_ring_length + 1e-9 || len < r.min_ring_length - 1e-9) {
            std::snprintf(buf, sizeof buf, "ring R%02d length %.2f m outside %.1f-%.1f m", ring, len,
                          r.min_ring_length, r.max_ring_length);
            d.issues.push_back(buf);
        }
        const double tol = 1.0;
        for (double bx : {st.cargo_x0(), st.cargo_x1()})
            if (xa < bx - tol && xb > bx + tol) {
                std::snprintf(buf, sizeof buf, "ring R%02d spans the cargo-region boundary at FR%.1f", ring,
                              st.frames.frame(bx));
                d.issues.push_back(buf);
            }

        auto push = [&](const std::string& zone, const Mass& m) {
            Block b;
            b.ring = ring;
            b.zone = zone;
            std::snprintf(buf, sizeof buf, "R%02d-%s", ring, zone.c_str());
            b.name = buf;
            b.x0 = xa; b.x1 = xb;
            b.fr0 = st.frames.frame(xa); b.fr1 = st.frames.frame(xb);
            b.mass = m;
            b.overweight = m.w > r.max_block_weight;
            if (b.overweight) {
                std::snprintf(buf, sizeof buf, "block %s weighs %.0f t (limit %.0f t)", b.name.c_str(), m.w,
                              r.max_block_weight);
                d.issues.push_back(buf);
            }
            d.blocks.push_back(b);
        };
        if (is_cargo(st, xa, xb)) {
            for (int z = 0; z < wm.nzone; ++z) {
                Mass m = wm.zone_mass(z, xa, xb);
                if (m.w >= 0.5) push(zones[z].name, m);
            }
        } else {
            push(0.5 * (xa + xb) < st.cargo_x0() ? "AFT" : "FORE", wm.ring_mass(xa, xb));
        }
    }
    return d;
}

Division auto_divide(const WeightModel& wm, const std::vector<Zone>& zones, const TankerParams& p,
                     const TankerStructure& st, const DivisionRules& r) {
    const std::vector<double> c = butt_candidates(p, st, r);
    const int K = static_cast<int>(c.size());
    std::vector<double> mandatory;
    for (double x : c)
        if (x > st.cargo_x0()) { mandatory.push_back(x); break; }
    for (int i = K - 1; i >= 0; --i)
        if (c[i] < st.cargo_x1()) { mandatory.push_back(c[i]); break; }

    auto max_block = [&](double xa, double xb) {
        if (!is_cargo(st, xa, xb)) return wm.ring_mass(xa, xb).w;
        double w = 0;
        for (int z = 0; z < wm.nzone; ++z) w = std::max(w, wm.zone_mass(z, xa, xb).w);
        return w;
    };

    // best[j] = (rings, sum over rings of (ring weight)^2) for [0, c_j], lexicographic.
    // With the ring count fixed the total weight is fixed, so the square sum is smallest when
    // the rings are evenly loaded (no tiny leftover rings).
    const int inf = std::numeric_limits<int>::max();
    std::vector<int> cnt(K, inf), prev(K, -1);
    std::vector<double> cost(K, 0.0);
    cnt[0] = 0;
    for (int j = 1; j < K; ++j) {
        for (int i = j - 1; i >= 0; --i) {
            const double len = c[j] - c[i];
            if (len > r.max_ring_length + 1e-9) break;
            if (len < r.min_ring_length - 1e-9 || cnt[i] == inf) continue;
            bool crosses = std::any_of(mandatory.begin(), mandatory.end(),
                                       [&](double m) { return m > c[i] + 1e-9 && m < c[j] - 1e-9; });
            if (crosses) continue;
            const double w = max_block(c[i], c[j]);
            if (w > r.max_block_weight) continue;
            const int n = cnt[i] + 1;
            const double wr = wm.ring_mass(c[i], c[j]).w;
            const double cst = cost[i] + wr * wr;
            if (n < cnt[j] || (n == cnt[j] && cst < cost[j])) { cnt[j] = n; cost[j] = cst; prev[j] = i; }
        }
    }
    if (cnt[K - 1] == inf) {
        Division d;
        d.issues.push_back("no feasible division: relax the weight limit or the ring length range");
        return d;
    }
    std::vector<double> butts;
    for (int j = prev[K - 1]; j > 0; j = prev[j]) butts.push_back(c[j]);
    return evaluate_division(wm, zones, p, st, butts, r);
}

void write_division_csv(const Division& d, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "block,ring,zone,fr_from,fr_to,x_from_m,x_to_m,length_m,weight_t,LCG_m,TCG_m,VCG_m,overweight\n";
    char buf[512];
    for (auto& b : d.blocks) {
        Vec3 g = b.mass.cog();
        std::snprintf(buf, sizeof buf, "%s,%d,%s,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%.2f,%.2f,%.2f,%s\n", b.name.c_str(),
                      b.ring, b.zone.c_str(), b.fr0, b.fr1, b.x0, b.x1, b.x1 - b.x0, b.mass.w, g.x, g.y, g.z,
                      b.overweight ? "yes" : "no");
        f << buf;
    }
}

void write_division_svg(const Division& d, const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                        const std::vector<Zone>& zones, const DivisionRules& r, const std::string& path) {
    const double W = 1600, pad = 40;
    const double k = (W - 2 * pad) / p.Lpp;  // profile scale, px/m
    const double prof_top = 60;
    const double prof_h = p.D * k;
    const double chart_top = prof_top + prof_h + 70, chart_h = 170;
    const double sec_top = chart_top + chart_h + 70;
    const double ks = 12.0;  // section scale
    const double H = sec_top + (p.D + 2) * ks + 60;
    const char* zone_col[] = {"#8fb8de", "#5b8fc7", "#e3a76f", "#c97d3f", "#9ccc9c", "#c4a6d8", "#9f7bbd", "#bbbbbb"};
    auto zc = [&](const std::string& name) {
        for (size_t i = 0; i < zones.size(); ++i)
            if (zones[i].name == name) return zone_col[std::min<size_t>(i, 6)];
        return zone_col[7];
    };

    std::ostringstream o;
    o << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W << "' height='" << H << "' viewBox='0 0 " << W << ' '
      << H << "' font-family='sans-serif' font-size='12'>\n<rect width='100%' height='100%' fill='#fbfaf7'/>\n";
    auto X = [&](double x) { return pad + x * k; };
    auto Zp = [&](double z) { return prof_top + (p.D - z) * k; };
    auto line = [&](double x1, double y1, double x2, double y2, const char* col, double w, const char* dash = nullptr) {
        o << "<line x1='" << x1 << "' y1='" << y1 << "' x2='" << x2 << "' y2='" << y2 << "' stroke='" << col
          << "' stroke-width='" << w << "'" << (dash ? std::string(" stroke-dasharray='") + dash + "'" : "") << "/>\n";
    };
    auto text = [&](double x, double y, const std::string& s, const char* anchor = "start", const char* col = "#333",
                    int size = 12) {
        o << "<text x='" << x << "' y='" << y << "' text-anchor='" << anchor << "' fill='" << col << "' font-size='"
          << size << "'>" << s << "</text>\n";
    };

    char buf[256];
    std::snprintf(buf, sizeof buf, "Block division: %zu rings, %zu blocks, limit %.0f t per block, ring length %.0f-%.0f m%s",
                  d.butts.size() - 1, d.blocks.size(), r.max_block_weight, r.min_ring_length, r.max_ring_length,
                  d.issues.empty() ? "  (all rules met)" : "  (RULE VIOLATIONS, see CSV/console)");
    text(pad, 28, buf, "start", d.issues.empty() ? "#222" : "#b00020", 15);

    // Profile: centreline profile, bulkheads, webs, butts, ring labels.
    for (double x : st.webs_x()) line(X(x), Zp(0), X(x), Zp(p.D), "#d8d2c4", 0.8);
    for (auto& g : slice(hull, 1, 1e-6)) line(X(g.first.x), Zp(g.first.z), X(g.second.x), Zp(g.second.z), "#222", 1.4);
    for (double x : st.bulkheads_x()) line(X(x), Zp(0), X(x), Zp(p.D), "#555", 1.6);
    line(X(st.cargo_x0()), Zp(st.double_bottom), X(st.cargo_x1()), Zp(st.double_bottom), "#888", 0.8, "4 3");
    for (size_t i = 1; i + 1 < d.butts.size(); ++i)
        line(X(d.butts[i]), Zp(-1.0), X(d.butts[i]), Zp(p.D + 1.5), "#c62828", 1.4, "6 3");
    for (size_t i = 0; i + 1 < d.butts.size(); ++i) {
        std::snprintf(buf, sizeof buf, "R%02zu", i + 1);
        text(X(0.5 * (d.butts[i] + d.butts[i + 1])), Zp(p.D + 2.0), buf, "middle", "#333", 11);
    }
    for (double f = 0; f <= st.frames.frame(p.Lpp); f += 10) {
        line(X(st.frames.x(f)), Zp(0) + 4, X(st.frames.x(f)), Zp(0) + 9, "#777", 0.8);
        std::snprintf(buf, sizeof buf, "%.0f", f);
        text(X(st.frames.x(f)), Zp(0) + 22, buf, "middle", "#777", 10);
    }
    text(pad, Zp(0) + 40, "Profile: transverse bulkheads (dark), web frames (light), butts (red dashed), frame numbers below", "start", "#555", 11);

    // Block weight chart: one bar per block, grouped by ring.
    double wmax = r.max_block_weight * 1.15;
    for (auto& b : d.blocks) wmax = std::max(wmax, b.mass.w * 1.05);
    auto Yc = [&](double w) { return chart_top + chart_h - w / wmax * chart_h; };
    line(pad, Yc(0), W - pad, Yc(0), "#888", 1);
    line(pad, Yc(r.max_block_weight), W - pad, Yc(r.max_block_weight), "#c62828", 1, "6 3");
    std::snprintf(buf, sizeof buf, "limit %.0f t", r.max_block_weight);
    text(W - pad, Yc(r.max_block_weight) - 4, buf, "end", "#c62828", 11);
    for (size_t i = 0; i + 1 < d.butts.size(); ++i) {
        std::vector<const Block*> ring;
        for (auto& b : d.blocks) if (b.ring == static_cast<int>(i) + 1) ring.push_back(&b);
        if (ring.empty()) continue;
        double x0 = X(d.butts[i]) + 2, x1 = X(d.butts[i + 1]) - 2, bw = (x1 - x0) / ring.size();
        for (size_t j = 0; j < ring.size(); ++j) {
            const Block& b = *ring[j];
            o << "<rect x='" << x0 + j * bw << "' y='" << Yc(b.mass.w) << "' width='" << std::max(1.0, bw - 1)
              << "' height='" << Yc(0) - Yc(b.mass.w) << "' fill='" << (b.overweight ? "#d32f2f" : zc(b.zone))
              << "'><title>" << b.name << " " << static_cast<int>(b.mass.w + 0.5) << " t</title></rect>\n";
        }
    }
    text(pad, chart_top - 10, "Block weights (t), bars grouped by ring; colours match the section zones below", "start", "#555", 11);

    // Midship section with zone rectangles and member traces.
    const double xs = 0.5 * (st.frames.x(120) + st.frames.x(150));
    HullSection hs = hull_section(hull, xs);
    const double cx = pad + (p.B / 2 + 1) * ks;
    auto SX = [&](double y) { return cx + y * ks; };
    auto SZ = [&](double z) { return sec_top + (p.D + 1 - z) * ks; };
    for (size_t i = 0; i < zones.size(); ++i) {
        const Zone& zn = zones[i];
        double y0 = std::max(zn.ymin, -p.B / 2 - 0.6), y1 = std::min(zn.ymax, p.B / 2 + 0.6);
        double z0 = std::max(zn.zmin, -0.6), z1 = std::min(zn.zmax, p.D + 0.6);
        o << "<rect x='" << SX(y0) << "' y='" << SZ(z1) << "' width='" << (y1 - y0) * ks << "' height='"
          << (z1 - z0) * ks << "' fill='" << zone_col[std::min<size_t>(i, 6)] << "' fill-opacity='0.35' stroke='"
          << zone_col[std::min<size_t>(i, 6)] << "'/>\n";
        text(SX(0.5 * (y0 + y1)), SZ(0.5 * (z0 + z1)) + 4, zn.name, "middle", "#222", 12);
    }
    for (auto& m : section_members(hs, xs, p, st))
        line(SX(m.seg.y0), SZ(m.seg.z0), SX(m.seg.y1), SZ(m.seg.z1), "#1d3557", 2.0);
    std::snprintf(buf, sizeof buf,
                  "Cargo-region section at FR%.0f: zones (seams %.1f m clear of members), shell, inner hull, inner bottom, "
                  "CL bulkhead, DB girders, side stringers",
                  st.frames.frame(xs), 0.3);
    text(pad, sec_top - 12, buf, "start", "#555", 11);

    // Block table for the same ring next to the section.
    int ring_mid = 0;
    for (size_t i = 0; i + 1 < d.butts.size(); ++i)
        if (d.butts[i] <= xs && xs < d.butts[i + 1]) ring_mid = static_cast<int>(i) + 1;
    double ty = sec_top + 10, tx = SX(p.B / 2 + 3);
    std::snprintf(buf, sizeof buf, "Ring R%02d blocks:", ring_mid);
    text(tx, ty, buf, "start", "#222", 13);
    for (auto& b : d.blocks) {
        if (b.ring != ring_mid) continue;
        ty += 18;
        Vec3 g = b.mass.cog();
        std::snprintf(buf, sizeof buf, "%-10s FR%.1f-FR%.1f  %6.1f t   LCG %.1f  TCG %+.1f  VCG %.1f m", b.name.c_str(),
                      b.fr0, b.fr1, b.mass.w, g.x, g.y, g.z);
        o << "<rect x='" << tx << "' y='" << ty - 10 << "' width='10' height='10' fill='" << zc(b.zone) << "'/>\n";
        text(tx + 16, ty, buf, "start", "#333", 12);
    }
    o << "</svg>\n";
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << o.str();
}

}  // namespace hullcad
