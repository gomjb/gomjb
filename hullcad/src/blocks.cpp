#include "hullcad/blocks.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include "hullcad/lines_plan.hpp"

namespace hullcad {

namespace {

constexpr double kSteel = 7.85;  // t/m^3
constexpr double kBig = 1e6;
constexpr double kFlatTol = 0.01;  // m, planarity tolerance for flat blocks

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
    // Zones are half-open [min, max): a member lying exactly on an upper boundary belongs to the
    // neighbouring zone, so nothing is counted twice when a seam coincides with a member.
    const double eps = 1e-9;
    if (std::fabs(c.y0 - r.ymax) < eps && std::fabs(c.y1 - r.ymax) < eps) return false;
    if (std::fabs(c.z0 - r.zmax) < eps && std::fabs(c.z1 - r.zmax) < eps) return false;
    s = c;
    return t1 > t0;
}

bool is_cargo(const TankerStructure& st, double xa, double xb) {
    double m = 0.5 * (xa + xb);
    return m > st.cargo_x0() && m < st.cargo_x1();
}

bool is_shell(const char* kind) {
    return !std::strcmp(kind, "deck") || !std::strcmp(kind, "bottom shell") || !std::strcmp(kind, "side shell");
}
bool is_skin(const char* kind) {
    return is_shell(kind) || !std::strcmp(kind, "inner bottom") || !std::strcmp(kind, "inner hull") ||
           !std::strcmp(kind, "CL bulkhead");
}

// Utilisation of fitting a x b into A x B, rotation allowed (<= 1 fits).
double fit2(double a, double b, double A, double B) {
    return std::min(std::max(a / A, b / B), std::max(b / A, a / B));
}
// Utilisation of fitting L x W x H into a space; L and W may swap, height stays vertical.
double fit3(double L, double W, double H, double SL, double SW, double SH) {
    return std::max(fit2(L, W, SL, SW), H / SH);
}

#if defined(__GNUC__) || defined(__clang__)
std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
#endif
std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

}  // namespace

namespace {

// Breadth bands between the centreline seam (at +off) and the outboard limit a, split by the
// extra longitudinal seams. Returns (name suffix, ymin, ymax); band 1 is next to the centreline.
std::vector<std::tuple<std::string, double, double>> breadth_bands(const YardProfile& y, double off, double a) {
    std::vector<double> ys;
    for (double v : y.division.long_seams_y)
        if (v > off + 0.5 && v < a - 0.5) ys.push_back(v);
    std::sort(ys.begin(), ys.end());
    const size_t n = ys.size() + 1;
    auto name = [&](size_t i, const char* side) { return n == 1 ? std::string(side) : std::to_string(i + 1) + "-" + side; };
    std::vector<std::tuple<std::string, double, double>> out;
    for (size_t i = 0; i < n; ++i) {  // starboard: off, -ys[0], -ys[1], ..., -a
        double hi = i == 0 ? off : -ys[i - 1], lo = i + 1 < n ? -ys[i] : -a;
        out.emplace_back(name(i, "S"), lo, hi);
    }
    for (size_t i = 0; i < n; ++i) {  // port: off, ys[0], ..., a
        double lo = i == 0 ? off : ys[i - 1], hi = i + 1 < n ? ys[i] : a;
        out.emplace_back(name(i, "P"), lo, hi);
    }
    return out;
}

}  // namespace

std::vector<Zone> default_cargo_zones(const TankerParams& p, const TankerStructure& st, const YardProfile& y) {
    const double off = y.division.seam_offset;
    const double a = std::min(0.5 * p.B - st.double_side, 0.5 * p.B - p.bilge_radius) - off;
    const double zdb = st.double_bottom + off;
    const double zdk = p.D - y.division.deck_zone_depth;
    const auto bands = breadth_bands(y, off, a);
    std::vector<Zone> z;
    for (auto& [sfx, lo, hi] : bands) z.push_back({"DB-" + sfx, lo, hi, -kBig, zdb, 'h'});
    const double zs = y.division.side_split_z;
    if (zs > 0) {
        z.push_back({"SIDE-L-S", -kBig, -a, -kBig, zs, 'v'});
        z.push_back({"SIDE-L-P", a, kBig, -kBig, zs, 'v'});
        z.push_back({"SIDE-U-S", -kBig, -a, zs, kBig, 'v'});
        z.push_back({"SIDE-U-P", a, kBig, zs, kBig, 'v'});
    } else {
        z.push_back({"SIDE-S", -kBig, -a, -kBig, kBig, 'v'});
        z.push_back({"SIDE-P", a, kBig, -kBig, kBig, 'v'});
    }
    for (auto& [sfx, lo, hi] : bands) z.push_back({"BHD-" + sfx, lo, hi, zdb, zdk, 'v'});
    for (auto& [sfx, lo, hi] : bands) z.push_back({"DECK-" + sfx, lo, hi, zdk, kBig, 'h'});
    return z;
}

std::vector<Zone> default_end_zones(const TankerParams&, const YardProfile& y) {
    const double off = y.division.seam_offset, zs = y.division.end_split_z;
    std::vector<Zone> z;
    for (auto& [sfx, lo, hi] : breadth_bands(y, off, kBig)) z.push_back({"LOW-" + sfx, lo, hi, -kBig, zs, 'v'});
    for (auto& [sfx, lo, hi] : breadth_bands(y, off, kBig)) z.push_back({"UP-" + sfx, lo, hi, zs, kBig, 'v'});
    return z;
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
    for (int z = 0; z < n_cargo_zones; ++z) m += zone_mass(z, xa, xb);
    return m;
}

WeightModel build_weight_model(const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                               const std::vector<Zone>& cargo_zones, const std::vector<Zone>& end_zones,
                               double dx_target, ThreadPool& pool) {
    WeightModel wm;
    wm.zones = cargo_zones;
    wm.zones.insert(wm.zones.end(), end_zones.begin(), end_zones.end());
    wm.n_cargo_zones = static_cast<int>(cargo_zones.size());
    wm.nzone = static_cast<int>(wm.zones.size());
    wm.nslice = static_cast<int>(std::ceil(p.Lpp / dx_target));
    wm.dx = p.Lpp / wm.nslice;
    wm.x0 = 0.0;
    const double tan_tol = std::tan(1.0 * 3.14159265358979 / 180.0);

    // Longitudinal members: one independent task per slice.
    std::vector<Mass> slice_mass(static_cast<size_t>(wm.nslice) * wm.nzone);
    wm.slices.assign(static_cast<size_t>(wm.nslice) * wm.nzone, {});
    pool.parallel_for(wm.nslice, [&](size_t k) {
        const double x = wm.x0 + (k + 0.5) * wm.dx;
        HullSection hs = hull_section(hull, x);
        for (auto& m : section_members(hs, x, p, st)) {
            const double mpa = plate_mass_per_area(*m.spec) * wm.dx;  // t per metre of girth
            for (int z = 0; z < wm.nzone; ++z) {
                Seg2 s = m.seg;
                if (!clip(s, wm.zones[z])) continue;
                const double dy = s.y1 - s.y0, dz = s.z1 - s.z0;
                const double len = std::hypot(dy, dz);
                if (len <= 0) continue;
                slice_mass[k * wm.nzone + z].add(len * mpa, x, 0.5 * (s.y0 + s.y1), 0.5 * (s.z0 + s.z1));
                auto& si = wm.slices[k * wm.nzone + z];
                si.extent.add(s.y0, s.z0);
                si.extent.add(s.y1, s.z1);
                si.member_len += len;
                si.t_len += len * m.spec->t_mm;
                if (is_skin(m.kind)) si.plate_len += len;
                if (is_shell(m.kind)) si.shell_len += len;
                // A member reaching this zone's upper seam continues into the neighbour zone:
                // the plates are joined there at erection (each seam counted once, by its lower zone).
                const Zone& r = wm.zones[z];
                const double eps = 1e-9;
                if (r.ymax < 0.5 * kBig && (std::fabs(s.y0 - r.ymax) < eps || std::fabs(s.y1 - r.ymax) < eps)) ++si.seam_cross;
                if (r.zmax < 0.5 * kBig && (std::fabs(s.z0 - r.zmax) < eps || std::fabs(s.z1 - r.zmax) < eps)) ++si.seam_cross;
                if (is_shell(m.kind)) {
                    // Planarity bookkeeping: horizontal plating records z, vertical records y,
                    // anything inclined records both (so the block cannot be flat).
                    const bool horiz = std::fabs(dz) <= tan_tol * std::fabs(dy);
                    const bool vert = std::fabs(dy) <= tan_tol * std::fabs(dz);
                    if (!vert) { si.flat_h.add(s.y0, s.z0); si.flat_h.add(s.y1, s.z1); }
                    if (!horiz) { si.flat_v.add(s.y0, s.z0); si.flat_v.add(s.y1, s.z1); }
                }
            }
        }
        for (auto& jt : section_joints(hs, x, p, st))
            for (int z = 0; z < wm.nzone; ++z) {
                const Zone& r = wm.zones[z];
                if (jt.y >= r.ymin && jt.y < r.ymax && jt.z >= r.zmin && jt.z < r.zmax) ++wm.slices[k * wm.nzone + z].joints;
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
    for (double x : st.bulkheads_x()) wm.transverse.push_back({x, "bulkhead", {}, {}, {}});
    for (double x : st.webs_x()) wm.transverse.push_back({x, "web", {}, {}, {}});
    std::sort(wm.transverse.begin(), wm.transverse.end(), [](const auto& a, const auto& b) { return a.x < b.x; });
    const int nz = static_cast<int>(std::ceil(p.D / 0.05));
    pool.parallel_for(wm.transverse.size(), [&](size_t i) {
        auto& item = wm.transverse[i];
        item.zone.assign(wm.nzone, Mass{});
        item.extent.assign(wm.nzone, Box2{});
        item.area.assign(wm.nzone, 0.0);
        const bool web = item.kind == "web";
        const PlateSpec& spec = web ? st.web : st.trans_bulkhead;
        const double mpa = plate_mass_per_area(spec) * (web ? 1.0 - st.web_opening_ratio : 1.0);
        HullSection hs = hull_section(hull, item.x);
        const double h = p.D / nz;
        for (int k = 0; k < nz; ++k) {
            const double z = (k + 0.5) * h;
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
                const Zone& r = wm.zones[zi];
                if (z < r.zmin || z >= r.zmax) continue;
                for (int j = 0; j < n; ++j) {
                    double a = std::max(iv[j][0], r.ymin), b = std::min(iv[j][1], r.ymax);
                    if (b <= a) continue;
                    item.zone[zi].add((b - a) * h * mpa, item.x, 0.5 * (a + b), z);
                    item.area[zi] += (b - a) * h;
                    item.extent[zi].add(a, z - 0.5 * h);
                    item.extent[zi].add(b, z + 0.5 * h);
                }
            }
        }
    });
    return wm;
}

bool Block::ok() const {
    return std::all_of(checks.begin(), checks.end(), [](const Check& c) { return c.ok(); });
}

const Check* Block::binding() const {
    const Check* b = nullptr;
    for (auto& c : checks)
        if (!b || c.ratio() > b->ratio()) b = &c;
    return b;
}

Block evaluate_block(const WeightModel& wm, int z, double xa, double xb, const YardProfile& y) {
    Block b;
    b.zone = wm.zones[z].name;
    b.x0 = xa; b.x1 = xb;
    b.mass = wm.zone_mass(z, xa, xb);
    const double steel = b.mass.w * (1.0 + y.weight.steel_margin);
    b.transport_weight = steel * (1.0 + y.weight.outfit_ratio);
    b.lift_weight = b.transport_weight + y.weight.rigging_weight;

    // Envelope and planarity from the slices whose centres lie in [xa, xb), plus transverse parts.
    Box2 ext, fh, fv;
    double girth = 0;
    int k_lo = std::max(0, static_cast<int>(std::ceil((xa - wm.x0) / wm.dx - 0.5)));
    int k_hi = std::min(wm.nslice - 1, static_cast<int>(std::ceil((xb - wm.x0) / wm.dx - 0.5)) - 1);
    for (int k = k_lo; k <= k_hi; ++k) {
        const auto& si = wm.slices[k * wm.nzone + z];
        ext.merge(si.extent);
        fh.merge(si.flat_h);
        fv.merge(si.flat_v);
        girth = std::max(girth, si.plate_len);
    }
    for (auto& t : wm.transverse)
        if (t.x >= xa && t.x < xb) ext.merge(t.extent[z]);
    b.L = xb - xa;
    b.B = ext.width();
    b.H = ext.height();
    b.flat = !ext.empty() && fh.height() <= kFlatTol && fv.width() <= kFlatTol;

    auto add = [&](const std::string& name, double value, double limit, std::string detail) {
        // Keep one entry per check name: the worst.
        for (auto& c : b.checks)
            if (c.name == name) {
                if (limit > 0 && value / limit > c.ratio()) c = {name, value, limit, std::move(detail)};
                return;
            }
        b.checks.push_back({name, value, limit, std::move(detail)});
    };

    if (y.cutting.enabled) {
        if (y.cutting.continuous_longitudinals)
            add("cutting.stiffener_length", b.L, y.cutting.max_stiffener_length,
                fmt("block length %.1f m vs stiffener %.1f m", b.L, y.cutting.max_stiffener_length));
        b.plate_butts = std::max(0, static_cast<int>(std::ceil(b.L / y.cutting.max_plate_length - 1e-9)) - 1);
        b.plate_seams = std::max(0, static_cast<int>(std::ceil(girth / y.cutting.max_plate_width - 1e-9)) - 1);
    }
    if (b.flat && y.panel_line.enabled) {
        double w = wm.zones[z].plane == 'h' ? b.B : b.H;
        add("panel_line.size", fit2(b.L, w, y.panel_line.max_length, y.panel_line.max_width), 1.0,
            fmt("main panel %.1f x %.1f m vs %.1f x %.1f m", b.L, w, y.panel_line.max_length, y.panel_line.max_width));
    }
    if (!b.flat && y.curved_shop.enabled) {
        const auto& c = y.curved_shop;
        add("curved_shop.size", fit3(b.L, b.B, b.H, c.max_length, c.max_width, c.max_height), 1.0,
            fmt("%.1f x %.1f x %.1f m vs %.1f x %.1f x %.1f m", b.L, b.B, b.H, c.max_length, c.max_width, c.max_height));
        if (c.crane_capacity > 0)
            add("curved_shop.crane", b.lift_weight, c.crane_capacity, fmt("lift %.0f t", b.lift_weight));
    }
    // Transverse parts built before block assembly: webs on the medium-assembly line,
    // bulkhead panels on the panel line.
    for (auto& t : wm.transverse) {
        if (t.x < xa || t.x >= xb || t.zone[z].w < 0.5) continue;
        const double pw = t.extent[z].width(), ph = t.extent[z].height();
        const double tw = t.zone[z].w * (1.0 + y.weight.steel_margin);
        if (t.kind == "web" && y.medium_assembly.enabled) {
            const auto& m = y.medium_assembly;
            add("medium_assembly.size", fit2(pw, ph, m.max_length, m.max_width), 1.0,
                fmt("web unit %.1f x %.1f m vs %.1f x %.1f m", pw, ph, m.max_length, m.max_width));
            add("medium_assembly.crane", tw, m.crane_capacity, fmt("web unit %.1f t", tw));
        }
        if (t.kind == "bulkhead" && y.panel_line.enabled) {
            add("panel_line.size", fit2(pw, ph, y.panel_line.max_length, y.panel_line.max_width), 1.0,
                fmt("bulkhead panel %.1f x %.1f m vs %.1f x %.1f m", pw, ph, y.panel_line.max_length,
                    y.panel_line.max_width));
            if (y.panel_line.max_weight > 0)
                add("panel_line.weight", tw, y.panel_line.max_weight, fmt("bulkhead panel %.0f t", tw));
        }
    }
    if (y.assembly.enabled) {
        const auto& a = y.assembly;
        double cap = std::max(a.crane_capacity, a.tandem_capacity);
        add("assembly.crane", b.lift_weight, cap,
            fmt("lift %.0f t vs %s %.0f t", b.lift_weight, a.tandem_capacity > a.crane_capacity ? "tandem" : "crane", cap));
        add("assembly.size", fit3(b.L, b.B, b.H, a.max_length, a.max_width, a.max_height), 1.0,
            fmt("%.1f x %.1f x %.1f m vs %.1f x %.1f x %.1f m", b.L, b.B, b.H, a.max_length, a.max_width, a.max_height));
        add("assembly.door", std::max(b.B / a.door_width, b.H / a.door_height), 1.0,
            fmt("section %.1f x %.1f m vs door %.1f x %.1f m", b.B, b.H, a.door_width, a.door_height));
    }
    if (y.turnover.enabled) {
        bool upside_down = std::any_of(y.turnover.zones.begin(), y.turnover.zones.end(), [&](const std::string& pre) {
            return b.zone.compare(0, pre.size(), pre) == 0;
        });
        if (upside_down)
            add("turnover.capacity", b.lift_weight, y.turnover.capacity, fmt("lift %.0f t", b.lift_weight));
    }
    if (y.painting.enabled) {
        const auto& pc = y.painting;
        add("painting.size", fit3(b.L, b.B, b.H, pc.cell_length, pc.cell_width, pc.cell_height), 1.0,
            fmt("%.1f x %.1f x %.1f m vs cell %.1f x %.1f x %.1f m", b.L, b.B, b.H, pc.cell_length, pc.cell_width,
                pc.cell_height));
        add("painting.weight", b.transport_weight, pc.max_weight, fmt("%.0f t", b.transport_weight));
    }
    if (y.transporter.enabled) {
        const auto& tr = y.transporter;
        add("transporter.weight", b.transport_weight, tr.combined_capacity(),
            fmt("%.0f t vs %d x %.0f t", b.transport_weight, tr.units, tr.unit_capacity));
        add("transporter.height", b.H + tr.deck_height, tr.route_height,
            fmt("block %.1f m + deck %.1f m", b.H, tr.deck_height));
        add("transporter.width", std::min(b.L, b.B), tr.route_width, fmt("load width %.1f m", std::min(b.L, b.B)));
    }
    if (y.erection.enabled) {
        double cap = std::max(y.erection.crane_capacity, y.erection.tandem_capacity);
        add("erection.crane", b.lift_weight, cap, fmt("lift %.0f t", b.lift_weight));
    }
    return b;
}

QuantityTakeoff takeoff_block(const WeightModel& wm, int z, double xa, double xb, bool flat,
                              const TankerStructure& st, const YardProfile& y) {
    QuantityTakeoff q;
    const auto& to = y.takeoff;
    const double sp = st.stiffener_spacing;
    const double L = xb - xa;
    int k_lo = std::max(0, static_cast<int>(std::ceil((xa - wm.x0) / wm.dx - 0.5)));
    int k_hi = std::min(wm.nslice - 1, static_cast<int>(std::ceil((xb - wm.x0) / wm.dx - 0.5)) - 1);
    double member_area = 0, skin_girth = 0, shell_area = 0, t_area = 0;
    for (int k = k_lo; k <= k_hi; ++k) {
        const auto& si = wm.slices[k * wm.nzone + z];
        member_area += si.member_len * wm.dx;
        shell_area += si.shell_len * wm.dx;
        t_area += si.t_len * wm.dx;
        skin_girth = std::max(skin_girth, si.plate_len);
        const double n_long = si.plate_len / sp;              // longitudinals on the skins
        q.stiffener_length += n_long * wm.dx;
        q.fillet_length += 2.0 * (n_long + si.joints) * wm.dx; // double fillets: stiffeners and plate joints
        q.erection_length += si.seam_cross * wm.dx;           // longitudinal seams to the neighbour zone
    }
    // The block's aft butt is welded at erection (the forward one belongs to the next block).
    if (xa > wm.x0 + 1e-9 && k_lo <= k_hi) q.erection_length += wm.slices[k_lo * wm.nzone + z].member_len;
    q.mean_thickness = member_area > 0 ? t_area / member_area : 0;

    // Plate butts and seams inside the block, from the stock plate size.
    const double Lp = y.cutting.max_plate_length, Wp = y.cutting.max_plate_width;
    const int butts = std::max(0, static_cast<int>(std::ceil(L / Lp - 1e-9)) - 1);
    const int seams = std::max(0, static_cast<int>(std::ceil(skin_girth / Wp - 1e-9)) - 1);
    q.shop_butt_length = butts * skin_girth + seams * L;

    // Transverse members: plate, stiffeners, perimeter fillets, openings and longitudinal slots.
    double trans_area = 0;
    for (auto& t : wm.transverse) {
        if (t.x < xa || t.x >= xb || t.area[z] <= 0) continue;
        const bool web = t.kind == "web";
        const double gross = t.area[z];
        const double net = web ? gross * (1.0 - st.web_opening_ratio) : gross;
        trans_area += net;
        const double stiff = gross / sp;
        q.stiffener_length += stiff;
        q.fillet_length += 2.0 * stiff + 2.0 * 2.0 * (t.extent[z].width() + t.extent[z].height());
        if (web) {
            const double holes = gross * st.web_opening_ratio / to.hole_area;
            const int k = std::clamp(static_cast<int>((t.x - wm.x0) / wm.dx), 0, wm.nslice - 1);
            const double slots = wm.slices[k * wm.nzone + z].plate_len / sp;
            q.cut_length += holes * to.hole_perimeter + slots * to.slot_length;
        }
    }
    q.plate_area = member_area + trans_area;
    q.paint_area = 2.0 * q.plate_area + q.stiffener_length * to.stiffener_paint_girth;
    q.cut_length += q.plate_area * 2.0 * (Lp + Wp) / (Lp * Wp);  // stock plates profile-cut on all edges
    if (!flat) q.heating_length = shell_area * to.heating_line_density;
    return q;
}

std::vector<double> butt_candidates(const TankerParams& p, const TankerStructure& st, const YardProfile& y) {
    std::vector<double> members = st.bulkheads_x();
    for (double x : st.webs_x()) members.push_back(x);
    std::vector<double> c{0.0};
    const int f_end = static_cast<int>(std::floor(st.frames.frame(p.Lpp)));
    for (int f = 0; f <= f_end; ++f) {
        double xm = 0.5 * (st.frames.x(f) + st.frames.x(f + 1));
        if (xm <= 0.0 || xm >= p.Lpp) continue;
        bool clear = std::all_of(members.begin(), members.end(), [&](double m) {
            return std::fabs(m - xm) >= y.division.min_butt_clearance;
        });
        if (clear) c.push_back(xm);
    }
    c.push_back(p.Lpp);
    return c;
}

namespace {

// Blocks of one ring (zones of the cargo or end scheme, empty zones skipped).
std::vector<Block> ring_blocks(const WeightModel& wm, const TankerStructure& st, double xa, double xb,
                               const YardProfile& y) {
    std::vector<Block> out;
    const bool cargo = is_cargo(st, xa, xb);
    const int z0 = cargo ? 0 : wm.n_cargo_zones, z1 = cargo ? wm.n_cargo_zones : wm.nzone;
    for (int z = z0; z < z1; ++z) {
        if (wm.zone_mass(z, xa, xb).w < 0.5) continue;
        out.push_back(evaluate_block(wm, z, xa, xb, y));
    }
    return out;
}

}  // namespace

Division evaluate_division(const WeightModel& wm, const TankerParams& p, const TankerStructure& st,
                           std::vector<double> butts, const YardProfile& y) {
    butts.push_back(0.0);
    butts.push_back(p.Lpp);
    std::sort(butts.begin(), butts.end());
    butts.erase(std::unique(butts.begin(), butts.end(), [](double a, double b) { return std::fabs(a - b) < 1e-6; }),
                butts.end());

    Division d;
    d.butts = butts;
    std::vector<double> members = st.bulkheads_x();
    for (double x : st.webs_x()) members.push_back(x);
    for (size_t i = 1; i + 1 < butts.size(); ++i)
        for (double m : members)
            if (std::fabs(butts[i] - m) < y.division.min_butt_clearance)
                d.issues.push_back(fmt("butt at FR%.2f is %.0f mm from a transverse member (min %.0f mm)",
                                       st.frames.frame(butts[i]), std::fabs(butts[i] - m) * 1000,
                                       y.division.min_butt_clearance * 1000));

    // Longitudinal seams must clear the longitudinal members they run along.
    {
        std::vector<std::pair<double, std::string>> ym;  // port offsets of vertical members
        for (double g : st.db_girder_y) ym.push_back({g, fmt("DB girder y=%.1f", g)});
        if (st.centreline_bulkhead) ym.push_back({0.0, "CL bulkhead"});
        ym.push_back({0.5 * p.B - st.double_side, "inner hull"});
        std::vector<std::pair<double, std::string>> zm;  // heights of horizontal members
        zm.push_back({st.double_bottom, "inner bottom"});
        for (double sz : st.side_stringer_z) zm.push_back({sz, fmt("side stringer z=%.1f", sz)});
        std::vector<double> yseams = y.division.long_seams_y;
        yseams.push_back(y.division.seam_offset);
        for (double s : yseams)
            for (auto& [m, what] : ym)
                if (std::fabs(std::fabs(s) - m) < y.division.seam_offset - 1e-9)
                    d.issues.push_back(fmt("longitudinal seam at y=%.2f m is %.0f mm from the %s (min %.0f mm)", s,
                                           std::fabs(std::fabs(s) - m) * 1000, what.c_str(), y.division.seam_offset * 1000));
        for (double s : {y.division.side_split_z, p.D - y.division.deck_zone_depth})
            if (s > 0)
                for (auto& [m, what] : zm)
                    if (std::fabs(s - m) < y.division.seam_offset - 1e-9)
                        d.issues.push_back(fmt("horizontal seam at z=%.2f m is %.0f mm from the %s (min %.0f mm)", s,
                                               std::fabs(s - m) * 1000, what.c_str(), y.division.seam_offset * 1000));
    }

    for (size_t i = 0; i + 1 < butts.size(); ++i) {
        const int ring = static_cast<int>(i) + 1;
        const double xa = butts[i], xb = butts[i + 1], len = xb - xa;
        if (len > y.division.max_ring_length + 1e-9 || len < y.division.min_ring_length - 1e-9)
            d.issues.push_back(fmt("ring R%02d length %.2f m outside %.1f-%.1f m", ring, len,
                                   y.division.min_ring_length, y.division.max_ring_length));
        for (double bx : {st.cargo_x0(), st.cargo_x1()})
            if (xa < bx - 1.0 && xb > bx + 1.0)
                d.issues.push_back(fmt("ring R%02d spans the cargo-region boundary at FR%.1f", ring, st.frames.frame(bx)));
        for (auto& b : ring_blocks(wm, st, xa, xb, y)) {
            b.ring = ring;
            b.name = fmt("R%02d-%s", ring, b.zone.c_str());
            b.fr0 = st.frames.frame(xa);
            b.fr1 = st.frames.frame(xb);
            for (int zi = 0; zi < wm.nzone; ++zi)
                if (wm.zones[zi].name == b.zone) b.q = takeoff_block(wm, zi, xa, xb, b.flat, st, y);
            for (auto& c : b.checks)
                if (!c.ok())
                    d.issues.push_back(fmt("block %s fails %s (%.0f %%): %s", b.name.c_str(), c.name.c_str(),
                                           100 * c.ratio(), c.detail.c_str()));
            d.blocks.push_back(std::move(b));
        }
    }
    return d;
}

Division auto_divide(const WeightModel& wm, const TankerParams& p, const TankerStructure& st, const YardProfile& y) {
    const std::vector<double> c = butt_candidates(p, st, y);
    const int K = static_cast<int>(c.size());
    std::vector<double> mandatory;
    for (double x : c)
        if (x > st.cargo_x0()) { mandatory.push_back(x); break; }
    for (int i = K - 1; i >= 0; --i)
        if (c[i] < st.cargo_x1()) { mandatory.push_back(c[i]); break; }
    const double minL = y.division.min_ring_length, maxL = y.division.max_ring_length;

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
            if (len > maxL + 1e-9) break;
            if (len < minL - 1e-9 || cnt[i] == inf) continue;
            bool crosses = std::any_of(mandatory.begin(), mandatory.end(),
                                       [&](double m) { return m > c[i] + 1e-9 && m < c[j] - 1e-9; });
            if (crosses) continue;
            auto blocks = ring_blocks(wm, st, c[i], c[j], y);
            if (!std::all_of(blocks.begin(), blocks.end(), [](const Block& b) { return b.ok(); })) continue;
            const double wr = wm.ring_mass(c[i], c[j]).w;
            const int n = cnt[i] + 1;
            const double cst = cost[i] + wr * wr;
            if (n < cnt[j] || (n == cnt[j] && cst < cost[j])) { cnt[j] = n; cost[j] = cst; prev[j] = i; }
        }
    }
    if (cnt[K - 1] != inf) {
        std::vector<double> butts;
        for (int j = prev[K - 1]; j > 0; j = prev[j]) butts.push_back(c[j]);
        return evaluate_division(wm, p, st, butts, y);
    }

    // Diagnose: which checks fail even for the shortest admissible rings?
    Division d;
    d.issues.push_back(fmt("no feasible division for yard '%s'", y.name.c_str()));
    std::map<std::string, std::pair<int, double>> fails;  // check -> (positions failing, worst ratio)
    std::map<std::string, std::string> worst_detail;
    int positions = 0;
    for (int i = 0; i < K; ++i) {
        int j = i + 1;
        while (j < K && c[j] - c[i] < minL - 1e-9) ++j;
        if (j >= K || c[j] - c[i] > maxL + 1e-9) continue;
        ++positions;
        std::map<std::string, double> here;
        for (auto& b : ring_blocks(wm, st, c[i], c[j], y))
            for (auto& ch : b.checks)
                if (!ch.ok() && ch.ratio() > here[ch.name]) {
                    here[ch.name] = ch.ratio();
                    if (ch.ratio() > fails[ch.name].second)
                        worst_detail[ch.name] = fmt("%s at FR%.1f: %s", b.zone.c_str(), st.frames.frame(c[i]), ch.detail.c_str());
                }
        for (auto& [name, r] : here) {
            fails[name].first++;
            fails[name].second = std::max(fails[name].second, r);
        }
    }
    for (auto& [name, v] : fails)
        d.issues.push_back(fmt("%s fails even for %.1f m rings at %d of %d positions (worst %.0f %%, %s)", name.c_str(),
                               minL, v.first, positions, 100 * v.second, worst_detail[name].c_str()));
    if (fails.empty())
        d.issues.push_back("every short ring passes; the ring length range or mandatory butts leave no path");
    return d;
}

std::vector<CheckSummary> summarize_checks(const Division& d) {
    std::map<std::string, CheckSummary> m;
    for (auto& b : d.blocks) {
        const Check* bind = b.binding();
        for (auto& c : b.checks) {
            auto& s = m[c.name];
            s.name = c.name;
            if (c.ratio() > s.max_ratio) { s.max_ratio = c.ratio(); s.worst_block = b.name; }
            if (bind == &c) s.binding_blocks++;
        }
    }
    std::vector<CheckSummary> out;
    for (auto& [k, v] : m) out.push_back(v);
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.max_ratio > b.max_ratio; });
    return out;
}

void write_division_csv(const Division& d, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "block,ring,zone,type,fr_from,fr_to,x_from_m,x_to_m,L_m,B_m,H_m,steel_t,transport_t,lift_t,"
         "LCG_m,TCG_m,VCG_m,plate_butts,plate_seams,binding_check,utilization,ok,failed_checks\n";
    for (auto& b : d.blocks) {
        Vec3 g = b.mass.cog();
        const Check* bind = b.binding();
        std::string failed;
        for (auto& c : b.checks)
            if (!c.ok()) failed += (failed.empty() ? "" : ";") + c.name;
        f << fmt("%s,%d,%s,%s,%.2f,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%d,%d,%s,%.3f,%s,%s\n",
                 b.name.c_str(), b.ring, b.zone.c_str(), b.flat ? "flat" : "curved", b.fr0, b.fr1, b.x0, b.x1, b.L, b.B,
                 b.H, b.mass.w, b.transport_weight, b.lift_weight, g.x, g.y, g.z, b.plate_butts, b.plate_seams,
                 bind ? bind->name.c_str() : "", bind ? bind->ratio() : 0.0, b.ok() ? "yes" : "no", failed.c_str());
    }
}

std::string division_svg(const Division& d, const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                         const WeightModel& wm, const YardProfile& y) {
    const double W = 1600, pad = 40;
    const double k = (W - 2 * pad) / p.Lpp;  // profile scale, px/m
    const double prof_top = 84;
    const double prof_h = p.D * k;
    const double chart_top = prof_top + prof_h + 80, chart_h = 170;
    const double sec_top = chart_top + chart_h + 70;
    const double ks = 12.0;  // section scale
    const double H = sec_top + (p.D + 2) * ks + 70;
    const char* palette[] = {"#8fb8de", "#5b8fc7", "#e3a76f", "#c97d3f", "#a8d5a2", "#6fae6a",
                             "#c4a6d8", "#9f7bbd", "#d9c27a", "#b89a3e", "#9ad0d0", "#4f9e9e"};
    auto zc = [&](const std::string& name) {
        for (int i = 0; i < wm.nzone; ++i)
            if (wm.zones[i].name == name) return palette[i % 12];
        return "#bbbbbb";
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
    auto text = [&](double x, double yy, const std::string& s, const char* anchor = "start", const char* col = "#333",
                    int size = 12) {
        o << "<text x='" << x << "' y='" << yy << "' text-anchor='" << anchor << "' fill='" << col << "' font-size='"
          << size << "'>" << s << "</text>\n";
    };

    text(pad, 26, fmt("Block division for %s: %zu rings, %zu blocks%s", y.name.c_str(), d.butts.empty() ? 0 : d.butts.size() - 1,
                      d.blocks.size(), d.issues.empty() ? " (all yard checks pass)" : " (RULE VIOLATIONS, see CSV/console)"),
         "start", d.issues.empty() ? "#222" : "#b00020", 15);
    text(pad, 44, fmt("Transporter %d x %.0f t, assembly crane %.0f t (tandem %.0f t), painting cell %.0f x %.0f x %.0f m, "
                      "erection crane %.0f t", y.transporter.units, y.transporter.unit_capacity, y.assembly.crane_capacity,
                      y.assembly.tandem_capacity, y.painting.cell_length, y.painting.cell_width, y.painting.cell_height,
                      y.erection.crane_capacity),
         "start", "#555", 11);

    // Profile: centreline profile, bulkheads, webs, butts, ring labels, end-zone split.
    for (double x : st.webs_x()) line(X(x), Zp(0), X(x), Zp(p.D), "#d8d2c4", 0.8);
    for (auto& g : slice(hull, 1, 1e-6)) line(X(g.first.x), Zp(g.first.z), X(g.second.x), Zp(g.second.z), "#222", 1.4);
    for (double x : st.bulkheads_x()) line(X(x), Zp(0), X(x), Zp(p.D), "#555", 1.6);
    line(X(st.cargo_x0()), Zp(st.double_bottom), X(st.cargo_x1()), Zp(st.double_bottom), "#888", 0.8, "4 3");
    line(X(0), Zp(y.division.end_split_z), X(st.cargo_x0()), Zp(y.division.end_split_z), "#4f9e9e", 0.8, "4 3");
    line(X(st.cargo_x1()), Zp(y.division.end_split_z), X(p.Lpp), Zp(y.division.end_split_z), "#4f9e9e", 0.8, "4 3");
    for (size_t i = 1; i + 1 < d.butts.size(); ++i)
        line(X(d.butts[i]), Zp(-1.0), X(d.butts[i]), Zp(p.D + 1.5), "#c62828", 1.4, "6 3");
    for (size_t i = 0; i + 1 < d.butts.size(); ++i)
        text(X(0.5 * (d.butts[i] + d.butts[i + 1])), Zp(p.D + 2.0), fmt("R%02zu", i + 1), "middle", "#333", 11);
    for (double f = 0; f <= st.frames.frame(p.Lpp); f += 10) {
        line(X(st.frames.x(f)), Zp(0) + 4, X(st.frames.x(f)), Zp(0) + 9, "#777", 0.8);
        text(X(st.frames.x(f)), Zp(0) + 22, fmt("%.0f", f), "middle", "#777", 10);
    }
    text(pad, Zp(0) + 40,
         "Profile: transverse bulkheads (dark), web frames (light), butts (red dashed), end-body lower/upper split (teal), frame numbers",
         "start", "#555", 11);

    // Utilisation chart: one bar per block = its binding (highest) yard check.
    auto Yc = [&](double r) { return chart_top + chart_h - std::min(r, 1.25) / 1.25 * chart_h; };
    line(pad, Yc(0), W - pad, Yc(0), "#888", 1);
    line(pad, Yc(1.0), W - pad, Yc(1.0), "#c62828", 1, "6 3");
    text(W - pad, Yc(1.0) - 4, "100 % of facility limit", "end", "#c62828", 11);
    for (size_t i = 0; i + 1 < d.butts.size(); ++i) {
        std::vector<const Block*> ring;
        for (auto& b : d.blocks)
            if (b.ring == static_cast<int>(i) + 1) ring.push_back(&b);
        if (ring.empty()) continue;
        double x0 = X(d.butts[i]) + 2, x1 = X(d.butts[i + 1]) - 2, bw = (x1 - x0) / ring.size();
        for (size_t j = 0; j < ring.size(); ++j) {
            const Block& b = *ring[j];
            const Check* bind = b.binding();
            double r = bind ? bind->ratio() : 0;
            o << "<rect x='" << x0 + j * bw << "' y='" << Yc(r) << "' width='" << std::max(1.0, bw - 1) << "' height='"
              << Yc(0) - Yc(r) << "' fill='" << (b.ok() ? zc(b.zone) : "#d32f2f") << "'><title>"
              << fmt("%s %s %.0f t: %s %.0f %%", b.name.c_str(), b.flat ? "flat" : "curved", b.lift_weight,
                     bind ? bind->name.c_str() : "-", 100 * r)
              << "</title></rect>\n";
        }
    }
    text(pad, chart_top - 10, "Utilisation of the binding facility check per block (colours = zones; red = fails)", "start",
         "#555", 11);

    // Cargo section with zone rectangles and member traces.
    const double xs = 0.5 * (st.frames.x(120) + st.frames.x(150));
    HullSection hs = hull_section(hull, xs);
    const double cx = pad + (p.B / 2 + 1) * ks;
    auto SX = [&](double yy) { return cx + yy * ks; };
    auto SZ = [&](double z) { return sec_top + (p.D + 1 - z) * ks; };
    for (int i = 0; i < wm.n_cargo_zones; ++i) {
        const Zone& zn = wm.zones[i];
        double y0 = std::max(zn.ymin, -p.B / 2 - 0.6), y1 = std::min(zn.ymax, p.B / 2 + 0.6);
        double z0 = std::max(zn.zmin, -0.6), z1 = std::min(zn.zmax, p.D + 0.6);
        o << "<rect x='" << SX(y0) << "' y='" << SZ(z1) << "' width='" << (y1 - y0) * ks << "' height='" << (z1 - z0) * ks
          << "' fill='" << palette[i % 12] << "' fill-opacity='0.35' stroke='" << palette[i % 12] << "'/>\n";
    }
    for (auto& m : section_members(hs, xs, p, st))
        line(SX(m.seg.y0), SZ(m.seg.z0), SX(m.seg.y1), SZ(m.seg.z1), "#1d3557", 2.0);
    for (int i = 0; i < wm.n_cargo_zones; ++i) {
        const Zone& zn = wm.zones[i];
        double y0 = std::max(zn.ymin, -p.B / 2 - 0.6), y1 = std::min(zn.ymax, p.B / 2 + 0.6);
        double z0 = std::max(zn.zmin, -0.6), z1 = std::min(zn.zmax, p.D + 0.6);
        bool side = zn.name.rfind("SIDE", 0) == 0;
        o << "<text x='" << SX(0.5 * (y0 + y1)) << "' y='" << SZ(0.5 * (z0 + z1)) + 4
          << "' text-anchor='middle' font-size='12' fill='#222' paint-order='stroke' stroke='#fbfaf7' stroke-width='3'"
          << (side ? fmt(" transform='rotate(-90 %.1f %.1f)'", SX(0.5 * (y0 + y1)), SZ(0.5 * (z0 + z1)) + 4) : "")
          << ">" << zn.name << "</text>\n";
    }
    text(pad, sec_top - 12, fmt("Cargo-region section at FR%.0f: zones and members", st.frames.frame(xs)), "start", "#555", 11);

    // Block table for the ring containing that section.
    int ring_mid = 0;
    for (size_t i = 0; i + 1 < d.butts.size(); ++i)
        if (d.butts[i] <= xs && xs < d.butts[i + 1]) ring_mid = static_cast<int>(i) + 1;
    double ty = sec_top + 6, tx = SX(p.B / 2 + 3);
    text(tx, ty, fmt("Ring R%02d blocks (lift weight incl. margins, outfitting, rigging):", ring_mid), "start", "#222", 13);
    for (auto& b : d.blocks) {
        if (b.ring != ring_mid) continue;
        ty += 19;
        const Check* bind = b.binding();
        o << "<rect x='" << tx << "' y='" << ty - 10 << "' width='10' height='10' fill='" << zc(b.zone) << "'/>\n";
        text(tx + 16, ty,
             fmt("%-10s %-6s %5.1f x %4.1f x %4.1f m  %5.0f t   binding: %s %.0f %%", b.name.c_str(), b.flat ? "flat" : "curved",
                 b.L, b.B, b.H, b.lift_weight, bind ? bind->name.c_str() : "-", bind ? 100 * bind->ratio() : 0.0),
             "start", b.ok() ? "#333" : "#b00020", 12);
    }
    o << "</svg>\n";
    return o.str();
}

void write_division_svg(const Division& d, const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                        const WeightModel& wm, const YardProfile& y, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << division_svg(d, hull, p, st, wm, y);
}

}  // namespace hullcad
