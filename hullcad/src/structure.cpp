#include "hullcad/structure.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hullcad/lines_plan.hpp"

namespace hullcad {

FrameTable::FrameTable(std::vector<FrameSegment> segs) : segs_(std::move(segs)) {
    if (segs_.empty() || segs_.front().from_frame != 0.0)
        throw std::invalid_argument("FrameTable: first segment must start at frame 0");
    x0_.push_back(0.0);
    for (size_t i = 1; i < segs_.size(); ++i) {
        if (segs_[i].from_frame <= segs_[i - 1].from_frame)
            throw std::invalid_argument("FrameTable: segments must be in increasing frame order");
        x0_.push_back(x0_[i - 1] + (segs_[i].from_frame - segs_[i - 1].from_frame) * segs_[i - 1].spacing);
    }
}

double FrameTable::x(double f) const {
    size_t i = 0;
    while (i + 1 < segs_.size() && f >= segs_[i + 1].from_frame) ++i;
    return x0_[i] + (f - segs_[i].from_frame) * segs_[i].spacing;
}

double FrameTable::frame(double xv) const {
    size_t i = 0;
    while (i + 1 < segs_.size() && xv >= x0_[i + 1]) ++i;
    return segs_[i].from_frame + (xv - x0_[i]) / segs_[i].spacing;
}

double FrameTable::spacing_at(double f) const {
    size_t i = 0;
    while (i + 1 < segs_.size() && f >= segs_[i + 1].from_frame) ++i;
    return segs_[i].spacing;
}

std::vector<double> TankerStructure::bulkheads_x() const {
    std::vector<double> xs;
    for (double f : bulkhead_frames) xs.push_back(frames.x(f));
    return xs;
}

std::vector<double> TankerStructure::webs_x() const {
    std::vector<double> xs;
    for (int f = static_cast<int>(std::ceil(cargo_aft_frame)); f <= cargo_fwd_frame; ++f) {
        if (f % web_every != 0) continue;
        bool on_bhd = std::any_of(bulkhead_frames.begin(), bulkhead_frames.end(),
                                  [&](double b) { return std::fabs(b - f) < 1e-9; });
        if (!on_bhd) xs.push_back(frames.x(f));
    }
    return xs;
}

TankerStructure default_tanker_structure(const TankerParams& p) {
    TankerStructure st;
    // Aft body 0.80 m, cargo region 0.95 m (webs every 5 frames = 4.75 m), fore body 0.80 m.
    st.frames = FrameTable({{0, 0.80}, {50, 0.95}, {240, 0.80}});
    st.cargo_aft_frame = 50;   // x = 40.0 m, engine-room front bulkhead
    st.cargo_fwd_frame = 240;  // x = 220.5 m, collision bulkhead (0.077 Lpp from FP)
    // Aft peak, ER front, slop | COT6, COT6|5, ... COT2|1, collision.
    st.bulkhead_frames = {10, 50, 60, 90, 120, 150, 180, 210, 240};
    st.db_girder_y = {0.0, 5.0, 10.0, 15.0};
    st.side_stringer_z = {7.0, 12.5, 17.0};
    if (st.cargo_x1() >= p.Lpp) throw std::runtime_error("default_tanker_structure: layout longer than Lpp");
    return st;
}

double HullSection::half_breadth(double z) const {
    double best = 0.0;
    for (auto& s : segs) {
        double lo = std::min(s.z0, s.z1), hi = std::max(s.z0, s.z1);
        if (z < lo || z > hi || hi - lo < 1e-12) continue;
        double t = (z - s.z0) / (s.z1 - s.z0);
        best = std::max(best, s.y0 + t * (s.y1 - s.y0));
    }
    return best;
}

double HullSection::bottom_z(double y) const {
    double best = std::numeric_limits<double>::infinity();
    for (auto& s : segs) {
        double lo = std::min(s.y0, s.y1), hi = std::max(s.y0, s.y1);
        if (y < lo || y > hi || hi - lo < 1e-12) continue;
        double t = (y - s.y0) / (s.y1 - s.y0);
        best = std::min(best, s.z0 + t * (s.z1 - s.z0));
    }
    return best;
}

HullSection hull_section(const TriMesh& hull, double x) {
    HullSection hs;
    for (auto& g : slice(hull, 0, x)) hs.segs.push_back({g.first.y, g.first.z, g.second.y, g.second.z});
    return hs;
}

double inner_hull_y(const HullSection& s, const TankerParams& p, const TankerStructure& st, double z) {
    return std::min(0.5 * p.B - st.double_side, s.half_breadth(z) - st.double_side);
}

std::vector<MemberTrace> section_members(const HullSection& s, double x, const TankerParams& p,
                                         const TankerStructure& st) {
    std::vector<MemberTrace> out;
    // Shell and deck plating straight from the hull section.
    for (auto& g : s.segs) {
        const PlateSpec* spec;
        const char* kind;
        if (std::min(g.z0, g.z1) >= p.D - 1e-6) { spec = &st.deck; kind = "deck"; }
        else if (std::fabs(g.z1 - g.z0) <= std::fabs(g.y1 - g.y0)) { spec = &st.bottom_shell; kind = "bottom shell"; }
        else { spec = &st.side_shell; kind = "side shell"; }
        out.push_back({g, spec, kind});
    }
    if (x < st.cargo_x0() || x > st.cargo_x1()) return out;

    // Double-hull members exist only in the cargo region.
    const double db = st.double_bottom;
    const double min_space = 0.5;  // no inner hull where the wing space would be narrower
    const double yib = inner_hull_y(s, p, st, db);
    if (yib > min_space) out.push_back({{-yib, db, yib, db}, &st.inner_bottom, "inner bottom"});

    const int nz = 30;
    for (int k = 0; k < nz; ++k) {
        double za = db + (p.D - db) * k / nz, zb = db + (p.D - db) * (k + 1) / nz;
        double ya = inner_hull_y(s, p, st, za), yb = inner_hull_y(s, p, st, zb);
        if (ya <= min_space || yb <= min_space) continue;
        out.push_back({{ya, za, yb, zb}, &st.inner_hull, "inner hull"});
        out.push_back({{-ya, za, -yb, zb}, &st.inner_hull, "inner hull"});
    }
    if (st.centreline_bulkhead && yib > min_space)
        out.push_back({{0.0, db, 0.0, p.D}, &st.long_bulkhead, "CL bulkhead"});

    for (double gy : st.db_girder_y) {
        if (gy > yib - 0.2) continue;
        double zb = s.bottom_z(gy);
        if (!std::isfinite(zb) || db - zb < 0.2) continue;
        out.push_back({{gy, zb, gy, db}, &st.db_girder, "DB girder"});
        if (gy > 0) out.push_back({{-gy, zb, -gy, db}, &st.db_girder, "DB girder"});
    }
    for (double sz : st.side_stringer_z) {
        double yi = inner_hull_y(s, p, st, sz), yo = s.half_breadth(sz);
        if (yi <= min_space || yo - yi < 0.2) continue;
        out.push_back({{yi, sz, yo, sz}, &st.stringer, "stringer"});
        out.push_back({{-yi, sz, -yo, sz}, &st.stringer, "stringer"});
    }
    return out;
}

}  // namespace hullcad
