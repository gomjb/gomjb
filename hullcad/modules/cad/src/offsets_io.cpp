#include "hullcad/cad/offsets_io.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace hullcad {

namespace {

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n\"'"), b = s.find_last_not_of(" \t\r\n\"'");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

std::string lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> split(const std::string& line, char sep) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    for (std::string t; std::getline(ss, t, sep);) out.push_back(trim(t));
    return out;
}

}  // namespace

OffsetTable read_offsets_csv(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::invalid_argument("cannot open offset table " + path);
    auto fail = [&](int ln, const std::string& msg) {
        throw std::invalid_argument(path + ":" + std::to_string(ln) + ": " + msg);
    };

    double unit = 1.0;
    char sep = 0;
    int col_station = -1, col_point = -1, col_x = -1, col_y = -1, col_z = -1;
    struct Row { std::string station; double point; double x, y, z; int order; };
    std::vector<Row> rows;
    std::string line;
    int ln = 0;
    while (std::getline(f, line)) {
        ++ln;
        const std::string t = trim(line);
        if (t.empty()) continue;
        if (t[0] == '#') {
            const std::string c = lower(t);
            const size_t u = c.find("units");
            if (u != std::string::npos) {
                const size_t eq = c.find_first_of("=:", u);
                const std::string v = eq == std::string::npos ? std::string() : trim(c.substr(eq + 1));
                if (v == "mm") unit = 1e-3;
                else if (v == "cm") unit = 1e-2;
                else if (v == "m") unit = 1.0;
                else fail(ln, "unknown units '" + v + "' (use m, cm or mm)");
            }
            continue;
        }
        if (!sep) {  // header line: separator and columns
            sep = t.find(';') != std::string::npos ? ';' : t.find('\t') != std::string::npos ? '\t' : ',';
            const auto h = split(t, sep);
            for (int i = 0; i < static_cast<int>(h.size()); ++i) {
                const std::string n = lower(h[i]);
                if (n == "station" || n == "stn" || n == "st") col_station = i;
                else if (n == "point" || n == "pt" || n == "index") col_point = i;
                else if (n == "x") col_x = i;
                else if (n == "y" || n == "half_breadth" || n == "halfbreadth") col_y = i;
                else if (n == "z" || n == "height") col_z = i;
            }
            if (col_x < 0 || col_y < 0 || col_z < 0) fail(ln, "header must name the columns x, y and z");
            continue;
        }
        const auto v = split(t, sep);
        auto num = [&](int col, const char* name) {
            if (col >= static_cast<int>(v.size())) fail(ln, std::string("missing column ") + name);
            errno = 0;
            char* end = nullptr;
            const double d = std::strtod(v[col].c_str(), &end);
            if (v[col].empty() || errno != 0 || *end != '\0' || !std::isfinite(d))
                fail(ln, std::string(name) + " '" + v[col] + "' is not a number");
            return d;
        };
        Row r;
        r.x = num(col_x, "x") * unit;
        r.y = num(col_y, "y") * unit;
        r.z = num(col_z, "z") * unit;
        if (r.y < -1e-9) fail(ln, "negative half-breadth (give the port side, y >= 0)");
        r.station = col_station >= 0 ? (col_station < static_cast<int>(v.size()) ? v[col_station] : std::string()) : std::string();
        r.point = col_point >= 0 ? num(col_point, "point") : 0.0;
        r.order = static_cast<int>(rows.size());
        rows.push_back(r);
    }
    if (!sep) throw std::invalid_argument(path + ": no header line");
    if (rows.empty()) throw std::invalid_argument(path + ": no offsets");

    // Group into stations.
    std::map<std::string, std::vector<Row>> by_name;
    std::vector<std::vector<Row>> groups;
    if (col_station >= 0) {
        for (auto& r : rows) by_name[r.station].push_back(r);
        for (auto& [k, g] : by_name) groups.push_back(g);
    } else {
        std::vector<Row> sorted = rows;
        std::stable_sort(sorted.begin(), sorted.end(), [](const Row& a, const Row& b) { return a.x < b.x; });
        for (auto& r : sorted) {
            if (groups.empty() || std::fabs(groups.back().front().x - r.x) > 1e-6) groups.emplace_back();
            groups.back().push_back(r);
        }
    }

    OffsetTable t;
    t.source = path;
    for (auto& g : groups) {
        if (col_point >= 0)
            std::stable_sort(g.begin(), g.end(), [](const Row& a, const Row& b) { return a.point < b.point; });
        else
            std::stable_sort(g.begin(), g.end(), [](const Row& a, const Row& b) { return a.order < b.order; });
        const double x = g.front().x;
        for (auto& r : g)
            if (std::fabs(r.x - x) > 1e-6)
                throw std::invalid_argument(path + ": station '" + g.front().station + "' mixes x values");
        if (g.size() < 4)
            throw std::invalid_argument(path + ": station at x = " + std::to_string(x) + " has fewer than 4 points");
        std::vector<Vec3> pts;
        for (auto& r : g) pts.push_back({r.x, r.y, r.z});
        if (pts.front().y > 1e-3 && pts.back().y <= 1e-3) std::reverse(pts.begin(), pts.end());  // deck-first
        if (pts.front().y > 1e-3)
            throw std::invalid_argument(path + ": station at x = " + std::to_string(x) +
                                        " must start (or end) on the centreline, y = 0");
        pts.front().y = 0.0;
        t.stations.push_back(std::move(pts));
    }
    std::sort(t.stations.begin(), t.stations.end(), [](const auto& a, const auto& b) { return a.front().x < b.front().x; });
    for (size_t i = 1; i < t.stations.size(); ++i)
        if (std::fabs(t.stations[i].front().x - t.stations[i - 1].front().x) < 1e-6)
            throw std::invalid_argument(path + ": two stations at x = " + std::to_string(t.stations[i].front().x));
    if (t.stations.size() < 4) throw std::invalid_argument(path + ": need at least 4 stations");
    t.x_shift = -t.stations.front().front().x;
    for (auto& s : t.stations)
        for (auto& p : s) p.x += t.x_shift;
    return t;
}

void write_offsets_csv(const OffsetTable& t, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "station,x,point,y,z\n";
    char buf[128];
    for (size_t i = 0; i < t.stations.size(); ++i)
        for (size_t j = 0; j < t.stations[i].size(); ++j) {
            const Vec3& p = t.stations[i][j];
            std::snprintf(buf, sizeof buf, "%zu,%.4f,%zu,%.4f,%.4f\n", i, p.x, j, p.y, p.z);
            f << buf;
        }
}

OffsetTableInfo describe(const OffsetTable& t) {
    OffsetTableInfo r;
    r.stations = static_cast<int>(t.stations.size());
    if (t.stations.empty()) return r;
    r.min_points = r.max_points = static_cast<int>(t.stations.front().size());
    for (auto& s : t.stations) {
        r.min_points = std::min(r.min_points, static_cast<int>(s.size()));
        r.max_points = std::max(r.max_points, static_cast<int>(s.size()));
        for (auto& p : s) {
            r.max_half_breadth = std::max(r.max_half_breadth, p.y);
            r.max_height = std::max(r.max_height, p.z);
        }
    }
    r.length = t.stations.back().front().x - t.stations.front().front().x;
    return r;
}

TankerParams params_for_table(const OffsetTable& t, TankerParams base) {
    const OffsetTableInfo info = describe(t);
    base.Lpp = info.length;
    base.B = 2.0 * info.max_half_breadth;
    double widest = -1;
    for (auto& s : t.stations)
        if (s.back().y > widest) { widest = s.back().y; base.D = s.back().z; }
    base.stations = info.stations;
    return base;
}

OffsetTable parametric_offset_table(const TankerParams& p, int points, double noise, double round_to, unsigned seed) {
    OffsetTable t;
    t.source = "parametric";
    for (double x : station_positions(p)) {
        auto pts = section_points(section_at(p, x), x, p.D, points);
        for (size_t i = 1; i + 1 < pts.size(); ++i) {
            for (double* c : {&pts[i].y, &pts[i].z}) {
                seed = seed * 1664525u + 1013904223u;
                *c += noise * (((seed >> 8) / 16777216.0) * 2.0 - 1.0);
                if (round_to > 0) *c = std::round(*c / round_to) * round_to;
            }
            pts[i].y = std::max(0.0, pts[i].y);
        }
        t.stations.push_back(std::move(pts));
    }
    return t;
}

}  // namespace hullcad
