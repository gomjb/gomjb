#include "hullcad/schedule.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace hullcad {

namespace {

long floor_div(long a, long b) { return a / b - ((a % b != 0) && ((a < 0) != (b < 0))); }

UtilityUse scaled(const UtilityUse& u, double f) {
    UtilityUse r;
    for (auto& it : utility_items()) r.*(it.member) = u.*(it.member) * f;
    return r;
}

// Erection rank of a zone inside its ring: bottom, side (lower), bulkhead, side (upper), deck.
int zone_rank(const std::string& z) {
    auto starts = [&](const char* p) { return z.rfind(p, 0) == 0; };
    if (starts("DB") || starts("LOW")) return 0;
    if (starts("SIDE-U")) return 3;
    if (starts("SIDE")) return 1;
    if (starts("BHD")) return 2;
    if (starts("UP")) return 3;
    if (starts("DECK")) return 4;
    return 5;
}

// Hourly capacity in the rate unit of each utility item (0 = no capacity entered).
double capacity_of(const std::string& key, const YardProfile::Capacity& c) {
    if (key == "electricity_kwh") return c.electricity_kw;
    if (key == "oxygen_nm3") return c.oxygen_nm3_h;
    if (key == "fuel_gas_nm3") return c.fuel_gas_nm3_h;
    if (key == "co2_nm3") return c.co2_nm3_h;
    if (key == "air_nm3") return c.air_nm3_min;
    if (key == "water_m3") return c.water_m3_h;
    return 0;
}

const char* rate_unit_of(const std::string& key) {
    if (key == "electricity_kwh") return "kW";
    if (key == "air_nm3") return "Nm3/min";
    if (key == "water_m3") return "m3/h";
    if (key == "paint_l" || key == "diesel_l") return "L/h";
    return "Nm3/h";
}

}  // namespace

WorkCalendar::WorkCalendar(long start_day, int work_days_per_week) {
    if (work_days_per_week < 1 || work_days_per_week > 7)
        throw std::invalid_argument("work_days_per_week must be 1..7");
    wdpw_ = work_days_per_week;
    while (weekday_mon0(start_day) >= wdpw_) ++start_day;  // start on a working day
    pos0_ = weekday_mon0(start_day);
    monday0_ = start_day - pos0_;
}

long WorkCalendar::day(int wd) const {
    const long g = pos0_ + static_cast<long>(wd);
    const long week = floor_div(g, wdpw_);
    return monday0_ + 7 * week + (g - week * wdpw_);
}

ScheduleResult schedule_utilities(const Division& d, const UtilityReport& u, const YardProfile& y) {
    if (u.blocks.size() != d.blocks.size())
        throw std::invalid_argument("schedule_utilities: utility report does not match the division");
    const auto& sc = y.schedule;
    if (sc.erection_blocks_per_day <= 0) throw std::invalid_argument("erection_blocks_per_day must be > 0");
    ScheduleResult r;
    r.calendar = WorkCalendar(days_from_date(sc.erection_start), sc.work_days_per_week);
    const int n = static_cast<int>(d.blocks.size());
    if (n == 0) return r;

    // Erection order: key ring, then alternately the next ring forward and aft.
    const int rings = static_cast<int>(d.butts.size()) - 1;
    const double Lpp = d.butts.back();
    int key = 1;
    for (int i = 0; i < rings; ++i)
        if (d.butts[i] <= sc.erection_key_x * Lpp && sc.erection_key_x * Lpp < d.butts[i + 1]) key = i + 1;
    std::vector<int> ring_order{key};
    for (int k = 1; static_cast<int>(ring_order.size()) < rings; ++k) {
        if (key + k <= rings) ring_order.push_back(key + k);
        if (key - k >= 1) ring_order.push_back(key - k);
    }
    std::vector<int> order;  // block indices in erection order
    for (int ring : ring_order) {
        std::vector<int> in_ring;
        for (int i = 0; i < n; ++i)
            if (d.blocks[i].ring == ring) in_ring.push_back(i);
        std::stable_sort(in_ring.begin(), in_ring.end(), [&](int a, int b) {
            return zone_rank(d.blocks[a].zone) < zone_rank(d.blocks[b].zone);
        });
        order.insert(order.end(), in_ring.begin(), in_ring.end());
    }

    // Stage spans, planned backwards from each block's erection day.
    r.blocks.resize(n);
    int last_erection = 0;
    for (int k = 0; k < n; ++k) {
        const int i = order[k];
        const Block& b = d.blocks[i];
        BlockSchedule& s = r.blocks[i];
        s.block = b.name;
        s.zone = b.zone;
        s.ring = b.ring;
        s.erection_order = k + 1;
        const int E = static_cast<int>(std::floor(k / sc.erection_blocks_per_day));
        s.erection_wd = E;
        last_erection = std::max(last_erection, E);
        auto& sp = s.span;
        auto S = [](Stage st) { return static_cast<int>(st); };
        sp[S(Stage::Erection)] = {E, E + std::max(1, sc.erection_weld_days)};
        sp[S(Stage::Handling)] = {E, E + 1};
        const int paint_end = E - std::max(0, sc.paint_to_erection_days);
        sp[S(Stage::Painting)] = {paint_end - std::max(1, sc.painting_days), paint_end};
        const int asm_days = std::max(1, static_cast<int>(std::lround(sc.assembly_days + sc.assembly_days_per_100t * b.mass.w / 100.0)));
        const int A1 = sp[S(Stage::Painting)].start;
        sp[S(Stage::Welding)] = {A1 - asm_days, A1};
        const int A0 = sp[S(Stage::Welding)].start;
        const bool curved = b.q.heating_length > 0;
        sp[S(Stage::Heating)] = curved ? StageSpan{A0 - std::max(1, sc.heating_days), A0} : StageSpan{A0, A0};
        const int H0 = sp[S(Stage::Heating)].start;
        sp[S(Stage::Cutting)] = {H0 - std::max(1, sc.cutting_days), H0};
    }
    r.testing.start = last_erection + std::max(0, sc.test_after_last_erection_days);
    r.testing.end = r.testing.start + std::max(1, sc.test_days);

    r.first_wd = r.testing.start;
    r.last_wd = r.testing.end - 1;
    for (auto& s : r.blocks)
        for (int st = 0; st < kStageCount; ++st) {
            if (st == static_cast<int>(Stage::Testing) || s.span[st].days() <= 0) continue;
            r.first_wd = std::min(r.first_wd, s.span[st].start);
            r.last_wd = std::max(r.last_wd, s.span[st].end - 1);
        }
    const int D = r.days();
    r.daily_by_stage.assign(D, {});
    r.active.assign(D, {});
    r.daily_base.assign(D, {});
    r.daily_total.assign(D, {});

    for (int i = 0; i < n; ++i)
        for (int st = 0; st < kStageCount; ++st) {
            const StageSpan sp = r.blocks[i].span[st];
            if (st == static_cast<int>(Stage::Testing) || sp.days() <= 0) continue;
            const UtilityUse per_day = scaled(u.blocks[i].by_stage[st], 1.0 / sp.days());
            for (int wd = sp.start; wd < sp.end; ++wd) {
                r.daily_by_stage[wd - r.first_wd][st] += per_day;
                r.active[wd - r.first_wd][st] += 1;
            }
        }
    {
        const int T = static_cast<int>(Stage::Testing);
        const UtilityUse per_day = scaled(u.by_stage[T], 1.0 / r.testing.days());
        for (int wd = r.testing.start; wd < r.testing.end; ++wd) {
            r.daily_by_stage[wd - r.first_wd][T] += per_day;
            r.active[wd - r.first_wd][T] = 1;
        }
    }
    const auto& cap = y.capacity;
    for (int k = 0; k < D; ++k) {
        r.daily_base[k].electricity_kwh = cap.base_electricity_kw * cap.hours_per_day;
        r.daily_base[k].air_nm3 = cap.base_air_nm3_min * 60.0 * cap.hours_per_day;
        UtilityUse t = r.daily_base[k];
        for (int st = 0; st < kStageCount; ++st) t += r.daily_by_stage[k][st];
        r.daily_total[k] = t;
    }

    // Peaks, hourly demand and capacity check per utility.
    const double hours = std::max(cap.hours_per_day, 1e-9);
    for (auto& it : utility_items()) {
        PeakInfo p;
        p.key = it.key;
        p.label = it.label;
        p.rate_unit = rate_unit_of(it.key);
        const double to_rate = (p.key == "air_nm3" ? 1.0 / 60.0 : 1.0) * cap.peak_factor / hours;
        double sum = 0;
        for (int k = 0; k < D; ++k) {
            const double v = r.daily_total[k].*(it.member);
            sum += v;
            if (v > p.daily_peak) { p.daily_peak = v; p.peak_wd = r.first_wd + k; }
        }
        p.daily_mean = sum / D;
        p.peak_rate = p.daily_peak * to_rate;
        p.capacity = capacity_of(p.key, cap);
        if (p.capacity > 0) {
            p.utilization = p.peak_rate / p.capacity;
            for (int k = 0; k < D; ++k)
                if (r.daily_total[k].*(it.member) * to_rate > p.capacity * (1 + 1e-12)) ++p.days_over;
        }
        // What drives the peak day.
        if (p.daily_peak > 0) {
            const int pk = p.peak_wd;
            std::vector<std::pair<std::string, double>> c;
            for (int i = 0; i < n; ++i)
                for (int st = 0; st < kStageCount; ++st) {
                    const StageSpan sp = r.blocks[i].span[st];
                    if (st == static_cast<int>(Stage::Testing) || pk < sp.start || pk >= sp.end) continue;
                    const double v = u.blocks[i].by_stage[st].*(it.member) / sp.days();
                    if (v > 0) c.emplace_back(r.blocks[i].block + " " + stage_label(static_cast<Stage>(st)), v);
                }
            const double tv = r.daily_by_stage[pk - r.first_wd][static_cast<int>(Stage::Testing)].*(it.member);
            if (tv > 0) c.emplace_back(stage_label(Stage::Testing), tv);
            const double bv = r.daily_base[pk - r.first_wd].*(it.member);
            if (bv > 0) c.emplace_back("기본 부하", bv);
            for (int st = 0; st < kStageCount; ++st)
                p.stage_share[st] = r.daily_by_stage[pk - r.first_wd][st].*(it.member) / p.daily_peak;
            p.stage_share[kStageCount] = bv / p.daily_peak;
            p.dominant_stage = static_cast<int>(std::max_element(p.stage_share.begin(), p.stage_share.end()) - p.stage_share.begin());
            std::sort(c.begin(), c.end(), [](auto& a, auto& b) { return a.second > b.second; });
            if (c.size() > 5) c.resize(5);
            p.contributors = std::move(c);
        }
        r.peaks.push_back(std::move(p));
    }
    return r;
}

void write_block_schedule_csv(const ScheduleResult& s, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "block,ring,zone,erection_order,erection_date";
    for (int st = 0; st < kStageCount; ++st)
        if (st != static_cast<int>(Stage::Testing))
            f << ',' << stage_key(static_cast<Stage>(st)) << "_start," << stage_key(static_cast<Stage>(st)) << "_end";
    f << '\n';
    for (auto& b : s.blocks) {
        f << b.block << ',' << b.ring << ',' << b.zone << ',' << b.erection_order << ',' << s.calendar.date(b.erection_wd);
        for (int st = 0; st < kStageCount; ++st) {
            if (st == static_cast<int>(Stage::Testing)) continue;
            const StageSpan sp = b.span[st];
            if (sp.days() <= 0) f << ",,";
            else f << ',' << s.calendar.date(sp.start) << ',' << s.calendar.date(sp.end - 1);
        }
        f << '\n';
    }
}

void write_daily_utilities_csv(const ScheduleResult& s, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "date,working_day";
    for (auto& it : utility_items()) f << ',' << it.key;
    for (int st = 0; st < kStageCount; ++st) f << ",active_" << stage_key(static_cast<Stage>(st));
    f << '\n';
    char buf[64];
    for (int k = 0; k < s.days(); ++k) {
        f << s.calendar.date(s.first_wd + k) << ',' << s.first_wd + k;
        for (auto& it : utility_items()) {
            std::snprintf(buf, sizeof buf, ",%.2f", s.daily_total[k].*(it.member));
            f << buf;
        }
        for (int st = 0; st < kStageCount; ++st) f << ',' << s.active[k][st];
        f << '\n';
    }
}

}  // namespace hullcad
