#include "hullcad/production/daily_load.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace hullcad {

namespace {

UtilityUse scaled(const UtilityUse& u, double f) {
    UtilityUse r;
    for (auto& it : utility_items()) r.*(it.member) = u.*(it.member) * f;
    return r;
}

// Hourly capacity in the rate unit of each utility item (0 = no capacity entered).
double capacity_of(const std::string& key, const SupplyCapacity& c) {
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

DailyUtilityProfile daily_utilities(const ProductionSchedule& s, const UtilityReport& u, const SupplyCapacity& cap) {
    if (u.blocks.size() != s.blocks.size())
        throw std::invalid_argument("daily_utilities: utility report does not match the schedule");
    DailyUtilityProfile r;
    r.calendar = s.calendar;
    const int n = static_cast<int>(s.blocks.size());
    if (n == 0) return r;
    r.first_wd = s.first_wd();
    r.last_wd = s.last_wd();
    const int D = r.days();
    r.by_stage.assign(D, {});
    r.active.assign(D, {});
    r.base.assign(D, {});
    r.total.assign(D, {});

    for (int i = 0; i < n; ++i)
        for (int st = 0; st < kStageCount; ++st) {
            const StageSpan sp = s.blocks[i].span[st];
            if (st == static_cast<int>(Stage::Testing) || sp.days() <= 0) continue;
            const UtilityUse per_day = scaled(u.blocks[i].by_stage[st], 1.0 / sp.days());
            for (int wd = sp.start; wd < sp.end; ++wd) {
                r.by_stage[wd - r.first_wd][st] += per_day;
                r.active[wd - r.first_wd][st] += 1;
            }
        }
    {
        const int T = static_cast<int>(Stage::Testing);
        const UtilityUse per_day = scaled(u.by_stage[T], 1.0 / s.testing.days());
        for (int wd = s.testing.start; wd < s.testing.end; ++wd) {
            r.by_stage[wd - r.first_wd][T] += per_day;
            r.active[wd - r.first_wd][T] = 1;
        }
    }
    for (int k = 0; k < D; ++k) {
        r.base[k].electricity_kwh = cap.base_electricity_kw * cap.hours_per_day;
        r.base[k].air_nm3 = cap.base_air_nm3_min * 60.0 * cap.hours_per_day;
        UtilityUse t = r.base[k];
        for (int st = 0; st < kStageCount; ++st) t += r.by_stage[k][st];
        r.total[k] = t;
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
            const double v = r.total[k].*(it.member);
            sum += v;
            if (v > p.daily_peak) { p.daily_peak = v; p.peak_wd = r.first_wd + k; }
        }
        p.daily_mean = sum / D;
        p.peak_rate = p.daily_peak * to_rate;
        p.capacity = capacity_of(p.key, cap);
        if (p.capacity > 0) {
            p.utilization = p.peak_rate / p.capacity;
            for (int k = 0; k < D; ++k)
                if (r.total[k].*(it.member) * to_rate > p.capacity * (1 + 1e-12)) ++p.days_over;
        }
        // What drives the peak day.
        if (p.daily_peak > 0) {
            const int pk = p.peak_wd;
            std::vector<std::pair<std::string, double>> c;
            for (int i = 0; i < n; ++i)
                for (int st = 0; st < kStageCount; ++st) {
                    const StageSpan sp = s.blocks[i].span[st];
                    if (st == static_cast<int>(Stage::Testing) || pk < sp.start || pk >= sp.end) continue;
                    const double v = u.blocks[i].by_stage[st].*(it.member) / sp.days();
                    if (v > 0) c.emplace_back(s.blocks[i].block + " " + stage_label(static_cast<Stage>(st)), v);
                }
            const double tv = r.by_stage[pk - r.first_wd][static_cast<int>(Stage::Testing)].*(it.member);
            if (tv > 0) c.emplace_back(stage_label(Stage::Testing), tv);
            const double bv = r.base[pk - r.first_wd].*(it.member);
            if (bv > 0) c.emplace_back("기본 부하", bv);
            for (int st = 0; st < kStageCount; ++st)
                p.stage_share[st] = r.by_stage[pk - r.first_wd][st].*(it.member) / p.daily_peak;
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

void write_daily_utilities_csv(const DailyUtilityProfile& p, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "date,working_day";
    for (auto& it : utility_items()) f << ',' << it.key;
    for (int st = 0; st < kStageCount; ++st) f << ",active_" << stage_key(static_cast<Stage>(st));
    f << '\n';
    char buf[64];
    for (int k = 0; k < p.days(); ++k) {
        f << p.calendar.date(p.first_wd + k) << ',' << p.first_wd + k;
        for (auto& it : utility_items()) {
            std::snprintf(buf, sizeof buf, ",%.2f", p.total[k].*(it.member));
            f << buf;
        }
        for (int st = 0; st < kStageCount; ++st) f << ',' << p.active[k][st];
        f << '\n';
    }
}

}  // namespace hullcad
