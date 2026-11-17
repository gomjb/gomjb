#pragma once
// Daily utility load (생산관리): every block's stage consumption spread over the stage's working
// days in the production schedule, base load added per day, and the daily totals converted to
// hourly demand (working hours x simultaneity factor) and checked against the supply capacity.
#include <array>
#include <string>
#include <utility>
#include <vector>

#include "hullcad/planning/schedule.hpp"
#include "hullcad/production/utilities.hpp"
#include "hullcad/yard/facility.hpp"

namespace hullcad {

struct PeakInfo {
    std::string key, label, rate_unit;  // e.g. "electricity_kwh", "전력", "kW"
    double daily_peak = 0;   // consumption on the peak day
    int peak_wd = 0;
    double daily_mean = 0;   // mean over the working days of the profile
    double peak_rate = 0;    // daily_peak / working hours x peak factor, in rate_unit
    double capacity = 0;     // in rate_unit, 0 = not checked
    double utilization = 0;  // peak_rate / capacity
    int days_over = 0;       // days whose peak rate exceeds the capacity
    std::vector<std::pair<std::string, double>> contributors;  // largest block-stage shares of the peak day
    std::array<double, kStageCount + 1> stage_share{};         // peak-day share by stage, last = base load
    int dominant_stage = -1;                                   // index into stage_share
};

struct DailyUtilityProfile {
    WorkCalendar calendar;
    int first_wd = 0, last_wd = 0;  // inclusive range covered by the profile
    std::vector<std::array<UtilityUse, kStageCount>> by_stage;  // index wd - first_wd
    std::vector<UtilityUse> base;
    std::vector<UtilityUse> total;
    std::vector<std::array<int, kStageCount>> active;  // blocks in each stage per day
    std::vector<PeakInfo> peaks;                        // one per utility item
    int days() const { return last_wd - first_wd + 1; }
};

// The UtilityReport and the schedule must come from the same Division (blocks in the same order).
DailyUtilityProfile daily_utilities(const ProductionSchedule& s, const UtilityReport& u, const SupplyCapacity& cap);
void write_daily_utilities_csv(const DailyUtilityProfile& p, const std::string& path);

}  // namespace hullcad
