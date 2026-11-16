#pragma once
// Production schedule and daily utility profile.
//
// Blocks are erected one after another (key ring first, then alternately forward and aft;
// bottom -> side -> bulkhead -> deck within a ring). Every block's production stages are planned
// backwards from its erection day, and each stage's utility consumption is spread evenly over
// its working days. Daily totals are converted to hourly demand (working hours x simultaneity
// factor) and checked against the supply capacity allocated to the ship.
#include <array>
#include <string>
#include <vector>

#include "hullcad/utilities.hpp"

namespace hullcad {

// Working-day calendar: working day 0 is the erection start (moved to the next working day if
// needed); negative indices are earlier working days. Days are counted since 1970-01-01.
class WorkCalendar {
public:
    WorkCalendar() = default;
    WorkCalendar(long start_day, int work_days_per_week);
    long day(int wd) const;  // calendar day of working day wd
    std::string date(int wd) const { return date_from_days(day(wd)); }
    int work_days_per_week() const { return wdpw_; }

private:
    long monday0_ = 0;  // Monday of the week containing working day 0
    int pos0_ = 0;      // weekday of working day 0
    int wdpw_ = 6;
};

struct StageSpan {
    int start = 0, end = 0;  // working days [start, end)
    int days() const { return end - start; }
};

struct BlockSchedule {
    std::string block, zone;
    int ring = 0;
    int erection_order = 0;
    int erection_wd = 0;
    std::array<StageSpan, kStageCount> span{};  // Stage::Testing is ship-level, left empty
};

struct PeakInfo {
    std::string key, label, rate_unit;  // e.g. "electricity_kwh", "전력", "kW"
    double daily_peak = 0;   // consumption on the peak day
    int peak_wd = 0;
    double daily_mean = 0;   // mean over the working days of the schedule
    double peak_rate = 0;    // daily_peak / working hours x peak factor, in rate_unit
    double capacity = 0;     // in rate_unit, 0 = not checked
    double utilization = 0;  // peak_rate / capacity
    int days_over = 0;       // days whose peak rate exceeds the capacity
    std::vector<std::pair<std::string, double>> contributors;  // largest block-stage shares of the peak day
    std::array<double, kStageCount + 1> stage_share{};         // peak-day share by stage, last = base load
    int dominant_stage = -1;                                   // index into stage_share
};

struct ScheduleResult {
    WorkCalendar calendar;
    std::vector<BlockSchedule> blocks;  // same order as Division::blocks
    StageSpan testing;
    int first_wd = 0, last_wd = 0;      // inclusive range covered by the profile
    std::vector<std::array<UtilityUse, kStageCount>> daily_by_stage;  // index wd - first_wd
    std::vector<UtilityUse> daily_base;
    std::vector<UtilityUse> daily_total;
    std::vector<std::array<int, kStageCount>> active;  // blocks in each stage per day
    std::vector<PeakInfo> peaks;        // one per utility item
    int days() const { return last_wd - first_wd + 1; }
};

// The UtilityReport must come from the same Division (blocks in the same order).
ScheduleResult schedule_utilities(const Division& d, const UtilityReport& u, const YardProfile& y);

void write_block_schedule_csv(const ScheduleResult& s, const std::string& path);
void write_daily_utilities_csv(const ScheduleResult& s, const std::string& path);

}  // namespace hullcad
