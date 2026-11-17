#pragma once
// Mid-term production schedule (중일정): when every block is cut, formed, assembled, painted,
// erected and welded, and when the tanks are tested.
//
// Blocks are erected one after another (key ring first, then alternately forward and aft;
// bottom -> side -> bulkhead -> deck within a ring). Every block's stages are planned backwards
// from its erection day on a working-day calendar.
#include <array>
#include <string>
#include <vector>

#include "hullcad/core/calendar.hpp"
#include "hullcad/production_design/blocks.hpp"
#include "hullcad/production_design/process.hpp"

namespace hullcad {

struct PlanningSettings {
    struct Schedule {
        std::string erection_start = "2027-03-02";  // first block erected (keel laying), YYYY-MM-DD
        int work_days_per_week = 6;                  // Mon..Sat
        double erection_blocks_per_day = 1.5;
        double erection_key_x = 0.35;   // erection starts at the ring containing this x / Lpp
        int cutting_days = 4;
        int heating_days = 5;           // curved blocks only
        double assembly_days = 8.0;
        double assembly_days_per_100t = 4.0;
        int painting_days = 6;
        int paint_to_erection_days = 5; // storage / pre-erection buffer
        int erection_weld_days = 6;
        int test_after_last_erection_days = 20;
        int test_days = 30;
    } schedule;
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

struct ProductionSchedule {
    WorkCalendar calendar;
    std::vector<BlockSchedule> blocks;  // same order as Division::blocks
    StageSpan testing;                  // ship-level tank tests
    int last_erection_wd = 0;
    // Working days spanned by all stages and the tank tests (inclusive).
    int first_wd() const;
    int last_wd() const;
};

ProductionSchedule schedule_blocks(const Division& d, const PlanningSettings& s);
void write_block_schedule_csv(const ProductionSchedule& s, const std::string& path);

}  // namespace hullcad
