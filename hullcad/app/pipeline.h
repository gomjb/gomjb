#pragma once
// Computation pipeline used by the desktop app. Plain C++ (no Qt), so it runs on worker threads
// and stays testable on its own.
#include <memory>
#include <string>
#include <vector>

#include "hullcad/production_design/blocks.hpp"
#include "hullcad/cad/hull_param.hpp"
#include "hullcad/naval/hydrostatics.hpp"
#include "hullcad/cad/mesh.hpp"
#include "hullcad/planning/schedule.hpp"
#include "hullcad/structure/structure.hpp"
#include "hullcad/production/daily_load.hpp"
#include "hullcad/production/utilities.hpp"
#include "hullcad/project/yard_profile.hpp"

namespace hullcad::app {

struct HullRun {
    TankerParams params;
    TriMesh mesh;
    HydroResult design;               // at design draft
    std::vector<HydroResult> table;   // upright hydrostatic table
    double KG = 0;
    std::vector<GZPoint> gz;
    std::string lines_svg;
    TankVolumes tanks;                // cargo / ballast tank volumes (tank-test utilities)
    double seconds = 0;
    std::string error;                // non-empty: the run failed
};

struct DivisionRun {
    Division division;
    std::vector<CheckSummary> summary;
    std::string svg;
    Mass steel_total;
    UtilityReport utilities;          // work-driven utility estimate of the blocks
    ProductionSchedule schedule;      // planning: block stage schedule
    DailyUtilityProfile daily;        // production: daily utility load and peaks
    std::string schedule_error;       // e.g. malformed schedule options
    YardProfile yard;                 // profile the run used
    bool manual = false;
    double seconds = 0;
    std::string error;
};

// Hull surface, mesh, hydrostatic table, GZ curve (fixed trim) and lines plan.
HullRun run_hull(const TankerParams& p, double KG, int threads);
// GZ curve only, on an existing hull.
std::vector<GZPoint> run_gz(const HullRun& hull, double KG, int threads);
// Structure, weight model and block division (automatic if manual_frames is empty).
DivisionRun run_division(const HullRun& hull, const YardProfile& yard, const std::vector<double>& manual_frames,
                         int threads);

}  // namespace hullcad::app
