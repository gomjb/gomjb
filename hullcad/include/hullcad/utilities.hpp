#pragma once
// Production utility estimate: work quantities from the block takeoff (cutting length, weld
// metal, line heating, coated area, lifts, tank volumes) times the yard's unit consumption.
//
//   consumption = sum over stages of (work quantity x unit consumption)
//
// Only work-driven consumption is estimated. Time-driven base load (lighting, HVAC, idling
// compressors) needs a production schedule and is not included.
#include <array>
#include <string>
#include <vector>

#include "hullcad/blocks.hpp"

namespace hullcad {

struct UtilityUse {
    double electricity_kwh = 0;
    double oxygen_nm3 = 0;
    double fuel_gas_nm3 = 0;   // LNG / LPG
    double co2_nm3 = 0;        // welding shielding gas
    double air_nm3 = 0;        // compressed air (free air delivered)
    double water_m3 = 0;
    double paint_l = 0;
    double diesel_l = 0;
    UtilityUse& operator+=(const UtilityUse& o);
};

// Display metadata, in report order.
struct UtilityItem {
    const char* key;
    const char* label;  // Korean label
    const char* unit;
    double UtilityUse::*member;
};
const std::vector<UtilityItem>& utility_items();

enum class Stage { Cutting, Welding, Heating, Painting, Handling, Erection, Testing, Count };
constexpr int kStageCount = static_cast<int>(Stage::Count);
const char* stage_key(Stage s);    // "cutting", ...
const char* stage_label(Stage s);  // Korean label

struct TankVolumes {
    double cargo_m3 = 0;    // inside the inner hull (cargo and slop tanks)
    double ballast_m3 = 0;  // double bottom and double side spaces of the cargo region
};
TankVolumes tank_volumes(const TriMesh& hull, const TankerParams& p, const TankerStructure& st, double dx = 0.5);

struct BlockUtility {
    std::string block;
    double steel_t = 0;
    double weld_metal_kg = 0;
    std::array<UtilityUse, kStageCount> by_stage{};
    UtilityUse total;
};

struct UtilityReport {
    std::vector<BlockUtility> blocks;
    std::array<UtilityUse, kStageCount> by_stage{};  // whole ship
    UtilityUse total;
    TankVolumes tanks;
    QuantityTakeoff quantities;  // summed over all blocks
    double steel_t = 0;
    double weld_metal_kg = 0, fcaw_kg = 0, saw_kg = 0;
    int coats = 0;  // coats applied over paint_area
};

UtilityReport estimate_utilities(const Division& d, const TankVolumes& tanks, const YardProfile& y);
void write_utilities_csv(const UtilityReport& r, const std::string& path);

}  // namespace hullcad
