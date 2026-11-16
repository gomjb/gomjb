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

#include "hullcad/production_design/blocks.hpp"
#include "hullcad/production_design/process.hpp"

namespace hullcad {

// Unit consumption per unit of work. All values are illustrative reference values: replace
// them with the yard's own measured figures before using the results for planning.
struct UtilitySettings {
    struct UtilCutting {
        double plasma_max_thickness = 30.0;  // mm; thicker plates are cut by oxy-fuel
        double plasma_kwh_per_m = 0.35;
        double plasma_air_nm3_per_m = 0.12;
        double oxy_o2_nm3_per_m = 0.20;
        double oxy_fuel_nm3_per_m = 0.03;
    } util_cutting;

    struct UtilWelding {
        double fillet_deposit = 0.16;        // kg weld metal per metre of fillet (one side, ~6 mm leg)
        double shop_butt_deposit = 1.8;      // kg/m, plate butts/seams in the shop (~18 mm plate)
        double erection_butt_deposit = 2.2;  // kg/m, block joints at erection
        bool saw_on_flat_blocks = true;      // shop butts of flat blocks by SAW (panel line)
        double fcaw_kwh_per_kg = 4.0;        // incl. power-source losses and idle
        double fcaw_co2_nm3_per_kg = 0.40;   // shielding gas
        double saw_kwh_per_kg = 2.2;
        double gouging_air_nm3_per_m = 1.5;  // back-gouging / grinding of butt welds
    } util_welding;

    struct UtilHeating {
        double fuel_nm3_per_m = 0.03;   // LNG/LPG for line heating
        double o2_nm3_per_m = 0.08;
        double water_m3_per_m = 0.01;   // quench water
    } util_heating;

    struct UtilPainting {
        int coats = 3;
        double blast_fraction = 1.0;              // share of the coated area blasted at block stage
        double blast_air_nm3_per_m2 = 25.0;
        double spray_air_nm3_per_m2_coat = 1.5;
        double paint_l_per_m2_coat = 0.25;
        double ventilation_kwh_per_m2 = 1.5;      // ventilation, dehumidification, lighting in the cell
        double drying_fuel_nm3_per_m2 = 0.05;
    } util_painting;

    struct UtilHandling {
        int lifts_per_block = 4;          // assembly, turnover, painting, erection
        double lift_height = 25.0;        // m
        double crane_efficiency = 0.5;
        int transport_moves = 3;
        double transport_km_per_move = 1.5;
        double diesel_l_per_tkm = 0.05;
    } util_handling;

    struct UtilTesting {
        double water_test_fraction = 0.5;  // share of tank volume hydro-tested with water
        double air_test_fraction = 0.5;    // share of tank volume air-tested
        double air_test_pressure_bar = 0.2;
        double air_leak_factor = 2.0;      // compressed air used per theoretical fill (leaks, repeats)
    } util_testing;
};

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

UtilityReport estimate_utilities(const Division& d, const TankVolumes& tanks, const UtilitySettings& y);
void write_utilities_csv(const UtilityReport& r, const std::string& path);

}  // namespace hullcad
