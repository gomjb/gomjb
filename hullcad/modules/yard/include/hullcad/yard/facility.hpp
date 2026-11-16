#pragma once
// Shipyard facility model (조선소 설비): the physical limits of every production stage a block
// passes through, and the utility supply allocated to the ship. Pure data, no dependencies.
// All lengths in metres, weights in tonnes. Values in the defaults are examples.
#include <string>
#include <vector>

namespace hullcad {

// Utility supply allocated to the ship (0 = not checked) and time-driven base load.
struct SupplyCapacity {
    double hours_per_day = 16.0;     // working hours over which a day's consumption is drawn
    double peak_factor = 1.3;        // simultaneity: peak hourly demand / average hourly demand
    double electricity_kw = 0;
    double oxygen_nm3_h = 0;
    double fuel_gas_nm3_h = 0;
    double co2_nm3_h = 0;
    double air_nm3_min = 0;
    double water_m3_h = 0;
    double base_electricity_kw = 0;  // lighting, HVAC during working hours
    double base_air_nm3_min = 0;     // compressor idling / network leakage
};

struct FacilityProfile {
    std::string name = "generic large yard (example values)";

    struct Cutting {
        bool enabled = true;
        bool continuous_longitudinals = true;  // no stiffener butts inside a block
        double max_stiffener_length = 25.0;
        double max_plate_length = 21.0;        // stock plate size: plate butts and seams per block
        double max_plate_width = 4.5;
    } cutting;

    struct PanelLine {
        bool enabled = true;
        double max_length = 25.0;  // flat block main panel and bulkhead panels
        double max_width = 22.0;
        double max_weight = 150.0; // per bulkhead panel; 0 = no limit
    } panel_line;

    struct CurvedShop {
        bool enabled = true;
        double max_length = 26.0;  // curved blocks (pin-jig area)
        double max_width = 24.0;
        double max_height = 24.0;
        double crane_capacity = 0; // 0 = not checked here (assembly shop crane applies)
    } curved_shop;

    struct MediumAssembly {
        bool enabled = true;
        double max_length = 22.0;  // web frames / floors built as sub-units
        double max_width = 10.0;
        double crane_capacity = 50.0;
    } medium_assembly;

    struct AssemblyShop {
        bool enabled = true;
        double crane_capacity = 300.0;   // single overhead crane
        double tandem_capacity = 500.0;  // two cranes together; 0 = no tandem lifting
        double max_length = 30.0;
        double max_width = 26.0;
        double max_height = 24.0;
        double door_width = 26.0;        // block leaves the shop lengthwise
        double door_height = 24.0;
    } assembly;

    struct Turnover {
        bool enabled = true;
        double capacity = 400.0;
        std::vector<std::string> zones = {"DB", "DECK"};  // zone name prefixes built upside-down
    } turnover;

    struct Painting {
        bool enabled = true;
        double cell_length = 30.0;
        double cell_width = 26.0;
        double cell_height = 22.0;
        double max_weight = 600.0;
        int cells = 8;  // reported only (throughput belongs to scheduling)
    } painting;

    struct Transporter {
        bool enabled = true;
        double unit_capacity = 300.0;
        int units = 2;                  // units that can work together on one block
        double multi_efficiency = 0.9;  // capacity factor for synchronised multi-unit moves
        double deck_height = 1.9;       // loaded height = block height + deck height
        double route_height = 25.0;     // lowest clearance on the route (gates, bridges)
        double route_width = 30.0;
        double combined_capacity() const {
            return units <= 1 ? unit_capacity : units * unit_capacity * multi_efficiency;
        }
    } transporter;

    struct Erection {
        bool enabled = true;
        double crane_capacity = 900.0;  // dock / goliath crane
        double tandem_capacity = 0.0;
    } erection;

    SupplyCapacity capacity;
};

}  // namespace hullcad
