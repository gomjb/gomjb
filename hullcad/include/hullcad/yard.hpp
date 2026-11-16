#pragma once
// Shipyard facility profile: the production limits that block division must respect.
//
// Every stage of the block's production route (cutting -> panel line or curved-block shop ->
// medium assembly of webs -> block assembly shop -> turnover -> painting -> transporter ->
// erection crane) can be enabled or disabled and has its own limits. Profiles are plain INI
// files, so the same division engine serves different yards:
//
//   [transporter]
//   enabled = true
//   unit_capacity = 300
//   units = 2
//
// All lengths are in metres, weights in tonnes.
#include <string>
#include <vector>

namespace hullcad {

struct YardProfile {
    std::string name = "generic large yard (example values)";

    struct Weight {
        double steel_margin = 0.03;    // welding, mill tolerance
        double outfit_ratio = 0.05;    // pre-outfitting weight as a fraction of steel
        double rigging_weight = 8.0;   // lifting beams and slings, added to every crane lift
    } weight;

    struct Division {
        double min_ring_length = 6.0;
        double max_ring_length = 30.0;
        double min_butt_clearance = 0.3;  // butt distance from transverse bulkheads and webs
        double seam_offset = 0.3;         // seam distance from longitudinal members
        double deck_zone_depth = 3.0;     // depth of the deck blocks below the upper deck
        double end_split_z = 10.0;        // aft/fore bodies: lower/upper block split height
        double side_split_z = 0.0;        // cargo side blocks: lower/upper split height, 0 = none
        std::vector<double> long_seams_y; // extra longitudinal seams (port offsets, mirrored) that
                                          // split DB/BHD/DECK and end-body blocks across the breadth
    } division;

    struct Cutting {
        bool enabled = true;
        bool continuous_longitudinals = true;  // no stiffener butts inside a block
        double max_stiffener_length = 25.0;
        double max_plate_length = 21.0;        // reported: plate butts per block
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

    // ---- Utility estimation. All rates are illustrative reference values: replace them with the
    // yard's own measured unit consumption before using the results for planning.
    struct Takeoff {
        double fillet_deposit = 0.16;        // kg weld metal per metre of fillet (one side, ~6 mm leg)
        double shop_butt_deposit = 1.8;      // kg/m, plate butts/seams in the shop (~18 mm plate)
        double erection_butt_deposit = 2.2;  // kg/m, block joints at erection
        double hole_area = 0.5;              // m^2 per web opening (manhole / lightening hole)
        double hole_perimeter = 2.7;         // m of cutting per opening
        double slot_length = 0.9;            // m of cutting per longitudinal slot in a web
        double stiffener_paint_girth = 0.7;  // m^2 of coated surface per metre of stiffener
        double heating_line_density = 1.5;   // m of line heating per m^2 of curved shell
        int coats = 3;
        bool saw_on_flat_blocks = true;      // shop butts of flat blocks by SAW (panel line)
    } takeoff;

    struct UtilCutting {
        double plasma_max_thickness = 30.0;  // mm; thicker plates are cut by oxy-fuel
        double plasma_kwh_per_m = 0.35;
        double plasma_air_nm3_per_m = 0.12;
        double oxy_o2_nm3_per_m = 0.20;
        double oxy_fuel_nm3_per_m = 0.03;
    } util_cutting;

    struct UtilWelding {
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

    // ---- Production schedule (working days) used to spread utilities over the calendar.
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

    // ---- Utility supply capacity allocated to this ship (0 = not checked) and base load.
    struct Capacity {
        double hours_per_day = 16.0;     // working hours over which a day's consumption is drawn
        double peak_factor = 1.3;        // simultaneity: peak hourly demand / average hourly demand
        double electricity_kw = 0;
        double oxygen_nm3_h = 0;
        double fuel_gas_nm3_h = 0;
        double co2_nm3_h = 0;
        double air_nm3_min = 0;
        double water_m3_h = 0;
        double base_electricity_kw = 0;  // time-driven base load (lighting, HVAC) during working hours
        double base_air_nm3_min = 0;     // compressor idling / network leakage
    } capacity;
};

// One INI key bound to a profile field, with its unit and help text (used for parsing, `--set`
// overrides, and writing an annotated template).
struct YardOption {
    enum class Type { Bool, Number, Integer, Text, List, NumberList, Date };
    std::string key;  // "section.name"
    Type type;
    void* ptr;
    std::string unit;
    std::string help;
};

std::vector<YardOption> yard_options(YardProfile& y);
std::vector<std::pair<std::string, std::string>> yard_sections();  // section -> description

// key = "section.name"; throws std::invalid_argument for unknown keys or malformed values.
void set_yard_option(YardProfile& y, const std::string& key, const std::string& value);
std::string get_yard_option(YardProfile& y, const std::string& key);

// Loads an INI file on top of `base` (unspecified keys keep their base values).
// Calendar helpers: days since 1970-01-01 <-> "YYYY-MM-DD" (proleptic Gregorian).
long days_from_date(const std::string& ymd);  // throws std::invalid_argument on malformed dates
std::string date_from_days(long days);
int weekday_mon0(long days);                  // Monday = 0 ... Sunday = 6

YardProfile load_yard_profile(const std::string& path, YardProfile base = {});
void write_yard_profile(YardProfile y, const std::string& path);  // annotated, all keys

}  // namespace hullcad
