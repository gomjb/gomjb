#pragma once
// Project settings (프로젝트): one yard profile that combines the settings of every module and
// reads/writes them as an INI file.
//
//   FacilityProfile           (yard)              facility limits, utility supply capacity
//   ProductionDesignSettings  (production_design) + weight margins, division rules, takeoff
//   PlanningSettings          (planning)          production schedule
//   UtilitySettings           (production)        unit utility consumption
//
// Each module only sees its own settings type; YardProfile is passed wherever one of them is
// expected. INI sections map to the nested structs:
//
//   [transporter]
//   unit_capacity = 300
//   units = 2
#include <string>
#include <utility>
#include <vector>

#include "hullcad/core/calendar.hpp"
#include "hullcad/planning/schedule.hpp"
#include "hullcad/production/utilities.hpp"
#include "hullcad/production_design/design_settings.hpp"

namespace hullcad {

struct YardProfile : ProductionDesignSettings, PlanningSettings, UtilitySettings {};

// One INI key bound to a profile field, with its unit and help text (used for parsing, `--set`
// overrides, the app's input form, and writing an annotated template).
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
// Module that owns each section (for documentation and the app's grouping).
std::string section_module(const std::string& section);

// key = "section.name"; throws std::invalid_argument for unknown keys or malformed values.
void set_yard_option(YardProfile& y, const std::string& key, const std::string& value);
std::string get_yard_option(YardProfile& y, const std::string& key);

// Loads an INI file on top of `base` (unspecified keys keep their base values).
YardProfile load_yard_profile(const std::string& path, YardProfile base = {});
void write_yard_profile(YardProfile y, const std::string& path);  // annotated, all keys

}  // namespace hullcad
