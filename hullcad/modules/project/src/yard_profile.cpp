#include "hullcad/project/yard_profile.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace hullcad {

namespace {

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

double parse_number(const std::string& key, const std::string& v) {
    errno = 0;
    char* end = nullptr;
    double d = std::strtod(v.c_str(), &end);
    if (v.empty() || errno != 0 || trim(end).size() != 0)
        throw std::invalid_argument("yard option " + key + ": '" + v + "' is not a number");
    return d;
}

}  // namespace

std::vector<std::pair<std::string, std::string>> yard_sections() {
    return {
        {"yard", "야드 이름"},
        {"weight", "중량 여유: 강재 여유, 선행의장 중량, 리깅 중량"},
        {"division", "분할 형상 규칙: 링 길이, 버트·심 이격, 존 높이"},
        {"cutting", "절단/강재: 론지 연속성(블록 안 보강재 버트 금지), 강판 규격"},
        {"panel_line", "판넬라인(평블록): 주판넬과 격벽 판넬의 크기·중량"},
        {"curved_shop", "곡블록 공장: 곡블록 크기(핀지그 영역)"},
        {"medium_assembly", "중조립: 웹·플로어 서브유닛 크기와 크레인"},
        {"assembly", "대조립 공장: 크레인(단독/텐덤), 정반·베이 크기, 출입문"},
        {"turnover", "반전: 뒤집어 제작하는 블록(DB, 데크 등)의 반전 능력"},
        {"painting", "도장 공장: 셀 크기, 바닥 하중"},
        {"transporter", "트랜스포터: 대당 능력, 동시 투입 대수, 운반 경로 클리어런스"},
        {"erection", "탑재: 도크/골리앗 크레인 능력"},
        {"takeoff", "물량 산출 가정: 웹 개구부, 슬롯, 보강재 도장 면적, 곡가공 선상가열 밀도 (참고값)"},
        {"util_cutting", "유틸리티 원단위 - 절단: 플라즈마 전력·공기, 가스절단 산소·연료 (참고값)"},
        {"util_welding", "유틸리티 원단위 - 용접: 용착량, 전력, CO2 보호가스, 가우징 공기 (참고값)"},
        {"util_heating", "유틸리티 원단위 - 곡가공 선상가열: 연료가스, 산소, 냉각수 (참고값)"},
        {"util_painting", "유틸리티 원단위 - 블라스팅·도장: 도장 횟수, 압축공기, 도료, 환기 전력, 건조 연료 (참고값)"},
        {"util_handling", "유틸리티 원단위 - 크레인 전력, 트랜스포터 경유 (참고값)"},
        {"util_testing", "유틸리티 원단위 - 탱크 수압·공기 시험 (참고값)"},
        {"schedule", "생산 일정: 탑재 시작일·속도·순서, 공정별 기간(작업일) (예시값)"},
        {"capacity", "유틸리티 공급 능력(이 호선 할당분, 0 = 검사 안 함), 시간당 환산, 기본 부하"},
    };
}

std::string section_module(const std::string& section) {
    static const std::vector<std::pair<std::string, std::string>> m = {
        {"yard", "yard"},          {"cutting", "yard"},       {"panel_line", "yard"},  {"curved_shop", "yard"},
        {"medium_assembly", "yard"}, {"assembly", "yard"},    {"turnover", "yard"},    {"painting", "yard"},
        {"transporter", "yard"},   {"erection", "yard"},      {"capacity", "yard"},
        {"weight", "production_design"}, {"division", "production_design"}, {"takeoff", "production_design"},
        {"schedule", "planning"},
    };
    for (auto& [k, v] : m)
        if (k == section) return v;
    if (section.rfind("util_", 0) == 0) return "production";
    return "project";
}

std::vector<YardOption> yard_options(YardProfile& y) {
    using T = YardOption::Type;
    return {
        {"yard.name", T::Text, &y.name, "", "프로파일 이름"},

        {"weight.steel_margin", T::Number, &y.weight.steel_margin, "-", "강재 중량 여유 (용접, 압연 공차)"},
        {"weight.outfit_ratio", T::Number, &y.weight.outfit_ratio, "-", "선행의장 중량 / 강재 중량"},
        {"weight.rigging_weight", T::Number, &y.weight.rigging_weight, "t", "크레인 인양 시 더하는 리깅 중량"},

        {"division.min_ring_length", T::Number, &y.division.min_ring_length, "m", "링 최소 길이"},
        {"division.max_ring_length", T::Number, &y.division.max_ring_length, "m", "링 최대 길이"},
        {"division.min_butt_clearance", T::Number, &y.division.min_butt_clearance, "m", "버트와 횡부재(격벽, 웹) 최소 이격"},
        {"division.seam_offset", T::Number, &y.division.seam_offset, "m", "심과 종부재 이격"},
        {"division.deck_zone_depth", T::Number, &y.division.deck_zone_depth, "m", "데크 블록 높이 (상갑판 아래)"},
        {"division.end_split_z", T::Number, &y.division.end_split_z, "m", "선수미부 상·하 블록 분할 높이"},
        {"division.long_seams_y", T::NumberList, &y.division.long_seams_y, "m", "추가 종방향 심 위치 (좌현 기준, 우현 대칭, 쉼표 구분; 거더와 이격)"},
        {"division.side_split_z", T::Number, &y.division.side_split_z, "m", "화물구역 사이드 블록 상·하 분할 높이 (0 = 분할 안 함, 스트링거와 이격)"},

        {"cutting.enabled", T::Bool, &y.cutting.enabled, "", "절단/강재 검사 사용"},
        {"cutting.continuous_longitudinals", T::Bool, &y.cutting.continuous_longitudinals, "", "블록 안 론지 버트 금지 (블록 길이 <= 보강재 길이)"},
        {"cutting.max_stiffener_length", T::Number, &y.cutting.max_stiffener_length, "m", "보강재(론지) 최대 길이"},
        {"cutting.max_plate_length", T::Number, &y.cutting.max_plate_length, "m", "강판 최대 길이 (판 버트 수 산출용)"},
        {"cutting.max_plate_width", T::Number, &y.cutting.max_plate_width, "m", "강판 최대 폭 (판 심 수 산출용)"},

        {"panel_line.enabled", T::Bool, &y.panel_line.enabled, "", "판넬라인 검사 사용"},
        {"panel_line.max_length", T::Number, &y.panel_line.max_length, "m", "판넬 최대 길이"},
        {"panel_line.max_width", T::Number, &y.panel_line.max_width, "m", "판넬 최대 폭"},
        {"panel_line.max_weight", T::Number, &y.panel_line.max_weight, "t", "격벽 판넬 최대 중량 (0 = 무제한)"},

        {"curved_shop.enabled", T::Bool, &y.curved_shop.enabled, "", "곡블록 공장 검사 사용"},
        {"curved_shop.max_length", T::Number, &y.curved_shop.max_length, "m", "곡블록 최대 길이"},
        {"curved_shop.max_width", T::Number, &y.curved_shop.max_width, "m", "곡블록 최대 폭"},
        {"curved_shop.max_height", T::Number, &y.curved_shop.max_height, "m", "곡블록 최대 높이"},
        {"curved_shop.crane_capacity", T::Number, &y.curved_shop.crane_capacity, "t", "곡블록 공장 크레인 (0 = 검사 안 함)"},

        {"medium_assembly.enabled", T::Bool, &y.medium_assembly.enabled, "", "중조립 검사 사용"},
        {"medium_assembly.max_length", T::Number, &y.medium_assembly.max_length, "m", "웹·플로어 서브유닛 최대 길이"},
        {"medium_assembly.max_width", T::Number, &y.medium_assembly.max_width, "m", "웹·플로어 서브유닛 최대 폭"},
        {"medium_assembly.crane_capacity", T::Number, &y.medium_assembly.crane_capacity, "t", "중조립 크레인"},

        {"assembly.enabled", T::Bool, &y.assembly.enabled, "", "대조립 공장 검사 사용"},
        {"assembly.crane_capacity", T::Number, &y.assembly.crane_capacity, "t", "천장 크레인 단독 능력"},
        {"assembly.tandem_capacity", T::Number, &y.assembly.tandem_capacity, "t", "텐덤(2대) 인양 능력 (0 = 텐덤 불가)"},
        {"assembly.max_length", T::Number, &y.assembly.max_length, "m", "정반/베이 최대 길이"},
        {"assembly.max_width", T::Number, &y.assembly.max_width, "m", "정반/베이 최대 폭"},
        {"assembly.max_height", T::Number, &y.assembly.max_height, "m", "크레인 훅 아래 최대 높이"},
        {"assembly.door_width", T::Number, &y.assembly.door_width, "m", "출입문 폭 (블록은 길이 방향으로 나감)"},
        {"assembly.door_height", T::Number, &y.assembly.door_height, "m", "출입문 높이"},

        {"turnover.enabled", T::Bool, &y.turnover.enabled, "", "반전 검사 사용"},
        {"turnover.capacity", T::Number, &y.turnover.capacity, "t", "반전 능력"},
        {"turnover.zones", T::List, &y.turnover.zones, "", "뒤집어 제작하는 존 이름 접두어 (쉼표 구분)"},

        {"painting.enabled", T::Bool, &y.painting.enabled, "", "도장 공장 검사 사용"},
        {"painting.cell_length", T::Number, &y.painting.cell_length, "m", "도장 셀 길이"},
        {"painting.cell_width", T::Number, &y.painting.cell_width, "m", "도장 셀 폭"},
        {"painting.cell_height", T::Number, &y.painting.cell_height, "m", "도장 셀 높이"},
        {"painting.max_weight", T::Number, &y.painting.max_weight, "t", "도장 셀 최대 블록 중량"},
        {"painting.cells", T::Integer, &y.painting.cells, "ea", "셀 수 (참고값, 일정 계획용)"},

        {"transporter.enabled", T::Bool, &y.transporter.enabled, "", "트랜스포터 검사 사용"},
        {"transporter.unit_capacity", T::Number, &y.transporter.unit_capacity, "t", "대당 적재 능력"},
        {"transporter.units", T::Integer, &y.transporter.units, "ea", "한 블록에 동시 투입 가능한 대수"},
        {"transporter.multi_efficiency", T::Number, &y.transporter.multi_efficiency, "-", "다대 동기 운반 시 능력 계수"},
        {"transporter.deck_height", T::Number, &y.transporter.deck_height, "m", "적재 데크 높이"},
        {"transporter.route_height", T::Number, &y.transporter.route_height, "m", "운반 경로 최소 높이 클리어런스"},
        {"transporter.route_width", T::Number, &y.transporter.route_width, "m", "운반 경로 최소 폭"},

        {"erection.enabled", T::Bool, &y.erection.enabled, "", "탑재 크레인 검사 사용"},
        {"erection.crane_capacity", T::Number, &y.erection.crane_capacity, "t", "도크/골리앗 크레인 능력"},
        {"erection.tandem_capacity", T::Number, &y.erection.tandem_capacity, "t", "텐덤 탑재 능력 (0 = 불가)"},

        {"takeoff.hole_area", T::Number, &y.takeoff.hole_area, "m2", "웹 개구부 1개 면적"},
        {"takeoff.hole_perimeter", T::Number, &y.takeoff.hole_perimeter, "m", "웹 개구부 1개 절단 길이"},
        {"takeoff.slot_length", T::Number, &y.takeoff.slot_length, "m", "론지 관통 슬롯 1개 절단 길이"},
        {"takeoff.stiffener_paint_girth", T::Number, &y.takeoff.stiffener_paint_girth, "m2/m", "보강재 1 m당 도장 면적"},
        {"takeoff.heating_line_density", T::Number, &y.takeoff.heating_line_density, "m/m2", "곡외판 1 m2당 선상가열 길이"},

        {"util_cutting.plasma_max_thickness", T::Number, &y.util_cutting.plasma_max_thickness, "mm", "플라즈마 절단 최대 두께 (초과 시 가스절단)"},
        {"util_cutting.plasma_kwh_per_m", T::Number, &y.util_cutting.plasma_kwh_per_m, "kWh/m", "플라즈마 전력"},
        {"util_cutting.plasma_air_nm3_per_m", T::Number, &y.util_cutting.plasma_air_nm3_per_m, "Nm3/m", "플라즈마 압축공기"},
        {"util_cutting.oxy_o2_nm3_per_m", T::Number, &y.util_cutting.oxy_o2_nm3_per_m, "Nm3/m", "가스절단 산소"},
        {"util_cutting.oxy_fuel_nm3_per_m", T::Number, &y.util_cutting.oxy_fuel_nm3_per_m, "Nm3/m", "가스절단 연료가스 (LNG/LPG)"},

        {"util_welding.fillet_deposit", T::Number, &y.util_welding.fillet_deposit, "kg/m", "필릿 용착량 (한쪽, 각장 약 6 mm)"},
        {"util_welding.shop_butt_deposit", T::Number, &y.util_welding.shop_butt_deposit, "kg/m", "공장 맞대기 용접 용착량"},
        {"util_welding.erection_butt_deposit", T::Number, &y.util_welding.erection_butt_deposit, "kg/m", "탑재 맞대기 용접 용착량"},
        {"util_welding.saw_on_flat_blocks", T::Bool, &y.util_welding.saw_on_flat_blocks, "", "평블록 판 이음은 SAW(판넬라인)로 용접"},
        {"util_welding.fcaw_kwh_per_kg", T::Number, &y.util_welding.fcaw_kwh_per_kg, "kWh/kg", "FCAW 전력 (용착금속 1 kg당)"},
        {"util_welding.fcaw_co2_nm3_per_kg", T::Number, &y.util_welding.fcaw_co2_nm3_per_kg, "Nm3/kg", "FCAW CO2 보호가스"},
        {"util_welding.saw_kwh_per_kg", T::Number, &y.util_welding.saw_kwh_per_kg, "kWh/kg", "SAW 전력"},
        {"util_welding.gouging_air_nm3_per_m", T::Number, &y.util_welding.gouging_air_nm3_per_m, "Nm3/m", "맞대기 용접 가우징·그라인딩 압축공기"},

        {"util_heating.fuel_nm3_per_m", T::Number, &y.util_heating.fuel_nm3_per_m, "Nm3/m", "선상가열 연료가스"},
        {"util_heating.o2_nm3_per_m", T::Number, &y.util_heating.o2_nm3_per_m, "Nm3/m", "선상가열 산소"},
        {"util_heating.water_m3_per_m", T::Number, &y.util_heating.water_m3_per_m, "m3/m", "선상가열 냉각수"},

        {"util_painting.coats", T::Integer, &y.util_painting.coats, "회", "도장 횟수"},
        {"util_painting.blast_fraction", T::Number, &y.util_painting.blast_fraction, "-", "블록 단계 블라스팅 비율 (도장 면적 대비)"},
        {"util_painting.blast_air_nm3_per_m2", T::Number, &y.util_painting.blast_air_nm3_per_m2, "Nm3/m2", "블라스팅 압축공기"},
        {"util_painting.spray_air_nm3_per_m2_coat", T::Number, &y.util_painting.spray_air_nm3_per_m2_coat, "Nm3/m2", "도장 스프레이 압축공기 (1회당)"},
        {"util_painting.paint_l_per_m2_coat", T::Number, &y.util_painting.paint_l_per_m2_coat, "L/m2", "도료 사용량 (1회당)"},
        {"util_painting.ventilation_kwh_per_m2", T::Number, &y.util_painting.ventilation_kwh_per_m2, "kWh/m2", "도장 셀 환기·제습·조명 전력"},
        {"util_painting.drying_fuel_nm3_per_m2", T::Number, &y.util_painting.drying_fuel_nm3_per_m2, "Nm3/m2", "건조 연료가스"},

        {"util_handling.lifts_per_block", T::Integer, &y.util_handling.lifts_per_block, "회", "블록당 크레인 인양 횟수"},
        {"util_handling.lift_height", T::Number, &y.util_handling.lift_height, "m", "평균 인양 높이"},
        {"util_handling.crane_efficiency", T::Number, &y.util_handling.crane_efficiency, "-", "크레인 효율 (전력→위치에너지)"},
        {"util_handling.transport_moves", T::Integer, &y.util_handling.transport_moves, "회", "블록당 트랜스포터 이동 횟수"},
        {"util_handling.transport_km_per_move", T::Number, &y.util_handling.transport_km_per_move, "km", "이동 1회 평균 거리"},
        {"util_handling.diesel_l_per_tkm", T::Number, &y.util_handling.diesel_l_per_tkm, "L/t·km", "트랜스포터 경유"},

        {"util_testing.water_test_fraction", T::Number, &y.util_testing.water_test_fraction, "-", "수압 시험하는 탱크 용적 비율"},
        {"util_testing.air_test_fraction", T::Number, &y.util_testing.air_test_fraction, "-", "공기 시험하는 탱크 용적 비율"},
        {"util_testing.air_test_pressure_bar", T::Number, &y.util_testing.air_test_pressure_bar, "bar", "공기 시험 압력 (게이지)"},
        {"util_testing.air_leak_factor", T::Number, &y.util_testing.air_leak_factor, "-", "공기 시험 손실·재시험 계수"},

        {"schedule.erection_start", T::Date, &y.schedule.erection_start, "", "첫 블록 탑재일 (YYYY-MM-DD)"},
        {"schedule.work_days_per_week", T::Integer, &y.schedule.work_days_per_week, "일", "주당 작업일 (월요일부터)"},
        {"schedule.erection_blocks_per_day", T::Number, &y.schedule.erection_blocks_per_day, "블록/일", "탑재 속도"},
        {"schedule.erection_key_x", T::Number, &y.schedule.erection_key_x, "x/Lpp", "탑재 시작 위치 (키 블록 링)"},
        {"schedule.cutting_days", T::Integer, &y.schedule.cutting_days, "일", "절단 기간"},
        {"schedule.heating_days", T::Integer, &y.schedule.heating_days, "일", "곡가공 기간 (곡블록만)"},
        {"schedule.assembly_days", T::Number, &y.schedule.assembly_days, "일", "조립 기본 기간"},
        {"schedule.assembly_days_per_100t", T::Number, &y.schedule.assembly_days_per_100t, "일", "강재 100 t당 추가 조립 기간"},
        {"schedule.painting_days", T::Integer, &y.schedule.painting_days, "일", "블라스팅·도장 기간"},
        {"schedule.paint_to_erection_days", T::Integer, &y.schedule.paint_to_erection_days, "일", "도장 완료 후 탑재까지 (적치·선행탑재)"},
        {"schedule.erection_weld_days", T::Integer, &y.schedule.erection_weld_days, "일", "탑재 후 용접 기간"},
        {"schedule.test_after_last_erection_days", T::Integer, &y.schedule.test_after_last_erection_days, "일", "마지막 탑재 후 탱크 시험 시작까지"},
        {"schedule.test_days", T::Integer, &y.schedule.test_days, "일", "탱크 시험 기간"},

        {"capacity.hours_per_day", T::Number, &y.capacity.hours_per_day, "h", "하루 작업 시간 (일 사용량을 시간당으로 환산)"},
        {"capacity.peak_factor", T::Number, &y.capacity.peak_factor, "-", "동시 사용 계수 (피크 / 평균 시간당 수요)"},
        {"capacity.electricity_kw", T::Number, &y.capacity.electricity_kw, "kW", "전력 공급 능력"},
        {"capacity.oxygen_nm3_h", T::Number, &y.capacity.oxygen_nm3_h, "Nm3/h", "산소 공급 능력"},
        {"capacity.fuel_gas_nm3_h", T::Number, &y.capacity.fuel_gas_nm3_h, "Nm3/h", "연료가스 공급 능력"},
        {"capacity.co2_nm3_h", T::Number, &y.capacity.co2_nm3_h, "Nm3/h", "CO2 공급 능력"},
        {"capacity.air_nm3_min", T::Number, &y.capacity.air_nm3_min, "Nm3/min", "압축공기 공급 능력 (컴프레서)"},
        {"capacity.water_m3_h", T::Number, &y.capacity.water_m3_h, "m3/h", "용수 공급 능력"},
        {"capacity.base_electricity_kw", T::Number, &y.capacity.base_electricity_kw, "kW", "기본 전력 부하 (조명·공조, 작업 시간 중)"},
        {"capacity.base_air_nm3_min", T::Number, &y.capacity.base_air_nm3_min, "Nm3/min", "기본 공기 부하 (공회전·누설)"},
    };
}

void set_yard_option(YardProfile& y, const std::string& key_in, const std::string& value) {
    // Keys that moved to the module owning them; old profiles keep loading.
    static const std::vector<std::pair<std::string, std::string>> renamed = {
        {"takeoff.fillet_deposit", "util_welding.fillet_deposit"},
        {"takeoff.shop_butt_deposit", "util_welding.shop_butt_deposit"},
        {"takeoff.erection_butt_deposit", "util_welding.erection_butt_deposit"},
        {"takeoff.saw_on_flat_blocks", "util_welding.saw_on_flat_blocks"},
        {"takeoff.coats", "util_painting.coats"},
    };
    std::string key = key_in;
    for (auto& [from, to] : renamed)
        if (key == from) key = to;
    const std::string v = trim(value);
    for (auto& o : yard_options(y)) {
        if (o.key != key) continue;
        switch (o.type) {
            case YardOption::Type::Bool:
                if (v == "true" || v == "1" || v == "yes" || v == "on") *static_cast<bool*>(o.ptr) = true;
                else if (v == "false" || v == "0" || v == "no" || v == "off") *static_cast<bool*>(o.ptr) = false;
                else throw std::invalid_argument("yard option " + key + ": '" + v + "' is not true/false");
                return;
            case YardOption::Type::Number:
                *static_cast<double*>(o.ptr) = parse_number(key, v);
                return;
            case YardOption::Type::Integer: {
                double d = parse_number(key, v);
                if (d != static_cast<int>(d)) throw std::invalid_argument("yard option " + key + ": not an integer");
                *static_cast<int*>(o.ptr) = static_cast<int>(d);
                return;
            }
            case YardOption::Type::Text:
                *static_cast<std::string*>(o.ptr) = v;
                return;
            case YardOption::Type::Date:
                days_from_date(v);  // validates
                *static_cast<std::string*>(o.ptr) = v;
                return;
            case YardOption::Type::List: {
                auto& list = *static_cast<std::vector<std::string>*>(o.ptr);
                list.clear();
                std::stringstream ss(v);
                for (std::string t; std::getline(ss, t, ',');)
                    if (!trim(t).empty()) list.push_back(trim(t));
                return;
            }
            case YardOption::Type::NumberList: {
                auto& list = *static_cast<std::vector<double>*>(o.ptr);
                list.clear();
                std::stringstream ss(v);
                for (std::string t; std::getline(ss, t, ',');)
                    if (!trim(t).empty()) list.push_back(parse_number(key, trim(t)));
                return;
            }
        }
    }
    throw std::invalid_argument("unknown yard option: " + key);
}

std::string get_yard_option(YardProfile& y, const std::string& key) {
    for (auto& o : yard_options(y)) {
        if (o.key != key) continue;
        std::ostringstream s;
        switch (o.type) {
            case YardOption::Type::Bool: s << (*static_cast<bool*>(o.ptr) ? "true" : "false"); break;
            case YardOption::Type::Number: s << *static_cast<double*>(o.ptr); break;
            case YardOption::Type::Integer: s << *static_cast<int*>(o.ptr); break;
            case YardOption::Type::Text:
            case YardOption::Type::Date: s << *static_cast<std::string*>(o.ptr); break;
            case YardOption::Type::List: {
                auto& l = *static_cast<std::vector<std::string>*>(o.ptr);
                for (size_t i = 0; i < l.size(); ++i) s << (i ? "," : "") << l[i];
                break;
            }
            case YardOption::Type::NumberList: {
                auto& l = *static_cast<std::vector<double>*>(o.ptr);
                for (size_t i = 0; i < l.size(); ++i) s << (i ? "," : "") << l[i];
                break;
            }
        }
        return s.str();
    }
    throw std::invalid_argument("unknown yard option: " + key);
}

YardProfile load_yard_profile(const std::string& path, YardProfile base) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open yard profile " + path);
    std::string line, section;
    int ln = 0;
    while (std::getline(f, line)) {
        ++ln;
        size_t c = line.find_first_of(";#");
        if (c != std::string::npos) line = line.substr(0, c);
        line = trim(line);
        if (line.empty()) continue;
        if (line.front() == '[') {
            if (line.back() != ']') throw std::invalid_argument(path + ":" + std::to_string(ln) + ": bad section header");
            section = trim(line.substr(1, line.size() - 2));
            continue;
        }
        size_t eq = line.find('=');
        if (eq == std::string::npos) throw std::invalid_argument(path + ":" + std::to_string(ln) + ": expected key = value");
        std::string key = trim(line.substr(0, eq));
        try {
            set_yard_option(base, section.empty() ? key : section + "." + key, line.substr(eq + 1));
        } catch (const std::invalid_argument& e) {
            throw std::invalid_argument(path + ":" + std::to_string(ln) + ": " + e.what());
        }
    }
    return base;
}

void write_yard_profile(YardProfile y, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "; hullcad shipyard profile. Lengths in m, weights in t. Unlisted keys keep their defaults.\n";
    auto opts = yard_options(y);
    for (auto& [sec, desc] : yard_sections()) {
        f << "\n[" << sec << "]  ; " << desc << "\n";
        for (auto& o : opts) {
            if (o.key.compare(0, sec.size() + 1, sec + ".") != 0) continue;
            std::string name = o.key.substr(sec.size() + 1);
            std::string val = get_yard_option(y, o.key);
            std::string lhs = name + " = " + val;
            if (lhs.size() < 34) lhs.append(34 - lhs.size(), ' ');
            f << lhs << " ; " << o.help << (o.unit.empty() || o.unit == "-" ? "" : " [" + o.unit + "]") << "\n";
        }
    }
}

}  // namespace hullcad
