#include "hullcad/utilities.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace hullcad {

UtilityUse& UtilityUse::operator+=(const UtilityUse& o) {
    electricity_kwh += o.electricity_kwh;
    oxygen_nm3 += o.oxygen_nm3;
    fuel_gas_nm3 += o.fuel_gas_nm3;
    co2_nm3 += o.co2_nm3;
    air_nm3 += o.air_nm3;
    water_m3 += o.water_m3;
    paint_l += o.paint_l;
    diesel_l += o.diesel_l;
    return *this;
}

const std::vector<UtilityItem>& utility_items() {
    static const std::vector<UtilityItem> items = {
        {"electricity_kwh", "전력", "kWh", &UtilityUse::electricity_kwh},
        {"oxygen_nm3", "산소", "Nm3", &UtilityUse::oxygen_nm3},
        {"fuel_gas_nm3", "연료가스", "Nm3", &UtilityUse::fuel_gas_nm3},
        {"co2_nm3", "CO2 (용접)", "Nm3", &UtilityUse::co2_nm3},
        {"air_nm3", "압축공기", "Nm3", &UtilityUse::air_nm3},
        {"water_m3", "용수", "m3", &UtilityUse::water_m3},
        {"paint_l", "도료", "L", &UtilityUse::paint_l},
        {"diesel_l", "경유", "L", &UtilityUse::diesel_l},
    };
    return items;
}

const char* stage_key(Stage s) {
    static const char* k[] = {"cutting", "welding", "heating", "painting", "handling", "erection", "testing"};
    return k[static_cast<int>(s)];
}

const char* stage_label(Stage s) {
    static const char* k[] = {"절단", "공장 용접", "곡가공 (선상가열)", "블라스팅·도장", "크레인·운반", "탑재 용접", "탱크 시험"};
    return k[static_cast<int>(s)];
}

TankVolumes tank_volumes(const TriMesh& hull, const TankerParams& p, const TankerStructure& st, double dx) {
    TankVolumes v;
    const double x0 = st.cargo_x0(), x1 = st.cargo_x1();
    const int nx = std::max(1, static_cast<int>(std::ceil((x1 - x0) / dx)));
    const double hx = (x1 - x0) / nx;
    const int nz = static_cast<int>(std::ceil(p.D / 0.05));
    const double hz = p.D / nz;
    for (int i = 0; i < nx; ++i) {
        const double x = x0 + (i + 0.5) * hx;
        HullSection hs = hull_section(hull, x);
        double total = 0, inner = 0;
        for (int k = 0; k < nz; ++k) {
            const double z = (k + 0.5) * hz;
            total += 2.0 * hs.half_breadth(z) * hz;
            if (z > st.double_bottom) {
                const double yi = inner_hull_y(hs, p, st, z);
                if (yi > 0.5) inner += 2.0 * yi * hz;
            }
        }
        v.cargo_m3 += inner * hx;
        v.ballast_m3 += (total - inner) * hx;
    }
    return v;
}

UtilityReport estimate_utilities(const Division& d, const TankVolumes& tanks, const YardProfile& y) {
    UtilityReport r;
    r.tanks = tanks;
    r.coats = y.takeoff.coats;
    const auto& to = y.takeoff;
    const auto& uc = y.util_cutting;
    const auto& uw = y.util_welding;
    const auto& uh = y.util_heating;
    const auto& up = y.util_painting;
    const auto& ul = y.util_handling;
    auto S = [](Stage s) { return static_cast<int>(s); };

    for (const Block& b : d.blocks) {
        const QuantityTakeoff& q = b.q;
        BlockUtility u;
        u.block = b.name;
        u.steel_t = b.mass.w;

        UtilityUse& cut = u.by_stage[S(Stage::Cutting)];
        if (q.mean_thickness <= uc.plasma_max_thickness) {
            cut.electricity_kwh = q.cut_length * uc.plasma_kwh_per_m;
            cut.air_nm3 = q.cut_length * uc.plasma_air_nm3_per_m;
        } else {
            cut.oxygen_nm3 = q.cut_length * uc.oxy_o2_nm3_per_m;
            cut.fuel_gas_nm3 = q.cut_length * uc.oxy_fuel_nm3_per_m;
        }

        // Shop welding: fillets by FCAW; plate butts/seams by SAW on flat blocks (panel line).
        const double fillet_kg = q.fillet_length * to.fillet_deposit;
        const double butt_kg = q.shop_butt_length * to.shop_butt_deposit;
        const bool saw = b.flat && to.saw_on_flat_blocks;
        const double fcaw_kg = fillet_kg + (saw ? 0.0 : butt_kg);
        const double saw_kg = saw ? butt_kg : 0.0;
        UtilityUse& weld = u.by_stage[S(Stage::Welding)];
        weld.electricity_kwh = fcaw_kg * uw.fcaw_kwh_per_kg + saw_kg * uw.saw_kwh_per_kg;
        weld.co2_nm3 = fcaw_kg * uw.fcaw_co2_nm3_per_kg;
        weld.air_nm3 = q.shop_butt_length * uw.gouging_air_nm3_per_m;

        UtilityUse& heat = u.by_stage[S(Stage::Heating)];
        heat.fuel_gas_nm3 = q.heating_length * uh.fuel_nm3_per_m;
        heat.oxygen_nm3 = q.heating_length * uh.o2_nm3_per_m;
        heat.water_m3 = q.heating_length * uh.water_m3_per_m;

        UtilityUse& paint = u.by_stage[S(Stage::Painting)];
        const double coat_area = q.paint_area * to.coats;
        paint.air_nm3 = q.paint_area * up.blast_fraction * up.blast_air_nm3_per_m2 + coat_area * up.spray_air_nm3_per_m2_coat;
        paint.paint_l = coat_area * up.paint_l_per_m2_coat;
        paint.electricity_kwh = q.paint_area * up.ventilation_kwh_per_m2;
        paint.fuel_gas_nm3 = q.paint_area * up.drying_fuel_nm3_per_m2;

        // Lifts: potential energy m g h per lift over the crane efficiency; transport by diesel.
        UtilityUse& hand = u.by_stage[S(Stage::Handling)];
        hand.electricity_kwh = ul.lifts_per_block * b.lift_weight * 1000.0 * 9.81 * ul.lift_height / 3.6e6 /
                               std::max(ul.crane_efficiency, 1e-6);
        hand.diesel_l = b.transport_weight * ul.transport_moves * ul.transport_km_per_move * ul.diesel_l_per_tkm;

        const double erect_kg = q.erection_length * to.erection_butt_deposit;
        UtilityUse& erect = u.by_stage[S(Stage::Erection)];
        erect.electricity_kwh = erect_kg * uw.fcaw_kwh_per_kg;
        erect.co2_nm3 = erect_kg * uw.fcaw_co2_nm3_per_kg;
        erect.air_nm3 = q.erection_length * uw.gouging_air_nm3_per_m;

        u.weld_metal_kg = fcaw_kg + saw_kg + erect_kg;
        for (int s = 0; s < kStageCount; ++s) {
            u.total += u.by_stage[s];
            r.by_stage[s] += u.by_stage[s];
        }
        r.fcaw_kg += fcaw_kg + erect_kg;
        r.saw_kg += saw_kg;
        r.weld_metal_kg += u.weld_metal_kg;
        r.steel_t += u.steel_t;
        auto& Q = r.quantities;
        Q.plate_area += q.plate_area;
        Q.stiffener_length += q.stiffener_length;
        Q.paint_area += q.paint_area;
        Q.cut_length += q.cut_length;
        Q.fillet_length += q.fillet_length;
        Q.shop_butt_length += q.shop_butt_length;
        Q.erection_length += q.erection_length;
        Q.heating_length += q.heating_length;
        r.blocks.push_back(u);
    }

    // Tank tests (ship level): water fill, and air delivered to raise the test pressure.
    const auto& ut = y.util_testing;
    const double vol = tanks.cargo_m3 + tanks.ballast_m3;
    UtilityUse& test = r.by_stage[S(Stage::Testing)];
    test.water_m3 = vol * ut.water_test_fraction;
    test.air_nm3 = vol * ut.air_test_fraction * (ut.air_test_pressure_bar / 1.01325) * ut.air_leak_factor;

    for (int s = 0; s < kStageCount; ++s) r.total += r.by_stage[s];
    return r;
}

void write_utilities_csv(const UtilityReport& r, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    const auto& items = utility_items();
    f << "scope,stage";
    for (auto& it : items) f << ',' << it.key;
    f << ",weld_metal_kg,steel_t\n";
    char buf[64];
    auto row = [&](const std::string& scope, const std::string& stage, const UtilityUse& u, double wm, double st) {
        f << scope << ',' << stage;
        for (auto& it : items) {
            std::snprintf(buf, sizeof buf, ",%.2f", u.*(it.member));
            f << buf;
        }
        std::snprintf(buf, sizeof buf, ",%.1f,%.1f\n", wm, st);
        f << buf;
    };
    for (int s = 0; s < kStageCount; ++s) row("ship", stage_key(static_cast<Stage>(s)), r.by_stage[s], 0, 0);
    row("ship", "total", r.total, r.weld_metal_kg, r.steel_t);
    for (auto& b : r.blocks) {
        for (int s = 0; s < kStageCount; ++s)
            if (s != static_cast<int>(Stage::Testing)) row(b.block, stage_key(static_cast<Stage>(s)), b.by_stage[s], 0, 0);
        row(b.block, "total", b.total, b.weld_metal_kg, b.steel_t);
    }
}

}  // namespace hullcad
