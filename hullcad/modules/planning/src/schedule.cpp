#include "hullcad/planning/schedule.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace hullcad {

namespace {

// Erection rank of a zone inside its ring: bottom, side (lower), bulkhead, side (upper), deck.
int zone_rank(const std::string& z) {
    auto starts = [&](const char* p) { return z.rfind(p, 0) == 0; };
    if (starts("DB") || starts("LOW")) return 0;
    if (starts("SIDE-U")) return 3;
    if (starts("SIDE")) return 1;
    if (starts("BHD")) return 2;
    if (starts("UP")) return 3;
    if (starts("DECK")) return 4;
    return 5;
}

}  // namespace

int ProductionSchedule::first_wd() const {
    int f = testing.start;
    for (auto& b : blocks)
        for (int st = 0; st < kStageCount; ++st)
            if (st != static_cast<int>(Stage::Testing) && b.span[st].days() > 0) f = std::min(f, b.span[st].start);
    return f;
}

int ProductionSchedule::last_wd() const {
    int l = testing.end - 1;
    for (auto& b : blocks)
        for (int st = 0; st < kStageCount; ++st)
            if (st != static_cast<int>(Stage::Testing) && b.span[st].days() > 0) l = std::max(l, b.span[st].end - 1);
    return l;
}

ProductionSchedule schedule_blocks(const Division& d, const PlanningSettings& ps) {
    const auto& sc = ps.schedule;
    if (sc.erection_blocks_per_day <= 0) throw std::invalid_argument("erection_blocks_per_day must be > 0");
    ProductionSchedule r;
    r.calendar = WorkCalendar(days_from_date(sc.erection_start), sc.work_days_per_week);
    const int n = static_cast<int>(d.blocks.size());
    if (n == 0) return r;

    // Erection order: key ring, then alternately the next ring forward and aft.
    const int rings = static_cast<int>(d.butts.size()) - 1;
    const double Lpp = d.butts.back();
    int key = 1;
    for (int i = 0; i < rings; ++i)
        if (d.butts[i] <= sc.erection_key_x * Lpp && sc.erection_key_x * Lpp < d.butts[i + 1]) key = i + 1;
    std::vector<int> ring_order{key};
    for (int k = 1; static_cast<int>(ring_order.size()) < rings; ++k) {
        if (key + k <= rings) ring_order.push_back(key + k);
        if (key - k >= 1) ring_order.push_back(key - k);
    }
    std::vector<int> order;  // block indices in erection order
    for (int ring : ring_order) {
        std::vector<int> in_ring;
        for (int i = 0; i < n; ++i)
            if (d.blocks[i].ring == ring) in_ring.push_back(i);
        std::stable_sort(in_ring.begin(), in_ring.end(), [&](int a, int b) {
            return zone_rank(d.blocks[a].zone) < zone_rank(d.blocks[b].zone);
        });
        order.insert(order.end(), in_ring.begin(), in_ring.end());
    }

    // Stage spans, planned backwards from each block's erection day.
    r.blocks.resize(n);
    auto S = [](Stage st) { return static_cast<int>(st); };
    for (int k = 0; k < n; ++k) {
        const int i = order[k];
        const Block& b = d.blocks[i];
        BlockSchedule& s = r.blocks[i];
        s.block = b.name;
        s.zone = b.zone;
        s.ring = b.ring;
        s.erection_order = k + 1;
        const int E = static_cast<int>(std::floor(k / sc.erection_blocks_per_day));
        s.erection_wd = E;
        r.last_erection_wd = std::max(r.last_erection_wd, E);
        auto& sp = s.span;
        sp[S(Stage::Erection)] = {E, E + std::max(1, sc.erection_weld_days)};
        sp[S(Stage::Handling)] = {E, E + 1};
        const int paint_end = E - std::max(0, sc.paint_to_erection_days);
        sp[S(Stage::Painting)] = {paint_end - std::max(1, sc.painting_days), paint_end};
        const int asm_days = std::max(1, static_cast<int>(std::lround(sc.assembly_days + sc.assembly_days_per_100t * b.mass.w / 100.0)));
        const int A1 = sp[S(Stage::Painting)].start;
        sp[S(Stage::Welding)] = {A1 - asm_days, A1};
        const int A0 = sp[S(Stage::Welding)].start;
        const bool curved = b.q.heating_length > 0;
        sp[S(Stage::Heating)] = curved ? StageSpan{A0 - std::max(1, sc.heating_days), A0} : StageSpan{A0, A0};
        const int H0 = sp[S(Stage::Heating)].start;
        sp[S(Stage::Cutting)] = {H0 - std::max(1, sc.cutting_days), H0};
    }
    r.testing.start = r.last_erection_wd + std::max(0, sc.test_after_last_erection_days);
    r.testing.end = r.testing.start + std::max(1, sc.test_days);
    return r;
}

void write_block_schedule_csv(const ProductionSchedule& s, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << "block,ring,zone,erection_order,erection_date";
    for (int st = 0; st < kStageCount; ++st)
        if (st != static_cast<int>(Stage::Testing))
            f << ',' << stage_key(static_cast<Stage>(st)) << "_start," << stage_key(static_cast<Stage>(st)) << "_end";
    f << '\n';
    for (auto& b : s.blocks) {
        f << b.block << ',' << b.ring << ',' << b.zone << ',' << b.erection_order << ',' << s.calendar.date(b.erection_wd);
        for (int st = 0; st < kStageCount; ++st) {
            if (st == static_cast<int>(Stage::Testing)) continue;
            const StageSpan sp = b.span[st];
            if (sp.days() <= 0) f << ",,";
            else f << ',' << s.calendar.date(sp.start) << ',' << s.calendar.date(sp.end - 1);
        }
        f << '\n';
    }
}

}  // namespace hullcad
