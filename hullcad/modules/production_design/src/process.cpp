#include "hullcad/production_design/process.hpp"

namespace hullcad {

const char* stage_key(Stage s) {
    static const char* k[] = {"cutting", "welding", "heating", "painting", "handling", "erection", "testing"};
    return k[static_cast<int>(s)];
}

const char* stage_label(Stage s) {
    static const char* k[] = {"절단", "공장 용접", "곡가공 (선상가열)", "블라스팅·도장", "크레인·운반", "탑재 용접", "탱크 시험"};
    return k[static_cast<int>(s)];
}

}  // namespace hullcad
