#include "hullcad/core/calendar.hpp"

#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace hullcad {

namespace {
long floor_div(long a, long b) { return a / b - ((a % b != 0) && ((a < 0) != (b < 0))); }
}  // namespace

// Howard Hinnant's days_from_civil / civil_from_days.
long days_from_date(const std::string& ymd_in) {
    std::string ymd = ymd_in;
    const size_t a = ymd.find_first_not_of(" \t\r\n"), b = ymd.find_last_not_of(" \t\r\n");
    ymd = a == std::string::npos ? std::string() : ymd.substr(a, b - a + 1);
    int y = 0, m = 0, d = 0;
    char c1 = 0, c2 = 0;
    std::istringstream in(ymd);
    in >> y >> c1 >> m >> c2 >> d;
    std::string rest;
    if (!in || c1 != '-' || c2 != '-' || (in >> rest) || m < 1 || m > 12 || d < 1)
        throw std::invalid_argument("'" + ymd_in + "' is not a date (YYYY-MM-DD)");
    static const int mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    const bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    if (d > mdays[m - 1] + (m == 2 && leap ? 1 : 0)) throw std::invalid_argument("'" + ymd_in + "' is not a valid day");
    y -= m <= 2;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<long>(doe) - 719468;
}

std::string date_from_days(long z) {
    z += 719468;
    const long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    long y = static_cast<long>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned m = mp < 10 ? mp + 3 : mp - 9;
    y += m <= 2;
    char buf[48];
    std::snprintf(buf, sizeof buf, "%04ld-%02u-%02u", y, m, d);
    return buf;
}

int weekday_mon0(long days) { return static_cast<int>(((days % 7) + 7 + 3) % 7); }  // 1970-01-01 was a Thursday

WorkCalendar::WorkCalendar(long start_day, int work_days_per_week) {
    if (work_days_per_week < 1 || work_days_per_week > 7)
        throw std::invalid_argument("work_days_per_week must be 1..7");
    wdpw_ = work_days_per_week;
    while (weekday_mon0(start_day) >= wdpw_) ++start_day;  // start on a working day
    pos0_ = weekday_mon0(start_day);
    monday0_ = start_day - pos0_;
}

long WorkCalendar::day(int wd) const {
    const long g = pos0_ + static_cast<long>(wd);
    const long week = floor_div(g, wdpw_);
    return monday0_ + 7 * week + (g - week * wdpw_);
}

}  // namespace hullcad
