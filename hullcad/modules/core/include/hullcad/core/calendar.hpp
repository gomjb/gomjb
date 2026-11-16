#pragma once
// Dates and working-day calendars.
#include <string>

namespace hullcad {

// Days since 1970-01-01 <-> "YYYY-MM-DD" (proleptic Gregorian).
long days_from_date(const std::string& ymd);  // throws std::invalid_argument on malformed dates
std::string date_from_days(long days);
int weekday_mon0(long days);                  // Monday = 0 ... Sunday = 6

// Working-day calendar: working day 0 is the start day (moved to the next working day if
// needed); negative indices are earlier working days. Weeks run Monday..(work_days_per_week).
class WorkCalendar {
public:
    WorkCalendar() = default;
    WorkCalendar(long start_day, int work_days_per_week);
    long day(int wd) const;  // calendar day of working day wd
    std::string date(int wd) const { return date_from_days(day(wd)); }
    int work_days_per_week() const { return wdpw_; }

private:
    long monday0_ = 0;  // Monday of the week containing working day 0
    int pos0_ = 0;      // weekday of working day 0
    int wdpw_ = 6;
};

}  // namespace hullcad
