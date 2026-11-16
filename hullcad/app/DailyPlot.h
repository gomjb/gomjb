#pragma once
// Daily utility profile: one stacked bar per working day (colour = production stage, grey = base
// load) with the supply capacity drawn as a line. Hover shows the day's breakdown.
#include <QStringList>
#include <QWidget>
#include <array>
#include <vector>

#include "hullcad/schedule.hpp"

class DailyPlot : public QWidget {
    Q_OBJECT
public:
    explicit DailyPlot(QWidget* parent = nullptr);
    // item: index into hullcad::utility_items(); capacity_per_day: 0 = none.
    void setData(const hullcad::ScheduleResult& s, int item, double capacity_per_day, const QString& unit);
    void clear();

protected:
    void paintEvent(QPaintEvent*) override;
    void mouseMoveEvent(QMouseEvent* e) override;

private:
    QRectF plotArea() const;
    std::vector<std::array<double, hullcad::kStageCount + 1>> days_;  // per stage + base
    QStringList dates_;
    double capacity_ = 0;
    double ymax_ = 1;
    QString unit_, title_;
};
