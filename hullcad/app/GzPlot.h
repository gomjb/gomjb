#pragma once
// GZ (righting lever) curve plot drawn with QPainter.
#include <QWidget>
#include <vector>

#include "hullcad/naval/hydrostatics.hpp"

class GzPlot : public QWidget {
    Q_OBJECT
public:
    explicit GzPlot(QWidget* parent = nullptr);
    void setCurve(const std::vector<hullcad::GZPoint>& gz, double GM);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    std::vector<hullcad::GZPoint> gz_;
    double GM_ = 0;
};
