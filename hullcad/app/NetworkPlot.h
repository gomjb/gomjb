#pragma once
// Curve-network inspection views.
//
// LongitudinalPlot: one waterline or buttock along the ship: its shape on top (vertical scale
// exaggerated) and its curvature against x underneath, for the unfaired, the section-faired and
// the network-faired sections. Wheel zooms along x, drag pans, double-click fits.
//
// CurvatureMapView: the normal curvature of the hull surface over (x, girth): red = convex,
// blue = concave, white = almost straight. Alternating red/blue stripes across the ship are
// stations that disagree with their neighbours.
#include <QImage>
#include <QPointF>
#include <QWidget>
#include <vector>

#include "hullcad/fairing/network.hpp"

class LongitudinalPlot : public QWidget {
    Q_OBJECT
public:
    explicit LongitudinalPlot(QWidget* parent = nullptr);
    void setCurve(const hullcad::LongitudinalCurve& c, bool network_applied);
    void clear();
    void setShow(int set, bool on);  // 0 raw, 1 section-faired, 2 network-faired

protected:
    void paintEvent(QPaintEvent*) override;
    void wheelEvent(QWheelEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;

private:
    struct Set {
        std::vector<hullcad::Vec3> pts;
        std::vector<hullcad::CurvatureSample> k;
    };
    Set sets_[3];
    bool show_[3] = {true, true, true};
    bool network_ = false;
    char kind_ = 'W';
    QString caption_;
    double x0_ = 0, x1_ = 1;      // full x range
    double v0_ = 0, v1_ = 1;      // view x range
    QPointF lastMouse_;
};

class CurvatureMapView : public QWidget {
    Q_OBJECT
public:
    explicit CurvatureMapView(QWidget* parent = nullptr);
    void setMap(const hullcad::CurvatureMap& m, const QString& caption);
    void setGirth(bool girth);  // false: longitudinal curvature, true: girthwise curvature
    void clear();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void rebuild();
    hullcad::CurvatureMap map_;
    bool girth_ = false;
    QImage image_;
    QString caption_;
    double scale_ = 1;  // curvature at full colour, 1/m
};
