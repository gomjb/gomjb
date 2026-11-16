#pragma once
// Section with curvature combs (before fairing: red, faired: blue) and the curvature along the
// girth underneath. Wheel zooms, drag pans, double-click fits.
#include <QPointF>
#include <QWidget>
#include <vector>

#include "hullcad/fairing/fairing.hpp"

class CombPlot : public QWidget {
    Q_OBJECT
public:
    explicit CombPlot(QWidget* parent = nullptr);
    void setSection(const hullcad::SectionFairing& s);
    void clear();
    void setShowBefore(bool on) { showBefore_ = on; update(); }
    void setShowAfter(bool on) { showAfter_ = on; update(); }

protected:
    void paintEvent(QPaintEvent*) override;
    void wheelEvent(QWheelEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent*) override;

private:
    QRectF topArea() const;
    QRectF bottomArea() const;
    std::vector<hullcad::CurvatureSample> before_, after_;
    std::vector<hullcad::Vec3> offsets_;
    double combScale_ = 1;
    bool focus_ = true;  // view: curved region (true) or whole section (false); double-click toggles
    double zoom_ = 1;
    QPointF pan_;
    QPointF lastMouse_;
    bool showBefore_ = true, showAfter_ = true;
    QString caption_;
};
