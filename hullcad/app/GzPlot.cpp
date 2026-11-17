#include "GzPlot.h"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

GzPlot::GzPlot(QWidget* parent) : QWidget(parent) {
    setMinimumSize(360, 240);
    setAutoFillBackground(true);
}

void GzPlot::setCurve(const std::vector<hullcad::GZPoint>& gz, double GM) {
    gz_ = gz;
    GM_ = GM;
    update();
}

void GzPlot::paintEvent(QPaintEvent*) {
    QPainter g(this);
    g.setRenderHint(QPainter::Antialiasing);
    g.fillRect(rect(), QColor(0xfb, 0xfa, 0xf7));
    const QRectF area = QRectF(rect()).adjusted(56, 34, -20, -40);
    if (gz_.empty()) {
        g.drawText(rect(), Qt::AlignCenter, tr("GZ 곡선 없음"));
        return;
    }
    double hmax = gz_.back().heel, zmin = 0, zmax = 0;
    for (auto& p : gz_) { zmin = std::min(zmin, p.GZ); zmax = std::max(zmax, p.GZ); }
    zmax = std::ceil(zmax * 2 + 0.01) / 2;
    zmin = std::floor(zmin * 2) / 2;
    auto X = [&](double h) { return area.left() + h / hmax * area.width(); };
    auto Y = [&](double z) { return area.bottom() - (z - zmin) / (zmax - zmin) * area.height(); };

    g.setPen(QPen(QColor(0xdd, 0xd8, 0xcc), 1));
    for (double h = 0; h <= hmax + 1e-9; h += 10) g.drawLine(QPointF(X(h), area.top()), QPointF(X(h), area.bottom()));
    for (double z = zmin; z <= zmax + 1e-9; z += 0.5) g.drawLine(QPointF(area.left(), Y(z)), QPointF(area.right(), Y(z)));
    g.setPen(QPen(QColor(0x55, 0x55, 0x55), 1.2));
    g.drawLine(QPointF(area.left(), Y(0)), QPointF(area.right(), Y(0)));
    g.drawLine(QPointF(area.left(), area.top()), QPointF(area.left(), area.bottom()));

    g.setPen(QColor(0x44, 0x44, 0x44));
    for (double h = 0; h <= hmax + 1e-9; h += 10)
        g.drawText(QRectF(X(h) - 20, area.bottom() + 4, 40, 16), Qt::AlignCenter, QString::number(h, 'f', 0));
    for (double z = zmin; z <= zmax + 1e-9; z += 0.5)
        g.drawText(QRectF(area.left() - 50, Y(z) - 8, 44, 16), Qt::AlignRight | Qt::AlignVCenter, QString::number(z, 'f', 1));
    g.drawText(QRectF(area.left(), area.bottom() + 20, area.width(), 18), Qt::AlignCenter, tr("횡경사각 (deg)"));
    g.save();
    g.translate(14, area.center().y());
    g.rotate(-90);
    g.drawText(QRectF(-60, -8, 120, 16), Qt::AlignCenter, tr("GZ (m)"));
    g.restore();

    // Initial slope = GM: tangent from the origin reaching GM at 1 rad (57.3 deg).
    g.save();
    g.setClipRect(area);
    g.setPen(QPen(QColor(0x9a, 0x9a, 0x9a), 1, Qt::DashLine));
    const double h1 = std::min(57.3, hmax);
    g.drawLine(QPointF(X(0), Y(0)), QPointF(X(h1), Y(GM_ * h1 / 57.3)));
    g.restore();

    QPainterPath path;
    for (size_t i = 0; i < gz_.size(); ++i) {
        QPointF pt(X(gz_[i].heel), Y(gz_[i].GZ));
        if (i == 0) path.moveTo(pt); else path.lineTo(pt);
    }
    g.setPen(QPen(QColor(0x1f, 0x6f, 0xb2), 2.2));
    g.drawPath(path);

    auto best = std::max_element(gz_.begin(), gz_.end(), [](auto& a, auto& b) { return a.GZ < b.GZ; });
    g.setPen(QColor(0x22, 0x22, 0x22));
    g.drawText(QRectF(area.left(), 8, area.width(), 18), Qt::AlignLeft,
               tr("GM %1 m,  GZmax %2 m @ %3°  (점선: GM 기울기)")
                   .arg(GM_, 0, 'f', 2).arg(best->GZ, 0, 'f', 2).arg(best->heel, 0, 'f', 1));
}
