#include "CombPlot.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

using namespace hullcad;

namespace {
const QColor kBefore(0xc6, 0x28, 0x28);
const QColor kAfter(0x1f, 0x6f, 0xb2);
}  // namespace

CombPlot::CombPlot(QWidget* parent) : QWidget(parent) {
    setMinimumSize(420, 360);
    setMouseTracking(false);
}

void CombPlot::clear() {
    before_.clear();
    after_.clear();
    offsets_.clear();
    update();
}

void CombPlot::setSection(const SectionFairing& s) {
    before_ = curvature_samples(s.before, 900);
    after_ = curvature_samples(s.composite, 900);
    offsets_ = s.offsets;
    double kmax = 1e-9, ext = 0;
    for (auto& c : before_) kmax = std::max(kmax, std::fabs(c.kappa));
    for (auto& c : after_) kmax = std::max(kmax, std::fabs(c.kappa));
    for (auto& c : after_) ext = std::max({ext, std::fabs(c.p.y), std::fabs(c.p.z)});
    combScale_ = 0.18 * std::max(ext, 1.0) / kmax;  // longest spike = 18 % of the section size
    caption_ = tr("x = %1 m   최대 편차 %2 mm   변곡점 %3 → %4   곡률 변화 에너지 %5 → %6%7")
                   .arg(s.x, 0, 'f', 1)
                   .arg(s.max_deviation * 1000, 0, 'f', 1)
                   .arg(s.q_before.inflections)
                   .arg(s.q_composite.inflections)
                   .arg(s.q_before.energy, 0, 'f', 3)
                   .arg(s.q_composite.energy, 0, 'f', 3)
                   .arg(s.ok ? QString() : tr("   (순정 적용 불가: 오프셋 유지)"));
    zoom_ = 1;
    pan_ = {};
    update();
}

QRectF CombPlot::topArea() const { return QRectF(rect()).adjusted(10, 30, -10, -(height() * 0.28 + 8)); }
QRectF CombPlot::bottomArea() const {
    const double h = height() * 0.28;
    return QRectF(56, height() - h + 4, width() - 70, h - 26);
}

void CombPlot::paintEvent(QPaintEvent*) {
    QPainter g(this);
    g.setRenderHint(QPainter::Antialiasing);
    g.fillRect(rect(), QColor(0xfb, 0xfa, 0xf7));
    if (after_.empty()) {
        g.drawText(rect(), Qt::AlignCenter, tr("단면을 선택하세요"));
        return;
    }
    g.setPen(QColor(0x22, 0x22, 0x22));
    g.drawText(QRectF(10, 6, width() - 20, 18), Qt::AlignLeft, caption_);

    // --- Section with combs (equal axes), port half: y to the right, z up.
    const QRectF a = topArea();
    double ymin = 1e9, ymax = -1e9, zmin = 1e9, zmax = -1e9;
    auto grow = [&](double y, double z) { ymin = std::min(ymin, y); ymax = std::max(ymax, y); zmin = std::min(zmin, z); zmax = std::max(zmax, z); };
    double kpk = 0;
    for (auto& c : after_) kpk = std::max(kpk, std::fabs(c.kappa));
    for (auto* v : {&after_, &before_})
        for (auto& c : *v)
            if (!focus_ || std::fabs(c.kappa) >= 0.05 * kpk) {  // focus: where the section bends
                grow(c.p.y, c.p.z);
                grow(c.p.y - c.kappa * combScale_ * c.n.y, c.p.z - c.kappa * combScale_ * c.n.z);
            }
    if (focus_) {
        const double pad = 0.3 * std::max(ymax - ymin, zmax - zmin);
        ymin -= pad; ymax += pad; zmin -= pad; zmax += pad;
    }
    const double sc = zoom_ * std::min(a.width() / std::max(ymax - ymin, 1e-6), a.height() / std::max(zmax - zmin, 1e-6)) * 0.94;
    const QPointF c0(a.center().x() - 0.5 * (ymin + ymax) * sc + pan_.x(), a.center().y() + 0.5 * (zmin + zmax) * sc + pan_.y());
    auto P = [&](double y, double z) { return QPointF(c0.x() + y * sc, c0.y() - z * sc); };
    g.save();
    g.setClipRect(a);
    auto drawSet = [&](const std::vector<CurvatureSample>& v, QColor col, bool comb) {
        if (comb) {
            QColor light = col;
            light.setAlpha(90);
            g.setPen(QPen(light, 1));
            QPainterPath env;
            for (size_t i = 0; i < v.size(); i += 3) {
                const auto& c = v[i];
                const QPointF tip = P(c.p.y - c.kappa * combScale_ * c.n.y, c.p.z - c.kappa * combScale_ * c.n.z);
                g.drawLine(P(c.p.y, c.p.z), tip);
                if (i == 0) env.moveTo(tip); else env.lineTo(tip);
            }
            g.setPen(QPen(col, 1.2));
            g.drawPath(env);
        }
        QPainterPath path;
        for (size_t i = 0; i < v.size(); ++i) {
            const QPointF q = P(v[i].p.y, v[i].p.z);
            if (i == 0) path.moveTo(q); else path.lineTo(q);
        }
        g.setPen(QPen(col, 2.0));
        g.drawPath(path);
    };
    if (showBefore_) drawSet(before_, kBefore, true);
    if (showAfter_) drawSet(after_, kAfter, true);
    g.setPen(Qt::NoPen);
    g.setBrush(QColor(0x33, 0x33, 0x33));
    for (auto& q : offsets_) g.drawEllipse(P(q.y, q.z), 2.0, 2.0);
    g.restore();

    // Legend.
    double lx = a.left() + 6, ly = a.top() + 6;
    auto legend = [&](QColor col, const QString& t) {
        g.fillRect(QRectF(lx, ly, 14, 3), col);
        g.setPen(QColor(0x33, 0x33, 0x33));
        g.drawText(QPointF(lx + 20, ly + 5), t);
        ly += 16;
    };
    legend(kBefore, tr("순정 전: 오프셋 보간 곡선과 곡률 콤"));
    legend(kAfter, tr("순정 후: 순정 단면과 곡률 콤"));
    legend(QColor(0x33, 0x33, 0x33), tr("오프셋 점"));

    // --- Curvature along the girth.
    const QRectF b = bottomArea();
    double smax = 0, kmax = 1e-9, kmin = 0;
    for (auto* v : {&before_, &after_})
        for (auto& c : *v) { smax = std::max(smax, c.s); kmax = std::max(kmax, c.kappa); kmin = std::min(kmin, c.kappa); }
    kmax *= 1.1;
    kmin = std::min(kmin * 1.1, -0.05 * kmax);
    auto X = [&](double s) { return b.left() + s / smax * b.width(); };
    auto Y = [&](double k) { return b.bottom() - (k - kmin) / (kmax - kmin) * b.height(); };
    g.setPen(QColor(0xdd, 0xd8, 0xcc));
    g.drawRect(b);
    g.setPen(QPen(QColor(0x88, 0x88, 0x88), 1, Qt::DashLine));
    g.drawLine(QPointF(b.left(), Y(0)), QPointF(b.right(), Y(0)));
    auto curve = [&](const std::vector<CurvatureSample>& v, QColor col) {
        QPainterPath path;
        for (size_t i = 0; i < v.size(); ++i) {
            const QPointF q(X(v[i].s), Y(v[i].kappa));
            if (i == 0) path.moveTo(q); else path.lineTo(q);
        }
        g.setPen(QPen(col, 1.6));
        g.drawPath(path);
    };
    if (showBefore_) curve(before_, kBefore);
    if (showAfter_) curve(after_, kAfter);
    g.setPen(QColor(0x44, 0x44, 0x44));
    g.drawText(QRectF(0, b.top() - 2, b.left() - 4, 14), Qt::AlignRight, QString::number(kmax, 'f', 2));
    g.drawText(QRectF(0, Y(0) - 7, b.left() - 4, 14), Qt::AlignRight, QStringLiteral("0"));
    g.drawText(QRectF(b.left(), b.bottom() + 2, b.width(), 16), Qt::AlignCenter,
               tr("곡률 κ (1/m) — 용골(왼쪽)에서 갑판 끝(오른쪽)까지 거스(girth) 방향"));
}

void CombPlot::wheelEvent(QWheelEvent* e) {
    const double f = e->angleDelta().y() > 0 ? 1.25 : 0.8;
    const QRectF a = topArea();
    const QPointF m = e->position() - a.center();
    pan_ = m + (pan_ - m) * f;  // keep the point under the cursor fixed
    zoom_ *= f;
    update();
}

void CombPlot::mousePressEvent(QMouseEvent* e) { lastMouse_ = e->position(); }

void CombPlot::mouseMoveEvent(QMouseEvent* e) {
    if (!(e->buttons() & Qt::LeftButton)) return;
    pan_ += e->position() - lastMouse_;
    lastMouse_ = e->position();
    update();
}

void CombPlot::mouseDoubleClickEvent(QMouseEvent*) {
    focus_ = !focus_;
    zoom_ = 1;
    pan_ = {};
    update();
}
