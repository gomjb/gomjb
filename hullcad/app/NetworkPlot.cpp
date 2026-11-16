#include "NetworkPlot.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

using namespace hullcad;

namespace {
const QColor kColors[3] = {QColor(0x8a, 0x8a, 0x8a), QColor(0xc6, 0x28, 0x28), QColor(0x1f, 0x6f, 0xb2)};
const QColor kPaper(0xfb, 0xfa, 0xf7);
const QColor kInk(0x22, 0x22, 0x22);
const QColor kGrid(0xdd, 0xd8, 0xcc);
}  // namespace

// ---------------------------------------------------------------- LongitudinalPlot

LongitudinalPlot::LongitudinalPlot(QWidget* parent) : QWidget(parent) { setMinimumSize(480, 360); }

void LongitudinalPlot::clear() {
    for (auto& s : sets_) { s.pts.clear(); s.k.clear(); }
    update();
}

void LongitudinalPlot::setShow(int set, bool on) {
    show_[set] = on;
    update();
}

void LongitudinalPlot::setCurve(const LongitudinalCurve& c, bool network_applied) {
    network_ = network_applied;
    kind_ = c.kind;
    const Vec3 pn = c.kind == 'W' ? Vec3{0, 0, 1} : Vec3{0, 1, 0};
    const std::vector<Vec3>* src[3] = {&c.raw, &c.sectioned, &c.faired};
    x0_ = 1e18;
    x1_ = -1e18;
    for (int i = 0; i < 3; ++i) {
        sets_[i].pts = *src[i];
        sets_[i].k.clear();
        if (sets_[i].pts.size() >= 4) sets_[i].k = curvature_samples(longitudinal_curve(sets_[i].pts), 1500, pn);
        for (auto& q : sets_[i].pts) { x0_ = std::min(x0_, q.x); x1_ = std::max(x1_, q.x); }
    }
    if (x1_ <= x0_) { x0_ = 0; x1_ = 1; }
    v0_ = x0_;
    v1_ = x1_;
    caption_ = tr("%1   곡률 변동량 Σ|Δκ|: 원래 %2 → 단면 순정 %3%4   변곡점 %5 → %6%7")
                   .arg(QString::fromStdString(c.name()))
                   .arg(c.q_raw.variation, 0, 'f', 3)
                   .arg(c.q_sectioned.variation, 0, 'f', 3)
                   .arg(network_ ? tr(" → 곡선망 순정 %1").arg(c.q_faired.variation, 0, 'f', 3) : QString())
                   .arg(c.q_raw.inflections)
                   .arg(c.q_sectioned.inflections)
                   .arg(network_ ? tr(" → %1").arg(c.q_faired.inflections) : QString());
    update();
}

void LongitudinalPlot::paintEvent(QPaintEvent*) {
    QPainter g(this);
    g.setRenderHint(QPainter::Antialiasing);
    g.fillRect(rect(), kPaper);
    if (sets_[1].pts.size() < 4) {
        g.drawText(rect(), Qt::AlignCenter, tr("수선 또는 버톡을 선택하세요"));
        return;
    }
    g.setPen(kInk);
    g.drawText(QRectF(10, 6, width() - 20, 18), Qt::AlignLeft, caption_);
    const QRectF top(64, 34, width() - 78, height() * 0.30);
    const QRectF bot(64, top.bottom() + 34, width() - 78, height() - top.bottom() - 34 - 30);
    auto X = [&](double x) { return top.left() + (x - v0_) / (v1_ - v0_) * top.width(); };
    auto coord = [&](const Vec3& q) { return kind_ == 'W' ? q.y : q.z; };
    const bool on[3] = {show_[0], show_[1], show_[2] && network_};

    // --- Shape (vertical scale exaggerated).
    double c0 = 1e18, c1 = -1e18;
    for (int i = 0; i < 3; ++i)
        if (on[i])
            for (auto& q : sets_[i].pts)
                if (q.x >= v0_ && q.x <= v1_) { c0 = std::min(c0, coord(q)); c1 = std::max(c1, coord(q)); }
    if (c1 <= c0) { c0 -= 0.5; c1 += 0.5; }
    const double pad = 0.08 * (c1 - c0);
    c0 -= pad;
    c1 += pad;
    auto Yc = [&](double v) { return top.bottom() - (v - c0) / (c1 - c0) * top.height(); };
    g.setPen(kGrid);
    g.drawRect(top);
    g.save();
    g.setClipRect(top);
    for (int i = 0; i < 3; ++i) {
        if (!on[i]) continue;
        QPainterPath path;
        bool first = true;
        for (auto& s : sets_[i].k) {
            const QPointF q(X(s.p.x), Yc(coord(s.p)));
            if (first) path.moveTo(q); else path.lineTo(q);
            first = false;
        }
        g.setPen(QPen(kColors[i], i == 2 ? 2.0 : 1.4));
        g.setBrush(Qt::NoBrush);
        g.drawPath(path);
        g.setPen(Qt::NoPen);
        g.setBrush(kColors[i]);
        for (auto& q : sets_[i].pts) g.drawEllipse(QPointF(X(q.x), Yc(coord(q))), 2.0, 2.0);
    }
    g.restore();
    g.setPen(QColor(0x44, 0x44, 0x44));
    const QString axis = kind_ == 'W' ? tr("반폭 y (m)") : tr("높이 z (m)");
    g.drawText(QRectF(0, top.top() - 2, top.left() - 6, 14), Qt::AlignRight, QString::number(c1, 'f', 2));
    g.drawText(QRectF(0, top.bottom() - 12, top.left() - 6, 14), Qt::AlignRight, QString::number(c0, 'f', 2));
    g.drawText(QRectF(top.left() + 6, top.top() + 4, 400, 14), Qt::AlignLeft, axis + tr(" — 세로 확대"));

    // --- Curvature against x.
    double kmax = 1e-9;
    for (int i = 0; i < 3; ++i)
        if (on[i])
            for (auto& s : sets_[i].k)
                if (s.p.x >= v0_ && s.p.x <= v1_) kmax = std::max(kmax, std::fabs(s.kappa));
    kmax *= 1.1;
    auto Yk = [&](double k) { return bot.center().y() - k / kmax * 0.5 * bot.height(); };
    g.setPen(kGrid);
    g.drawRect(bot);
    g.setPen(QPen(QColor(0x88, 0x88, 0x88), 1, Qt::DashLine));
    g.drawLine(QPointF(bot.left(), Yk(0)), QPointF(bot.right(), Yk(0)));
    g.save();
    g.setClipRect(bot);
    for (int i = 0; i < 3; ++i) {
        if (!on[i]) continue;
        QColor light = kColors[i];
        light.setAlpha(60);
        g.setPen(QPen(light, 1));
        for (size_t j = 0; j < sets_[i].k.size(); j += 4) {
            const auto& s = sets_[i].k[j];
            g.drawLine(QPointF(X(s.p.x), Yk(0)), QPointF(X(s.p.x), Yk(s.kappa)));
        }
        QPainterPath path;
        bool first = true;
        for (auto& s : sets_[i].k) {
            const QPointF q(X(s.p.x), Yk(s.kappa));
            if (first) path.moveTo(q); else path.lineTo(q);
            first = false;
        }
        g.setPen(QPen(kColors[i], i == 2 ? 1.8 : 1.3));
        g.drawPath(path);
    }
    g.restore();
    g.setPen(QColor(0x44, 0x44, 0x44));
    g.drawText(QRectF(0, bot.top() - 2, bot.left() - 6, 14), Qt::AlignRight, QString::number(kmax, 'g', 2));
    g.drawText(QRectF(0, Yk(0) - 7, bot.left() - 6, 14), Qt::AlignRight, QStringLiteral("0"));
    g.drawText(QRectF(0, bot.bottom() - 12, bot.left() - 6, 14), Qt::AlignRight, QString::number(-kmax, 'g', 2));
    g.drawText(QRectF(bot.left(), bot.bottom() + 4, bot.width(), 16), Qt::AlignLeft,
               QStringLiteral("x = %1 m").arg(v0_, 0, 'f', 1));
    g.drawText(QRectF(bot.left(), bot.bottom() + 4, bot.width(), 16), Qt::AlignRight,
               QStringLiteral("x = %1 m").arg(v1_, 0, 'f', 1));
    g.drawText(QRectF(bot.left(), bot.bottom() + 4, bot.width(), 16), Qt::AlignCenter,
               tr("곡률 κ (1/m), 선미(왼쪽) → 선수(오른쪽)"));

    // Legend.
    double lx = bot.left() + 8, ly = bot.top() + 8;
    const QString names[3] = {tr("원래 단면에서 자른 곡선"), tr("단면 순정 후"), tr("곡선망 순정 후")};
    for (int i = 0; i < 3; ++i) {
        if (!on[i]) continue;
        g.fillRect(QRectF(lx, ly, 14, 3), kColors[i]);
        g.setPen(QColor(0x33, 0x33, 0x33));
        g.drawText(QPointF(lx + 20, ly + 5), names[i]);
        ly += 16;
    }
}

void LongitudinalPlot::wheelEvent(QWheelEvent* e) {
    const double f = e->angleDelta().y() > 0 ? 0.8 : 1.25;
    const double left = 64, w = std::max(1.0, width() - 78.0);
    const double xm = v0_ + (e->position().x() - left) / w * (v1_ - v0_);
    double a = xm + (v0_ - xm) * f, b = xm + (v1_ - xm) * f;
    if (b - a > x1_ - x0_) { a = x0_; b = x1_; }
    if (b - a < 2.0) return;
    v0_ = a;
    v1_ = b;
    update();
}

void LongitudinalPlot::mousePressEvent(QMouseEvent* e) { lastMouse_ = e->position(); }

void LongitudinalPlot::mouseMoveEvent(QMouseEvent* e) {
    if (!(e->buttons() & Qt::LeftButton)) return;
    const double w = std::max(1.0, width() - 78.0);
    const double dx = -(e->position().x() - lastMouse_.x()) / w * (v1_ - v0_);
    lastMouse_ = e->position();
    v0_ += dx;
    v1_ += dx;
    update();
}

void LongitudinalPlot::mouseDoubleClickEvent(QMouseEvent*) {
    v0_ = x0_;
    v1_ = x1_;
    update();
}

// ---------------------------------------------------------------- CurvatureMapView

CurvatureMapView::CurvatureMapView(QWidget* parent) : QWidget(parent) { setMinimumSize(480, 260); }

void CurvatureMapView::clear() {
    map_ = {};
    image_ = {};
    update();
}

void CurvatureMapView::setMap(const CurvatureMap& m, const QString& caption) {
    map_ = m;
    caption_ = caption;
    rebuild();
}

void CurvatureMapView::setGirth(bool girth) {
    girth_ = girth;
    rebuild();
}

void CurvatureMapView::rebuild() {
    if (map_.nu == 0) { image_ = {}; update(); return; }
    const auto& k = girth_ ? map_.k_girth : map_.k_long;
    std::vector<double> a;
    a.reserve(k.size());
    for (double v : k) a.push_back(std::fabs(v));
    std::sort(a.begin(), a.end());
    scale_ = std::max(1e-6, a[static_cast<size_t>(0.95 * (a.size() - 1))]);
    const double thr = girth_ ? 1e-3 : 2e-4;  // almost straight: white
    image_ = QImage(map_.nu, map_.nv, QImage::Format_RGB32);
    for (int i = 0; i < map_.nu; ++i)
        for (int j = 0; j < map_.nv; ++j) {
            const double v = k[i * map_.nv + j];
            QColor c(255, 255, 255);
            if (std::fabs(v) >= thr) {
                // Gamma < 1 so the small curvature of the long flat curves still shows its sign.
                const double t = std::min(1.0, std::pow(std::fabs(v) / scale_, 0.4));
                const QColor full = v > 0 ? QColor(0xc6, 0x28, 0x28) : QColor(0x1f, 0x6f, 0xb2);
                c = QColor(static_cast<int>(255 + (full.red() - 255) * (0.25 + 0.75 * t)),
                           static_cast<int>(255 + (full.green() - 255) * (0.25 + 0.75 * t)),
                           static_cast<int>(255 + (full.blue() - 255) * (0.25 + 0.75 * t)));
            }
            image_.setPixel(i, map_.nv - 1 - j, c.rgb());  // keel at the bottom
        }
    update();
}

void CurvatureMapView::paintEvent(QPaintEvent*) {
    QPainter g(this);
    g.fillRect(rect(), kPaper);
    if (image_.isNull()) {
        g.drawText(rect(), Qt::AlignCenter, tr("곡면을 계산하면 곡률 지도가 표시됩니다"));
        return;
    }
    g.setPen(kInk);
    g.drawText(QRectF(10, 6, width() - 20, 18), Qt::AlignLeft, caption_);
    const QRectF r(64, 32, width() - 78, height() - 32 - 46);
    g.drawImage(r, image_);
    g.setPen(kGrid);
    g.drawRect(r);
    g.setPen(QColor(0x44, 0x44, 0x44));
    g.drawText(QRectF(0, r.top(), r.left() - 6, 14), Qt::AlignRight, tr("갑판"));
    g.drawText(QRectF(0, r.bottom() - 14, r.left() - 6, 14), Qt::AlignRight, tr("용골"));
    g.drawText(QRectF(r.left(), r.bottom() + 4, r.width(), 16), Qt::AlignLeft,
               QStringLiteral("x = %1 m").arg(map_.x_min, 0, 'f', 1));
    g.drawText(QRectF(r.left(), r.bottom() + 4, r.width(), 16), Qt::AlignRight,
               QStringLiteral("x = %1 m").arg(map_.x_max, 0, 'f', 1));
    g.drawText(QRectF(r.left(), r.bottom() + 4, r.width(), 16), Qt::AlignCenter,
               girth_ ? tr("거스 방향 법곡률 (단면 방향)") : tr("길이 방향 법곡률 (수선·버톡 방향)"));
    // Colour key.
    const double ky = r.bottom() + 24;
    auto key = [&](double x, QColor c, const QString& t) {
        g.fillRect(QRectF(x, ky, 14, 10), c);
        g.setPen(QColor(0x33, 0x33, 0x33));
        g.drawText(QPointF(x + 20, ky + 9), t);
    };
    key(r.left(), QColor(0xc6, 0x28, 0x28), tr("볼록 (+)"));
    key(r.left() + 110, QColor(0x1f, 0x6f, 0xb2), tr("오목 (−)"));
    key(r.left() + 220, QColor(255, 255, 255), tr("거의 직선"));
    g.drawText(QPointF(r.left() + 340, ky + 9), tr("색 최대 |κ| = %1 1/m (95 %ile)").arg(scale_, 0, 'g', 2));
}
