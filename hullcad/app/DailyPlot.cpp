#include "DailyPlot.h"

#include <QLocale>
#include <QMouseEvent>
#include <QPainter>
#include <QToolTip>
#include <algorithm>
#include <cmath>

using namespace hullcad;

namespace {

const QColor kStageColors[kStageCount + 1] = {
    QColor(0x8f, 0xb8, 0xde),  // cutting
    QColor(0x5b, 0x8f, 0xc7),  // welding
    QColor(0xe3, 0xa7, 0x6f),  // heating
    QColor(0x6f, 0xae, 0x6a),  // painting
    QColor(0xd9, 0xc2, 0x7a),  // handling
    QColor(0x9f, 0x7b, 0xbd),  // erection
    QColor(0x4f, 0x9e, 0x9e),  // testing
    QColor(0xbb, 0xbb, 0xbb),  // base load
};

QString stageName(int s) {
    return s < kStageCount ? QString::fromUtf8(stage_label(static_cast<Stage>(s))) : QStringLiteral("기본 부하");
}

QString fmt(double v) { return QLocale(QLocale::English).toString(v, 'f', 0); }

// "Nice" axis step: 1, 2 or 5 x 10^n.
double niceStep(double range, int ticks) {
    const double raw = range / std::max(1, ticks);
    const double p = std::pow(10.0, std::floor(std::log10(raw)));
    for (double m : {1.0, 2.0, 5.0, 10.0})
        if (m * p >= raw) return m * p;
    return 10 * p;
}

}  // namespace

DailyPlot::DailyPlot(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(260);
    setMouseTracking(true);
}

void DailyPlot::clear() {
    days_.clear();
    dates_.clear();
    update();
}

void DailyPlot::setData(const ScheduleResult& s, int item, double capacity_per_day, const QString& unit) {
    const auto& it = utility_items().at(item);
    days_.assign(s.days(), {});
    dates_.clear();
    ymax_ = capacity_per_day;
    for (int k = 0; k < s.days(); ++k) {
        double sum = 0;
        for (int st = 0; st < kStageCount; ++st) sum += days_[k][st] = s.daily_by_stage[k][st].*(it.member);
        sum += days_[k][kStageCount] = s.daily_base[k].*(it.member);
        ymax_ = std::max(ymax_, sum);
        dates_ << QString::fromStdString(s.calendar.date(s.first_wd + k));
    }
    ymax_ = std::max(ymax_ * 1.08, 1e-9);
    capacity_ = capacity_per_day;
    unit_ = unit;
    title_ = tr("%1 일별 사용량 (%2/일)").arg(QString::fromUtf8(it.label), unit);
    update();
}

QRectF DailyPlot::plotArea() const { return QRectF(rect()).adjusted(78, 46, -16, -42); }

void DailyPlot::paintEvent(QPaintEvent*) {
    QPainter g(this);
    g.setRenderHint(QPainter::Antialiasing);
    g.fillRect(rect(), QColor(0xfb, 0xfa, 0xf7));
    if (days_.empty()) {
        g.drawText(rect(), Qt::AlignCenter, tr("일정 결과 없음"));
        return;
    }
    const QRectF a = plotArea();
    const int D = static_cast<int>(days_.size());
    const double bw = a.width() / D;
    auto Y = [&](double v) { return a.bottom() - v / ymax_ * a.height(); };

    // Grid and y labels.
    const double step = niceStep(ymax_, 5);
    g.setPen(QColor(0xdd, 0xd8, 0xcc));
    for (double v = 0; v <= ymax_ + 1e-9; v += step) g.drawLine(QPointF(a.left(), Y(v)), QPointF(a.right(), Y(v)));
    g.setPen(QColor(0x55, 0x55, 0x55));
    for (double v = 0; v <= ymax_ + 1e-9; v += step)
        g.drawText(QRectF(0, Y(v) - 8, a.left() - 6, 16), Qt::AlignRight | Qt::AlignVCenter, fmt(v));

    // Stacked bars.
    for (int k = 0; k < D; ++k) {
        double base = 0;
        for (int st = kStageCount; st >= 0; --st) {  // base load at the bottom, then the stages
            const double v = days_[k][st];
            if (v <= 0) continue;
            g.fillRect(QRectF(a.left() + k * bw, Y(base + v), std::max(bw - 0.3, 0.6), Y(base) - Y(base + v)), kStageColors[st]);
            base += v;
        }
    }
    // Capacity line.
    if (capacity_ > 0) {
        g.setPen(QPen(QColor(0xc6, 0x28, 0x28), 1.6, Qt::DashLine));
        g.drawLine(QPointF(a.left(), Y(capacity_)), QPointF(a.right(), Y(capacity_)));
        g.drawText(QRectF(a.left(), Y(capacity_) - 18, a.width() - 4, 16), Qt::AlignRight,
                   tr("공급 능력 %1 %2/일").arg(fmt(capacity_), unit_));
    }
    // X labels: about eight dates.
    g.setPen(QColor(0x55, 0x55, 0x55));
    const int every = std::max(1, D / 8);
    for (int k = 0; k < D; k += every) {
        g.drawLine(QPointF(a.left() + (k + 0.5) * bw, a.bottom()), QPointF(a.left() + (k + 0.5) * bw, a.bottom() + 4));
        g.drawText(QRectF(a.left() + (k + 0.5) * bw - 40, a.bottom() + 6, 80, 16), Qt::AlignCenter, dates_[k]);
    }
    g.drawLine(a.bottomLeft(), a.bottomRight());

    // Title and legend.
    g.setPen(QColor(0x22, 0x22, 0x22));
    QFont f = font();
    f.setBold(true);
    g.setFont(f);
    g.drawText(QRectF(a.left(), 4, a.width(), 18), Qt::AlignLeft, title_);
    g.setFont(font());
    double lx = a.left();
    for (int st = 0; st <= kStageCount; ++st) {
        const QString n = stageName(st);
        g.fillRect(QRectF(lx, 27, 10, 10), kStageColors[st]);
        g.drawText(QPointF(lx + 14, 36), n);
        lx += 22 + g.fontMetrics().horizontalAdvance(n);
    }
    g.drawText(QRectF(a.left(), a.bottom() + 22, a.width(), 16), Qt::AlignCenter, tr("작업일 (일요일 등 휴일 제외)"));
}

void DailyPlot::mouseMoveEvent(QMouseEvent* e) {
    const QRectF a = plotArea();
    if (days_.empty() || !a.contains(e->position())) {
        QToolTip::hideText();
        return;
    }
    const int k = std::clamp(static_cast<int>((e->position().x() - a.left()) / (a.width() / days_.size())), 0,
                             static_cast<int>(days_.size()) - 1);
    double sum = 0;
    QString lines;
    for (int st = 0; st <= kStageCount; ++st) {
        const double v = days_[k][st];
        sum += v;
        if (v > 0) lines += QStringLiteral("\n  %1: %2").arg(stageName(st), fmt(v));
    }
    QToolTip::showText(e->globalPosition().toPoint(),
                       QStringLiteral("%1   합계 %2 %3%4").arg(dates_[k], fmt(sum), unit_, lines), this);
}
