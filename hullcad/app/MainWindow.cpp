#include "MainWindow.h"

#include <QApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QLocale>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextBrowser>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <stdexcept>

#include "GzPlot.h"
#include "SvgView.h"
#include "YardForm.h"

using namespace hullcad;
using namespace hullcad::app;

namespace {

struct HullField {
    const char* label;
    double TankerParams::*member;
    double lo, hi;
    int decimals;
    const char* unit;
};

const HullField kHullFields[] = {
    {"Lpp (수선간장)", &TankerParams::Lpp, 50, 400, 2, "m"},
    {"B (형폭)", &TankerParams::B, 10, 80, 2, "m"},
    {"D (형깊이)", &TankerParams::D, 5, 40, 2, "m"},
    {"T (계획흘수)", &TankerParams::T, 2, 30, 2, "m"},
    {"빌지 반경", &TankerParams::bilge_radius, 0.5, 6, 2, "m"},
    {"평행부 후단 (x/Lpp)", &TankerParams::xi_pmb_aft, 0.1, 0.5, 3, ""},
    {"선수 어깨 (x/Lpp)", &TankerParams::xi_bow_shoulder, 0.55, 0.9, 3, ""},
    {"트랜섬 반폭비", &TankerParams::transom_half_breadth_ratio, 0.2, 1.0, 3, ""},
    {"선미 컷업 높이", &TankerParams::stern_cutup_height, 0, 15, 2, "m"},
    {"선미 단면 지수 (V형<2)", &TankerParams::n_aft, 1.1, 3.0, 2, ""},
    {"선수 단면 지수 (U형>2)", &TankerParams::n_fwd, 1.5, 4.0, 2, ""},
};

// Numeric cell: shown with thousands separators, sorted by value.
class NumItem : public QTableWidgetItem {
public:
    NumItem(double v, int decimals) : QTableWidgetItem(QLocale(QLocale::English).toString(v, 'f', decimals)) {
        setData(Qt::UserRole, v);
        setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    }
    bool operator<(const QTableWidgetItem& o) const override {
        return data(Qt::UserRole).toDouble() < o.data(Qt::UserRole).toDouble();
    }
};

QTableWidgetItem* num(double v, int decimals) { return new NumItem(v, decimals); }

QTableWidgetItem* text(const QString& s) { return new QTableWidgetItem(s); }

void setupTable(QTableWidget* t, const QStringList& headers) {
    t->setColumnCount(headers.size());
    t->setHorizontalHeaderLabels(headers);
    t->verticalHeader()->setVisible(false);
    t->setEditTriggers(QAbstractItemView::NoEditTriggers);
    t->setSelectionBehavior(QAbstractItemView::SelectRows);
    t->setAlternatingRowColors(true);
    t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    t->horizontalHeader()->setStretchLastSection(true);
}

QString checkLabel(const std::string& name) {
    static const QMap<QString, QString> ko = {
        {"cutting", "절단/강재"},     {"panel_line", "판넬라인"},     {"curved_shop", "곡블록"},
        {"medium_assembly", "중조립"}, {"assembly", "대조립"},        {"turnover", "반전"},
        {"painting", "도장"},         {"transporter", "트랜스포터"},  {"erection", "탑재"},
    };
    QString n = QString::fromStdString(name);
    QString stage = n.section('.', 0, 0);
    return ko.contains(stage) ? QStringLiteral("%1 (%2)").arg(ko[stage], n) : n;
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(tr("hullcad — 선형 · 유체정역학 · 블록 분할"));
    tabs_ = new QTabWidget(this);
    tabs_->addTab(buildHullTab(), tr("선형"));
    tabs_->addTab(buildHydroTab(), tr("유체정역학 · 복원성"));
    tabs_->addTab(buildDivisionTab(), tr("블록 분할"));
    tabs_->addTab(buildUtilityTab(), tr("유틸리티"));
    setCentralWidget(tabs_);
    buildMenus();
    connect(&hullWatch_, &QFutureWatcher<HullRun>::finished, this, &MainWindow::onHullDone);
    connect(&gzWatch_, &QFutureWatcher<std::vector<GZPoint>>::finished, this, [this] {
        if (!hull_) return;
        const double GM = hull_->design.KM_T - kgSpin_->value();
        gzPlot_->setCurve(gzWatch_.result(), GM);
    });
    connect(&divWatch_, &QFutureWatcher<DivisionRun>::finished, this, &MainWindow::onDivisionDone);
    statusBar()->showMessage(tr("준비 (CPU 스레드 %1개)").arg(threads()));
    resize(1500, 950);
}

MainWindow::~MainWindow() {
    hullWatch_.waitForFinished();
    gzWatch_.waitForFinished();
    divWatch_.waitForFinished();
}

int MainWindow::threads() const { return std::max(1, QThread::idealThreadCount()); }

void MainWindow::start() { runHull(); }

QWidget* MainWindow::buildHullTab() {
    auto* split = new QSplitter(Qt::Horizontal);
    auto* left = new QWidget;
    auto* lv = new QVBoxLayout(left);
    auto* box = new QGroupBox(tr("선형 파라미터 (120k 탱커 파라메트릭 선형)"));
    auto* form = new QFormLayout(box);
    TankerParams defaults;
    for (auto& f : kHullFields) {
        auto* s = new QDoubleSpinBox;
        s->setRange(f.lo, f.hi);
        s->setDecimals(f.decimals);
        s->setSingleStep(f.decimals >= 3 ? 0.01 : 0.5);
        if (*f.unit) s->setSuffix(QStringLiteral(" ") + QString::fromUtf8(f.unit));
        s->setValue(defaults.*(f.member));
        form->addRow(QString::fromUtf8(f.label), s);
        hullSpins_.push_back(s);
    }
    lv->addWidget(box);
    hullButton_ = new QPushButton(tr("선형 생성 · 계산"));
    hullButton_->setDefault(true);
    connect(hullButton_, &QPushButton::clicked, this, &MainWindow::runHull);
    lv->addWidget(hullButton_);
    particulars_ = new QTextBrowser;
    lv->addWidget(particulars_, 1);
    split->addWidget(left);
    linesView_ = new SvgView;
    split->addWidget(linesView_);
    split->setStretchFactor(1, 3);
    split->setSizes({360, 1100});
    return split;
}

QWidget* MainWindow::buildHydroTab() {
    auto* w = new QWidget;
    auto* v = new QVBoxLayout(w);
    auto* top = new QHBoxLayout;
    top->addWidget(new QLabel(tr("KG (가정값):")));
    kgSpin_ = new QDoubleSpinBox;
    kgSpin_->setRange(0, 40);
    kgSpin_->setDecimals(2);
    kgSpin_->setSuffix(QStringLiteral(" m"));
    kgSpin_->setValue(13.0);
    top->addWidget(kgSpin_);
    auto* gzBtn = new QPushButton(tr("GZ 다시 계산"));
    connect(gzBtn, &QPushButton::clicked, this, &MainWindow::runGz);
    top->addWidget(gzBtn);
    top->addWidget(new QLabel(tr("설계흘수 배수량 기준, 트림 고정")));
    top->addStretch(1);
    v->addLayout(top);
    auto* split = new QSplitter(Qt::Horizontal);
    hydroTable_ = new QTableWidget;
    setupTable(hydroTable_, {tr("흘수 m"), tr("배수량 t"), tr("LCB m"), tr("KB m"), tr("Awp m²"), tr("LCF m"),
                             tr("TPC t/cm"), tr("MCT t·m/cm"), tr("KMt m"), tr("KMl m"), tr("Cb"), tr("Cw")});
    hydroTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    split->addWidget(hydroTable_);
    gzPlot_ = new GzPlot;
    split->addWidget(gzPlot_);
    split->setSizes({850, 600});
    v->addWidget(split, 1);
    return w;
}

QWidget* MainWindow::buildDivisionTab() {
    auto* split = new QSplitter(Qt::Horizontal);

    auto* left = new QWidget;
    auto* lv = new QVBoxLayout(left);
    lv->setContentsMargins(6, 6, 6, 6);
    auto* fileRow = new QHBoxLayout;
    auto* openBtn = new QPushButton(tr("불러오기…"));
    auto* saveBtn = new QPushButton(tr("저장…"));
    auto* resetBtn = new QPushButton(tr("기본값"));
    connect(openBtn, &QPushButton::clicked, this, &MainWindow::openYard);
    connect(saveBtn, &QPushButton::clicked, this, &MainWindow::saveYard);
    connect(resetBtn, &QPushButton::clicked, this, &MainWindow::resetYard);
    fileRow->addWidget(new QLabel(tr("야드 프로파일")));
    fileRow->addStretch(1);
    fileRow->addWidget(openBtn);
    fileRow->addWidget(saveBtn);
    fileRow->addWidget(resetBtn);
    lv->addLayout(fileRow);
    yardForm_ = new YardForm;
    auto* scroll = new QScrollArea;
    scroll->setWidget(yardForm_);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);  // help text wraps instead
    lv->addWidget(scroll, 1);
    auto* buttsForm = new QFormLayout;
    buttsEdit_ = new QLineEdit;
    buttsEdit_->setPlaceholderText(tr("비우면 자동 분할, 예: 30.5, 50.5, 75.5"));
    buttsForm->addRow(tr("수동 버트 (FR)"), buttsEdit_);
    lv->addLayout(buttsForm);
    divButton_ = new QPushButton(tr("블록 분할 실행"));
    connect(divButton_, &QPushButton::clicked, this, &MainWindow::runDivision);
    lv->addWidget(divButton_);
    divStatus_ = new QLabel;
    divStatus_->setWordWrap(true);
    lv->addWidget(divStatus_);
    split->addWidget(left);

    auto* right = new QSplitter(Qt::Vertical);
    divView_ = new SvgView;
    right->addWidget(divView_);
    auto* results = new QTabWidget;
    blocksTable_ = new QTableWidget;
    setupTable(blocksTable_, {tr("블록"), tr("형식"), tr("FR 시작"), tr("FR 끝"), tr("L m"), tr("B m"), tr("H m"),
                              tr("인양 t"), tr("지배 검사"), tr("이용률 %"), tr("판정")});
    blocksTable_->setSortingEnabled(true);
    results->addTab(blocksTable_, tr("블록 목록"));
    checksTable_ = new QTableWidget;
    setupTable(checksTable_, {tr("검사 항목"), tr("최대 이용률 %"), tr("지배 블록 수"), tr("최악 블록")});
    results->addTab(checksTable_, tr("설비 병목"));
    issuesList_ = new QListWidget;
    results->addTab(issuesList_, tr("규칙 위반 · 진단"));
    right->addWidget(results);
    right->setSizes({560, 330});
    split->addWidget(right);
    split->setSizes({430, 1070});
    return split;
}

QWidget* MainWindow::buildUtilityTab() {
    auto* w = new QWidget;
    auto* v = new QVBoxLayout(w);
    auto* note = new QLabel(tr("유틸리티 사용량 = 블록별 작업 물량 × 원단위.  원단위는 <b>참고값</b>입니다. "
                               "'블록 분할' 탭의 야드 프로파일에서 [takeoff], [util_*] 항목을 야드 실측값으로 바꾸세요. "
                               "시간에 비례하는 기본 부하(조명, 공조, 압축기 공회전)는 일정 정보가 필요해 포함하지 않았습니다."));
    note->setWordWrap(true);
    v->addWidget(note);
    utilSummary_ = new QLabel;
    utilSummary_->setWordWrap(true);
    utilSummary_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    v->addWidget(utilSummary_);
    QStringList cols;
    for (auto& it : utility_items()) cols << QStringLiteral("%1 (%2)").arg(QString::fromUtf8(it.label), QString::fromUtf8(it.unit));
    auto* split = new QSplitter(Qt::Vertical);
    utilStageTable_ = new QTableWidget;
    setupTable(utilStageTable_, QStringList{tr("공정")} + cols);
    utilStageTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    split->addWidget(utilStageTable_);
    utilBlockTable_ = new QTableWidget;
    setupTable(utilBlockTable_, QStringList{tr("블록"), tr("강재 t"), tr("용착금속 kg")} + cols);
    utilBlockTable_->setSortingEnabled(true);
    utilBlockTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    split->addWidget(utilBlockTable_);
    split->setSizes({330, 520});
    v->addWidget(split, 1);
    return w;
}

void MainWindow::showUtilities(const DivisionRun& d) {
    const UtilityReport& u = d.utilities;
    const auto& items = utility_items();
    utilStageTable_->setRowCount(0);
    utilBlockTable_->setSortingEnabled(false);
    utilBlockTable_->setRowCount(0);
    if (u.blocks.empty()) {
        utilSummary_->setText(tr("블록 분할 결과가 없어 계산하지 않았습니다."));
        return;
    }
    const auto& Q = u.quantities;
    utilSummary_->setText(
        tr("<b>작업 물량</b>: 판 %1 m² · 도장 %2 m² (×%3회) · 절단 %4 m · 필릿 %5 m · 공장 맞대기 %6 m · 탑재 용접 %7 m · "
           "선상가열 %8 m · 용착금속 %9 t (FCAW %10 t, SAW %11 t) · 탱크 화물 %12 m³ / 밸러스트 %13 m³")
            .arg(Q.plate_area, 0, 'f', 0).arg(Q.paint_area, 0, 'f', 0).arg(u.coats)
            .arg(Q.cut_length, 0, 'f', 0).arg(Q.fillet_length, 0, 'f', 0).arg(Q.shop_butt_length, 0, 'f', 0)
            .arg(Q.erection_length, 0, 'f', 0).arg(Q.heating_length, 0, 'f', 0)
            .arg(u.weld_metal_kg / 1000, 0, 'f', 1).arg(u.fcaw_kg / 1000, 0, 'f', 1).arg(u.saw_kg / 1000, 0, 'f', 1)
            .arg(u.tanks.cargo_m3, 0, 'f', 0).arg(u.tanks.ballast_m3, 0, 'f', 0));

    auto addRow = [&](const QString& name, const UtilityUse& use, bool bold, double scale = 1.0, int dec = 0) {
        const int r = utilStageTable_->rowCount();
        utilStageTable_->insertRow(r);
        utilStageTable_->setItem(r, 0, text(name));
        for (int c = 0; c < static_cast<int>(items.size()); ++c)
            utilStageTable_->setItem(r, c + 1, num(use.*(items[c].member) * scale, dec));
        if (bold)
            for (int c = 0; c < utilStageTable_->columnCount(); ++c) {
                QFont f = utilStageTable_->item(r, c)->font();
                f.setBold(true);
                utilStageTable_->item(r, c)->setFont(f);
            }
    };
    for (int s = 0; s < kStageCount; ++s) addRow(QString::fromUtf8(stage_label(static_cast<Stage>(s))), u.by_stage[s], false);
    addRow(tr("합계"), u.total, true);
    if (u.steel_t > 0) addRow(tr("강재 1 t당"), u.total, false, 1.0 / u.steel_t, 2);

    utilBlockTable_->setRowCount(static_cast<int>(u.blocks.size()));
    for (int i = 0; i < static_cast<int>(u.blocks.size()); ++i) {
        const auto& b = u.blocks[i];
        utilBlockTable_->setItem(i, 0, text(QString::fromStdString(b.block)));
        utilBlockTable_->setItem(i, 1, num(b.steel_t, 1));
        utilBlockTable_->setItem(i, 2, num(b.weld_metal_kg, 0));
        for (int c = 0; c < static_cast<int>(items.size()); ++c)
            utilBlockTable_->setItem(i, c + 3, num(b.total.*(items[c].member), 0));
    }
    utilBlockTable_->setSortingEnabled(true);
}

void MainWindow::buildMenus() {
    auto* file = menuBar()->addMenu(tr("파일(&F)"));
    file->addAction(tr("야드 프로파일 열기…"), QKeySequence::Open, this, &MainWindow::openYard);
    file->addAction(tr("야드 프로파일 저장…"), QKeySequence::Save, this, &MainWindow::saveYard);
    file->addSeparator();
    file->addAction(tr("블록 목록 CSV 내보내기…"), this, &MainWindow::exportBlocksCsv);
    file->addAction(tr("유틸리티 CSV 내보내기…"), this, [this] {
        if (division_.utilities.blocks.empty()) return;
        const QString path = QFileDialog::getSaveFileName(this, tr("유틸리티 내보내기"), QStringLiteral("utilities.csv"), tr("CSV (*.csv)"));
        if (path.isEmpty()) return;
        try {
            write_utilities_csv(division_.utilities, QDir::toNativeSeparators(path).toStdString());
        } catch (const std::exception& e) {
            QMessageBox::warning(this, tr("내보내기 실패"), QString::fromUtf8(e.what()));
        }
    });
    file->addAction(tr("현재 도면 SVG 내보내기…"), this, &MainWindow::exportSvg);
    file->addAction(tr("선체 메쉬 OBJ 내보내기…"), this, &MainWindow::exportObj);
    file->addSeparator();
    file->addAction(tr("종료"), QKeySequence::Quit, qApp, &QApplication::quit);
    auto* help = menuBar()->addMenu(tr("도움말(&H)"));
    help->addAction(tr("hullcad 정보"), this, [this] {
        QMessageBox::about(this, tr("hullcad"),
                           tr("<b>hullcad</b><br>B-spline 선형, 유체정역학, 블록 분할<br>"
                              "Qt %1, %2, CPU 스레드 %3개")
                               .arg(QString::fromLatin1(qVersion()), QSysInfo::buildCpuArchitecture())
                               .arg(threads()));
    });
}

TankerParams MainWindow::hullParams() const {
    TankerParams p;
    for (int i = 0; i < hullSpins_.size(); ++i) p.*(kHullFields[i].member) = hullSpins_[i]->value();
    return p;
}

void MainWindow::runHull() {
    if (hullWatch_.isRunning()) return;
    hullButton_->setEnabled(false);
    divButton_->setEnabled(false);
    statusBar()->showMessage(tr("선형 생성 · 유체정역학 계산 중…"));
    const TankerParams p = hullParams();
    const double KG = kgSpin_->value();
    const int n = threads();
    hullWatch_.setFuture(QtConcurrent::run([p, KG, n] { return run_hull(p, KG, n); }));
}

void MainWindow::onHullDone() {
    hullButton_->setEnabled(true);
    divButton_->setEnabled(true);
    HullRun h = hullWatch_.result();
    if (!h.error.empty()) {
        statusBar()->showMessage(tr("선형 생성 실패"));
        QMessageBox::warning(this, tr("선형 생성 실패"), QString::fromStdString(h.error));
        return;
    }
    hull_ = std::make_shared<const HullRun>(std::move(h));
    showHull(*hull_);
    statusBar()->showMessage(tr("선형 계산 완료 (%1 s). 블록 분할 계산 중…").arg(hull_->seconds, 0, 'f', 2));
    runDivision();  // structure and blocks follow the hull
}

void MainWindow::showHull(const HullRun& h) {
    const auto& d = h.design;
    const auto& p = h.params;
    particulars_->setHtml(
        tr("<h3>주요 요목 (설계흘수)</h3><table cellspacing=4>"
           "<tr><td>Lpp × B × D × T</td><td>%1 × %2 × %3 × %4 m</td></tr>"
           "<tr><td>배수량</td><td>%5 t (%6 m³)</td></tr>"
           "<tr><td>Cb / Cw</td><td>%7 / %8</td></tr>"
           "<tr><td>LCB / LCF (midship 기준)</td><td>%9 / %10 m</td></tr>"
           "<tr><td>KB / KMt</td><td>%11 / %12 m</td></tr>"
           "<tr><td>TPC / MCT</td><td>%13 t/cm / %14 t·m/cm</td></tr>"
           "<tr><td>메쉬</td><td>삼각형 %15개</td></tr></table>")
            .arg(p.Lpp).arg(p.B).arg(p.D).arg(p.T)
            .arg(d.displacement, 0, 'f', 0).arg(d.volume, 0, 'f', 0)
            .arg(d.Cb, 0, 'f', 3).arg(d.Cw, 0, 'f', 3)
            .arg(d.B.x - 0.5 * p.Lpp, 0, 'f', 2).arg(d.F.x - 0.5 * p.Lpp, 0, 'f', 2)
            .arg(d.B.z, 0, 'f', 2).arg(d.KM_T, 0, 'f', 2)
            .arg(d.TPC, 0, 'f', 1).arg(d.MCT, 0, 'f', 0)
            .arg(h.mesh.tris.size()));
    linesView_->setSvg(QByteArray::fromStdString(h.lines_svg));

    hydroTable_->setSortingEnabled(false);
    hydroTable_->setRowCount(static_cast<int>(h.table.size()));
    for (int r = 0; r < static_cast<int>(h.table.size()); ++r) {
        const auto& t = h.table[r];
        const double vals[] = {t.draft, t.displacement, t.B.x - 0.5 * p.Lpp, t.B.z, t.Awp, t.F.x - 0.5 * p.Lpp,
                               t.TPC, t.MCT, t.KM_T, t.KM_L, t.Cb, t.Cw};
        const int dec[] = {2, 0, 2, 2, 0, 2, 1, 0, 2, 1, 3, 3};
        for (int c = 0; c < 12; ++c) hydroTable_->setItem(r, c, num(vals[c], dec[c]));
    }
    gzPlot_->setCurve(h.gz, d.KM_T - h.KG);
}

void MainWindow::runGz() {
    if (!hull_ || gzWatch_.isRunning()) return;
    auto hull = hull_;
    const double KG = kgSpin_->value();
    const int n = threads();
    gzWatch_.setFuture(QtConcurrent::run([hull, KG, n] { return run_gz(*hull, KG, n); }));
}

void MainWindow::runDivision() {
    if (!hull_ || divWatch_.isRunning()) return;
    YardProfile yard;
    std::vector<double> frames;
    try {
        yard = yardForm_->profile();
        for (const QString& t : buttsEdit_->text().split(QLatin1Char(','), Qt::SkipEmptyParts)) {
            bool ok = false;
            double f = t.trimmed().toDouble(&ok);
            if (!ok) throw std::invalid_argument("수동 버트 '" + t.trimmed().toStdString() + "'는 숫자가 아닙니다");
            frames.push_back(f);
        }
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("입력 오류"), QString::fromUtf8(e.what()));
        return;
    }
    divButton_->setEnabled(false);
    divStatus_->setText(tr("계산 중…"));
    auto hull = hull_;
    const int n = threads();
    divWatch_.setFuture(QtConcurrent::run([hull, yard, frames, n] { return run_division(*hull, yard, frames, n); }));
}

void MainWindow::onDivisionDone() {
    divButton_->setEnabled(true);
    division_ = divWatch_.result();
    showDivision(division_);
    showUtilities(division_);
    statusBar()->showMessage(tr("블록 분할 완료 (%1 s)").arg(division_.seconds, 0, 'f', 2));
    if (!screenshotDir_.isEmpty()) QTimer::singleShot(300, this, &MainWindow::takeScreenshots);
}

void MainWindow::showDivision(const DivisionRun& r) {
    const Division& d = r.division;
    issuesList_->clear();
    if (!r.error.empty()) {
        divStatus_->setText(tr("<span style='color:#b00020'>오류: %1</span>").arg(QString::fromStdString(r.error)));
        return;
    }
    int flat = 0;
    for (auto& b : d.blocks) flat += b.flat;
    if (d.butts.empty()) {
        divStatus_->setText(tr("<span style='color:#b00020'><b>분할 불가</b> — 원인은 '규칙 위반 · 진단' 탭을 보세요.</span>"));
        divView_->clear();
    } else {
        divStatus_->setText(tr("%1: 링 %2개, 블록 %3개 (평 %4 / 곡 %5), 강재 %6 t%7")
                                .arg(r.manual ? tr("수동 분할") : tr("자동 분할"))
                                .arg(d.butts.size() - 1).arg(d.blocks.size()).arg(flat).arg(d.blocks.size() - flat)
                                .arg(r.steel_total.w, 0, 'f', 0)
                                .arg(d.issues.empty() ? tr(" — 모든 검사 통과") : tr(" — <span style='color:#b00020'>규칙 위반 %1건</span>").arg(d.issues.size())));
        divView_->setSvg(QByteArray::fromStdString(r.svg));
    }
    for (auto& s : d.issues) issuesList_->addItem(QString::fromStdString(s));
    if (d.issues.empty()) issuesList_->addItem(tr("위반 없음"));

    blocksTable_->setSortingEnabled(false);
    blocksTable_->setRowCount(static_cast<int>(d.blocks.size()));
    for (int i = 0; i < static_cast<int>(d.blocks.size()); ++i) {
        const Block& b = d.blocks[i];
        const Check* bind = b.binding();
        blocksTable_->setItem(i, 0, text(QString::fromStdString(b.name)));
        blocksTable_->setItem(i, 1, text(b.flat ? tr("평") : tr("곡")));
        blocksTable_->setItem(i, 2, num(b.fr0, 1));
        blocksTable_->setItem(i, 3, num(b.fr1, 1));
        blocksTable_->setItem(i, 4, num(b.L, 2));
        blocksTable_->setItem(i, 5, num(b.B, 2));
        blocksTable_->setItem(i, 6, num(b.H, 2));
        blocksTable_->setItem(i, 7, num(b.lift_weight, 1));
        blocksTable_->setItem(i, 8, text(bind ? checkLabel(bind->name) : QString()));
        blocksTable_->setItem(i, 9, num(bind ? 100 * bind->ratio() : 0.0, 1));
        blocksTable_->setItem(i, 10, text(b.ok() ? tr("OK") : tr("위반")));
        if (bind) blocksTable_->item(i, 8)->setToolTip(QString::fromStdString(bind->detail));
        if (!b.ok())
            for (int c = 0; c < blocksTable_->columnCount(); ++c) blocksTable_->item(i, c)->setForeground(QColor(0xb0, 0x00, 0x20));
    }
    blocksTable_->setSortingEnabled(true);

    checksTable_->setRowCount(static_cast<int>(r.summary.size()));
    for (int i = 0; i < static_cast<int>(r.summary.size()); ++i) {
        const auto& s = r.summary[i];
        checksTable_->setItem(i, 0, text(checkLabel(s.name)));
        checksTable_->setItem(i, 1, num(100 * s.max_ratio, 1));
        checksTable_->setItem(i, 2, num(s.binding_blocks, 0));
        checksTable_->setItem(i, 3, text(QString::fromStdString(s.worst_block)));
        if (s.max_ratio > 1.0) checksTable_->item(i, 1)->setForeground(QColor(0xb0, 0x00, 0x20));
    }
}

bool MainWindow::loadYard(const QString& path) {
    try {
        YardProfile y = load_yard_profile(QDir::toNativeSeparators(path).toStdString());
        yardForm_->setProfile(y);
        yardPath_ = path;
        statusBar()->showMessage(tr("야드 프로파일: %1").arg(path));
        return true;
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("야드 프로파일 오류"), QString::fromUtf8(e.what()));
        return false;
    }
}

void MainWindow::openYard() {
    const QString path = QFileDialog::getOpenFileName(this, tr("야드 프로파일 열기"), yardPath_, tr("야드 프로파일 (*.ini);;모든 파일 (*)"));
    if (!path.isEmpty() && loadYard(path)) runDivision();
}

void MainWindow::saveYard() {
    YardProfile y;
    try {
        y = yardForm_->profile();
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("입력 오류"), QString::fromUtf8(e.what()));
        return;
    }
    const QString path = QFileDialog::getSaveFileName(this, tr("야드 프로파일 저장"),
                                                      yardPath_.isEmpty() ? QStringLiteral("my_yard.ini") : yardPath_,
                                                      tr("야드 프로파일 (*.ini)"));
    if (path.isEmpty()) return;
    try {
        write_yard_profile(y, QDir::toNativeSeparators(path).toStdString());
        yardPath_ = path;
        statusBar()->showMessage(tr("저장됨: %1").arg(path));
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("저장 실패"), QString::fromUtf8(e.what()));
    }
}

void MainWindow::resetYard() { yardForm_->setProfile(YardProfile{}); }

void MainWindow::exportBlocksCsv() {
    if (division_.division.blocks.empty()) return;
    const QString path = QFileDialog::getSaveFileName(this, tr("블록 목록 내보내기"), QStringLiteral("blocks.csv"), tr("CSV (*.csv)"));
    if (path.isEmpty()) return;
    try {
        write_division_csv(division_.division, QDir::toNativeSeparators(path).toStdString());
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("내보내기 실패"), QString::fromUtf8(e.what()));
    }
}

void MainWindow::exportSvg() {
    SvgView* view = tabs_->currentIndex() == 0 ? linesView_ : divView_;
    if (view->svg().isEmpty()) return;
    const QString path = QFileDialog::getSaveFileName(this, tr("도면 내보내기"),
                                                      tabs_->currentIndex() == 0 ? QStringLiteral("lines.svg") : QStringLiteral("block_division.svg"),
                                                      tr("SVG (*.svg)"));
    if (path.isEmpty()) return;
    QFile f(path);
    if (f.open(QIODevice::WriteOnly)) f.write(view->svg());
}

void MainWindow::exportObj() {
    if (!hull_) return;
    const QString path = QFileDialog::getSaveFileName(this, tr("메쉬 내보내기"), QStringLiteral("hull.obj"), tr("OBJ (*.obj)"));
    if (path.isEmpty()) return;
    try {
        write_obj(hull_->mesh, QDir::toNativeSeparators(path).toStdString());
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("내보내기 실패"), QString::fromUtf8(e.what()));
    }
}

void MainWindow::takeScreenshots() {
    QDir().mkpath(screenshotDir_);
    for (int i = 0; i < tabs_->count(); ++i) {
        tabs_->setCurrentIndex(i);
        QApplication::processEvents();
        grab().save(QDir(screenshotDir_).filePath(QStringLiteral("tab%1.png").arg(i + 1)));
    }
    QApplication::quit();
}
