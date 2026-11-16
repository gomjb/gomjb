#include "MainWindow.h"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QSpinBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
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

#include "CombPlot.h"
#include "NetworkPlot.h"
#include "DailyPlot.h"
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

void MainWindow::setBilgeG2(bool g2) { bilgeCombo_->setCurrentIndex(g2 ? 1 : 0); }

void MainWindow::setNetworkFairing(bool on) {
    if (on) fairSurfaceCheck_->setChecked(true);
    networkCheck_->setChecked(on);
}

bool MainWindow::loadOffsets(const QString& path) {
    try {
        auto t = std::make_shared<OffsetTable>(read_offsets_csv(QDir::toNativeSeparators(path).toStdString()));
        const OffsetTableInfo info = describe(*t);
        table_ = t;
        fairSurfaceCheck_->setChecked(true);  // measured / entered offsets: build from the faired sections
        networkCheck_->setChecked(true);      // and fair the girth lines (stations entered one by one)
        bilgeCombo_->setEnabled(false);
        sourceLabel_->setText(tr("<b>가져온 오프셋</b>: %1<br>스테이션 %2개 × 점 %3~%4개, Lpp %5 m, 최대 반폭 %6 m, 최대 높이 %7 m. "
                                 "주요 치수(Lpp, B, D)는 오프셋에서 정합니다.")
                                  .arg(QFileInfo(path).fileName()).arg(info.stations).arg(info.min_points).arg(info.max_points)
                                  .arg(info.length, 0, 'f', 2).arg(info.max_half_breadth, 0, 'f', 2).arg(info.max_height, 0, 'f', 2));
        statusBar()->showMessage(tr("오프셋 테이블: %1").arg(path));
        return true;
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("오프셋 테이블 오류"), QString::fromUtf8(e.what()));
        return false;
    }
}

void MainWindow::useParametric() {
    table_.reset();
    fairSurfaceCheck_->setChecked(false);
    networkCheck_->setChecked(false);
    bilgeCombo_->setEnabled(true);
    sourceLabel_->setText(tr("파라메트릭 선형 (위 파라미터)"));
}

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
    auto* fbox = new QGroupBox(tr("빌지 형상 · 단면 순정"));
    auto* fform = new QFormLayout(fbox);
    bilgeCombo_ = new QComboBox;
    bilgeCombo_->addItem(tr("원호 빌지 (G1: 곡률이 0 → 1/R로 점프)"));
    bilgeCombo_->addItem(tr("곡률 연속 빌지 (G2: n = 4, 같은 단면적)"));
    bilgeCombo_->setToolTip(tr("곡률 연속 빌지는 평저·평측과 만나는 곳에서 곡률이 0부터 서서히 커집니다.\n"
                               "원호 빌지와 같은 단면적을 빼도록 크기를 맞춥니다."));
    fform->addRow(tr("빌지"), bilgeCombo_);
    fairTolSpin_ = new QDoubleSpinBox;
    fairTolSpin_->setRange(0.5, 50);
    fairTolSpin_->setDecimals(1);
    fairTolSpin_->setSuffix(QStringLiteral(" mm"));
    fairTolSpin_->setValue(3.0);
    fform->addRow(tr("순정 허용 오차"), fairTolSpin_);
    fairSurfaceCheck_ = new QCheckBox(tr("순정 단면으로 곡면 생성"));
    fairSurfaceCheck_->setToolTip(tr("파라메트릭 선형처럼 오차 없는 단면은 원래 단면으로 곡면을 만드는 편이 더 정확합니다.\n"
                                     "측정·입력 오차가 있는 오프셋에 사용하세요."));
    fform->addRow(QString(), fairSurfaceCheck_);
    networkCheck_ = new QCheckBox(tr("곡선망 순정 (거스 라인 · 평행부 직선)"));
    networkCheck_->setToolTip(tr("같은 거스 위치의 점을 배 길이 방향으로 이어 순정합니다. 평행 중앙부는 정확히 직선이 되고\n"
                                 "선수미부는 G2로 이어집니다. 스테이션마다 따로 입력·측정한 오프셋의 단차(줄무늬)를 없앱니다."));
    networkCheck_->setEnabled(false);
    connect(fairSurfaceCheck_, &QCheckBox::toggled, networkCheck_, &QCheckBox::setEnabled);
    fform->addRow(QString(), networkCheck_);
    networkTolSpin_ = new QDoubleSpinBox;
    networkTolSpin_->setRange(0.5, 50);
    networkTolSpin_->setDecimals(1);
    networkTolSpin_->setSuffix(QStringLiteral(" mm"));
    networkTolSpin_->setValue(5.0);
    networkTolSpin_->setToolTip(tr("곡선망 순정이 단면점을 옮길 수 있는 최대 거리. 스테이션별 오차보다 크게 잡으세요."));
    networkTolSpin_->setEnabled(false);
    connect(networkCheck_, &QCheckBox::toggled, networkTolSpin_, &QDoubleSpinBox::setEnabled);
    fform->addRow(tr("곡선망 허용 오차"), networkTolSpin_);
    accuracySpin_ = new QDoubleSpinBox;
    accuracySpin_->setRange(0, 20);
    accuracySpin_->setDecimals(1);
    accuracySpin_->setSuffix(QStringLiteral(" mm"));
    accuracySpin_->setValue(1.0);
    accuracySpin_->setToolTip(tr("가져온 오프셋의 예상 오차(측정·입력·반올림). 평저·평측 직선 구간을 찾을 때 씁니다."));
    fform->addRow(tr("오프셋 정밀도"), accuracySpin_);
    lv->addWidget(fbox);

    auto* srcBox = new QGroupBox(tr("선형 출처"));
    auto* sv = new QVBoxLayout(srcBox);
    sourceLabel_ = new QLabel(tr("파라메트릭 선형 (위 파라미터)"));
    sourceLabel_->setWordWrap(true);
    sv->addWidget(sourceLabel_);
    auto* srcRow = new QHBoxLayout;
    auto* importBtn = new QPushButton(tr("오프셋 불러오기…"));
    auto* paramBtn = new QPushButton(tr("파라메트릭 선형 사용"));
    connect(importBtn, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getOpenFileName(this, tr("오프셋 테이블 열기"), QString(),
                                                          tr("오프셋 테이블 (*.csv *.txt);;모든 파일 (*)"));
        if (!path.isEmpty() && loadOffsets(path)) runHull();
    });
    connect(paramBtn, &QPushButton::clicked, this, [this] {
        useParametric();
        runHull();
    });
    srcRow->addWidget(importBtn);
    srcRow->addWidget(paramBtn);
    sv->addLayout(srcRow);
    lv->addWidget(srcBox);
    hullButton_ = new QPushButton(tr("선형 생성 · 계산"));
    hullButton_->setDefault(true);
    connect(hullButton_, &QPushButton::clicked, this, &MainWindow::runHull);
    lv->addWidget(hullButton_);
    particulars_ = new QTextBrowser;
    lv->addWidget(particulars_, 1);
    split->addWidget(left);
    auto* right = new QTabWidget;
    right->setObjectName(QStringLiteral("innerTabs"));
    linesView_ = new SvgView;
    right->addTab(linesView_, tr("선도"));

    auto* fw = new QWidget;
    auto* fv = new QVBoxLayout(fw);
    fairSummary_ = new QLabel;
    fairSummary_->setWordWrap(true);
    fv->addWidget(fairSummary_);
    auto* row = new QHBoxLayout;
    row->addWidget(new QLabel(tr("스테이션")));
    stationSpin_ = new QSpinBox;
    connect(stationSpin_, &QSpinBox::valueChanged, this, [this](int i) {
        if (!hull_ || i < 0 || i >= static_cast<int>(hull_->fairing.sections.size())) return;
        combPlot_->setSection(hull_->fairing.sections[i]);
        fairTable_->selectRow(i);
    });
    row->addWidget(stationSpin_);
    auto* showBefore = new QCheckBox(tr("순정 전"));
    showBefore->setChecked(true);
    connect(showBefore, &QCheckBox::toggled, this, [this](bool on) { combPlot_->setShowBefore(on); });
    auto* showAfter = new QCheckBox(tr("순정 후"));
    showAfter->setChecked(true);
    connect(showAfter, &QCheckBox::toggled, this, [this](bool on) { combPlot_->setShowAfter(on); });
    row->addWidget(showBefore);
    row->addWidget(showAfter);
    row->addWidget(new QLabel(tr("휠: 확대 · 드래그: 이동 · 더블클릭: 맞춤")));
    row->addStretch(1);
    fv->addLayout(row);
    auto* fsplit = new QSplitter(Qt::Vertical);
    combPlot_ = new CombPlot;
    fsplit->addWidget(combPlot_);
    fairTable_ = new QTableWidget;
    setupTable(fairTable_, {tr("스테이션"), tr("x (m)"), tr("최대 편차 (mm)"), tr("변곡점 순정 전"), tr("변곡점 순정 후"),
                            tr("곡률 변화 에너지 전"), tr("곡률 변화 에너지 후"), tr("상태")});
    fairTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    connect(fairTable_, &QTableWidget::cellClicked, this, [this](int r, int) { stationSpin_->setValue(r); });
    fsplit->addWidget(fairTable_);
    fsplit->setSizes({560, 260});
    fv->addWidget(fsplit, 1);
    right->addTab(fw, tr("단면 순정 (곡률 콤)"));

    auto* nw = new QWidget;
    auto* nv = new QVBoxLayout(nw);
    networkSummary_ = new QLabel;
    networkSummary_->setWordWrap(true);
    nv->addWidget(networkSummary_);
    auto* nrow = new QHBoxLayout;
    nrow->addWidget(new QLabel(tr("검사 곡선")));
    networkCurve_ = new QComboBox;
    connect(networkCurve_, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (!hull_ || i < 0 || i >= static_cast<int>(hull_->network.curves.size())) return;
        longPlot_->setCurve(hull_->network.curves[i], hull_->network_faired);
        networkTable_->selectRow(i);
    });
    nrow->addWidget(networkCurve_);
    const QString setNames[3] = {tr("원래"), tr("단면 순정"), tr("곡선망 순정")};
    for (int k = 0; k < 3; ++k) {
        auto* cb = new QCheckBox(setNames[k]);
        cb->setChecked(k != 0);
        connect(cb, &QCheckBox::toggled, this, [this, k](bool on) { longPlot_->setShow(k, on); });
        nrow->addWidget(cb);
    }
    nrow->addWidget(new QLabel(tr("휠: x 확대 · 드래그: 이동 · 더블클릭: 전체")));
    nrow->addStretch(1);
    nv->addLayout(nrow);
    auto* nsplit = new QSplitter(Qt::Vertical);
    longPlot_ = new LongitudinalPlot;
    longPlot_->setShow(0, false);
    nsplit->addWidget(longPlot_);
    networkTable_ = new QTableWidget;
    setupTable(networkTable_, {tr("곡선"), tr("점"), tr("변곡점 원래"), tr("변곡점 단면 순정"), tr("변곡점 곡선망"),
                               tr("곡률 변동량 원래"), tr("곡률 변동량 단면 순정"), tr("곡률 변동량 곡선망")});
    networkTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    connect(networkTable_, &QTableWidget::cellClicked, this, [this](int r, int) { networkCurve_->setCurrentIndex(r); });
    nsplit->addWidget(networkTable_);
    nsplit->setSizes({560, 260});
    nv->addWidget(nsplit, 1);
    right->addTab(nw, tr("곡선망 검사 (수선·버톡)"));

    auto* cw = new QWidget;
    auto* cv = new QVBoxLayout(cw);
    curvSummary_ = new QLabel;
    curvSummary_->setWordWrap(true);
    cv->addWidget(curvSummary_);
    auto* crow = new QHBoxLayout;
    auto* longBtn = new QCheckBox(tr("거스 방향 곡률 보기"));
    connect(longBtn, &QCheckBox::toggled, this, [this](bool on) { curvMap_->setGirth(on); });
    crow->addWidget(longBtn);
    crow->addStretch(1);
    cv->addLayout(crow);
    curvMap_ = new CurvatureMapView;
    cv->addWidget(curvMap_, 1);
    right->addTab(cw, tr("곡면 곡률 지도"));
    split->addWidget(right);
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
                               "시간에 비례하는 기본 부하(조명, 공조, 압축기 공회전)는 [capacity]의 base_* 항목으로 일별 부하에만 더해집니다."));
    note->setWordWrap(true);
    v->addWidget(note);
    utilSummary_ = new QLabel;
    utilSummary_->setWordWrap(true);
    utilSummary_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    v->addWidget(utilSummary_);
    QStringList cols;
    for (auto& it : utility_items()) cols << QStringLiteral("%1 (%2)").arg(QString::fromUtf8(it.label), QString::fromUtf8(it.unit));
    auto* inner = new QTabWidget;
    inner->setObjectName(QStringLiteral("innerTabs"));
    v->addWidget(inner, 1);
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
    inner->addTab(split, tr("합계 (공정별 · 블록별)"));

    // Daily load: stacked chart per utility, peak / capacity table.
    auto* daily = new QWidget;
    auto* dv = new QVBoxLayout(daily);
    scheduleSummary_ = new QLabel;
    scheduleSummary_->setWordWrap(true);
    dv->addWidget(scheduleSummary_);
    auto* pickRow = new QHBoxLayout;
    pickRow->addWidget(new QLabel(tr("유틸리티:")));
    dailyItem_ = new QComboBox;
    for (auto& it : utility_items()) dailyItem_->addItem(QString::fromUtf8(it.label));
    connect(dailyItem_, &QComboBox::currentIndexChanged, this, &MainWindow::showDailyPlot);
    pickRow->addWidget(dailyItem_);
    pickRow->addWidget(new QLabel(tr("공급 능력·기본 부하·일정은 '블록 분할' 탭의 [capacity], [schedule] 항목에서 입력합니다.")));
    pickRow->addStretch(1);
    dv->addLayout(pickRow);
    auto* dsplit = new QSplitter(Qt::Vertical);
    dailyPlot_ = new DailyPlot;
    dsplit->addWidget(dailyPlot_);
    peaksTable_ = new QTableWidget;
    setupTable(peaksTable_, {tr("유틸리티"), tr("피크일 사용량"), tr("피크일"), tr("일평균"), tr("피크 시간당"),
                             tr("공급 능력"), tr("이용률 %"), tr("초과 일수"), tr("피크일 주요 공정"), tr("피크일 최대 항목")});
    dsplit->addWidget(peaksTable_);
    dsplit->setSizes({480, 300});
    dv->addWidget(dsplit, 1);
    inner->addTab(daily, tr("일별 부하 · 피크"));

    scheduleTable_ = new QTableWidget;
    setupTable(scheduleTable_, {tr("순서"), tr("블록"), tr("탑재일"), tr("절단"), tr("곡가공"), tr("조립"), tr("도장"),
                                tr("탑재 용접")});
    scheduleTable_->setSortingEnabled(true);
    inner->addTab(scheduleTable_, tr("블록 일정"));
    return w;
}

void MainWindow::showSchedule(const DivisionRun& d) {
    const ProductionSchedule& s = d.schedule;  // planning
    const DailyUtilityProfile& p = d.daily;    // production
    peaksTable_->setRowCount(0);
    scheduleTable_->setSortingEnabled(false);
    scheduleTable_->setRowCount(0);
    if (!d.schedule_error.empty()) {
        scheduleSummary_->setText(tr("<span style='color:#b00020'>일정 오류: %1</span>").arg(QString::fromUtf8(d.schedule_error.c_str())));
        dailyPlot_->clear();
        return;
    }
    if (s.blocks.empty()) {
        scheduleSummary_->setText(tr("블록 분할 결과가 없어 일정을 만들지 않았습니다."));
        dailyPlot_->clear();
        return;
    }
    const int last_erection = s.last_erection_wd;
    int over = 0;
    for (auto& pk : p.peaks) over += pk.capacity > 0 && pk.days_over > 0;
    scheduleSummary_->setText(
        tr("첫 절단 %1 · 탑재 %2 ~ %3 (블록 %4개, 하루 %5개) · 탱크 시험 %6 ~ %7 · 작업일 %8일 · 시간당 환산: %9시간/일 × 동시계수 %10%11")
            .arg(QString::fromStdString(s.calendar.date(p.first_wd)), QString::fromStdString(s.calendar.date(0)),
                 QString::fromStdString(s.calendar.date(last_erection)))
            .arg(s.blocks.size()).arg(d.yard.schedule.erection_blocks_per_day)
            .arg(QString::fromStdString(s.calendar.date(s.testing.start)), QString::fromStdString(s.calendar.date(s.testing.end - 1)))
            .arg(p.days()).arg(d.yard.capacity.hours_per_day).arg(d.yard.capacity.peak_factor)
            .arg(over ? tr(" — <span style='color:#b00020'><b>공급 능력 초과 %1건</b></span>").arg(over) : QString()));

    peaksTable_->setRowCount(static_cast<int>(p.peaks.size()));
    for (int i = 0; i < static_cast<int>(p.peaks.size()); ++i) {
        const PeakInfo& pk = p.peaks[i];
        const QString unit = QString::fromUtf8(utility_items()[i].unit);
        const QString ru = QString::fromStdString(pk.rate_unit);
        peaksTable_->setItem(i, 0, text(QStringLiteral("%1 (%2)").arg(QString::fromUtf8(pk.label.c_str()), unit)));
        peaksTable_->setItem(i, 1, num(pk.daily_peak, 0));
        peaksTable_->setItem(i, 2, text(pk.daily_peak > 0 ? QString::fromStdString(s.calendar.date(pk.peak_wd)) : QString()));
        peaksTable_->setItem(i, 3, num(pk.daily_mean, 0));
        peaksTable_->setItem(i, 4, text(QStringLiteral("%1 %2").arg(QLocale(QLocale::English).toString(pk.peak_rate, 'f', 1), ru)));
        peaksTable_->setItem(i, 5, text(pk.capacity > 0 ? QStringLiteral("%1 %2").arg(QLocale(QLocale::English).toString(pk.capacity, 'f', 1), ru) : tr("-")));
        peaksTable_->setItem(i, 6, pk.capacity > 0 ? num(100 * pk.utilization, 0) : text(tr("-")));
        peaksTable_->setItem(i, 7, pk.capacity > 0 ? num(pk.days_over, 0) : text(tr("-")));
        QString dom;
        if (pk.dominant_stage >= 0)
            dom = QStringLiteral("%1 %2%")
                      .arg(pk.dominant_stage < kStageCount ? QString::fromUtf8(stage_label(static_cast<Stage>(pk.dominant_stage))) : tr("기본 부하"))
                      .arg(std::lround(100 * pk.stage_share[pk.dominant_stage]));
        peaksTable_->setItem(i, 8, text(dom));
        peaksTable_->setItem(i, 9, text(pk.contributors.empty() ? QString() : QString::fromUtf8(pk.contributors.front().first.c_str())));
        if (pk.capacity > 0 && pk.days_over > 0)
            for (int c = 0; c < peaksTable_->columnCount(); ++c) peaksTable_->item(i, c)->setForeground(QColor(0xb0, 0x00, 0x20));
    }

    auto span = [&](const StageSpan& sp) {
        if (sp.days() <= 0) return QString();
        return QStringLiteral("%1 ~ %2").arg(QString::fromStdString(s.calendar.date(sp.start)).mid(5),
                                              QString::fromStdString(s.calendar.date(sp.end - 1)).mid(5));
    };
    scheduleTable_->setRowCount(static_cast<int>(s.blocks.size()));
    for (int i = 0; i < static_cast<int>(s.blocks.size()); ++i) {
        const BlockSchedule& b = s.blocks[i];
        auto st = [&](Stage x) { return b.span[static_cast<int>(x)]; };
        scheduleTable_->setItem(i, 0, num(b.erection_order, 0));
        scheduleTable_->setItem(i, 1, text(QString::fromStdString(b.block)));
        scheduleTable_->setItem(i, 2, text(QString::fromStdString(s.calendar.date(b.erection_wd))));
        scheduleTable_->setItem(i, 3, text(span(st(Stage::Cutting))));
        scheduleTable_->setItem(i, 4, text(span(st(Stage::Heating))));
        scheduleTable_->setItem(i, 5, text(span(st(Stage::Welding))));
        scheduleTable_->setItem(i, 6, text(span(st(Stage::Painting))));
        scheduleTable_->setItem(i, 7, text(span(st(Stage::Erection))));
    }
    scheduleTable_->setSortingEnabled(true);
    scheduleTable_->sortItems(0);
    showDailyPlot();
}

void MainWindow::showDailyPlot() {
    const DailyUtilityProfile& s = division_.daily;
    const int item = dailyItem_->currentIndex();
    if (s.total.empty() || item < 0 || item >= static_cast<int>(s.peaks.size())) {
        dailyPlot_->clear();
        return;
    }
    const PeakInfo& pk = s.peaks[item];
    // Capacity is an hourly rate; the chart is per day: rate x hours / simultaneity factor.
    const auto& cap = division_.yard.capacity;
    double per_day = 0;
    if (pk.capacity > 0)
        per_day = pk.capacity * (pk.key == "air_nm3" ? 60.0 : 1.0) * cap.hours_per_day / std::max(cap.peak_factor, 1e-9);
    dailyPlot_->setData(s, item, per_day, QString::fromUtf8(utility_items()[item].unit));
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
    file->addAction(tr("오프셋 테이블 불러오기…"), this, [this] {
        const QString path = QFileDialog::getOpenFileName(this, tr("오프셋 테이블 열기"), QString(),
                                                          tr("오프셋 테이블 (*.csv *.txt);;모든 파일 (*)"));
        if (!path.isEmpty() && loadOffsets(path)) runHull();
    });
    file->addAction(tr("오프셋 테이블 내보내기 (곡면 단면)…"), this, [this] {
        if (!hull_) return;
        const QString path = QFileDialog::getSaveFileName(this, tr("오프셋 테이블 저장"), QStringLiteral("offsets.csv"), tr("CSV (*.csv)"));
        if (path.isEmpty()) return;
        OffsetTable used;
        for (auto& sc : hull_->fairing.sections) used.stations.push_back(hull_->faired_surface ? sc.faired : sc.offsets);
        try {
            write_offsets_csv(used, QDir::toNativeSeparators(path).toStdString());
        } catch (const std::exception& e) {
            QMessageBox::warning(this, tr("내보내기 실패"), QString::fromUtf8(e.what()));
        }
    });
    file->addSeparator();
    file->addAction(tr("야드 프로파일 열기…"), QKeySequence::Open, this, &MainWindow::openYard);
    file->addAction(tr("야드 프로파일 저장…"), QKeySequence::Save, this, &MainWindow::saveYard);
    file->addSeparator();
    file->addAction(tr("블록 목록 CSV 내보내기…"), this, &MainWindow::exportBlocksCsv);
    file->addAction(tr("일별 유틸리티·블록 일정 CSV 내보내기…"), this, [this] {
        if (division_.schedule.blocks.empty()) return;
        const QString dir = QFileDialog::getExistingDirectory(this, tr("저장할 폴더"));
        if (dir.isEmpty()) return;
        try {
            write_daily_utilities_csv(division_.daily, QDir::toNativeSeparators(QDir(dir).filePath(QStringLiteral("daily_utilities.csv"))).toStdString());
            write_block_schedule_csv(division_.schedule, QDir::toNativeSeparators(QDir(dir).filePath(QStringLiteral("block_schedule.csv"))).toStdString());
            statusBar()->showMessage(tr("저장됨: %1").arg(dir));
        } catch (const std::exception& e) {
            QMessageBox::warning(this, tr("내보내기 실패"), QString::fromUtf8(e.what()));
        }
    });
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
    p.n_mid = bilgeCombo_->currentIndex() == 1 ? 4.0 : 2.0;
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
    FairingOptions fo;
    fo.tolerance = fairTolSpin_->value() / 1000.0;
    fo.junction_continuity = (!table_ && p.n_mid > 2.5) ? 2 : 1;  // G2 joints only where the design bilge is G2
    fo.offset_accuracy = table_ ? accuracySpin_->value() / 1000.0 : 0.0;
    const bool fair_surface = fairSurfaceCheck_->isChecked();
    const double network = fair_surface && networkCheck_->isChecked() ? networkTolSpin_->value() / 1000.0 : 0.0;
    auto table = table_;
    hullWatch_.setFuture(QtConcurrent::run([p, KG, n, fo, fair_surface, table, network] {
        return run_hull(p, KG, n, fo, fair_surface, table, network);
    }));
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
            .arg(h.mesh.tris.size())
        + (h.offsets ? tr("<p>선형: 가져온 오프셋 (%1), 곡면: %2 단면</p>").arg(QFileInfo(QString::fromStdString(h.offsets->source)).fileName(),
                                                                       h.faired_surface ? tr("순정") : tr("원래"))
                   : QString()));
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

    // Section fairing.
    const auto& secs = h.fairing.sections;
    const FairingSummary fs = summarize(h.fairing);
    fairSummary_->setText(
        tr("스테이션 %1개 · 순정 적용 %2개 (적용 불가 %3개: 선수재처럼 바닥 반경이 수 mm인 단면) · 최대 편차 %4 mm (허용 %5 mm) · "
           "변곡점 합계 %6 → %7 · 접속 조건 %8 · 곡면: %9%10")
            .arg(fs.stations).arg(fs.stations - fs.not_applicable).arg(fs.not_applicable)
            .arg(fs.max_deviation * 1000, 0, 'f', 1).arg(h.fairing.options.tolerance * 1000, 0, 'f', 1)
            .arg(fs.inflections_before).arg(fs.inflections_faired)
            .arg(h.fairing.options.junction_continuity >= 2 ? tr("G2 (곡률 연속)") : tr("G1 (접선 연속)"))
            .arg(h.faired_surface ? tr("순정 단면 사용") : tr("원래 단면 사용"))
            .arg(h.offsets || h.params.n_mid > 2.5 ? QString()
                                      : tr("<br><span style='color:#8a5a00'>원호 빌지는 평저·평측과 만나는 곳에서 곡률이 0 → 1/R로 계단처럼 뜁니다(G1). "
                                           "순정은 이 계단을 설계대로 유지하므로 곡률 변화 에너지가 큽니다. 2차 미분까지 매끄럽게 하려면 "
                                           "'곡률 연속 빌지 (G2)'를 선택하세요.</span>")));
    fairTable_->setRowCount(static_cast<int>(secs.size()));
    for (int i = 0; i < static_cast<int>(secs.size()); ++i) {
        const auto& sc = secs[i];
        fairTable_->setItem(i, 0, num(i, 0));
        fairTable_->setItem(i, 1, num(sc.x, 1));
        fairTable_->setItem(i, 2, num(sc.max_deviation * 1000, 1));
        fairTable_->setItem(i, 3, num(sc.q_before.inflections, 0));
        fairTable_->setItem(i, 4, num(sc.q_composite.inflections, 0));
        fairTable_->setItem(i, 5, num(sc.q_before.energy, 3));
        fairTable_->setItem(i, 6, num(sc.q_composite.energy, 3));
        fairTable_->setItem(i, 7, text(sc.ok ? tr("순정") : tr("적용 불가 (오프셋 유지)")));
        if (!sc.ok)
            for (int c = 0; c < fairTable_->columnCount(); ++c) fairTable_->item(i, c)->setForeground(QColor(0xb0, 0x00, 0x20));
    }
    stationSpin_->setRange(0, std::max(0, static_cast<int>(secs.size()) - 1));
    const int mid = static_cast<int>(secs.size()) / 2;
    if (stationSpin_->value() == mid && !secs.empty()) combPlot_->setSection(secs[mid]);
    stationSpin_->setValue(mid);
    showNetwork(h);
}

void MainWindow::showNetwork(const HullRun& h) {
    const auto& nr = h.network;
    double vr = 0, vs = 0, vf = 0;
    int ir = 0, is = 0, inf = 0;
    for (auto& c : nr.curves) {
        vr += c.q_raw.variation; vs += c.q_sectioned.variation; vf += c.q_faired.variation;
        ir += c.q_raw.inflections; is += c.q_sectioned.inflections; inf += c.q_faired.inflections;
    }
    networkSummary_->setText(
        h.network_faired
            ? tr("곡선망 순정: 거스 라인 %1개 (평행부 직선 %2개), 단면점 최대 이동 %3 mm (허용 %4 mm) · "
                 "수선·버톡 %5개의 곡률 변동량 합계 %6 → %7 → %8, 변곡점 %9 → %10 → %11 (원래 → 단면 순정 → 곡선망 순정)")
                  .arg(nr.girth_lines).arg(nr.straight_lines).arg(nr.max_move * 1000, 0, 'f', 1)
                  .arg(h.network_tolerance * 1000, 0, 'f', 1).arg(nr.curves.size())
                  .arg(vr, 0, 'f', 2).arg(vs, 0, 'f', 2).arg(vf, 0, 'f', 2).arg(ir).arg(is).arg(inf)
            : tr("검사만 (곡선망 순정 꺼짐): 수선·버톡 %1개의 곡률 변동량 합계 %2 → %3, 변곡점 %4 → %5 (원래 → 단면 순정). "
                 "'순정 단면으로 곡면 생성'과 '곡선망 순정'을 켜면 거스 라인도 순정합니다.")
                  .arg(nr.curves.size()).arg(vr, 0, 'f', 2).arg(vs, 0, 'f', 2).arg(ir).arg(is));
    networkTable_->setRowCount(static_cast<int>(nr.curves.size()));
    const int keep = networkCurve_->currentIndex();
    networkCurve_->blockSignals(true);
    networkCurve_->clear();
    for (int i = 0; i < static_cast<int>(nr.curves.size()); ++i) {
        const auto& c = nr.curves[i];
        networkCurve_->addItem(QString::fromStdString(c.name()));
        networkTable_->setItem(i, 0, text(QString::fromStdString(c.name())));
        networkTable_->setItem(i, 1, num(static_cast<double>(c.sectioned.size()), 0));
        networkTable_->setItem(i, 2, num(c.q_raw.inflections, 0));
        networkTable_->setItem(i, 3, num(c.q_sectioned.inflections, 0));
        networkTable_->setItem(i, 4, num(c.q_faired.inflections, 0));
        networkTable_->setItem(i, 5, num(c.q_raw.variation, 3));
        networkTable_->setItem(i, 6, num(c.q_sectioned.variation, 3));
        networkTable_->setItem(i, 7, num(c.q_faired.variation, 3));
    }
    networkCurve_->blockSignals(false);
    if (!nr.curves.empty()) {
        // Keep the selected curve across runs; first run: WL 1 m, just above the flat bottom.
        int pick = keep >= 0 && keep < static_cast<int>(nr.curves.size()) ? keep : 0;
        networkCurve_->setCurrentIndex(pick);
        longPlot_->setCurve(nr.curves[pick], h.network_faired);
    } else {
        longPlot_->clear();
    }

    const int stripes = h.curvature.long_sign_changes();
    curvSummary_->setText(
        h.stripes_before_network >= 0
            ? tr("곡면의 길이 방향 곡률 부호 변화 %1 → %2회 (곡선망 순정 전 → 후). 배 길이 방향으로 빨강·파랑이 번갈아 나오는 "
                 "세로 줄무늬는 이웃 스테이션끼리 어긋난 곳입니다.").arg(h.stripes_before_network).arg(stripes)
            : tr("곡면의 길이 방향 곡률 부호 변화 %1회. 배 길이 방향으로 빨강·파랑이 번갈아 나오는 세로 줄무늬는 이웃 스테이션끼리 "
                 "어긋난 곳입니다 (제브라 검사와 같은 목적).").arg(stripes));
    curvMap_->setMap(h.curvature, h.faired_surface ? (h.network_faired ? tr("곡면: 곡선망 순정 단면") : tr("곡면: 순정 단면"))
                                                   : tr("곡면: 원래 단면"));
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
    showSchedule(division_);
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
        if (auto* inner = tabs_->widget(i)->findChild<QTabWidget*>(QStringLiteral("innerTabs"))) {
            for (int j = 1; j < inner->count(); ++j) {
                inner->setCurrentIndex(j);
                QApplication::processEvents();
                grab().save(QDir(screenshotDir_).filePath(QStringLiteral("tab%1_%2.png").arg(i + 1).arg(j + 1)));
            }
            inner->setCurrentIndex(0);
        }
    }
    QApplication::quit();
}
