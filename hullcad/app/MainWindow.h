#pragma once
#include <QFutureWatcher>
#include <QMainWindow>
#include <memory>

#include "pipeline.h"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTabWidget;
class QTableWidget;
class QTextBrowser;
class CombPlot;
class CurvatureMapView;
class LongitudinalPlot;
class DailyPlot;
class QCheckBox;
class QSpinBox;
class GzPlot;
class SvgView;
class YardForm;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    bool loadYard(const QString& path);
    // Test/documentation aid: after the first division, save one PNG per tab and quit.
    void setScreenshotDir(const QString& dir) { screenshotDir_ = dir; }
    void start();
    void setBilgeG2(bool g2);
    void setNetworkFairing(bool on);
    bool loadOffsets(const QString& path);
    void useParametric();

private slots:
    void runHull();
    void onHullDone();
    void runGz();
    void runDivision();
    void onDivisionDone();
    void openYard();
    void saveYard();
    void resetYard();
    void exportBlocksCsv();
    void exportSvg();
    void exportObj();

private:
    QWidget* buildHullTab();
    QWidget* buildHydroTab();
    QWidget* buildDivisionTab();
    QWidget* buildUtilityTab();
    void showUtilities(const hullcad::app::DivisionRun& d);
    void showSchedule(const hullcad::app::DivisionRun& d);
    void showDailyPlot();
    void buildMenus();
    hullcad::TankerParams hullParams() const;
    void showHull(const hullcad::app::HullRun& h);
    void showNetwork(const hullcad::app::HullRun& h);
    void showDivision(const hullcad::app::DivisionRun& d);
    void takeScreenshots();
    int threads() const;

    QTabWidget* tabs_ = nullptr;
    // Hull tab
    QVector<QDoubleSpinBox*> hullSpins_;
    QComboBox* bilgeCombo_ = nullptr;
    QDoubleSpinBox* fairTolSpin_ = nullptr;
    QCheckBox* fairSurfaceCheck_ = nullptr;
    QDoubleSpinBox* accuracySpin_ = nullptr;
    QLabel* sourceLabel_ = nullptr;
    std::shared_ptr<const hullcad::OffsetTable> table_;  // imported offsets (null: parametric hull)
    QLabel* fairSummary_ = nullptr;
    QSpinBox* stationSpin_ = nullptr;
    CombPlot* combPlot_ = nullptr;
    QTableWidget* fairTable_ = nullptr;
    QCheckBox* networkCheck_ = nullptr;
    QDoubleSpinBox* networkTolSpin_ = nullptr;
    QLabel* networkSummary_ = nullptr;
    QComboBox* networkCurve_ = nullptr;
    LongitudinalPlot* longPlot_ = nullptr;
    QTableWidget* networkTable_ = nullptr;
    CurvatureMapView* curvMap_ = nullptr;
    QLabel* curvSummary_ = nullptr;
    QPushButton* hullButton_ = nullptr;
    QTextBrowser* particulars_ = nullptr;
    SvgView* linesView_ = nullptr;
    // Hydrostatics tab
    QDoubleSpinBox* kgSpin_ = nullptr;
    QTableWidget* hydroTable_ = nullptr;
    GzPlot* gzPlot_ = nullptr;
    // Division tab
    YardForm* yardForm_ = nullptr;
    QLineEdit* buttsEdit_ = nullptr;
    QPushButton* divButton_ = nullptr;
    QLabel* divStatus_ = nullptr;
    SvgView* divView_ = nullptr;
    QTableWidget* blocksTable_ = nullptr;
    QTableWidget* checksTable_ = nullptr;
    QListWidget* issuesList_ = nullptr;
    // Utilities tab
    QLabel* utilSummary_ = nullptr;
    QTableWidget* utilStageTable_ = nullptr;
    QTableWidget* utilBlockTable_ = nullptr;
    QComboBox* dailyItem_ = nullptr;
    DailyPlot* dailyPlot_ = nullptr;
    QLabel* scheduleSummary_ = nullptr;
    QTableWidget* peaksTable_ = nullptr;
    QTableWidget* scheduleTable_ = nullptr;

    std::shared_ptr<const hullcad::app::HullRun> hull_;
    hullcad::app::DivisionRun division_;
    QFutureWatcher<hullcad::app::HullRun> hullWatch_;
    QFutureWatcher<std::vector<hullcad::GZPoint>> gzWatch_;
    QFutureWatcher<hullcad::app::DivisionRun> divWatch_;
    QString yardPath_;
    QString screenshotDir_;
};
