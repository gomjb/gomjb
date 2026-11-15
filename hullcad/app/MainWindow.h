#pragma once
#include <QFutureWatcher>
#include <QMainWindow>
#include <memory>

#include "pipeline.h"

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTabWidget;
class QTableWidget;
class QTextBrowser;
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
    void buildMenus();
    hullcad::TankerParams hullParams() const;
    void showHull(const hullcad::app::HullRun& h);
    void showDivision(const hullcad::app::DivisionRun& d);
    void takeScreenshots();
    int threads() const;

    QTabWidget* tabs_ = nullptr;
    // Hull tab
    QVector<QDoubleSpinBox*> hullSpins_;
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

    std::shared_ptr<const hullcad::app::HullRun> hull_;
    hullcad::app::DivisionRun division_;
    QFutureWatcher<hullcad::app::HullRun> hullWatch_;
    QFutureWatcher<std::vector<hullcad::GZPoint>> gzWatch_;
    QFutureWatcher<hullcad::app::DivisionRun> divWatch_;
    QString yardPath_;
    QString screenshotDir_;
};
