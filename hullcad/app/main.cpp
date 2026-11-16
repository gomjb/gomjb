#include <QApplication>
#include <QCommandLineParser>

#include "MainWindow.h"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("hullcad"));
    QApplication::setOrganizationName(QStringLiteral("hullcad"));
    QApplication::setApplicationVersion(QStringLiteral("0.3.0"));
    QApplication::setStyle(QStringLiteral("Fusion"));

    QCommandLineParser cli;
    cli.setApplicationDescription(QStringLiteral("hullcad: hull form, hydrostatics and block division"));
    cli.addHelpOption();
    cli.addVersionOption();
    QCommandLineOption yard(QStringLiteral("yard"), QStringLiteral("Load a yard profile (.ini) at start."), QStringLiteral("file"));
    QCommandLineOption shots(QStringLiteral("screenshot"),
                             QStringLiteral("Save a PNG of every tab to <dir> after the first calculation, then quit."),
                             QStringLiteral("dir"));
    QCommandLineOption bilge(QStringLiteral("bilge"), QStringLiteral("Midship bilge: g1 (circular) or g2 (curvature-continuous)."),
                             QStringLiteral("g1|g2"));
    cli.addOption(bilge);
    QCommandLineOption offsets(QStringLiteral("offsets"), QStringLiteral("Build the hull from an offset table (.csv)."),
                               QStringLiteral("file"));
    cli.addOption(offsets);
    QCommandLineOption network(QStringLiteral("network"),
                               QStringLiteral("Curve-network (girth-line) fairing on or off (default: on for offset tables)."),
                               QStringLiteral("on|off"));
    cli.addOption(network);
    cli.addOption(yard);
    cli.addOption(shots);
    cli.process(app);

    MainWindow w;
    if (cli.isSet(yard) && !w.loadYard(cli.value(yard))) return 1;
    if (cli.isSet(shots)) w.setScreenshotDir(cli.value(shots));
    if (cli.isSet(bilge)) w.setBilgeG2(cli.value(bilge).toLower() == QStringLiteral("g2"));
    if (cli.isSet(offsets) && !w.loadOffsets(cli.value(offsets))) return 1;
    if (cli.isSet(network)) w.setNetworkFairing(cli.value(network).toLower() != QStringLiteral("off"));
    w.show();
    w.start();
    return app.exec();
}
