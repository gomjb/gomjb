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
    cli.addOption(yard);
    cli.addOption(shots);
    cli.process(app);

    MainWindow w;
    if (cli.isSet(yard) && !w.loadYard(cli.value(yard))) return 1;
    if (cli.isSet(shots)) w.setScreenshotDir(cli.value(shots));
    w.show();
    w.start();
    return app.exec();
}
