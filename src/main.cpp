#include "gui/MainWindow.h"

#include <QApplication>
#include <QStyleFactory>

// Usage:
//   SoftGpsReceiver [recording.bin]
//   SoftGpsReceiver recording.bin --screenshots <dir>   (process, save PNGs of every page, exit)
int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("SoftGpsReceiver"));
    QCoreApplication::setApplicationName(QStringLiteral("SoftGpsReceiver"));
    QCoreApplication::setApplicationVersion(QStringLiteral("1.0"));
    app.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));

    QString file, shots;
    const QStringList args = app.arguments();
    for (int i = 1; i < args.size(); ++i) {
        if (args[i] == QLatin1String("--screenshots") && i + 1 < args.size()) shots = args[++i];
        else if (!args[i].startsWith(QLatin1String("--"))) file = args[i];
    }

    if (!shots.isEmpty()) app.setFont(QFont(QStringLiteral("Segoe UI"), 9));   // deterministic font for offscreen rendering

    gui::MainWindow w;
    if (!shots.isEmpty()) {
        w.resize(1700, 1050);
        w.show();
        w.openInitialFile(file);
        w.captureScreenshots(shots);
        return 0;
    }
    w.show();
    w.openInitialFile(file);
    return app.exec();
}
