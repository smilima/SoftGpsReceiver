#include "MainWindow.h"
#include "ui_MainWindow.h"

#include "AcquisitionPage.h"
#include "BitSyncPage.h"
#include "CaCodePage.h"
#include "EphemerisPage.h"
#include "GuiUtil.h"
#include "MeasurementsPage.h"
#include "NavMessagePage.h"
#include "OverviewPage.h"
#include "PositionPage.h"
#include "RawSignalPage.h"
#include "SatellitesPage.h"
#include "SettingsDialog.h"
#include "TrackingPage.h"

#include <QApplication>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QSettings>
#include <QStyle>
#include <QThread>

using namespace gnss;

namespace gui {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pages_ = {ui->overviewPage, ui->rawSignalPage, ui->caCodePage, ui->acquisitionPage, ui->trackingPage,
              ui->bitSyncPage, ui->navMessagePage, ui->ephemerisPage, ui->satellitesPage, ui->measurementsPage,
              ui->positionPage};
    for (PageBase* p : pages_) p->setReceiver(&rx_);

    // icons from the platform style (no external resources required)
    QStyle* st = style();
    ui->actionOpen->setIcon(st->standardIcon(QStyle::SP_DialogOpenButton));
    ui->actionSettings->setIcon(st->standardIcon(QStyle::SP_FileDialogDetailedView));
    ui->actionRunAll->setIcon(st->standardIcon(QStyle::SP_MediaPlay));
    ui->actionAcquire->setIcon(st->standardIcon(QStyle::SP_MediaSeekForward));
    ui->actionTrack->setIcon(st->standardIcon(QStyle::SP_MediaSeekForward));
    ui->actionDecode->setIcon(st->standardIcon(QStyle::SP_MediaSeekForward));
    ui->actionSolve->setIcon(st->standardIcon(QStyle::SP_MediaSeekForward));
    ui->actionStop->setIcon(st->standardIcon(QStyle::SP_MediaStop));
    ui->actionAbout->setIcon(st->standardIcon(QStyle::SP_MessageBoxInformation));
    ui->menuView->addAction(ui->logDock->toggleViewAction());
    ui->menuView->addAction(ui->toolBar->toggleViewAction());

    stageLabel_ = new QLabel(tr("Ready"));
    progress_ = new QProgressBar;
    progress_->setRange(0, 100);
    progress_->setMaximumWidth(300);
    progress_->setVisible(false);
    ui->statusbar->addWidget(stageLabel_, 1);
    ui->statusbar->addPermanentWidget(progress_);

    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::onOpen);
    connect(ui->actionSettings, &QAction::triggered, this, &MainWindow::onSettings);
    connect(ui->actionRunAll, &QAction::triggered, this, [this] { startRun(Receiver::StageSolved, true); });
    connect(ui->actionAcquire, &QAction::triggered, this, [this] { startRun(Receiver::StageAcquired, true); });
    connect(ui->actionTrack, &QAction::triggered, this, [this] { startRun(Receiver::StageTracked, false); });
    connect(ui->actionDecode, &QAction::triggered, this, [this] { startRun(Receiver::StageDecoded, false); });
    connect(ui->actionSolve, &QAction::triggered, this, [this] { startRun(Receiver::StageSolved, false); });
    connect(ui->actionStop, &QAction::triggered, this, [this] {
        rx_.cancel();
        onLog(tr("Stop requested"));
    });
    connect(ui->actionExportCsv, &QAction::triggered, ui->positionPage, &PositionPage::exportCsv);
    connect(ui->actionExportKml, &QAction::triggered, ui->positionPage, &PositionPage::exportKml);
    connect(ui->actionAbout, &QAction::triggered, this, &MainWindow::onAbout);
    connect(ui->actionAlgorithms, &QAction::triggered, this, [this] { ui->pages->setCurrentWidget(ui->overviewPage); });
    connect(ui->overviewPage, &OverviewPage::prnSelected, this, &MainWindow::selectPrnEverywhere);

    connect(&rx_, &Receiver::logMessage, this, &MainWindow::onLog, Qt::QueuedConnection);
    connect(&rx_, &Receiver::progressChanged, this, &MainWindow::onProgress, Qt::QueuedConnection);
    connect(&rx_, &Receiver::stageCompleted, this, &MainWindow::onStageCompleted, Qt::QueuedConnection);

    QSettings qs;
    rx_.settings.load(qs);
    restoreGeometry(qs.value("window/geometry").toByteArray());
    restoreState(qs.value("window/state").toByteArray());
    updateActions();
    refreshPages(-1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openInitialFile(const QString& commandLinePath)
{
    if (!commandLinePath.isEmpty() && openFile(commandLinePath)) return;
    const QString last = QSettings().value("lastFile").toString();
    if (!last.isEmpty() && QFileInfo::exists(last) && openFile(last)) return;
    // Look for a recording next to (or a few levels above) the executable and in the working directory
    QStringList dirs{QDir::currentPath()};
    QDir d(QCoreApplication::applicationDirPath());
    for (int i = 0; i < 5; ++i) {
        dirs << d.absolutePath();
        if (!d.cdUp()) break;
    }
    for (const QString& dir : dirs) {
        const QFileInfoList files = QDir(dir).entryInfoList({"*fs*if*.bin", "*.bin"}, QDir::Files, QDir::Size | QDir::Reversed);
        for (const QFileInfo& f : files)
            if (f.size() > 10 * 1024 * 1024) {
                openFile(f.absoluteFilePath());
                return;
            }
    }
    onLog(tr("Use File > Open to load a recording. Sampling and IF frequencies are read from file names such as "
             "\"...-fs38_192-if9_55.bin\" (MHz)."));
}

bool MainWindow::openFile(const QString& path)
{
    if (worker_) return false;
    QString err;
    if (!rx_.openFile(path, &err)) {
        QMessageBox::warning(this, tr("Open recording"), err);
        return false;
    }
    setReadableStage(Receiver::StageNone);
    QSettings().setValue("lastFile", path);
    setWindowTitle(tr("Software GPS Receiver - %1").arg(QFileInfo(path).fileName()));
    stageLabel_->setText(tr("Opened %1 - press F5 (Run all) to process").arg(QFileInfo(path).fileName()));
    refreshPages(-1);
    updateActions();
    return true;
}

void MainWindow::onOpen()
{
    QSettings qs;
    const QString dir = qs.value("lastDir", QFileInfo(rx_.settings.fileName).absolutePath()).toString();
    const QString fn = QFileDialog::getOpenFileName(this, tr("Open recorded IF samples"), dir,
                                                    tr("Sample files (*.bin *.dat *.raw *.bb *.int8 *.iq);;All files (*)"));
    if (fn.isEmpty()) return;
    qs.setValue("lastDir", QFileInfo(fn).absolutePath());
    openFile(fn);
}

void MainWindow::onSettings()
{
    if (worker_) return;
    SettingsDialog dlg(rx_.settings, this);
    if (dlg.exec() != QDialog::Accepted) return;
    const ReceiverSettings ns = dlg.settings();
    const bool reopen = ns.format != rx_.settings.format || ns.skipBytes != rx_.settings.skipBytes;
    rx_.settings = ns;
    QSettings qs;
    rx_.settings.save(qs);
    if (reopen) {
        QString err;
        if (!rx_.reopen(&err)) QMessageBox::warning(this, tr("Settings"), err);
    } else {
        rx_.resetResults();
    }
    setReadableStage(Receiver::StageNone);
    onLog(tr("Settings changed: %1 - results cleared").arg(rx_.settings.summary()));
    refreshPages(-1);
    updateActions();
}

void MainWindow::startRun(int target, bool fromScratch)
{
    if (worker_) return;
    if (!rx_.source.isOpen()) {
        onOpen();
        if (!rx_.source.isOpen()) return;
    }
    // first stage that must be (re)computed
    int from = fromScratch ? int(Receiver::StageAcquired) : std::min(target, readableStage() + 1);
    from = std::max(from, int(Receiver::StageAcquired));
    rx_.invalidateFrom(from);
    setReadableStage(from - 1);
    refreshPages(-1);
    setWorkerRunning(true);
    worker_ = QThread::create([this, target] {
        switch (target) {
        case Receiver::StageAcquired: rx_.runAcquisition(); break;
        case Receiver::StageTracked: rx_.runTracking(); break;
        case Receiver::StageDecoded: rx_.runNavigation(); break;
        default: rx_.runPvt(); break;
        }
    });
    connect(worker_, &QThread::finished, this, &MainWindow::onRunFinished);
    progress_->setValue(0);
    progress_->setVisible(true);
    stageLabel_->setText(tr("Processing..."));
    updateActions();
    worker_->start();
}

void MainWindow::onRunFinished()
{
    worker_->deleteLater();
    worker_ = nullptr;
    setWorkerRunning(false);
    progress_->setVisible(false);
    if (rx_.stage() > readableStage()) setReadableStage(rx_.stage());
    stageLabel_->setText(rx_.isCancelled() ? tr("Stopped") : tr("Finished (stage %1 of 4)").arg(readableStage()));
    ui->overviewPage->refresh();
    refreshPages(Receiver::StageSolved);
    updateActions();
}

void MainWindow::onStageCompleted(int stage)
{
    if (stage > readableStage()) setReadableStage(stage);
    refreshPages(stage);
    ui->overviewPage->refresh();
}

void MainWindow::refreshPages(int stage)
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    for (PageBase* p : pages_)
        if (stage < 0 || p->stage() == stage) p->refresh();
    QApplication::restoreOverrideCursor();
}

void MainWindow::updateActions()
{
    const bool busy = worker_ != nullptr;
    const bool open = rx_.source.isOpen();
    ui->actionOpen->setEnabled(!busy);
    ui->actionSettings->setEnabled(!busy);
    ui->actionRunAll->setEnabled(!busy && open);
    ui->actionAcquire->setEnabled(!busy && open);
    ui->actionTrack->setEnabled(!busy && open);
    ui->actionDecode->setEnabled(!busy && open);
    ui->actionSolve->setEnabled(!busy && open);
    ui->actionStop->setEnabled(busy);
    const bool solved = !busy && readableStage() >= Receiver::StageSolved;
    ui->actionExportCsv->setEnabled(solved);
    ui->actionExportKml->setEnabled(solved);
}

void MainWindow::onLog(const QString& text)
{
    ui->logView->appendPlainText(QStringLiteral("[%1] %2").arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"), text));
}

void MainWindow::onProgress(const QString& stage, int percent)
{
    stageLabel_->setText(tr("%1...").arg(stage));
    progress_->setValue(percent);
}

void MainWindow::selectPrnEverywhere(int prn)
{
    ui->acquisitionPage->selectPrn(prn);
    ui->trackingPage->selectPrn(prn);
    ui->bitSyncPage->selectPrn(prn);
    ui->navMessagePage->selectPrn(prn);
    ui->satellitesPage->selectPrn(prn);
    ui->pages->setCurrentWidget(readableStage() >= Receiver::StageTracked ? static_cast<QWidget*>(ui->trackingPage)
                                                                           : static_cast<QWidget*>(ui->acquisitionPage));
}

void MainWindow::captureScreenshots(const QString& dir)
{
    if (!rx_.source.isOpen()) return;
    QDir().mkpath(dir);
    // Same threaded path as "Run all", including the per-stage page refreshes
    startRun(Receiver::StageSolved, true);
    if (worker_ && !worker_->isFinished()) {
        QEventLoop loop;
        connect(worker_, &QThread::finished, &loop, &QEventLoop::quit);
        loop.exec();
    }
    QCoreApplication::processEvents();
    auto safe = [](QString s) { return s.replace('&', "").replace('/', '-').replace(' ', '_'); };
    int shot = 0;
    for (int i = 0; i < ui->pages->count(); ++i) {
        ui->pages->setCurrentIndex(i);
        QCoreApplication::processEvents();
        QWidget* page = ui->pages->widget(i);
        QTabWidget* sub = page->findChild<QTabWidget*>();
        const int nSub = sub ? sub->count() : 1;
        for (int k = 0; k < nSub; ++k) {
            if (sub) sub->setCurrentIndex(k);
            QCoreApplication::processEvents();
            QString name = QStringLiteral("%1_%2").arg(++shot, 2, 10, QLatin1Char('0')).arg(safe(ui->pages->tabText(i)));
            if (sub) name += QLatin1Char('_') + safe(sub->tabText(k));
            grab().save(QDir(dir).filePath(name + QStringLiteral(".png")));
        }
        if (sub) sub->setCurrentIndex(0);
    }
    ui->pages->setCurrentIndex(0);
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, tr("About Software GPS Receiver"),
                       tr("<h3>Software GPS Receiver</h3>"
                          "<p>A post-processing GPS L1 C/A and Galileo E1 receiver for recorded IF samples, built to expose every "
                          "step of the signal processing: raw signal statistics, C/A code generation, FFT-based "
                          "acquisition, FLL-assisted PLL / DLL tracking, bit synchronisation, LNAV decoding with "
                          "parity, ephemeris/almanac/health, satellite orbits, pseudoranges and least-squares "
                          "position, velocity and time.</p>"
                          "<p>Reference: IS-GPS-200; Galileo OS SIS ICD Issue 2.1; K. Borre et al., <i>A Software-Defined GPS and Galileo Receiver</i>; "
                          "E. D. Kaplan, <i>Understanding GPS</i>.</p>"
                          "<p>Built with Qt %1.</p>").arg(QT_VERSION_STR));
}

void MainWindow::closeEvent(QCloseEvent* e)
{
    if (worker_) {
        rx_.cancel();
        worker_->wait();
    }
    QSettings qs;
    qs.setValue("window/geometry", saveGeometry());
    qs.setValue("window/state", saveState());
    rx_.settings.save(qs);
    e->accept();
}

} // namespace gui
