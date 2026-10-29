#pragma once

#include "core/Receiver.h"

#include <QList>
#include <QMainWindow>

class QLabel;
class QProgressBar;
class QThread;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

namespace gui {

class PageBase;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    bool openFile(const QString& path);
    void openInitialFile(const QString& commandLinePath);
    // Runs the whole pipeline synchronously and saves a PNG of every page and
    // sub-tab into 'dir' (command line option --screenshots <dir>).
    void captureScreenshots(const QString& dir);

protected:
    void closeEvent(QCloseEvent* e) override;

private slots:
    void onOpen();
    void onSettings();
    void onAbout();
    void onLog(const QString& text);
    void onProgress(const QString& stage, int percent);
    void onStageCompleted(int stage);
    void onRunFinished();
    void selectPrnEverywhere(int prn);

private:
    void startRun(int targetStage, bool fromScratch);
    void refreshPages(int stage);
    void updateActions();

    Ui::MainWindow* ui;
    gnss::Receiver rx_;
    QThread* worker_ = nullptr;
    QProgressBar* progress_ = nullptr;
    QLabel* stageLabel_ = nullptr;
    QList<PageBase*> pages_;
};

} // namespace gui
