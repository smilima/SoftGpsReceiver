#pragma once

#include "core/Receiver.h"

#include <QColor>
#include <QGroupBox>
#include <QLabel>
#include <QString>
#include <QVector>
#include <QWidget>
#include <vector>

namespace gui {

// Base class of every inspection page. Pages are laid out in Qt Designer
// (.ui files) and promoted into MainWindow.ui, so they only take a parent;
// the receiver is attached afterwards with setReceiver(). 'refresh' is called
// (in the GUI thread) whenever the results it depends on have changed.
class PageBase : public QWidget
{
    Q_OBJECT
public:
    explicit PageBase(QWidget* parent = nullptr) : QWidget(parent) {}
    void setReceiver(gnss::Receiver* rx) { rx_ = rx; }
    virtual void refresh() = 0;
    // Stage whose results this page shows (refreshed when that stage completes).
    virtual int stage() const = 0;

protected:
    bool readable(int stage) const;
    gnss::Receiver* rx_ = nullptr;
};

// Highest pipeline stage whose results may be read from the GUI thread.
// Only ever touched from the GUI thread (see MainWindow).
int readableStage();
void setReadableStage(int s);
// True while a processing stage runs on the worker thread.
bool workerRunning();
void setWorkerRunning(bool on);

QColor seriesColor(int i);
QColor prnColor(int prn);
QColor goodColor();
QColor badColor();
QColor warnColor();
QColor neutralColor();

template <typename T>
QVector<double> toQ(const std::vector<T>& v, double scale = 1.0, double offset = 0.0)
{
    QVector<double> r(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) r[i] = double(v[size_t(i)]) * scale + offset;
    return r;
}

QVector<double> range(int n, double scale = 1.0, double offset = 0.0);
QString fmt(double v, int prec);
QString fmtSci(double v, int prec = 6);
QString deg(double rad, int prec = 6);

QGroupBox* group(const QString& title, QWidget* content);
QLabel* valueLabel();
QString channelLabel(const gnss::TrackingChannel& ch);

} // namespace gui
