#pragma once

#include "GuiUtil.h"

#include <QVector>

QT_BEGIN_NAMESPACE
namespace Ui { class TrackingPage; }
QT_END_NAMESPACE

namespace gui {

class PlotWidget;

class TrackingPage : public PageBase
{
    Q_OBJECT
public:
    explicit TrackingPage(QWidget* parent = nullptr);
    ~TrackingPage() override;
    void refresh() override;
    int stage() const override { return gnss::Receiver::StageTracked; }

public slots:
    void selectPrn(int prn);

private slots:
    void showChannel();
    void computeShape();

private:
    void fillAllChannels();
    void fillSummary();
    void linkPlots();
    QVector<PlotWidget*> channelPlots() const;

    Ui::TrackingPage* ui;
    QVector<PlotWidget*> timePlots_;
    bool syncing_ = false;
};

} // namespace gui
