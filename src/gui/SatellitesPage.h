#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class SatellitesPage; }
QT_END_NAMESPACE

namespace gui {

class SatellitesPage : public PageBase
{
    Q_OBJECT
public:
    explicit SatellitesPage(QWidget* parent = nullptr);
    ~SatellitesPage() override;
    void refresh() override;
    int stage() const override { return gnss::Receiver::StageSolved; }

public slots:
    void selectPrn(int prn);

private slots:
    void showEpoch();
    void showKepler();

private:
    void fillTimePlots();
    const gnss::NavSolution* currentSolution() const;
    Ui::SatellitesPage* ui;
};

} // namespace gui
