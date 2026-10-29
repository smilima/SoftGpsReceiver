#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class PositionPage; }
QT_END_NAMESPACE

namespace gui {

class PositionPage : public PageBase
{
    Q_OBJECT
public:
    explicit PositionPage(QWidget* parent = nullptr);
    ~PositionPage() override;
    void refresh() override;
    int stage() const override { return gnss::Receiver::StageSolved; }

public slots:
    void exportCsv();
    void exportKml();

private slots:
    void showEpoch();

private:
    void fillSummary();
    void fillPlots();
    void fillSolutions();
    Ui::PositionPage* ui;
};

} // namespace gui
