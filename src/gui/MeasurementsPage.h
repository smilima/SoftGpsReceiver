#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MeasurementsPage; }
QT_END_NAMESPACE

namespace gui {

class MeasurementsPage : public PageBase
{
    Q_OBJECT
public:
    explicit MeasurementsPage(QWidget* parent = nullptr);
    ~MeasurementsPage() override;
    void refresh() override;
    int stage() const override { return gnss::Receiver::StageSolved; }

private slots:
    void showEpoch();

private:
    void fillPlots();
    Ui::MeasurementsPage* ui;
};

} // namespace gui
