#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class AcquisitionPage; }
QT_END_NAMESPACE

namespace gui {

class AcquisitionPage : public PageBase
{
    Q_OBJECT
public:
    explicit AcquisitionPage(QWidget* parent = nullptr);
    ~AcquisitionPage() override;
    void refresh() override;
    int stage() const override { return gnss::Receiver::StageAcquired; }

public slots:
    void selectPrn(int prn);

private:
    void showPrn(int prn);
    Ui::AcquisitionPage* ui;
    int prn_ = 0;
};

} // namespace gui
