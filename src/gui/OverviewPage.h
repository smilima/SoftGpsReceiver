#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class OverviewPage; }
QT_END_NAMESPACE

namespace gui {

class OverviewPage : public PageBase
{
    Q_OBJECT
public:
    explicit OverviewPage(QWidget* parent = nullptr);
    ~OverviewPage() override;
    void refresh() override;
    int stage() const override { return 0; }

signals:
    void prnSelected(int prn);

private:
    void clearPosition(const QString& text);
    Ui::OverviewPage* ui;
};

} // namespace gui
