#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class BitSyncPage; }
QT_END_NAMESPACE

namespace gui {

class BitSyncPage : public PageBase
{
    Q_OBJECT
public:
    explicit BitSyncPage(QWidget* parent = nullptr);
    ~BitSyncPage() override;
    void refresh() override;
    int stage() const override { return gnss::Receiver::StageDecoded; }

public slots:
    void selectPrn(int prn);

private slots:
    void showChannel();

private:
    Ui::BitSyncPage* ui;
};

} // namespace gui
