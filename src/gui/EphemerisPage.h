#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class EphemerisPage; }
QT_END_NAMESPACE

namespace gui {

class EphemerisPage : public PageBase
{
    Q_OBJECT
public:
    explicit EphemerisPage(QWidget* parent = nullptr);
    ~EphemerisPage() override;
    void refresh() override;
    int stage() const override { return gnss::Receiver::StageDecoded; }

private:
    void fillEphemeris();
    void fillAlmanac();
    void fillHealth();
    void fillIonoAndPages();
    Ui::EphemerisPage* ui;
};

} // namespace gui
