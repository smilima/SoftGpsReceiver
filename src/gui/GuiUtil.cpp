#include "GuiUtil.h"
#include "core/Constants.h"

#include <QVBoxLayout>
#include <cmath>

namespace gui {

static int g_readableStage = 0;

static bool g_workerRunning = false;

int readableStage() { return g_readableStage; }
void setReadableStage(int s) { g_readableStage = s; }
bool workerRunning() { return g_workerRunning; }
void setWorkerRunning(bool on) { g_workerRunning = on; }

bool PageBase::readable(int stage) const { return rx_ && g_readableStage >= stage; }

QColor seriesColor(int i)
{
    static const QColor c[] = {QColor(31, 119, 180), QColor(255, 127, 14), QColor(44, 160, 44), QColor(214, 39, 40),
                               QColor(148, 103, 189), QColor(140, 86, 75), QColor(227, 119, 194), QColor(127, 127, 127),
                               QColor(188, 189, 34), QColor(23, 190, 207), QColor(0, 0, 128), QColor(128, 128, 0)};
    const int n = int(sizeof(c) / sizeof(c[0]));
    return c[((i % n) + n) % n];
}

QColor prnColor(int prn)
{
    return QColor::fromHsv((prn * 47) % 360, 200, 200);
}

QString satLabel(int id)
{
    return gnss::isGalileo(id) ? gnss::satName(id) : QString::number(id);
}

double satAxis(int id)
{
    return gnss::isGalileo(id) ? 35.0 + gnss::svidOf(id) : double(id);
}

QColor goodColor() { return QColor(170, 230, 170); }
QColor badColor() { return QColor(245, 170, 170); }
QColor warnColor() { return QColor(250, 225, 150); }
QColor neutralColor() { return QColor(225, 225, 225); }

QVector<double> range(int n, double scale, double offset)
{
    QVector<double> r(n);
    for (int i = 0; i < n; ++i) r[i] = i * scale + offset;
    return r;
}

QString fmt(double v, int prec)
{
    if (!std::isfinite(v)) return QStringLiteral("-");
    return QString::number(v, 'f', prec);
}

QString fmtSci(double v, int prec)
{
    if (!std::isfinite(v)) return QStringLiteral("-");
    return QString::number(v, 'g', prec);
}

QString deg(double rad, int prec)
{
    return QString::number(rad * 180.0 / gnss::kPi, 'f', prec);
}

QGroupBox* group(const QString& title, QWidget* content)
{
    auto* g = new QGroupBox(title);
    auto* l = new QVBoxLayout(g);
    l->setContentsMargins(4, 4, 4, 4);
    l->addWidget(content);
    return g;
}

QLabel* valueLabel()
{
    auto* l = new QLabel(QStringLiteral("-"));
    l->setTextInteractionFlags(Qt::TextSelectableByMouse);
    return l;
}

QString channelLabel(const gnss::TrackingChannel& ch)
{
    return QStringLiteral("Ch %1 - %2").arg(ch.channel).arg(gnss::satName(ch.prn));
}

} // namespace gui
