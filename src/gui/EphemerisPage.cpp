#include "EphemerisPage.h"
#include "ui_EphemerisPage.h"
#include "DataTable.h"
#include "core/Constants.h"

#include <cmath>
#include <functional>

using namespace gnss;

namespace gui {

EphemerisPage::EphemerisPage(QWidget* parent) : PageBase(parent), ui(new Ui::EphemerisPage)
{
    ui->setupUi(this);
    ui->almTable->setColumns({tr("SV"), tr("Source PRN"), tr("Subframe/page"), tr("Health (8 bit)"), tr("Health meaning"),
                              tr("e"), tr("t_oa (s)"), tr("delta i (deg)"), tr("i (deg)"), tr("OMEGA DOT (deg/s)"),
                              tr("sqrt(A)"), tr("A (km)"), tr("Omega_0 (deg)"), tr("omega (deg)"), tr("M_0 (deg)"),
                              tr("a_f0 (us)"), tr("a_f1 (ns/s)")});
    ui->healthTable->setColumns({tr("SV"), tr("Tracked"), tr("Ephemeris health (SF1)"), tr("Meaning"), tr("Almanac health (8 bit)"),
                                 tr("Meaning"), tr("Page-25 health (6 bit)"), tr("Meaning"), tr("A-S / config (4 bit)"),
                                 tr("Meaning")});
    ui->ionoTable->setColumns({tr("Parameter"), tr("Value"), tr("Units"), tr("Comment")});
    ui->pagesTable->setColumns({tr("Transmitting PRN"), tr("Subframe"), tr("Page"), tr("SV ID"), tr("TOW (s)"),
                                tr("Time of week"), tr("Parity"), tr("Content")});
    QStringList nc{tr("PRN"), tr("AI")};
    for (int i = 1; i <= 30; ++i) nc << QStringLiteral("ERD%1").arg(i);
    ui->nmctTable->setColumns(nc);
}

EphemerisPage::~EphemerisPage()
{
    delete ui;
}

void EphemerisPage::refresh()
{
    for (DataTable* t : {ui->ephTable, ui->almTable, ui->healthTable, ui->ionoTable, ui->nmctTable, ui->pagesTable}) {
        t->beginFill(0);
        t->endFill();
    }
    ui->messageLabel->setText(QStringLiteral("-"));
    if (!readable(Receiver::StageDecoded)) return;
    fillEphemeris();
    fillAlmanac();
    fillHealth();
    fillIonoAndPages();
}

void EphemerisPage::fillEphemeris()
{
    DataTable* t = ui->ephTable;
    std::vector<const NavChannel*> chans;
    for (const auto& n : rx_->nav)
        if (n.eph.sf1 || n.eph.sf2 || n.eph.sf3) chans.push_back(&n);
    QStringList cols{tr("Parameter"), tr("Units")};
    for (const auto* n : chans) cols << satName(n->prn);
    t->setColumns(cols);

    using Getter = std::function<QString(const Ephemeris&)>;
    struct Row { QString name; QString units; Getter get; };
    auto num = [](double v, int p) { return QString::number(v, 'f', p); };
    auto sci = [](double v) { return QString::number(v, 'e', 10); };
    const double r2d = 180.0 / kGpsPi;
    const double r2sc = 1.0 / kGpsPi;
    const auto mu = [](const Ephemeris& e) { return isGalileo(e.prn) ? kMuGal : kMu; };
    const std::vector<Row> rows = {
        {tr("Subframes complete (1/2/3)"), "", [](const Ephemeris& e) { return QStringLiteral("%1 %2 %3").arg(e.sf1 ? "1" : "-").arg(e.sf2 ? "2" : "-").arg(e.sf3 ? "3" : "-"); }},
        {tr("IODC / IODE(SF2) / IODE(SF3)"), "", [](const Ephemeris& e) { return QStringLiteral("%1 / %2 / %3").arg(e.iodc).arg(e.iode2).arg(e.iode3); }},
        {tr("Issue-of-data consistency"), "", [](const Ephemeris& e) { return e.consistent() ? QStringLiteral("consistent") : QStringLiteral("MISMATCH"); }},
        {tr("Week number (mod 1024)"), tr("weeks"), [](const Ephemeris& e) { return QString::number(e.weekNumber); }},
        {tr("Codes on L2"), "", [](const Ephemeris& e) { return isGalileo(e.prn) ? QStringLiteral("-") : nav::l2CodeText(e.l2Code); }},
        {tr("L2 P data flag"), "", [](const Ephemeris& e) { return isGalileo(e.prn) ? QStringLiteral("-") : QString::number(e.l2pFlag); }},
        {tr("URA index (Galileo: SISA index)"), "", [](const Ephemeris& e) { return isGalileo(e.prn) ? QString::number(e.uraIndex) : QStringLiteral("%1 (%2)").arg(e.uraIndex).arg(nav::uraText(e.uraIndex)); }},
        {tr("SV health"), "", [](const Ephemeris& e) { return isGalileo(e.prn) ? (e.svHealth ? QStringLiteral("%1: not healthy or not guaranteed").arg(e.svHealth) : QStringLiteral("0: signal OK, data valid")) : QStringLiteral("%1: %2").arg(e.svHealth).arg(nav::health6Text(e.svHealth)); }},
        {tr("T_GD (Galileo: BGD(E1,E5b))"), tr("ns"), [=](const Ephemeris& e) { return num(e.tgd * 1e9, 3); }},
        {tr("t_oc"), tr("s"), [=](const Ephemeris& e) { return QStringLiteral("%1 (%2)").arg(e.toc, 0, 'f', 0).arg(nav::towText(e.toc)); }},
        {tr("a_f0 (clock bias)"), tr("s"), [=](const Ephemeris& e) { return sci(e.af0); }},
        {tr("a_f0 as range"), tr("km"), [=](const Ephemeris& e) { return num(e.af0 * kSpeedOfLight / 1000, 4); }},
        {tr("a_f1 (clock drift)"), tr("s/s"), [=](const Ephemeris& e) { return sci(e.af1); }},
        {tr("a_f2 (drift rate)"), tr("s/s^2"), [=](const Ephemeris& e) { return sci(e.af2); }},
        {tr("t_oe"), tr("s"), [=](const Ephemeris& e) { return QStringLiteral("%1 (%2)").arg(e.toe, 0, 'f', 0).arg(nav::towText(e.toe)); }},
        {tr("Fit interval flag"), "", [](const Ephemeris& e) { return isGalileo(e.prn) ? QStringLiteral("-") : e.fitFlag ? QStringLiteral("> 4 hours") : QStringLiteral("4 hours"); }},
        {tr("AODO"), tr("s"), [](const Ephemeris& e) { return isGalileo(e.prn) ? QStringLiteral("-") : QString::number(e.aodo); }},
        {tr("sqrt(A)"), tr("m^1/2"), [=](const Ephemeris& e) { return num(e.sqrtA, 6); }},
        {tr("A (semi-major axis)"), tr("km"), [=](const Ephemeris& e) { return num(e.sqrtA * e.sqrtA / 1000, 3); }},
        {tr("e (eccentricity)"), "", [=](const Ephemeris& e) { return num(e.e, 10); }},
        {tr("Perigee / apogee altitude"), tr("km"), [=](const Ephemeris& e) {
             const double A = e.sqrtA * e.sqrtA;
             return QStringLiteral("%1 / %2").arg((A * (1 - e.e) - kWgs84A) / 1000, 0, 'f', 1).arg((A * (1 + e.e) - kWgs84A) / 1000, 0, 'f', 1); }},
        {tr("Mean motion n0"), tr("rad/s"), [=](const Ephemeris& e) { const double A = e.sqrtA * e.sqrtA; return sci(std::sqrt(mu(e) / (A * A * A))); }},
        {tr("Orbital period"), tr("h"), [=](const Ephemeris& e) { const double A = e.sqrtA * e.sqrtA; return num(kTwoPi * std::sqrt(A * A * A / mu(e)) / 3600, 5); }},
        {tr("Delta n"), tr("semicircles/s"), [=](const Ephemeris& e) { return sci(e.deltaN * r2sc); }},
        {tr("M_0 (mean anomaly)"), tr("deg"), [=](const Ephemeris& e) { return num(e.m0 * r2d, 8); }},
        {tr("omega (argument of perigee)"), tr("deg"), [=](const Ephemeris& e) { return num(e.omega * r2d, 8); }},
        {tr("i_0 (inclination)"), tr("deg"), [=](const Ephemeris& e) { return num(e.i0 * r2d, 8); }},
        {tr("IDOT"), tr("deg/s"), [=](const Ephemeris& e) { return sci(e.idot * r2d); }},
        {tr("Omega_0 (long. of ascending node)"), tr("deg"), [=](const Ephemeris& e) { return num(e.omega0 * r2d, 8); }},
        {tr("OMEGA DOT"), tr("deg/s"), [=](const Ephemeris& e) { return sci(e.omegaDot * r2d); }},
        {tr("C_uc"), tr("rad"), [=](const Ephemeris& e) { return sci(e.cuc); }},
        {tr("C_us"), tr("rad"), [=](const Ephemeris& e) { return sci(e.cus); }},
        {tr("C_rc"), tr("m"), [=](const Ephemeris& e) { return num(e.crc, 5); }},
        {tr("C_rs"), tr("m"), [=](const Ephemeris& e) { return num(e.crs, 5); }},
        {tr("C_ic"), tr("rad"), [=](const Ephemeris& e) { return sci(e.cic); }},
        {tr("C_is"), tr("rad"), [=](const Ephemeris& e) { return sci(e.cis); }},
    };
    t->beginFill(int(rows.size()));
    for (int r = 0; r < int(rows.size()); ++r) {
        t->setText(r, 0, rows[size_t(r)].name);
        t->setText(r, 1, rows[size_t(r)].units);
        for (int c = 0; c < int(chans.size()); ++c) {
            const QString v = rows[size_t(r)].get(chans[size_t(c)]->eph);
            QColor bg;
            if (v.contains("MISMATCH") || (r == 7 && chans[size_t(c)]->eph.svHealth != 0)) bg = badColor();
            t->setText(r, 2 + c, v, bg);
        }
    }
    t->endFill();
    t->setSortingEnabled(false);
}

void EphemerisPage::fillAlmanac()
{
    DataTable* t = ui->almTable;
    const GlobalNavData& g = rx_->gnav;
    int n = 0;
    for (int sv = 1; sv <= 32; ++sv) n += g.almanac[sv].valid ? 1 : 0;
    t->beginFill(n);
    int r = 0;
    const double r2d = 180.0 / kGpsPi;
    for (int sv = 1; sv <= 32; ++sv) {
        const AlmanacEntry& a = g.almanac[sv];
        if (!a.valid) continue;
        t->setNum(r, 0, sv, 0);
        t->setNum(r, 1, a.sourcePrn, 0);
        t->setText(r, 2, QStringLiteral("SF%1 p%2").arg(a.subframe).arg(a.page));
        t->setText(r, 3, QStringLiteral("%1").arg(a.health, 8, 2, QLatin1Char('0')), a.health ? badColor() : goodColor());
        t->setText(r, 4, nav::health8Text(a.health));
        t->setNum(r, 5, a.e, 8);
        t->setNum(r, 6, a.toa, 0);
        t->setNum(r, 7, a.deltaI * r2d, 6);
        t->setNum(r, 8, 54.0 + a.deltaI * r2d, 6);
        t->setSci(r, 9, a.omegaDot * r2d, 6);
        t->setNum(r, 10, a.sqrtA, 5);
        t->setNum(r, 11, a.sqrtA * a.sqrtA / 1000, 3);
        t->setNum(r, 12, a.omega0 * r2d, 6);
        t->setNum(r, 13, a.omega * r2d, 6);
        t->setNum(r, 14, a.m0 * r2d, 6);
        t->setNum(r, 15, a.af0 * 1e6, 3);
        t->setNum(r, 16, a.af1 * 1e9, 4);
        ++r;
    }
    t->endFill();
}

void EphemerisPage::fillHealth()
{
    DataTable* t = ui->healthTable;
    const GlobalNavData& g = rx_->gnav;
    t->beginFill(32);
    for (int sv = 1; sv <= 32; ++sv) {
        const int r = sv - 1;
        t->setNum(r, 0, sv, 0);
        const NavChannel* nc = nullptr;
        for (const auto& n : rx_->nav)
            if (n.prn == sv) nc = &n;
        t->setText(r, 1, nc ? tr("yes") : QString(), nc ? goodColor() : QColor());
        if (nc && nc->eph.sf1) {
            t->setText(r, 2, QStringLiteral("%1").arg(nc->eph.svHealth, 6, 2, QLatin1Char('0')), nc->eph.svHealth ? badColor() : goodColor());
            t->setText(r, 3, nav::health6Text(nc->eph.svHealth));
        }
        if (g.almanac[sv].valid) {
            t->setText(r, 4, QStringLiteral("%1").arg(g.almanac[sv].health, 8, 2, QLatin1Char('0')),
                       g.almanac[sv].health ? badColor() : goodColor());
            t->setText(r, 5, nav::health8Text(g.almanac[sv].health));
        }
        if (g.health6Valid[sv]) {
            t->setText(r, 6, QStringLiteral("%1").arg(g.health6[sv], 6, 2, QLatin1Char('0')), g.health6[sv] ? badColor() : goodColor());
            t->setText(r, 7, g.health6[sv] == 63 ? tr("SV not available / not in constellation") : nav::health6Text(g.health6[sv]));
        }
        if (g.svConfigValid[sv]) {
            t->setText(r, 8, QStringLiteral("%1").arg(g.svConfig[sv], 4, 2, QLatin1Char('0')));
            t->setText(r, 9, nav::svConfigText(g.svConfig[sv]));
        }
    }
    t->endFill();
}

void EphemerisPage::fillIonoAndPages()
{
    const GlobalNavData& g = rx_->gnav;
    DataTable* it = ui->ionoTable;
    it->beginFill(0);
    auto row = [&](const QString& a, const QString& b, const QString& u, const QString& c = QString()) {
        const int r = it->rowCount();
        it->setRowCount(r + 1);
        it->setText(r, 0, a);
        it->setText(r, 1, b);
        it->setText(r, 2, u);
        it->setText(r, 3, c);
    };
    if (!g.ionoUtc.valid) {
        row(tr("Page 18 not received"), tr("-"), QString(),
            tr("Subframe 4 page 18 is broadcast once every 12.5 minutes; it is not contained in this recording, "
               "so no ionospheric correction is applied."));
    } else {
        const IonoUtc& u = g.ionoUtc;
        row(tr("Source PRN"), QString::number(u.sourcePrn), QString());
        const char* au[4] = {"s", "s/semicircle", "s/semicircle^2", "s/semicircle^3"};
        for (int i = 0; i < 4; ++i) row(QStringLiteral("alpha_%1").arg(i), QString::number(u.alpha[i], 'e', 6), au[i], tr("Klobuchar amplitude"));
        for (int i = 0; i < 4; ++i) row(QStringLiteral("beta_%1").arg(i), QString::number(u.beta[i], 'e', 6), au[i], tr("Klobuchar period"));
        row(tr("A_0"), QString::number(u.A0, 'e', 8), "s", tr("GPS-UTC bias"));
        row(tr("A_1"), QString::number(u.A1, 'e', 8), "s/s", tr("GPS-UTC drift"));
        row(tr("t_ot"), QString::number(u.tot), "s");
        row(tr("WN_t"), QString::number(u.wnt), tr("weeks (mod 256)"));
        row(tr("Delta t_LS"), QString::number(u.dtls), "s", tr("current leap seconds"));
        row(tr("WN_LSF / DN"), QStringLiteral("%1 / %2").arg(u.wnlsf).arg(u.dn), QString(), tr("leap second event"));
        row(tr("Delta t_LSF"), QString::number(u.dtlsf), "s", tr("leap seconds after the event"));
    }
    const GalileoGlobal& gg = g.galileo;
    if (gg.ionoValid) {
        row(tr("Galileo a_i0 / a_i1 / a_i2"), QStringLiteral("%1 / %2 / %3").arg(gg.ai0).arg(gg.ai1, 0, 'e', 4).arg(gg.ai2, 0, 'e', 4),
            tr("sfu, sfu/deg, sfu/deg^2"), tr("for the NeQuick model, which this receiver does not implement"));
    }
    if (gg.utcValid) {
        row(tr("Galileo A_0 / A_1"), QStringLiteral("%1 / %2").arg(gg.A0, 0, 'e', 6).arg(gg.A1, 0, 'e', 6), tr("s, s/s"), tr("GST-UTC"));
        row(tr("Galileo Delta t_LS"), QString::number(gg.dtLs), "s", tr("current leap seconds"));
    }
    if (gg.ggtoValid) {
        row(tr("A_0G"), QStringLiteral("%1 ns").arg(gg.a0g * 1e9, 0, 'f', 3), QString(), tr("GPS to Galileo time offset"));
        row(tr("A_1G"), QString::number(gg.a1g, 'e', 4), "s/s", tr("drift of that offset"));
        row(tr("t_0G / WN_0G"), QStringLiteral("%1 / %2").arg(gg.t0g).arg(gg.wn0g), QString());
    }
    it->endFill();

    ui->messageLabel->setText(g.specialMessage.isEmpty() ? tr("(page 17 not received)")
                                                         : tr("\"%1\"  (from PRN %2)").arg(g.specialMessage).arg(g.specialMessagePrn));

    DataTable* nt = ui->nmctTable;
    nt->beginFill(int(g.nmct.size()));
    int r = 0;
    for (const auto& kv : g.nmct) {
        nt->setNum(r, 0, kv.first, 0);
        for (int i = 0; i < int(kv.second.size()); ++i) {
            if (i == 0) { nt->setNum(r, 1, kv.second[0], 0); continue; }
            const int v = kv.second[size_t(i)];
            if (v == -32) nt->setText(r, 1 + i, tr("n/a"));
            else nt->setNum(r, 1 + i, v * 0.3, 1);
        }
        ++r;
    }
    nt->endFill();

    DataTable* pt = ui->pagesTable;
    pt->beginFill(int(g.pages.size()));
    for (int i = 0; i < int(g.pages.size()); ++i) {
        const PageRecord& p = g.pages[size_t(i)];
        pt->setNum(i, 0, p.prn, 0);
        pt->setNum(i, 1, p.subframe, 0);
        pt->setNum(i, 2, p.page, 0);
        pt->setNum(i, 3, p.svId, 0);
        pt->setNum(i, 4, p.tow, 0);
        pt->setText(i, 5, nav::towText(p.tow));
        pt->setText(i, 6, p.parityOk ? tr("OK") : tr("FAIL"), p.parityOk ? goodColor() : badColor());
        pt->setText(i, 7, nav::pageDescription(p.subframe, p.svId));
    }
    pt->endFill();
}

} // namespace gui
