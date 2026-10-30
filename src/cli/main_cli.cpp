// Command-line front end: runs the full receiver pipeline on a recording and
// prints the intermediate results. Useful for batch processing and testing.
//
//   gnss_cli <file.bin> [--ms N] [--channels N] [--fs Hz] [--if Hz] [--gnss all|gps|gal] [--csv out.csv]

#include "core/Constants.h"
#include "core/Geodesy.h"
#include "core/Receiver.h"

#include <QCoreApplication>
#include <QFile>
#include <QTextStream>
#include <cmath>
#include <cstdio>

using namespace gnss;

static QTextStream& out()
{
    static QTextStream s(stdout);
    return s;
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    if (args.size() < 2) {
        out() << "usage: gnss_cli <file.bin> [--ms N] [--channels N] [--fs Hz] [--if Hz] [--gnss all|gps|gal] [--csv out.csv]\n";
        return 1;
    }
    Receiver rx;
    QObject::connect(&rx, &Receiver::logMessage, [](const QString& t) { out() << t << "\n"; out().flush(); });
    QString err;
    if (!rx.openFile(args[1], &err)) {
        out() << err << "\n";
        return 2;
    }
    QString csv;
    for (int i = 2; i + 1 < args.size(); ++i) {
        if (args[i] == "--ms") rx.settings.msToProcess = args[++i].toInt();
        else if (args[i] == "--channels") rx.settings.numChannels = args[++i].toInt();
        else if (args[i] == "--fs") rx.settings.samplingFreq = args[++i].toDouble();
        else if (args[i] == "--if") rx.settings.ifFreq = args[++i].toDouble();
        else if (args[i] == "--gnss") {
            const QString g = args[++i];
            rx.settings.constellations = g == "gps" ? Constellations::GpsOnly
                                         : g == "gal" ? Constellations::GalileoOnly : Constellations::GpsGalileo;
        }
        else if (args[i] == "--csv") csv = args[++i];
    }

    if (!rx.runAcquisition()) return 3;
    out() << "\n PRN  metric   code(samp)  Doppler(Hz)  fine-coarse  C/N0\n";
    for (const auto& r : rx.acq.results)
        out() << QString::asprintf(" %3s  %6.2f  %10d  %11.1f  %11.1f  %5.1f %s\n",
                                   qPrintable(isGalileo(r.prn) ? satName(r.prn) : QString::number(r.prn)), r.metric, r.codePhase,
                                   r.doppler, r.fineDelta, r.cn0, r.acquired ? "ACQ" : "");
    out().flush();

    if (!rx.runTracking()) return 4;
    if (!rx.runNavigation()) return 5;
    for (const auto& n : rx.nav) {
        if (isGalileo(n.prn)) {
            out() << QString::asprintf(" %s words:", qPrintable(satName(n.prn)));
            for (size_t i = 0; i < n.inavWords.size() && i < 24; ++i)
                out() << QString::asprintf(" %d%s", n.inavWords[i].type, n.inavWords[i].crcOk ? "" : "*");
        } else {
            out() << QString::asprintf(" PRN %2d subframes:", n.prn);
            for (const auto& sf : n.subframes)
                out() << QString::asprintf(" %d%s(%d)", sf.id, sf.allParityOk() ? "" : "*", sf.towCount);
        }
        out() << "\n";
        if (n.eph.complete())
            out() << QString::asprintf("   WN %d toe %.0f toc %.0f IODE %d/%d IODC %d health %d URA %d e %.6f sqrtA %.3f\n",
                                       n.eph.weekNumber, n.eph.toe, n.eph.toc, n.eph.iode2, n.eph.iode3, n.eph.iodc,
                                       n.eph.svHealth, n.eph.uraIndex, n.eph.e, n.eph.sqrtA);
    }
    out().flush();
    if (!rx.runPvt()) return 6;

    const auto& p = rx.pvt;
    out() << "\nEpoch   rxTime        nSat   lat(deg)      lon(deg)      h(m)      E(m)    N(m)    U(m)   bias(m)      PDOP  vE     vN     vU\n";
    for (const auto& s : p.sols) {
        if (!s.valid) { out() << QString::asprintf("%4d  invalid: %s\n", s.index, qPrintable(s.message)); continue; }
        out() << QString::asprintf("%4d  %12.4f  %2d  %12.8f  %12.8f  %8.2f  %6.2f  %6.2f  %6.2f  %12.2f  %4.2f  %5.2f  %5.2f  %5.2f\n",
                                   s.index, s.gpsTime, s.nUsed, s.llh.lat * 180 / kPi, s.llh.lon * 180 / kPi, s.llh.h,
                                   s.enu.x, s.enu.y, s.enu.z, s.bias, s.pdop, s.velEnu.x, s.velEnu.y, s.velEnu.z);
    }
    out() << QString::asprintf("\nMean: lat %.8f lon %.8f h %.2f  (ECEF %.2f %.2f %.2f)\n", p.meanLlh.lat * 180 / kPi,
                               p.meanLlh.lon * 180 / kPi, p.meanLlh.h, p.meanPos.x, p.meanPos.y, p.meanPos.z);
    out() << QString::asprintf("UTM zone %d%c  E %.2f  N %.2f\n", p.utm.zone, p.utm.band.toLatin1(), p.utm.easting, p.utm.northing);
    out() << QString::asprintf("std E/N/U %.2f %.2f %.2f m, CEP50 %.2f m, 2DRMS %.2f m\n", p.stdE, p.stdN, p.stdU, p.cep50, p.drms2);
    out() << QString::asprintf("clock bias drift %.3f m/s, Doppler clock drift %.3f m/s, common freq offset %.1f Hz -> effective IF %.1f Hz\n",
                               p.biasDrift, p.meanDrift, p.commonFreqOffset, p.effectiveIf);
    if (p.weekNumber >= 0) {
        for (int k = 0; k < 3; ++k) {
            const QDateTime dt = geo::gpsToDateTime(p.weekNumber + 1024 * k, p.firstGpsTime);
            out() << QString::asprintf("If %d week rollovers: %s (GPS time)\n", k, qPrintable(dt.toString(Qt::ISODate)));
        }
    }
    if (!csv.isEmpty()) {
        QFile f(csv);
        if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream ts(&f);
            ts << "epoch,gpsTow,nSat,lat,lon,h,x,y,z,bias,pdop,hdop,vdop,ve,vn,vu\n";
            for (const auto& s : p.sols)
                if (s.valid)
                    ts << QString::asprintf("%d,%.4f,%d,%.9f,%.9f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                                            s.index, s.gpsTime, s.nUsed, s.llh.lat * 180 / kPi, s.llh.lon * 180 / kPi,
                                            s.llh.h, s.pos.x, s.pos.y, s.pos.z, s.bias, s.pdop, s.hdop, s.vdop,
                                            s.velEnu.x, s.velEnu.y, s.velEnu.z);
        }
    }
    out().flush();
    return 0;
}
