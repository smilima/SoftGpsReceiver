#include "Geodesy.h"
#include "Constants.h"

#include <QTimeZone>
#include <algorithm>
#include <cmath>

namespace gnss {
namespace geo {

Geodetic ecefToGeodetic(const Vec3& p)
{
    Geodetic g;
    const double a = kWgs84A;
    const double f = kWgs84F;
    const double e2 = f * (2 - f);
    const double r = std::hypot(p.x, p.y);
    if (r < 1e-9 && std::fabs(p.z) < 1e-9) return g;
    g.lon = std::atan2(p.y, p.x);
    double lat = std::atan2(p.z, r * (1 - e2));
    double h = 0;
    for (int i = 0; i < 10; ++i) {
        const double s = std::sin(lat);
        const double N = a / std::sqrt(1 - e2 * s * s);
        h = r / std::cos(lat) - N;
        const double nl = std::atan2(p.z, r * (1 - e2 * N / (N + h)));
        if (std::fabs(nl - lat) < 1e-14) { lat = nl; break; }
        lat = nl;
    }
    const double s = std::sin(lat);
    const double N = a / std::sqrt(1 - e2 * s * s);
    if (std::fabs(std::cos(lat)) > 1e-10) h = r / std::cos(lat) - N;
    else h = std::fabs(p.z) - N * (1 - e2);
    g.lat = lat;
    g.h = h;
    return g;
}

Vec3 geodeticToEcef(const Geodetic& g)
{
    const double e2 = kWgs84F * (2 - kWgs84F);
    const double s = std::sin(g.lat), c = std::cos(g.lat);
    const double N = kWgs84A / std::sqrt(1 - e2 * s * s);
    return Vec3((N + g.h) * c * std::cos(g.lon), (N + g.h) * c * std::sin(g.lon), (N * (1 - e2) + g.h) * s);
}

Vec3 ecefToEnu(const Vec3& d, double lat, double lon)
{
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    return Vec3(-so * d.x + co * d.y,
                -sl * co * d.x - sl * so * d.y + cl * d.z,
                cl * co * d.x + cl * so * d.y + sl * d.z);
}

void azimuthElevation(const Vec3& rx, const Vec3& sat, double* az, double* el)
{
    const Geodetic g = ecefToGeodetic(rx);
    const Vec3 enu = ecefToEnu(sat - rx, g.lat, g.lon);
    double a = std::atan2(enu.x, enu.y);
    if (a < 0) a += kTwoPi;
    if (az) *az = a;
    if (el) *el = std::atan2(enu.z, std::hypot(enu.x, enu.y));
}

Vec3 rotateEarth(const Vec3& p, double travelTime)
{
    const double th = kOmegaEDot * travelTime;
    const double c = std::cos(th), s = std::sin(th);
    return Vec3(c * p.x + s * p.y, -s * p.x + c * p.y, p.z);
}

double tropoSaastamoinen(double lat, double h, double el)
{
    if (el <= 0 || h < -100.0 || h > 1e4) return 0.0;
    const double humi = 0.5;
    const double hgt = std::max(h, 0.0);
    const double pres = 1013.25 * std::pow(1.0 - 2.2557e-5 * hgt, 5.2568);
    const double temp = 15.0 - 6.5e-3 * hgt + 273.16;
    const double e = 6.108 * humi * std::exp((17.15 * temp - 4684.0) / (temp - 38.45));
    const double z = kPi / 2.0 - el;
    const double trph = 0.0022768 * pres / (1.0 - 0.00266 * std::cos(2.0 * lat) - 0.00028 * hgt / 1e3) / std::cos(z);
    const double trpw = 0.002277 * (1255.0 / temp + 0.05) * e / std::cos(z);
    return trph + trpw;
}

double ionoKlobuchar(const double alpha[4], const double beta[4], double lat, double lon,
                     double az, double el, double gpsTow)
{
    if (el <= 0) return 0.0;
    const double E = el / kGpsPi;                  // semicircles
    const double phiU = lat / kGpsPi, lamU = lon / kGpsPi;
    const double psi = 0.0137 / (E + 0.11) - 0.022;
    double phiI = phiU + psi * std::cos(az);
    phiI = std::clamp(phiI, -0.416, 0.416);
    const double lamI = lamU + psi * std::sin(az) / std::cos(phiI * kGpsPi);
    const double phiM = phiI + 0.064 * std::cos((lamI - 1.617) * kGpsPi);
    double t = 4.32e4 * lamI + gpsTow;
    t = std::fmod(t, 86400.0);
    if (t < 0) t += 86400.0;
    const double F = 1.0 + 16.0 * std::pow(0.53 - E, 3);
    double per = beta[0] + phiM * (beta[1] + phiM * (beta[2] + phiM * beta[3]));
    if (per < 72000.0) per = 72000.0;
    double amp = alpha[0] + phiM * (alpha[1] + phiM * (alpha[2] + phiM * alpha[3]));
    if (amp < 0) amp = 0;
    const double x = kTwoPi * (t - 50400.0) / per;
    double tIono;
    if (std::fabs(x) < 1.57) tIono = F * (5e-9 + amp * (1 - x * x / 2 + x * x * x * x / 24));
    else tIono = F * 5e-9;
    return tIono * kSpeedOfLight;
}

Utm toUtm(double latRad, double lonRad)
{
    Utm u;
    const double latD = latRad * 180.0 / kPi;
    const double lonD = lonRad * 180.0 / kPi;
    int zone = int(std::floor((lonD + 180.0) / 6.0)) + 1;
    if (latD >= 56.0 && latD < 64.0 && lonD >= 3.0 && lonD < 12.0) zone = 32;          // Norway
    if (latD >= 72.0 && latD < 84.0) {                                                    // Svalbard
        if (lonD >= 0.0 && lonD < 9.0) zone = 31;
        else if (lonD >= 9.0 && lonD < 21.0) zone = 33;
        else if (lonD >= 21.0 && lonD < 33.0) zone = 35;
        else if (lonD >= 33.0 && lonD < 42.0) zone = 37;
    }
    zone = std::clamp(zone, 1, 60);
    const double lon0 = ((zone - 1) * 6 - 180 + 3) * kPi / 180.0;
    const double a = kWgs84A, f = kWgs84F;
    const double e2 = f * (2 - f);
    const double ep2 = e2 / (1 - e2);
    const double k0 = 0.9996;
    const double s = std::sin(latRad), c = std::cos(latRad), t = std::tan(latRad);
    const double N = a / std::sqrt(1 - e2 * s * s);
    const double T = t * t;
    const double C = ep2 * c * c;
    const double A = c * (lonRad - lon0);
    const double e4 = e2 * e2, e6 = e4 * e2;
    const double M = a * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * latRad
                          - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * std::sin(2 * latRad)
                          + (15 * e4 / 256 + 45 * e6 / 1024) * std::sin(4 * latRad)
                          - (35 * e6 / 3072) * std::sin(6 * latRad));
    u.easting = k0 * N * (A + (1 - T + C) * A * A * A / 6
                          + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * std::pow(A, 5) / 120) + 500000.0;
    u.northing = k0 * (M + N * t * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * std::pow(A, 4) / 24
                                    + (61 - 58 * T + T * T + 600 * C - 330 * ep2) * std::pow(A, 6) / 720));
    u.north = latRad >= 0;
    if (!u.north) u.northing += 10000000.0;
    u.zone = zone;
    static const char bands[] = "CDEFGHJKLMNPQRSTUVWXX";
    const int bi = std::clamp(int(std::floor((latD + 80.0) / 8.0)), 0, 20);
    u.band = QChar(bands[bi]);
    return u;
}

QDateTime gpsToDateTime(int fullWeek, double tow)
{
    QDateTime epoch(QDate(1980, 1, 6), QTime(0, 0, 0), QTimeZone::UTC);
    const qint64 ms = qint64(fullWeek) * 604800000LL + qint64(std::llround(tow * 1000.0));
    return epoch.addMSecs(ms);
}

int leapSeconds(const QDateTime& gps)
{
    static const struct { int y, m, d, ls; } table[] = {
        {1981, 7, 1, 1}, {1982, 7, 1, 2}, {1983, 7, 1, 3}, {1985, 7, 1, 4}, {1988, 1, 1, 5},
        {1990, 1, 1, 6}, {1991, 1, 1, 7}, {1992, 7, 1, 8}, {1993, 7, 1, 9}, {1994, 7, 1, 10},
        {1996, 1, 1, 11}, {1997, 7, 1, 12}, {1999, 1, 1, 13}, {2006, 1, 1, 14}, {2009, 1, 1, 15},
        {2012, 7, 1, 16}, {2015, 7, 1, 17}, {2017, 1, 1, 18}};
    int ls = 0;
    for (const auto& e : table)
        if (gps >= QDateTime(QDate(e.y, e.m, e.d), QTime(0, 0), QTimeZone::UTC).addSecs(e.ls - 1)) ls = e.ls;
    return ls;
}

QString formatDms(double rad, bool isLat)
{
    const double deg = rad * 180.0 / kPi;
    const QChar hemi = isLat ? (deg >= 0 ? QChar('N') : QChar('S')) : (deg >= 0 ? QChar('E') : QChar('W'));
    double a = std::fabs(deg);
    const int d = int(a);
    a = (a - d) * 60.0;
    const int m = int(a);
    const double s = (a - m) * 60.0;
    return QStringLiteral("%1° %2' %3\" %4").arg(d).arg(m, 2, 10, QLatin1Char('0'))
        .arg(s, 7, 'f', 4, QLatin1Char('0')).arg(hemi);
}

} // namespace geo
} // namespace gnss
