#pragma once

#include "MathUtil.h"

#include <QDateTime>
#include <QString>

namespace gnss {

struct Geodetic
{
    double lat = 0;   // rad
    double lon = 0;   // rad
    double h = 0;     // m above the WGS-84 ellipsoid
};

struct Utm
{
    int zone = 0;
    QChar band;
    bool north = true;
    double easting = 0, northing = 0;
};

namespace geo {

Geodetic ecefToGeodetic(const Vec3& p);
Vec3 geodeticToEcef(const Geodetic& g);
Vec3 ecefToEnu(const Vec3& d, double lat, double lon);   // rotate a difference vector
void azimuthElevation(const Vec3& rx, const Vec3& sat, double* az, double* el);   // rad
Vec3 rotateEarth(const Vec3& p, double travelTime);      // Sagnac / earth-rotation correction

// Saastamoinen model with a standard atmosphere (returns slant delay in m).
double tropoSaastamoinen(double lat, double h, double el);
// Klobuchar broadcast model (returns slant delay on L1 in m).
double ionoKlobuchar(const double alpha[4], const double beta[4], double lat, double lon,
                     double az, double el, double gpsTow);

Utm toUtm(double latRad, double lonRad);

// GPS week/time-of-week to calendar time. GPS time (no leap seconds) and UTC.
QDateTime gpsToDateTime(int fullWeek, double tow);
int leapSeconds(const QDateTime& gpsTime);

QString formatDms(double rad, bool isLat);

} // namespace geo
} // namespace gnss
