#pragma once

namespace gnss {

constexpr double kSpeedOfLight   = 299792458.0;          // m/s
constexpr double kGpsPi          = 3.1415926535898;      // value of pi mandated by IS-GPS-200
constexpr double kPi             = 3.14159265358979323846;
constexpr double kTwoPi          = 2.0 * kPi;
constexpr double kMu             = 3.986005e14;          // WGS-84 earth gravitational constant (m^3/s^2)
constexpr double kOmegaEDot      = 7.2921151467e-5;      // WGS-84 earth rotation rate (rad/s)
constexpr double kRelF           = -4.442807633e-10;     // relativistic constant F (s/m^1/2)
constexpr double kL1Freq         = 1575.42e6;            // Hz
constexpr double kL1Wavelength   = kSpeedOfLight / kL1Freq;
constexpr double kCaChipRate     = 1.023e6;              // chips/s
constexpr int    kCaCodeLength   = 1023;                 // chips
constexpr double kCarrierToCode  = 1540.0;               // L1 carrier cycles per C/A chip
constexpr double kWgs84A         = 6378137.0;
constexpr double kWgs84F         = 1.0 / 298.257223563;
constexpr int    kSecondsPerWeek = 604800;
constexpr int    kHalfWeek       = 302400;
constexpr int    kBitsPerSubframe = 300;
constexpr int    kMsPerBit       = 20;

} // namespace gnss
