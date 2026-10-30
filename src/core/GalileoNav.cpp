#include "GalileoNav.h"
#include "Constants.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>

namespace gnss {
namespace inav {

namespace {

constexpr int kRows = 8, kCols = 30;
constexpr int kStates = 64;

double p2(int e) { return std::ldexp(1.0, e); }

inline int parity(unsigned v)
{
    int p = 0;
    for (; v; v &= v - 1) p ^= 1;
    return p;
}

qint64 sfield(const InavWord& w, int start, int len)
{
    const quint64 u = w.field(start, len);
    return (u >> (len - 1)) & 1 ? qint64(u) - (qint64(1) << len) : qint64(u);
}

// Time of week carried by word types 5, 6 and 0 (type 0 only when its Time field is '10')
void readTow(InavWord& w)
{
    if (w.type == 5) { w.hasTow = true; w.tow = double(w.field(85, 20)); }
    else if (w.type == 6) { w.hasTow = true; w.tow = double(w.field(105, 20)); }
    else if (w.type == 0 && w.field(6, 2) == 2) { w.hasTow = true; w.tow = double(w.field(108, 20)); }
}

// Deinterleaves and FEC-decodes every page part at symbol offset 'phase', pairs even and odd parts and checks the CRC
std::vector<InavWord> decodeWords(const NavChannel& nc, int start, int phase)
{
    struct Part { int start; bool inverted; std::array<uint8_t, kPartBits> bits; };
    std::vector<Part> parts;
    bool inv = false;
    float deint[kCodedSymbols], sym[kCodedSymbols];
    const int nb = int(nc.softBits.size());
    for (int j = phase; j + kInavPartSymbols <= nb; j += kInavPartSymbols) {
        const int c = nc.preambleCorr[size_t(j)];
        if (std::abs(c) >= kSyncSymbols - 2) inv = c < 0;    // a cycle slip flips the polarity
        for (int i = 0; i < kCodedSymbols; ++i) {
            const float v = nc.softBits[size_t(j + kSyncSymbols + i)];
            sym[i] = inv ? -v : v;
        }
        deinterleave(sym, deint);
        Part p;
        p.start = start + j;
        p.inverted = inv;
        viterbi(deint, p.bits.data());
        parts.push_back(p);
    }

    std::vector<InavWord> words;
    for (size_t i = 0; i + 1 < parts.size(); ++i) {
        const Part& even = parts[i];
        const Part& odd = parts[i + 1];
        if (even.bits[0] != 0 || odd.bits[0] != 1) continue;
        if (even.bits[1] != 0 || odd.bits[1] != 0) continue;  // alert pages carry no word
        InavWord w;
        w.startMs = even.start;
        w.inverted = even.inverted;
        std::copy_n(even.bits.begin() + 2, 112, w.bits.begin());
        std::copy_n(odd.bits.begin() + 2, 16, w.bits.begin() + 112);

        uint8_t covered[196];
        std::copy_n(even.bits.begin(), 114, covered);
        std::copy_n(odd.bits.begin(), 82, covered + 114);
        uint32_t rx = 0;
        for (int b = 82; b < 106; ++b) rx = (rx << 1) | odd.bits[size_t(b)];
        w.crcOk = crc24q(covered, 196) == rx;
        w.type = int(w.field(0, 6));
        if (w.crcOk) readTow(w);
        words.push_back(w);
        ++i;
    }
    return words;
}

void assembleEphemeris(NavChannel& nc)
{
    Ephemeris& e = nc.eph;
    std::map<int, std::array<const InavWord*, 5>> sets;
    const std::array<const InavWord*, 5>* found = nullptr;
    for (const InavWord& w : nc.inavWords) {
        if (!w.crcOk || w.type < 1 || w.type > 4) continue;
        auto it = sets.find(int(w.field(6, 10)));
        if (it == sets.end()) it = sets.insert({int(w.field(6, 10)), {nullptr, nullptr, nullptr, nullptr, nullptr}}).first;
        auto& set = it->second;
        if (!set[size_t(w.type)]) set[size_t(w.type)] = &w;
        if (set[1] && set[2] && set[3] && set[4]) { found = &set; break; }
    }
    const InavWord* w5 = nullptr;
    for (const InavWord& w : nc.inavWords)
        if (w.crcOk && w.type == 5) { w5 = &w; break; }
    if (!found || !w5) return;

    const InavWord& w1 = *(*found)[1];
    const InavWord& w2 = *(*found)[2];
    const InavWord& w3 = *(*found)[3];
    const InavWord& w4 = *(*found)[4];
    e.iode2 = int(w1.field(6, 10));
    e.toe = double(w1.field(16, 14)) * 60.0;
    e.m0 = double(sfield(w1, 30, 32)) * p2(-31) * kGpsPi;
    e.e = double(w1.field(62, 32)) * p2(-33);
    e.sqrtA = double(w1.field(94, 32)) * p2(-19);

    e.omega0 = double(sfield(w2, 16, 32)) * p2(-31) * kGpsPi;
    e.i0 = double(sfield(w2, 48, 32)) * p2(-31) * kGpsPi;
    e.omega = double(sfield(w2, 80, 32)) * p2(-31) * kGpsPi;
    e.idot = double(sfield(w2, 112, 14)) * p2(-43) * kGpsPi;

    e.iode3 = int(w3.field(6, 10));
    e.omegaDot = double(sfield(w3, 16, 24)) * p2(-43) * kGpsPi;
    e.deltaN = double(sfield(w3, 40, 16)) * p2(-43) * kGpsPi;
    e.cuc = double(sfield(w3, 56, 16)) * p2(-29);
    e.cus = double(sfield(w3, 72, 16)) * p2(-29);
    e.crc = double(sfield(w3, 88, 16)) * p2(-5);
    e.crs = double(sfield(w3, 104, 16)) * p2(-5);
    e.uraIndex = int(w3.field(120, 8));                      // SISA(E1,E5b) index

    e.iodc = int(w4.field(6, 10));
    e.cic = double(sfield(w4, 22, 16)) * p2(-29);
    e.cis = double(sfield(w4, 38, 16)) * p2(-29);
    e.toc = double(w4.field(54, 14)) * 60.0;
    e.af0 = double(sfield(w4, 68, 31)) * p2(-34);
    e.af1 = double(sfield(w4, 99, 21)) * p2(-46);
    e.af2 = double(sfield(w4, 120, 6)) * p2(-59);

    e.tgd = double(sfield(*w5, 57, 10)) * p2(-32);           // BGD(E1,E5b), for single-frequency E1 users
    e.svHealth = int(w5->field(69, 2)) * 2 + int(w5->field(72, 1));   // E1-BHS and E1-BDVS
    e.weekNumber = int(w5->field(73, 12));
    e.sf1 = e.sf2 = e.sf3 = true;
}

} // namespace

void deinterleave(const float* in, float* out)
{
    for (int r = 0; r < kRows; ++r)
        for (int c = 0; c < kCols; ++c)
            out[c * kRows + r] = in[r * kCols + c];
}

void viterbi(const float* soft, uint8_t* bits)
{
    constexpr double kNever = -std::numeric_limits<double>::infinity();
    std::array<double, kStates> metric, next;
    metric.fill(kNever);
    metric[0] = 0.0;
    std::vector<std::array<uint8_t, kStates>> from(kPartBits);
    for (int t = 0; t < kPartBits; ++t) {
        next.fill(kNever);
        for (int st = 0; st < kStates; ++st) {
            if (metric[size_t(st)] == kNever) continue;
            for (int b = 0; b < 2; ++b) {
                const unsigned reg = (unsigned(b) << 6) | unsigned(st);
                const int g1 = parity(reg & 0171);
                const int g2 = parity(reg & 0133) ^ 1;
                const double m = metric[size_t(st)] + soft[2 * t] * (1 - 2 * g1) + soft[2 * t + 1] * (1 - 2 * g2);
                const size_t ns = reg >> 1;
                if (m > next[ns]) {
                    next[ns] = m;
                    from[size_t(t)][ns] = uint8_t(st);
                }
            }
        }
        metric = next;
    }
    int st = 0;                                              // the tail bits return the encoder to zero
    for (int t = kPartBits - 1; t >= 0; --t) {
        bits[t] = uint8_t((st >> 5) & 1);
        st = from[size_t(t)][size_t(st)];
    }
}

uint32_t crc24q(const uint8_t* bits, int n)
{
    uint32_t crc = 0;
    for (int i = 0; i < n; ++i) {
        const uint32_t fb = ((crc >> 23) & 1) ^ bits[i];
        crc = (crc << 1) & 0xFFFFFF;
        if (fb) crc ^= 0x864CFB;                             // G(X) = (1 + X) P(X)
    }
    return crc;
}

QString wordDescription(int type)
{
    switch (type) {
    case 0: return QStringLiteral("Spare word, GST");
    case 1: return QStringLiteral("Ephemeris (1/4)");
    case 2: return QStringLiteral("Ephemeris (2/4)");
    case 3: return QStringLiteral("Ephemeris (3/4) and SISA");
    case 4: return QStringLiteral("SVID, ephemeris (4/4) and clock");
    case 5: return QStringLiteral("Ionosphere, BGD, health and GST");
    case 6: return QStringLiteral("GST-UTC conversion");
    case 7: case 8: case 9: return QStringLiteral("Almanac");
    case 10: return QStringLiteral("Almanac and GST-GPS conversion");
    case 16: return QStringLiteral("Reduced clock and ephemeris");
    case 17: case 18: case 19: case 20: return QStringLiteral("FEC2 Reed-Solomon");
    case 22: return QStringLiteral("ARAIM integrity support");
    case 63: return QStringLiteral("Dummy word: no valid data for this satellite");
    default: return QStringLiteral("Not defined in the OS SIS ICD");
    }
}

std::vector<DecodedField> decodeFields(const InavWord& w)
{
    struct Row { const char* name; int start, len; bool isSigned; int exp; double mul; const char* units; };
    static const char* const kSc = "semicircles";
    static const char* const kScS = "semicircles/s";
    std::vector<Row> rows{{"Word type", 0, 6, false, 0, 1.0, ""}};
    const auto add = [&](std::initializer_list<Row> r) { rows.insert(rows.end(), r); };
    switch (w.type) {
    case 0: add({{"Time", 6, 2, false, 0, 1, ""}, {"Spare", 8, 88, false, 0, 1, ""}, {"WN", 96, 12, false, 0, 1, "weeks"},
                 {"TOW", 108, 20, false, 0, 1, "s"}}); break;
    case 1: add({{"IODnav", 6, 10, false, 0, 1, ""}, {"t0e", 16, 14, false, 0, 60, "s"}, {"M0", 30, 32, true, -31, 1, kSc},
                 {"e", 62, 32, false, -33, 1, ""}, {"sqrt(A)", 94, 32, false, -19, 1, "m^1/2"}, {"Reserved", 126, 2, false, 0, 1, ""}}); break;
    case 2: add({{"IODnav", 6, 10, false, 0, 1, ""}, {"Omega0", 16, 32, true, -31, 1, kSc}, {"i0", 48, 32, true, -31, 1, kSc},
                 {"omega", 80, 32, true, -31, 1, kSc}, {"IDOT", 112, 14, true, -43, 1, kScS}, {"Reserved", 126, 2, false, 0, 1, ""}}); break;
    case 3: add({{"IODnav", 6, 10, false, 0, 1, ""}, {"OMEGA DOT", 16, 24, true, -43, 1, kScS}, {"Delta n", 40, 16, true, -43, 1, kScS},
                 {"C_uc", 56, 16, true, -29, 1, "rad"}, {"C_us", 72, 16, true, -29, 1, "rad"}, {"C_rc", 88, 16, true, -5, 1, "m"},
                 {"C_rs", 104, 16, true, -5, 1, "m"}, {"SISA(E1,E5b)", 120, 8, false, 0, 1, "index"}}); break;
    case 4: add({{"IODnav", 6, 10, false, 0, 1, ""}, {"SVID", 16, 6, false, 0, 1, ""}, {"C_ic", 22, 16, true, -29, 1, "rad"},
                 {"C_is", 38, 16, true, -29, 1, "rad"}, {"t0c", 54, 14, false, 0, 60, "s"}, {"a_f0", 68, 31, true, -34, 1, "s"},
                 {"a_f1", 99, 21, true, -46, 1, "s/s"}, {"a_f2", 120, 6, true, -59, 1, "s/s^2"}, {"Spare", 126, 2, false, 0, 1, ""}}); break;
    case 5: add({{"a_i0", 6, 11, false, -2, 1, "sfu"}, {"a_i1", 17, 11, true, -8, 1, "sfu/deg"}, {"a_i2", 28, 14, true, -15, 1, "sfu/deg^2"},
                 {"Disturbance flags", 42, 5, false, 0, 1, ""}, {"BGD(E1,E5a)", 47, 10, true, -32, 1, "s"}, {"BGD(E1,E5b)", 57, 10, true, -32, 1, "s"},
                 {"E5b-HS", 67, 2, false, 0, 1, ""}, {"E1-B HS", 69, 2, false, 0, 1, ""}, {"E5b-DVS", 71, 1, false, 0, 1, ""},
                 {"E1-B DVS", 72, 1, false, 0, 1, ""}, {"WN", 73, 12, false, 0, 1, "weeks"}, {"TOW", 85, 20, false, 0, 1, "s"},
                 {"Spare", 105, 23, false, 0, 1, ""}}); break;
    case 6: add({{"A0", 6, 32, true, -30, 1, "s"}, {"A1", 38, 24, true, -50, 1, "s/s"}, {"Delta t_LS", 62, 8, true, 0, 1, "s"},
                 {"t0t", 70, 8, false, 0, 3600, "s"}, {"WN0t", 78, 8, false, 0, 1, "weeks"}, {"WN_LSF", 86, 8, false, 0, 1, "weeks"},
                 {"DN", 94, 3, false, 0, 1, "days"}, {"Delta t_LSF", 97, 8, true, 0, 1, "s"}, {"TOW", 105, 20, false, 0, 1, "s"},
                 {"Spare", 125, 3, false, 0, 1, ""}}); break;
    case 10: add({{"Almanac", 6, 80, false, 0, 1, ""}, {"A0G", 86, 16, true, -35, 1, "s"}, {"A1G", 102, 12, true, -51, 1, "s/s"},
                  {"t0G", 114, 8, false, 0, 3600, "s"}, {"WN0G", 122, 6, false, 0, 1, "weeks"}}); break;
    default: add({{"Data", 6, 122, false, 0, 1, ""}}); break;
    }

    std::vector<DecodedField> out;
    for (const Row& r : rows) {
        DecodedField d;
        d.def.key = QLatin1String(r.name);
        d.def.name = QLatin1String(r.name);
        d.def.s1 = r.start + 1;
        d.def.l1 = r.len;
        d.def.isSigned = r.isSigned;
        d.def.scale = p2(r.exp) * r.mul;
        d.def.scaleText = r.exp ? QStringLiteral("2^%1").arg(r.exp) : QString::number(r.mul);
        d.def.units = QLatin1String(r.units);
        for (int i = 0; i < r.len; ++i) d.bitString += QLatin1Char(w.bits[size_t(r.start + i)] ? '1' : '0');
        if (r.len <= 64) {
            d.raw = w.field(r.start, r.len);
            d.rawSigned = r.isSigned ? sfield(w, r.start, r.len) : qint64(d.raw);
            d.value = double(d.rawSigned) * d.def.scale;
            d.valueText = r.exp ? QString::number(d.value, 'g', 12) : QString::number(d.value, 'f', 0);
        }
        if (d.def.key == QLatin1String("Word type")) d.interpretation = wordDescription(w.type);
        else if (d.def.key == QLatin1String("TOW") || d.def.key == QLatin1String("t0e") || d.def.key == QLatin1String("t0c"))
            d.interpretation = nav::towText(d.value);
        out.push_back(d);
    }
    return out;
}

NavChannel decodeChannel(const TrackingChannel& trk, const ReceiverSettings& s)
{
    NavChannel nc;
    nc.channel = trk.channel;
    nc.prn = trk.prn;
    nc.eph.prn = trk.prn;
    nc.epochSec = kE1CodeMs * 1e-3;
    const int n = trk.numMs;
    if (!trk.valid || n < 4 * kInavPartSymbols) {
        nc.status = QStringLiteral("Too little tracking data");
        return nc;
    }

    // ---- symbols: one per code period, so there is no bit synchronisation step
    const int start = std::clamp(s.bitSyncStartMs / kE1CodeMs, 1, n / 2);
    nc.bitSyncStartMs = start;
    nc.firstBitMs = start;
    const int nb = n - start;
    nc.softBits.resize(size_t(nb));
    nc.hardBits.resize(size_t(nb));
    double m1 = 0, m2 = 0;
    for (int j = 0; j < nb; ++j) {
        const float v = trk.IB[size_t(start + j)];
        nc.softBits[size_t(j)] = v;
        nc.hardBits[size_t(j)] = v > 0 ? 1 : 0;
        m1 += std::fabs(double(v));
        m2 += double(v) * v;
    }
    m1 /= nb; m2 /= nb;
    const double var = std::max(1e-30, m2 - m1 * m1);
    const double snr = m1 * m1 / var;
    nc.bitSnrDb = 10.0 * std::log10(snr);
    nc.estimatedBer = 0.5 * std::erfc(std::sqrt(snr / 2.0));

    // ---- page synchronisation: pattern 0101100000 every 250 symbols, either polarity
    static const int pat[kSyncSymbols] = {0, 1, 0, 1, 1, 0, 0, 0, 0, 0};
    const int nc10 = std::max(0, nb - kSyncSymbols + 1);
    nc.preambleCorr.resize(size_t(nc10));
    std::array<int, kInavPartSymbols> hist{};
    for (int j = 0; j < nc10; ++j) {
        int c = 0;
        for (int i = 0; i < kSyncSymbols; ++i) c += (2 * nc.hardBits[size_t(j + i)] - 1) * (1 - 2 * pat[i]);
        nc.preambleCorr[size_t(j)] = c;
        if (std::abs(c) == kSyncSymbols) {
            nc.preambleMatches.push_back(j);
            hist[size_t(j % kInavPartSymbols)]++;
        }
    }
    // Sync-like runs also occur inside the data, and dummy pages repeat them at a fixed offset every page, so counting
    // matches only proposes candidates. The CRC decides, as parity does for the GPS preamble.
    std::vector<int> candidates;
    for (int i = 0; i < kInavPartSymbols; ++i)
        if (hist[size_t(i)] >= 3) candidates.push_back(i);
    std::sort(candidates.begin(), candidates.end(), [&](int a, int b) { return hist[size_t(a)] > hist[size_t(b)]; });
    if (candidates.size() > 8) candidates.resize(8);
    int phase = -1, bestOk = 0;
    for (int c : candidates) {
        std::vector<InavWord> trial = decodeWords(nc, start, c);
        const int ok = int(std::count_if(trial.begin(), trial.end(), [](const InavWord& w) { return w.crcOk; }));
        if (ok > bestOk) {
            bestOk = ok;
            phase = c;
            nc.inavWords = std::move(trial);
        }
    }
    int total = 0;
    for (int v : hist) total += v;
    nc.bitOffset = std::max(phase, 0);
    nc.bitSyncConfidence = total > 0 && phase >= 0 ? double(hist[size_t(phase)]) / total : 0.0;
    nc.bitSyncOk = phase >= 0;
    if (!nc.bitSyncOk) {
        nc.status = QStringLiteral("Page synchronisation failed");
        return nc;
    }
    for (int j = phase; j < nc10; j += kInavPartSymbols)
        if (std::abs(nc.preambleCorr[size_t(j)]) == kSyncSymbols) nc.preambleVerified.push_back(j);
    nc.parityOkWords = bestOk;
    nc.parityTotalWords = int(nc.inavWords.size());
    nc.verifySpan = 4000 / kE1CodeMs;
    for (const InavWord& w : nc.inavWords)
        if (w.crcOk) nc.verifiedMs.push_back(w.startMs);

    // ---- timing: TOW refers to the first chip of the first symbol of the even part
    for (const InavWord& w : nc.inavWords) {
        if (!w.hasTow) continue;
        nc.timingValid = true;
        nc.refMs = w.startMs;
        nc.refTow = w.tow;
        break;
    }
    for (const InavWord& w : nc.inavWords)
        if (w.hasTow && std::fabs(nc.txTimeAtMs(w.startMs) - w.tow) > 1e-6) nc.inavTowErrors++;

    assembleEphemeris(nc);
    nc.status = nc.eph.complete() ? QStringLiteral("Ephemeris complete")
                                  : QStringLiteral("Ephemeris incomplete (%1 of %2 words passed the CRC)")
                                        .arg(nc.parityOkWords).arg(nc.parityTotalWords);
    return nc;
}

} // namespace inav
} // namespace gnss
