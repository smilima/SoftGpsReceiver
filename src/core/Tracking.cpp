#include "Tracking.h"
#include "CaCode.h"
#include "Constants.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gnss {

namespace {

constexpr int kLutBits = 10;
constexpr int kLutSize = 1 << kLutBits;

struct CarrierLut
{
    float c[kLutSize];
    float s[kLutSize];
    CarrierLut()
    {
        for (int i = 0; i < kLutSize; ++i) {
            const double a = kTwoPi * (i + 0.5) / kLutSize;
            c[i] = float(std::cos(a));
            s[i] = float(std::sin(a));
        }
    }
};

const CarrierLut& lut()
{
    static const CarrierLut l;
    return l;
}

inline double frac(double x) { return x - std::floor(x); }

inline uint32_t phaseToFixed(double cycles)
{
    return uint32_t(uint64_t(frac(cycles) * 4294967296.0));
}

// Early/prompt/late correlation over one block. The carrier NCO is a 32-bit
// phase accumulator addressing a sine/cosine table; the code NCO is a
// double-precision phase, sampled with a padded code table so that the
// early and late replicas can run off either end of the code period.
template <bool Complex>
void correlate(const float* sI, const float* sQ, int n, double remCode, double codeStep, double d,
               const float* codeP, double remCarr, double carrStep, float acc[6])
{
    const CarrierLut& L = lut();
    uint32_t ph = phaseToFixed(remCarr);
    const uint32_t dph = uint32_t(uint64_t(std::llround(frac(carrStep) * 4294967296.0)));
    float ie = 0, qe = 0, ip = 0, qp = 0, il = 0, ql = 0;
    for (int i = 0; i < n; ++i) {
        const double cp = remCode + double(i) * codeStep;
        const float ce = codeP[int(cp + d + 1.0)];
        const float cpv = codeP[int(cp + 1.0)];
        const float cl = codeP[int(cp - d + 1.0)];
        const unsigned idx = ph >> (32 - kLutBits);
        ph += dph;
        const float c = L.c[idx], sn = L.s[idx];
        float bi, bq;
        if (Complex) {
            bi = sI[i] * c + sQ[i] * sn;
            bq = sQ[i] * c - sI[i] * sn;
        } else {
            bi = sI[i] * c;
            bq = -sI[i] * sn;
        }
        ie += ce * bi;  qe += ce * bq;
        ip += cpv * bi; qp += cpv * bq;
        il += cl * bi;  ql += cl * bq;
    }
    acc[0] = ie; acc[1] = qe; acc[2] = ip; acc[3] = qp; acc[4] = il; acc[5] = ql;
}

inline double safeAtan(double num, double den)
{
    if (den == 0.0) return num >= 0 ? kPi / 2 : -kPi / 2;
    return std::atan(num / den);
}

} // namespace

void TrackingChannel::reserve(int n)
{
    blockStart.reserve(n); blockSize.reserve(n); codeEpochSample.reserve(n);
    carrFreq.reserve(n); codeFreq.reserve(n); remCarrPhase.reserve(n); remCodePhase.reserve(n);
    IE.reserve(n); IP.reserve(n); IL.reserve(n); QE.reserve(n); QP.reserve(n); QL.reserve(n);
    pllDisc.reserve(n); fllDisc.reserve(n); dllDisc.reserve(n); carrNco.reserve(n); codeNco.reserve(n);
    cn0.reserve(n); pli.reserve(n); adr.reserve(n); lock.reserve(n);
}

void Tracking::trackChannel(const SignalSource& src, const ReceiverSettings& s, const AcqResult& acq,
                            qint64 startSample, TrackingChannel& ch,
                            const std::atomic<bool>* cancel, std::atomic<qint64>* msCounter)
{
    QElapsedTimer timer;
    timer.start();
    const double fs = s.samplingFreq;
    const bool cpx = src.isComplex();
    const qint64 totalSamples = src.numSamples();
    ch.prn = acq.prn;
    ch.startSample = startSample;
    ch.initCarrFreq = acq.fineFreq;

    int maxMs = int(double(totalSamples - startSample) / (fs / 1000.0)) - 2;
    if (s.msToProcess > 0) maxMs = std::min(maxMs, s.msToProcess);
    if (maxMs <= 0) { ch.status = QStringLiteral("No data"); return; }
    ch.reserve(maxMs);

    // padded code: codeP[k] = chip[(k-1) mod 1023], index = floor(phase) + 1
    const auto chips = CaCode::chips(ch.prn);
    std::vector<float> codeP(kCaCodeLength + 3);
    for (int k = 0; k < kCaCodeLength + 3; ++k)
        codeP[k] = chips[(k - 1 + kCaCodeLength) % kCaCodeLength];

    // ---- loop filter coefficients (2nd-order PLL + 1st-order FLL assist, 2nd-order DLL)
    const double zp = s.pllDamping;
    const double wnp = s.pllBandwidth * 8.0 * zp / (4.0 * zp * zp + 1.0);
    const double w0f = s.fllBandwidth / 0.25;
    const double zd = s.dllDamping;
    const double wnd = s.dllBandwidth * 8.0 * zd / (4.0 * zd * zd + 1.0);
    ch.pllWn = wnp; ch.pllK1 = 2.0 * zp * wnp; ch.pllK2 = wnp * wnp; ch.fllW0 = w0f;
    ch.dllWn = wnd; ch.dllK1 = 2.0 * zd * wnd; ch.dllK2 = wnd * wnd;
    const double d = s.dllSpacing;
    const double dopSign = s.dopplerSignValue();

    const double carrBase = acq.fineFreq;
    double carrFreq = carrBase;
    double codeFreq = kCaChipRate;
    double remCode = 0.0, remCarr = 0.0;
    double carrInteg = 0.0, codeInteg = 0.0;
    double adr = 0.0;
    double sI2 = 0.0, sQ2 = 0.0;
    float prevI = 0, prevQ = 0;
    float cn0 = 0;
    const int cn0Win = std::max(20, s.cn0WindowMs);
    std::vector<double> pwr(cn0Win, 0.0);
    qint64 pos = startSample;

    std::vector<float> bufI(size_t(fs / 1000.0 * 1.2) + 64), bufQ(bufI.size());
    int reported = 0;

    for (int ms = 0; ms < maxMs; ++ms) {
        if (cancel && cancel->load(std::memory_order_relaxed)) { ch.status = QStringLiteral("Cancelled"); break; }
        const double codeStep = codeFreq / fs;
        const int blk = int(std::ceil((double(kCaCodeLength) - remCode) / codeStep));
        if (blk <= 0 || pos + blk > totalSamples) break;
        if (size_t(blk) > bufI.size()) { bufI.resize(blk); bufQ.resize(blk); }
        src.read(pos, blk, bufI.data(), cpx ? bufQ.data() : nullptr);

        const double carrStep = carrFreq / fs;
        float acc[6];
        if (cpx) correlate<true>(bufI.data(), bufQ.data(), blk, remCode, codeStep, d, codeP.data(), remCarr, carrStep, acc);
        else     correlate<false>(bufI.data(), nullptr, blk, remCode, codeStep, d, codeP.data(), remCarr, carrStep, acc);
        const float IE = acc[0], QE = acc[1], IP = acc[2], QP = acc[3], IL = acc[4], QL = acc[5];
        const double T = double(blk) / fs;

        ch.blockStart.push_back(pos);
        ch.blockSize.push_back(blk);
        ch.codeEpochSample.push_back(double(pos) - remCode / codeStep);
        ch.remCodePhase.push_back(remCode);
        ch.remCarrPhase.push_back(remCarr);
        ch.carrFreq.push_back(carrFreq);
        ch.codeFreq.push_back(codeFreq);
        ch.adr.push_back(adr);
        ch.IE.push_back(IE); ch.IP.push_back(IP); ch.IL.push_back(IL);
        ch.QE.push_back(QE); ch.QP.push_back(QP); ch.QL.push_back(QL);

        // advance NCO phases to the start of the next block
        remCode = remCode + blk * codeStep - kCaCodeLength;
        remCarr = frac(remCarr + carrFreq * blk / fs);
        adr += (carrFreq - s.ifFreq) * T;
        pos += blk;

        // ---- carrier loop: Costas PLL with FLL assist during pull-in
        const double ep = safeAtan(QP, IP);                      // rad
        double efHz = 0.0;
        if (ms > 0) {
            const double cross = double(prevI) * QP - double(prevQ) * IP;
            const double dot = double(prevI) * IP + double(prevQ) * QP;
            efHz = safeAtan(cross, dot) / (kTwoPi * T);
        }
        prevI = IP; prevQ = QP;
        const bool fllOn = ms < s.fllAssistMs;
        carrInteg += T * (wnp * wnp * ep + (fllOn ? w0f * kTwoPi * efHz : 0.0));
        const double ncoHz = (carrInteg + 2.0 * zp * wnp * ep) / kTwoPi;
        carrFreq = carrBase + ncoHz;

        // ---- code loop: normalised early-minus-late envelope DLL
        const double E = std::hypot(double(IE), double(QE));
        const double Lm = std::hypot(double(IL), double(QL));
        const double ed = (E + Lm) > 0 ? (1.0 - d) * (E - Lm) / (E + Lm) : 0.0;  // chips
        codeInteg += T * wnd * wnd * ed;
        const double codeNco = codeInteg + 2.0 * zd * wnd * ed;               // chips/s
        const double aid = s.carrierAiding ? dopSign * (carrFreq - s.ifFreq) / kCarrierToCode : 0.0;
        codeFreq = kCaChipRate + codeNco + aid;

        ch.pllDisc.push_back(float(ep));
        ch.fllDisc.push_back(float(efHz));
        ch.dllDisc.push_back(float(ed));
        ch.carrNco.push_back(float(ncoHz));
        ch.codeNco.push_back(float(codeNco));

        // ---- C/N0 (M2M4 moment method) and phase lock indicator
        const double p = double(IP) * IP + double(QP) * QP;
        pwr[ms % cn0Win] = p;
        if (ms + 1 >= cn0Win && (ms + 1) % 20 == 0) {
            double m2 = 0, m4 = 0;
            for (double v : pwr) { m2 += v; m4 += v * v; }
            m2 /= cn0Win; m4 /= cn0Win;
            const double pd = std::sqrt(std::max(0.0, 2.0 * m2 * m2 - m4));
            const double pn = m2 - pd;
            if (pd > 0 && pn > 0) cn0 = float(10.0 * std::log10(pd / (pn * T)));
            else if (pd > 0) cn0 = 60.0f;
            else cn0 = 0.0f;
        }
        const double a = 0.02;
        sI2 = (1 - a) * sI2 + a * double(IP) * IP;
        sQ2 = (1 - a) * sQ2 + a * double(QP) * QP;
        const float pli = (sI2 + sQ2) > 0 ? float((sI2 - sQ2) / (sI2 + sQ2)) : 0.0f;
        ch.cn0.push_back(cn0);
        ch.pli.push_back(pli);
        ch.lock.push_back(pli > 0.6f ? 1 : 0);

        if (msCounter && (ms - reported) >= 200) {
            msCounter->fetch_add(ms - reported, std::memory_order_relaxed);
            reported = ms;
        }
    }
    if (msCounter) msCounter->fetch_add(int(ch.IP.size()) - reported, std::memory_order_relaxed);

    ch.numMs = int(ch.IP.size());
    ch.valid = ch.numMs > 0;
    if (ch.status.isEmpty()) ch.status = ch.valid ? QStringLiteral("Tracked") : QStringLiteral("Failed");
    // summary
    const int from = std::min(ch.numMs, std::max(0, s.bitSyncStartMs));
    double sc = 0;
    int nl = 0, cnt = 0;
    for (int i = from; i < ch.numMs; ++i) { sc += ch.cn0[i]; nl += ch.lock[i]; ++cnt; }
    ch.meanCn0 = cnt ? sc / cnt : 0;
    ch.lockFraction = cnt ? double(nl) / cnt : 0;
    ch.finalDoppler = ch.numMs ? ch.carrFreq.back() - s.ifFreq : 0;
    ch.elapsedSec = timer.elapsed() / 1000.0;
}

std::vector<CorrShapePoint> Tracking::correlationShape(const SignalSource& src, const ReceiverSettings& s,
                                                       const TrackingChannel& ch, int ms,
                                                       double span, int taps, int avgMs)
{
    std::vector<CorrShapePoint> out;
    if (!ch.valid || ms < 0 || ms >= ch.numMs || taps < 2) return out;
    const double fs = s.samplingFreq;
    const auto chips = CaCode::chips(ch.prn);
    out.resize(taps);
    for (int t = 0; t < taps; ++t) {
        out[t].offset = -span + 2.0 * span * t / (taps - 1);
        out[t].magnitude = out[t].inPhase = out[t].quadrature = 0;
    }
    const int last = std::min(ch.numMs, ms + std::max(1, avgMs));
    int used = 0;
    std::vector<float> bi, bq, rI, rQ;
    for (int m = ms; m < last; ++m) {
        const int blk = ch.blockSize[m];
        bi.resize(blk); bq.resize(blk); rI.resize(blk); rQ.resize(blk);
        if (src.read(ch.blockStart[m], blk, rI.data(), rQ.data()) < blk) break;
        const double carrStep = ch.carrFreq[m] / fs;
        for (int i = 0; i < blk; ++i) {
            const double ph = kTwoPi * (ch.remCarrPhase[m] + i * carrStep);
            const double c = std::cos(ph), sn = std::sin(ph);
            bi[i] = float(rI[i] * c + rQ[i] * sn);
            bq[i] = float(rQ[i] * c - rI[i] * sn);
        }
        const double codeStep = ch.codeFreq[m] / fs;
        const double sgn = ch.IP[m] >= 0 ? 1.0 : -1.0;
        for (int t = 0; t < taps; ++t) {
            double ai = 0, aq = 0;
            for (int i = 0; i < blk; ++i) {
                const double cp = ch.remCodePhase[m] + i * codeStep + out[t].offset;
                int k = int(std::floor(cp)) % kCaCodeLength;
                if (k < 0) k += kCaCodeLength;
                ai += chips[k] * bi[i];
                aq += chips[k] * bq[i];
            }
            out[t].magnitude += std::hypot(ai, aq);
            out[t].inPhase += sgn * ai;
            out[t].quadrature += sgn * aq;
        }
        ++used;
    }
    if (used > 0)
        for (auto& p : out) { p.magnitude /= used; p.inPhase /= used; p.quadrature /= used; }
    return out;
}

} // namespace gnss
