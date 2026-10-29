#include "SignalAnalysis.h"
#include "Constants.h"
#include "Fft.h"

#include <algorithm>
#include <cmath>

namespace gnss {
namespace analysis {

RawStats computeStats(const SignalSource& src, qint64 start, qint64 count)
{
    RawStats st;
    if (!src.isOpen()) return st;
    count = std::min(count, src.numSamples() - start);
    if (count <= 0) return st;

    const int chunk = 1 << 20;
    std::vector<float> bi(chunk), bq(chunk);
    double s1 = 0, s2 = 0, s3 = 0, s4 = 0, q1 = 0, q2 = 0, iq = 0;
    double mn = 1e300, mx = -1e300;
    qint64 done = 0;
    while (done < count) {
        const int n = src.read(start + done, int(std::min<qint64>(chunk, count - done)), bi.data(), bq.data());
        if (n <= 0) break;
        for (int i = 0; i < n; ++i) {
            const double v = bi[i];
            s1 += v; s2 += v * v;
            mn = std::min(mn, v); mx = std::max(mx, v);
            st.histogram[int(std::lround(v))]++;
            if (src.isComplex()) { q1 += bq[i]; q2 += double(bq[i]) * bq[i]; iq += v * bq[i]; }
        }
        done += n;
    }
    st.count = done;
    const double N = double(done);
    st.mean = s1 / N;
    const double var = std::max(0.0, s2 / N - st.mean * st.mean);
    st.stddev = std::sqrt(var);
    st.rms = std::sqrt(s2 / N);
    st.minV = mn; st.maxV = mx;
    // central moments from the histogram (exact for integer data)
    for (const auto& kv : st.histogram) {
        const double d = kv.first - st.mean;
        s3 += kv.second * d * d * d;
        s4 += kv.second * d * d * d * d;
    }
    if (var > 0) {
        st.skewness = (s3 / N) / std::pow(var, 1.5);
        st.kurtosis = (s4 / N) / (var * var);
    }
    st.distinctLevels = int(st.histogram.size());
    st.bitsUsed = st.distinctLevels > 0 ? std::log2(double(st.distinctLevels)) : 0;
    qint64 clip = 0;
    if (!st.histogram.empty()) clip = st.histogram.begin()->second + st.histogram.rbegin()->second;
    st.clipFraction = double(clip) / N;
    if (src.isComplex()) {
        st.hasQ = true;
        st.meanQ = q1 / N;
        st.stddevQ = std::sqrt(std::max(0.0, q2 / N - st.meanQ * st.meanQ));
        const double cov = iq / N - st.mean * st.meanQ;
        st.iqCorrelation = (st.stddev > 0 && st.stddevQ > 0) ? cov / (st.stddev * st.stddevQ) : 0;
    }
    return st;
}

static std::vector<double> hann(int n)
{
    std::vector<double> w(n);
    for (int i = 0; i < n; ++i) w[i] = 0.5 - 0.5 * std::cos(kTwoPi * i / (n - 1));
    return w;
}

// Accumulates |FFT|^2 of 'avg' segments starting at 'start' (stride nfft/2).
static int accumulateSpectra(const SignalSource& src, Fft& fft, const std::vector<double>& w,
                             qint64 start, qint64 count, int maxSegments, std::vector<double>& acc)
{
    const int nfft = fft.size();
    const int hop = nfft / 2;
    std::vector<float> bi(nfft), bq(nfft);
    std::vector<cplx> x(nfft);
    int seg = 0;
    for (qint64 pos = start; pos + nfft <= start + count && seg < maxSegments; pos += hop, ++seg) {
        if (src.read(pos, nfft, bi.data(), bq.data()) < nfft) break;
        double mi = 0, mq = 0;
        for (int i = 0; i < nfft; ++i) { mi += bi[i]; mq += bq[i]; }
        mi /= nfft; mq /= nfft;
        for (int i = 0; i < nfft; ++i) x[i] = cplx((bi[i] - mi) * w[i], (bq[i] - mq) * w[i]);
        fft.forward(x);
        for (int i = 0; i < nfft; ++i) acc[i] += std::norm(x[i]);
    }
    return seg;
}

Psd welchPsd(const SignalSource& src, double fs, qint64 start, qint64 count, int nfft)
{
    Psd p;
    if (!src.isOpen()) return p;
    count = std::min(count, src.numSamples() - start);
    if (count < nfft) return p;
    Fft fft(nfft);
    const auto w = hann(nfft);
    std::vector<double> acc(nfft, 0.0);
    const int seg = accumulateSpectra(src, fft, w, start, count, 1 << 30, acc);
    if (seg == 0) return p;
    p.nfft = nfft;
    p.segments = seg;
    p.resolution = fs / nfft;
    double wp = 0;
    for (double v : w) wp += v * v;
    const double norm = 1.0 / (seg * wp * fs);
    if (src.isComplex()) {
        p.freq.resize(nfft); p.powerDb.resize(nfft);
        for (int i = 0; i < nfft; ++i) {
            const int k = (i + nfft / 2) % nfft;     // fftshift
            p.freq[i] = (i - nfft / 2) * fs / nfft;
            p.powerDb[i] = 10.0 * std::log10(std::max(acc[k] * norm, 1e-30));
        }
    } else {
        const int half = nfft / 2 + 1;
        p.freq.resize(half); p.powerDb.resize(half);
        for (int i = 0; i < half; ++i) {
            p.freq[i] = i * fs / nfft;
            p.powerDb[i] = 10.0 * std::log10(std::max(2.0 * acc[i] * norm, 1e-30));
        }
    }
    return p;
}

Spectrogram spectrogram(const SignalSource& src, double fs, qint64 start, qint64 count, int nfft,
                        int nSlices, int avg)
{
    Spectrogram sg;
    if (!src.isOpen()) return sg;
    count = std::min(count, src.numSamples() - start);
    if (count < nfft || nSlices <= 0) return sg;
    Fft fft(nfft);
    const auto w = hann(nfft);
    const bool cpx = src.isComplex();
    const int nf = cpx ? nfft : nfft / 2 + 1;
    sg.nt = nSlices;
    sg.nf = nf;
    sg.db.assign(size_t(nSlices) * nf, -200.0f);
    sg.t0 = double(start) / fs;
    sg.t1 = double(start + count) / fs;
    sg.f0 = cpx ? -fs / 2 : 0;
    sg.f1 = cpx ? fs / 2 : fs / 2;
    const qint64 sliceLen = count / nSlices;
    std::vector<double> acc(nfft);
    for (int t = 0; t < nSlices; ++t) {
        std::fill(acc.begin(), acc.end(), 0.0);
        const qint64 s0 = start + qint64(t) * sliceLen;
        const int seg = accumulateSpectra(src, fft, w, s0, std::max<qint64>(sliceLen, nfft), avg, acc);
        if (seg == 0) continue;
        for (int i = 0; i < nf; ++i) {
            const int k = cpx ? (i + nfft / 2) % nfft : i;
            sg.db[size_t(t) * nf + i] = float(10.0 * std::log10(std::max(acc[k] / seg, 1e-30)));
        }
    }
    return sg;
}

} // namespace analysis
} // namespace gnss
