#pragma once

#include "SignalSource.h"

#include <map>
#include <vector>

namespace gnss {

struct RawStats
{
    qint64 count = 0;
    double mean = 0, stddev = 0, rms = 0, minV = 0, maxV = 0;
    double skewness = 0, kurtosis = 0;          // kurtosis = 3 for a Gaussian
    int distinctLevels = 0;
    double bitsUsed = 0;                        // log2(number of distinct levels)
    double clipFraction = 0;                    // fraction of samples at min or max level
    std::map<int, qint64> histogram;            // value -> count (I channel)
    // Optional quadrature-channel statistics
    bool hasQ = false;
    double meanQ = 0, stddevQ = 0;
    double iqCorrelation = 0;                   // normalised I/Q correlation (quadrature balance)
};

struct Psd
{
    std::vector<double> freq;                   // Hz (absolute, 0..fs/2 for real input)
    std::vector<double> powerDb;                // dB (relative)
    int nfft = 0;
    int segments = 0;
    double resolution = 0;                      // Hz per bin
};

struct Spectrogram
{
    int nt = 0, nf = 0;
    std::vector<float> db;                      // row-major [t * nf + f]
    double t0 = 0, t1 = 0;                      // seconds
    double f0 = 0, f1 = 0;                      // Hz
};

namespace analysis {

RawStats computeStats(const SignalSource& src, qint64 start, qint64 count);

// Welch averaged periodogram with a Hann window and 50 % overlap.
Psd welchPsd(const SignalSource& src, double fs, qint64 start, qint64 count, int nfft);

// Waterfall over [start, start+count): nSlices spectra, each averaging 'avg' FFTs.
Spectrogram spectrogram(const SignalSource& src, double fs, qint64 start, qint64 count, int nfft,
                        int nSlices, int avg);

} // namespace analysis
} // namespace gnss
