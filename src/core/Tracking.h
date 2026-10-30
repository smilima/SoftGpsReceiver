#pragma once

#include "Acquisition.h"
#include "SignalSource.h"
#include "Settings.h"

#include <QString>
#include <atomic>
#include <cstdint>
#include <vector>

namespace gnss {

// Everything recorded by one tracking channel, one entry per code period: about
// 1 ms for the GPS C/A code and 4 ms for Galileo E1. All arrays have numMs elements.
struct TrackingChannel
{
    int channel = 0;
    int prn = 0;
    bool valid = false;
    QString status;
    qint64 startSample = 0;
    double initCarrFreq = 0;
    int numMs = 0;
    int periodMs = 1;                     // nominal code period, so 1 entry = periodMs milliseconds
    int codeLength = 1023;                // chips

    std::vector<qint64> blockStart;       // first sample of the integration block
    std::vector<int> blockSize;           // samples integrated
    std::vector<double> codeEpochSample;  // (fractional) sample at which the prompt code epoch occurred
    std::vector<double> carrFreq;         // NCO carrier frequency used for the block (Hz)
    std::vector<double> codeFreq;         // NCO code frequency used for the block (chips/s)
    std::vector<double> remCarrPhase;     // carrier phase at block start (cycles)
    std::vector<double> remCodePhase;     // code phase at block start (chips)
    std::vector<float> IE, IP, IL, QE, QP, QL;   // on E1 these belong to the E1-C pilot
    std::vector<float> IB, QB;            // E1-B data prompt (Galileo only)
    std::vector<float> pllDisc;           // Costas discriminator atan(Q/I) (rad)
    std::vector<float> fllDisc;           // cross/dot FLL discriminator (Hz)
    std::vector<float> dllDisc;           // normalised early-minus-late envelope (chips)
    std::vector<float> carrNco;           // carrier loop filter output (Hz, relative to initial freq)
    std::vector<float> codeNco;           // code loop filter output (chips/s)
    std::vector<float> cn0;               // dB-Hz (moment method)
    std::vector<float> pli;               // phase lock indicator cos(2*dphi)
    std::vector<double> adr;              // accumulated Doppler (cycles, relative to IF)
    std::vector<uint8_t> lock;            // 1 when phase locked

    // loop parameters used (for display)
    double pllWn = 0, pllK1 = 0, pllK2 = 0, fllW0 = 0;
    double dllWn = 0, dllK1 = 0, dllK2 = 0;
    double meanCn0 = 0, lockFraction = 0, finalDoppler = 0;
    double elapsedSec = 0;

    void reserve(int n);
};

struct CorrShapePoint
{
    double offset;     // chips, positive = early (advanced replica)
    double magnitude;  // |I + jQ| averaged non-coherently
    double inPhase;    // I, sign-normalised to the prompt
    double quadrature;
};

class Tracking
{
public:
    static void trackChannel(const SignalSource& src, const ReceiverSettings& s, const AcqResult& acq,
                             qint64 startSample, TrackingChannel& ch,
                             const std::atomic<bool>* cancel, std::atomic<qint64>* msCounter);

    // Re-computes a bank of correlators around the prompt for a given ms using
    // the NCO state stored during tracking (the "correlation triangle").
    static std::vector<CorrShapePoint> correlationShape(const SignalSource& src, const ReceiverSettings& s,
                                                        const TrackingChannel& ch, int ms,
                                                        double span, int taps, int avgMs);
};

} // namespace gnss
