#pragma once

#include <QString>
#include <QVector>

class QSettings;

namespace gnss {

enum class SampleFormat { Int8Real = 0, UInt8Real, Int16Real, Int8IQ, Int16IQ };
QString sampleFormatName(SampleFormat f);

// Sign relationship between the IF carrier offset and the true Doppler.
// "Inverted" occurs with high-side LO injection in the front end.
enum class DopplerSign { Auto = 0, Normal, Inverted };

struct ReceiverSettings
{
    // ---------------- Input file
    QString fileName;
    SampleFormat format = SampleFormat::Int8Real;
    double samplingFreq = 38.192e6;     // Hz
    double ifFreq = 9.55e6;             // Hz
    qint64 skipBytes = 0;
    int msToProcess = 0;                // 0 = whole file

    // ---------------- Acquisition
    QVector<int> acqPrns;               // PRNs to search (empty = 1..32)
    int acqStartMs = 0;                 // where in the file the acquisition block starts
    double acqSearchBand = 10000.0;     // +/- Hz around the IF
    double acqStep = 500.0;             // Doppler bin width (Hz)
    int acqNonCoherent = 4;             // number of 1-ms correlations summed non-coherently
    double acqThreshold = 2.5;          // peak / second-peak ratio
    int fineFreqMs = 40;                // data used for fine frequency estimation

    // ---------------- Tracking
    int numChannels = 12;
    double dllBandwidth = 2.0;          // Hz
    double dllDamping = 0.7;
    double dllSpacing = 0.5;            // early/late offset from prompt (chips)
    double pllBandwidth = 20.0;         // Hz
    double pllDamping = 0.7;
    double fllBandwidth = 10.0;         // Hz (FLL assist during pull-in)
    int fllAssistMs = 500;              // FLL assist duration
    bool carrierAiding = false;         // carrier aiding of the DLL
    int cn0WindowMs = 100;              // M2M4 C/N0 estimator window

    // ---------------- Navigation
    int bitSyncStartMs = 1000;          // ignore pull-in transient before this
    int navSolPeriodMs = 500;
    double elevationMask = 5.0;         // degrees
    double minCn0 = 25.0;               // dB-Hz required to use a channel in a fix
    bool useIono = true;
    bool useTropo = true;
    bool useEarthRotation = true;
    bool elevationWeighting = false;
    DopplerSign dopplerSign = DopplerSign::Auto;

    ReceiverSettings();

    int samplesPerCode() const;                   // samples in one C/A code period (rounded)
    bool isComplex() const;
    int bytesPerSample() const;                   // per (complex) sample
    QVector<int> prnList() const;
    double dopplerSignValue() const;              // +1 / -1 (Auto -> +1)

    // Extract the sampling and IF frequencies from a file name such as
    // "GPSdata-DiscreteComponents-fs38_192-if9_55.bin" (values in MHz).
    static bool parseFileName(const QString& path, double* fs, double* fif, SampleFormat* fmt = nullptr);

    QString summary() const;
    void save(QSettings& qs) const;
    void load(QSettings& qs);
};

} // namespace gnss
