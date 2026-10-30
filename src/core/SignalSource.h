#pragma once

#include "Settings.h"

#include <QFile>
#include <QString>

namespace gnss {

// Memory-mapped access to a recorded IF sample file. All read functions are
// const and thread-safe so that every tracking channel can read in parallel.
class SignalSource
{
public:
    SignalSource() = default;
    ~SignalSource();
    SignalSource(const SignalSource&) = delete;
    SignalSource& operator=(const SignalSource&) = delete;

    bool open(const QString& path, SampleFormat fmt, qint64 skipBytes, QString* error);
    void close();

    bool isOpen() const { return data_ != nullptr; }
    QString path() const { return path_; }
    qint64 fileSize() const { return fileSize_; }
    qint64 numSamples() const { return numSamples_; }
    SampleFormat format() const { return fmt_; }
    bool isComplex() const { return fmt_ == SampleFormat::Int8IQ || fmt_ == SampleFormat::Int16IQ; }

    // Reads up to 'count' samples starting at sample index 'start'. The in-phase
    // (or real) component goes to outI; the quadrature component to outQ when
    // outQ is non-null (zero for real formats). Returns the number of samples read.
    int read(qint64 start, int count, float* outI, float* outQ = nullptr) const;

private:
    QFile file_;
    QString path_;
    const uchar* data_ = nullptr;
    qint64 fileSize_ = 0;
    qint64 skip_ = 0;
    qint64 numSamples_ = 0;
    int bytesPerSample_ = 1;
    int samplesPerByte_ = 1;
    SampleFormat fmt_ = SampleFormat::Int8Real;
};

} // namespace gnss
