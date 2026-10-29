#pragma once

#include <complex>
#include <vector>

namespace gnss {

using cplx = std::complex<double>;

inline cplx cmul(const cplx& a, const cplx& b)
{
    return {a.real() * b.real() - a.imag() * b.imag(), a.real() * b.imag() + a.imag() * b.real()};
}

// Arbitrary-length FFT. Power-of-two sizes use an iterative radix-2
// Cooley-Tukey transform; other sizes (e.g. 38192 samples per millisecond)
// use Bluestein's chirp-z algorithm on a radix-2 transform of size >= 2N-1.
// An Fft object owns scratch memory, so use one object per thread.
class Fft
{
public:
    explicit Fft(int n);
    int size() const { return n_; }
    bool isPowerOfTwo() const { return pow2_; }
    int internalSize() const { return m_; }

    void forward(cplx* data);            // X[k] = sum x[n] exp(-j 2 pi n k / N)
    void inverse(cplx* data);            // includes the 1/N scaling
    void forward(std::vector<cplx>& v) { forward(v.data()); }
    void inverse(std::vector<cplx>& v) { inverse(v.data()); }

private:
    void radix2(cplx* a, bool inverse) const;

    int n_ = 0, m_ = 0;
    bool pow2_ = true;
    std::vector<cplx> tw_;
    std::vector<int> rev_;
    std::vector<cplx> chirp_, chirpSpec_, work_;
};

} // namespace gnss
