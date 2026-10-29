#include "Fft.h"
#include "Constants.h"

#include <algorithm>
#include <cmath>

namespace gnss {

static int nextPow2(int n)
{
    int m = 1;
    while (m < n) m <<= 1;
    return m;
}

Fft::Fft(int n) : n_(n)
{
    pow2_ = n > 0 && (n & (n - 1)) == 0;
    m_ = pow2_ ? n : nextPow2(2 * n - 1);

    tw_.resize(std::max(1, m_ / 2));
    for (int k = 0; k < m_ / 2; ++k) {
        const double a = -kTwoPi * double(k) / double(m_);
        tw_[k] = cplx(std::cos(a), std::sin(a));
    }
    rev_.resize(m_);
    int bits = 0;
    while ((1 << bits) < m_) ++bits;
    for (int i = 0; i < m_; ++i) {
        int r = 0;
        for (int b = 0; b < bits; ++b)
            if (i & (1 << b)) r |= 1 << (bits - 1 - b);
        rev_[i] = r;
    }

    if (!pow2_) {
        // chirp w[k] = exp(+j pi k^2 / N); k^2 is reduced modulo 2N for accuracy
        chirp_.resize(n);
        const long long twoN = 2LL * n;
        for (int k = 0; k < n; ++k) {
            const long long k2 = (long long)k * k % twoN;
            const double a = kPi * double(k2) / double(n);
            chirp_[k] = cplx(std::cos(a), std::sin(a));
        }
        chirpSpec_.assign(m_, cplx(0, 0));
        chirpSpec_[0] = chirp_[0];
        for (int k = 1; k < n; ++k) {
            chirpSpec_[k] = chirp_[k];
            chirpSpec_[m_ - k] = chirp_[k];
        }
        radix2(chirpSpec_.data(), false);
        work_.resize(m_);
    }
}

void Fft::radix2(cplx* a, bool inv) const
{
    const int m = m_;
    for (int i = 0; i < m; ++i) {
        const int j = rev_[i];
        if (i < j) std::swap(a[i], a[j]);
    }
    for (int len = 2; len <= m; len <<= 1) {
        const int half = len >> 1;
        const int step = m / len;
        for (int i = 0; i < m; i += len) {
            cplx* p = a + i;
            cplx* q = a + i + half;
            for (int k = 0; k < half; ++k) {
                cplx w = tw_[k * step];
                if (inv) w = std::conj(w);
                const cplx u = p[k];
                const cplx v = cmul(q[k], w);
                p[k] = u + v;
                q[k] = u - v;
            }
        }
    }
}

void Fft::forward(cplx* x)
{
    if (pow2_) {
        radix2(x, false);
        return;
    }
    // Bluestein: X[k] = conj(w[k]) * sum_n (x[n] conj(w[n])) w[k-n]
    for (int k = 0; k < n_; ++k) work_[k] = cmul(x[k], std::conj(chirp_[k]));
    std::fill(work_.begin() + n_, work_.end(), cplx(0, 0));
    radix2(work_.data(), false);
    for (int k = 0; k < m_; ++k) work_[k] = cmul(work_[k], chirpSpec_[k]);
    radix2(work_.data(), true);
    const double s = 1.0 / double(m_);
    for (int k = 0; k < n_; ++k) x[k] = cmul(work_[k], std::conj(chirp_[k])) * s;
}

void Fft::inverse(cplx* x)
{
    for (int k = 0; k < n_; ++k) x[k] = std::conj(x[k]);
    forward(x);
    const double s = 1.0 / double(n_);
    for (int k = 0; k < n_; ++k) x[k] = std::conj(x[k]) * s;
}

} // namespace gnss
