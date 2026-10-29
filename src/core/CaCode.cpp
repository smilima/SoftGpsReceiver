#include "CaCode.h"
#include "Constants.h"

#include <stdexcept>

namespace gnss {

static const CaCodeInfo kTable[CaCode::kMaxPrn] = {
    { 1, 2, 6,   5, 01440}, { 2, 3, 7,   6, 01620}, { 3, 4, 8,   7, 01710}, { 4, 5, 9,   8, 01744},
    { 5, 1, 9,  17, 01133}, { 6, 2,10,  18, 01455}, { 7, 1, 8, 139, 01131}, { 8, 2, 9, 140, 01454},
    { 9, 3,10, 141, 01626}, {10, 2, 3, 251, 01504}, {11, 3, 4, 252, 01642}, {12, 5, 6, 254, 01750},
    {13, 6, 7, 255, 01764}, {14, 7, 8, 256, 01772}, {15, 8, 9, 257, 01775}, {16, 9,10, 258, 01776},
    {17, 1, 4, 469, 01156}, {18, 2, 5, 470, 01467}, {19, 3, 6, 471, 01633}, {20, 4, 7, 472, 01715},
    {21, 5, 8, 473, 01746}, {22, 6, 9, 474, 01763}, {23, 1, 3, 509, 01063}, {24, 4, 6, 512, 01706},
    {25, 5, 7, 513, 01743}, {26, 6, 8, 514, 01761}, {27, 7, 9, 515, 01770}, {28, 8,10, 516, 01774},
    {29, 1, 6, 859, 01127}, {30, 2, 7, 860, 01453}, {31, 3, 8, 861, 01625}, {32, 4, 9, 862, 01712},
};

const CaCodeInfo& CaCode::info(int prn)
{
    if (prn < 1 || prn > kMaxPrn) throw std::out_of_range("PRN out of range");
    return kTable[prn - 1];
}

std::vector<uint8_t> CaCode::g1Sequence()
{
    // G1 = 1 + x^3 + x^10, register initialised to all ones
    uint8_t r[11];
    for (int i = 1; i <= 10; ++i) r[i] = 1;
    std::vector<uint8_t> out(kCaCodeLength);
    for (int i = 0; i < kCaCodeLength; ++i) {
        out[i] = r[10];
        const uint8_t fb = r[3] ^ r[10];
        for (int j = 10; j > 1; --j) r[j] = r[j - 1];
        r[1] = fb;
    }
    return out;
}

std::vector<uint8_t> CaCode::g2Sequence()
{
    // G2 = 1 + x^2 + x^3 + x^6 + x^8 + x^9 + x^10
    uint8_t r[11];
    for (int i = 1; i <= 10; ++i) r[i] = 1;
    std::vector<uint8_t> out(kCaCodeLength);
    for (int i = 0; i < kCaCodeLength; ++i) {
        out[i] = r[10];
        const uint8_t fb = r[2] ^ r[3] ^ r[6] ^ r[8] ^ r[9] ^ r[10];
        for (int j = 10; j > 1; --j) r[j] = r[j - 1];
        r[1] = fb;
    }
    return out;
}

std::vector<uint8_t> CaCode::bits(int prn)
{
    const CaCodeInfo& ci = info(prn);
    uint8_t g1[11], g2[11];
    for (int i = 1; i <= 10; ++i) { g1[i] = 1; g2[i] = 1; }
    std::vector<uint8_t> out(kCaCodeLength);
    for (int i = 0; i < kCaCodeLength; ++i) {
        out[i] = g1[10] ^ (g2[ci.tap1] ^ g2[ci.tap2]);
        const uint8_t f1 = g1[3] ^ g1[10];
        const uint8_t f2 = g2[2] ^ g2[3] ^ g2[6] ^ g2[8] ^ g2[9] ^ g2[10];
        for (int j = 10; j > 1; --j) { g1[j] = g1[j - 1]; g2[j] = g2[j - 1]; }
        g1[1] = f1;
        g2[1] = f2;
    }
    return out;
}

std::vector<uint8_t> CaCode::bitsByDelay(int prn)
{
    const CaCodeInfo& ci = info(prn);
    const auto g1 = g1Sequence();
    const auto g2 = g2Sequence();
    std::vector<uint8_t> out(kCaCodeLength);
    for (int i = 0; i < kCaCodeLength; ++i)
        out[i] = g1[i] ^ g2[(i - ci.g2Delay + 1023 * 2) % kCaCodeLength];
    return out;
}

std::vector<int8_t> CaCode::chips(int prn)
{
    const auto b = bits(prn);
    std::vector<int8_t> out(kCaCodeLength);
    for (int i = 0; i < kCaCodeLength; ++i) out[i] = b[i] ? -1 : 1;
    return out;
}

int CaCode::firstTenChipsOctal(const std::vector<uint8_t>& b)
{
    int v = 0;
    for (int i = 0; i < 10 && i < int(b.size()); ++i) v = (v << 1) | (b[i] & 1);
    return v;   // value; print with %o to get the ICD representation
}

std::vector<int> CaCode::circularCorrelation(const std::vector<int8_t>& a, const std::vector<int8_t>& b)
{
    const int n = int(a.size());
    std::vector<int> r(n, 0);
    for (int lag = 0; lag < n; ++lag) {
        int s = 0;
        for (int i = 0; i < n; ++i) s += a[i] * b[(i + lag) % n];
        r[lag] = s;
    }
    return r;
}

} // namespace gnss
