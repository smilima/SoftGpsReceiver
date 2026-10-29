#pragma once

#include <cstdint>
#include <vector>

namespace gnss {

// IS-GPS-200 Table 3-Ia: code phase assignments for the C/A code.
struct CaCodeInfo
{
    int prn;
    int tap1, tap2;      // G2 phase selector taps (exclusive-or of two G2 stages)
    int g2Delay;         // equivalent G2 delay in chips
    int icdOctal;        // first 10 chips in octal, as published in the ICD
};

class CaCode
{
public:
    static constexpr int kMaxPrn = 32;

    static const CaCodeInfo& info(int prn);

    // Logic chips (0/1) generated with the G2 phase-selector taps (the hardware method).
    static std::vector<uint8_t> bits(int prn);
    // Logic chips generated from the G2 delay (the equivalent shifted-sequence method).
    static std::vector<uint8_t> bitsByDelay(int prn);
    // +1/-1 chips: logic 0 -> +1, logic 1 -> -1.
    static std::vector<int8_t> chips(int prn);

    static std::vector<uint8_t> g1Sequence();
    static std::vector<uint8_t> g2Sequence();

    static int firstTenChipsOctal(const std::vector<uint8_t>& bits);

    // Periodic (circular) correlation of two +/-1 sequences of equal length.
    static std::vector<int> circularCorrelation(const std::vector<int8_t>& a, const std::vector<int8_t>& b);
};

} // namespace gnss
