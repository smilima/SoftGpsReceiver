#pragma once

#include "NavMessage.h"

namespace gnss {
namespace inav {

// I/NAV as broadcast on E1-B (OS SIS ICD sections 4.1.4 and 4.3): 1 s page parts of 10 sync
// symbols plus 240 interleaved, rate 1/2 convolutionally coded symbols. An even and an odd
// part together carry one 128-bit word protected by a CRC-24Q.
constexpr int kSyncSymbols = 10;
constexpr int kCodedSymbols = 240;
constexpr int kPartBits = 120;

// 30 column x 8 row block deinterleaver
void deinterleave(const float* in, float* out);
// Soft-decision Viterbi decoder (k = 7, G1 = 171o, G2 = 133o with the second branch inverted).
// Positive soft symbols mean logic 0. Each part starts and ends in the zero state.
void viterbi(const float* soft, uint8_t* bits);
// CRC-24Q remainder of n bits
uint32_t crc24q(const uint8_t* bits, int n);

NavChannel decodeChannel(const TrackingChannel& trk, const ReceiverSettings& s);

// For display: the fields of a word with their scale factors, and what a word type carries
std::vector<DecodedField> decodeFields(const InavWord& w);
QString wordDescription(int type);

} // namespace inav
} // namespace gnss
