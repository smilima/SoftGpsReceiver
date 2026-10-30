#pragma once

#include <cstdint>
#include <vector>

namespace gnss {

// Galileo E1 Open Service ranging codes (memory codes, so they come from a table).
class GalileoCode
{
public:
    // +1/-1 chips of the 4092-chip primary codes: logic 0 -> +1, logic 1 -> -1. SVID 1..36.
    static std::vector<int8_t> e1b(int svid);
    static std::vector<int8_t> e1c(int svid);

    // Sine-phased BOC(1,1) replica with two values per chip: a chip c becomes +c, -c.
    static std::vector<int8_t> boc11(const std::vector<int8_t>& chips);
};

} // namespace gnss
