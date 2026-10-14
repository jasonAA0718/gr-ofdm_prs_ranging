/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_PHY_GEOMETRY_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_PHY_GEOMETRY_H

namespace gr {
namespace ofdm_prs_ranging {
namespace phy {

// Production burst geometry. These samples are included in the UHD TX time
// reference and in RX frame-start reconstruction; they are never configurable.
constexpr int leading_guard_samples = 1000;
constexpr int tail_guard_samples = 1000;

constexpr int coarse_start_samples(int preamble_len, int preamble_repeats)
{
    return leading_guard_samples + preamble_len * preamble_repeats;
}

constexpr int prs_start_samples(int preamble_len, int preamble_repeats, int zc_len)
{
    return coarse_start_samples(preamble_len, preamble_repeats) + zc_len;
}

constexpr int frame_samples(int preamble_len,
                            int preamble_repeats,
                            int zc_len,
                            int fft_len,
                            int cp_len,
                            int prs_symbols)
{
    return prs_start_samples(preamble_len, preamble_repeats, zc_len) +
           prs_symbols * (fft_len + cp_len) + tail_guard_samples;
}

} // namespace phy
} // namespace ofdm_prs_ranging
} // namespace gr
#endif
