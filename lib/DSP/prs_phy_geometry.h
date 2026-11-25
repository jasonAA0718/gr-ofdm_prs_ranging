/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_PHY_GEOMETRY_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_PHY_GEOMETRY_H

namespace gr {
namespace ofdm_prs_ranging {
namespace phy {

// Production burst geometry. These samples are included in the UHD TX time
// reference and in RX frame-start reconstruction; they are never configurable.
constexpr int fft_len = 512;
constexpr int cp_len = 64;
constexpr int active_bins = 512;
constexpr int prs_symbols = 127;
constexpr int preamble_len = 128;
constexpr int preamble_repeats = 17;
constexpr int coarse_sync_len = 819;
constexpr int seed = 13990001;
constexpr int mc_ds_gold_code_id = 2;
constexpr int poll_zc_root = 25;
constexpr int response_zc_root = 29;

constexpr int zc_root(bool response)
{
    return response ? response_zc_root : poll_zc_root;
}

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
