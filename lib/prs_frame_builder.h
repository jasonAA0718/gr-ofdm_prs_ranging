/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_BUILDER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_BUILDER_H

#include <gnuradio/gr_complex.h>
#include <cstdint>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

struct prs_frame_config {
    double samp_rate;
    int fft_len;
    int cp_len;
    int active_bins;
    int prs_symbols;
    int preamble_len;
    int preamble_repeats;
    int coarse_sync_len;
    int payload_len;
    int zero_guard_len;
    int tail_guard_len;
    float tx_amp;
    uint32_t seed;
    int coarse_zc_root = 25;
};

struct prs_frame {
    std::vector<gr_complex> samples;
    int prs_start = 0;
    int prs_len = 0;
    int payload_start = 0;
    int payload_len = 0;
};

class prs_frame_builder
{
public:
    static prs_frame build(const prs_frame_config& cfg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_BUILDER_H */
