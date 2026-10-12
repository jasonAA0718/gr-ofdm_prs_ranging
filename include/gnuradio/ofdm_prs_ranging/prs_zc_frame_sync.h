/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

/*! \brief Message-only CFO-assisted ZC synchronization and frame extraction. */
class OFDM_PRS_RANGING_API prs_zc_frame_sync : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<prs_zc_frame_sync>;

    static sptr make(double samp_rate = 30e6,
                     int fft_len = 512,
                     int cp_len = 64,
                     int active_bins = 512,
                     int prs_symbols = 127,
                     int preamble_len = 128,
                     int preamble_repeats = 17,
                     int coarse_sync_len = 819,
                     int zero_guard_len = 1000,
                     int tail_guard_len = 1000,
                     float preamble_threshold = 0.09f,
                     float zc_threshold = 0.35f,
                     int coarse_zc_root = 25,
                     int channel_id = 0,
                     bool enable_profiling = false);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_H */
