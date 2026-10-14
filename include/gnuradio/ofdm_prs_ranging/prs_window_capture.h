/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_WINDOW_CAPTURE_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_WINDOW_CAPTURE_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

/*! \brief Lightweight repeated-preamble acquisition and candidate-window capture. */
class OFDM_PRS_RANGING_API prs_window_capture : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<prs_window_capture>;

    static sptr make(double samp_rate = 30e6,
                     int fft_len = 512,
                     int cp_len = 64,
                     int active_bins = 512,
                     int prs_symbols = 127,
                     int preamble_len = 128,
                     int preamble_repeats = 17,
                     int coarse_sync_len = 819,
                     float threshold = 0.09f,
                     int min_frame_gap = 10000,
                     int candidate_margin_before = 4096,
                     int candidate_margin_after = 4096,
                     int queue_depth = 3,
                     bool acquisition_enabled = true,
                     bool enable_profiling = false);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_WINDOW_CAPTURE_H */
