/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_DETECTOR_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_DETECTOR_H

#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_frame_detector : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_frame_detector> sptr;
    static sptr make(double samp_rate = 10e6,
                     int fft_len = 1024,
                     int cp_len = 128,
                     int active_bins = 1024,
                     int prs_symbols = 16,
                     int preamble_len = 128,
                     int preamble_repeats = 16,
                     int coarse_sync_len = 839,
                     int zero_guard_len = 1000,
                     int tail_guard_len = 1000,
                     float threshold = 0.35f,
                     int min_frame_gap = 10000,
                     int coarse_zc_root = 25,
                     int channel_id = 0,
                     bool time_gating = false,
                     double reply_delay_s = 0.05,
                     double window_before_s = 0.0002,
                     double window_after_s = 0.004,
                     float zc_threshold = 0.35f);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_DETECTOR_H */
