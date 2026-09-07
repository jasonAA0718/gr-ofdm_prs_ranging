/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_channel_estimator : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_channel_estimator> sptr;
    static sptr make(double samp_rate = 10e6,
                     int fft_len = 512,
                     int active_bins = 512,
                     int prs_symbols = 127,
                     uint32_t seed = 13990001,
                     bool enable_profiling = false,
                     int mc_ds_gold_code_id = 2);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_H */
