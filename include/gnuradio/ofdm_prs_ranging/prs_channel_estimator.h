/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_H

#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_channel_estimator : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_channel_estimator> sptr;
    static sptr make(double samp_rate = 10e6,
                     int fft_len = 1024,
                     int active_bins = 600,
                     int prs_symbols = 16,
                     uint32_t seed = 13990001);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_H */
