/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_PHASE_SLOPE_ESTIMATOR_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_PHASE_SLOPE_ESTIMATOR_H

#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_phase_slope_estimator : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_phase_slope_estimator> sptr;
    static sptr make(double samp_rate = 10e6,
                     int fft_len = 1024,
                     int active_bins = 1024,
                     float max_residual_rms = 1.0f);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_PHASE_SLOPE_ESTIMATOR_H */
