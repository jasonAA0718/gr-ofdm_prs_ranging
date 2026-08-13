/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_PHASE_SLOPE_ESTIMATOR_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_PHASE_SLOPE_ESTIMATOR_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/ofdm_prs_ranging/prs_phase_slope_estimator.h>

namespace gr {
namespace ofdm_prs_ranging {

class prs_phase_slope_estimator_impl : public prs_phase_slope_estimator
{
public:
    prs_phase_slope_estimator_impl(double samp_rate,
                                   int fft_len,
                                   int active_bins,
                                   float max_residual_rms);
    ~prs_phase_slope_estimator_impl() override = default;

private:
    prs_rx_config d_cfg;
    float d_max_residual_rms;
    std::vector<float> d_freq;
    void handle_channel(pmt::pmt_t msg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_PHASE_SLOPE_ESTIMATOR_IMPL_H */
