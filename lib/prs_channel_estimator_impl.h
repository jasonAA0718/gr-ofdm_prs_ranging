/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/ofdm_prs_ranging/prs_channel_estimator.h>

namespace gr {
namespace ofdm_prs_ranging {

class prs_channel_estimator_impl : public prs_channel_estimator
{
public:
    prs_channel_estimator_impl(double samp_rate,
                               int fft_len,
                               int active_bins,
                               int prs_symbols,
                               uint32_t seed);
    ~prs_channel_estimator_impl() override = default;

private:
    prs_rx_config d_cfg;
    std::vector<gr_complex> d_pilot_reciprocals;
    std::vector<gr_complex> d_channel;
    std::vector<double> d_channel_energy;
    void handle_symbols(pmt::pmt_t msg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_CHANNEL_ESTIMATOR_IMPL_H */
