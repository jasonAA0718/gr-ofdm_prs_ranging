/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_FFT_RECEIVER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_FFT_RECEIVER_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/fft/fft.h>
#include <gnuradio/ofdm_prs_ranging/prs_fft_receiver.h>
#include <memory>

namespace gr {
namespace ofdm_prs_ranging {

class prs_fft_receiver_impl : public prs_fft_receiver
{
public:
    prs_fft_receiver_impl(double samp_rate,
                          int fft_len,
                          int cp_len,
                          int active_bins,
                          int prs_symbols);
    ~prs_fft_receiver_impl() override = default;

private:
    prs_rx_config d_cfg;
    std::unique_ptr<gr::fft::fft_complex_fwd> d_fft;
    void handle_frame(pmt::pmt_t msg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_FFT_RECEIVER_IMPL_H */
