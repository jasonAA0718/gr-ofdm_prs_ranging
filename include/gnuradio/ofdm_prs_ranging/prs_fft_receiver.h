/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_FFT_RECEIVER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_FFT_RECEIVER_H

#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_fft_receiver : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_fft_receiver> sptr;
    static sptr make(double samp_rate = 10e6,
                     int fft_len = 1024,
                     int cp_len = 128,
                     int active_bins = 1024,
                     int prs_symbols = 16);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_FFT_RECEIVER_H */
