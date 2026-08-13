/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_BURST_SOURCE_H
#define INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_BURST_SOURCE_H

#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API pluto_prs_burst_source : virtual public gr::sync_block
{
public:
    using sptr = std::shared_ptr<pluto_prs_burst_source>;

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
                     float tx_amp = 0.6f,
                     uint32_t seed = 13990001,
                     int default_packet_type = 1,
                     uint64_t guard_samples = 0,
                     int coarse_zc_root = 25);

    virtual int frame_len() const = 0;
    virtual int prs_start() const = 0;
    virtual int prs_len() const = 0;
    virtual std::vector<gr_complex> frame_samples() const = 0;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_BURST_SOURCE_H */
