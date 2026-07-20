/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_RECEIVER_UTILS_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_RECEIVER_UTILS_H

#include <gnuradio/gr_complex.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

struct prs_rx_config {
    double samp_rate = 10e6;
    int fft_len = 1024;
    int cp_len = 128;
    int active_bins = 600;
    int prs_symbols = 16;
    int preamble_len = 128;
    int preamble_repeats = 16;
    int coarse_sync_len = 839;
    int payload_len = 616;
    int zero_guard_len = 1000;
    int tail_guard_len = 1000;
    uint32_t seed = 13990001;
    int coarse_zc_root = 25;
    int channel_id = 0;
};

int prs_start_offset(const prs_rx_config& cfg);
int prs_len(const prs_rx_config& cfg);
int frame_len(const prs_rx_config& cfg);
std::vector<gr_complex> coarse_sync_sequence(int len, int root = 25);
std::vector<gr_complex> qpsk_pilots(const prs_rx_config& cfg);
std::vector<float> active_frequencies(const prs_rx_config& cfg);
std::vector<float> unwrap_phase(const std::vector<gr_complex>& samples);
bool pdu_get_c32(const pmt::pmt_t& msg, pmt::pmt_t& meta, std::vector<gr_complex>& data);
pmt::pmt_t dict_add_double(pmt::pmt_t dict, const std::string& key, double value);
pmt::pmt_t dict_add_int(pmt::pmt_t dict, const std::string& key, int64_t value);
double dict_ref_double(const pmt::pmt_t& dict, const std::string& key, double fallback);
uint64_t dict_ref_uint64(const pmt::pmt_t& dict, const std::string& key, uint64_t fallback);

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_RECEIVER_UTILS_H */
