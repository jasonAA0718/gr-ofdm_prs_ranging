/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_RECEIVER_UTILS_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_RECEIVER_UTILS_H

#include "prs_payload_codec.h"
#include "prs_phy_geometry.h"
#include <gnuradio/gr_complex.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <string>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

struct prs_rx_config {
    double samp_rate = 10e6;
    int fft_len = phy::fft_len;
    int cp_len = phy::cp_len;
    int active_bins = phy::active_bins;
    int prs_symbols = phy::prs_symbols;
    int preamble_len = phy::preamble_len;
    int preamble_repeats = phy::preamble_repeats;
    int coarse_sync_len = phy::coarse_sync_len;
    int payload_len = 0;
    uint32_t seed = phy::seed;
    int coarse_zc_root = phy::poll_zc_root;
    int channel_id = 0;
    int mc_ds_gold_code_id = phy::mc_ds_gold_code_id;
};

struct prs_cfo_estimate {
    double hz = 0.0;
    double coherence = 0.0;
    bool valid = false;
};

int prs_start_offset(const prs_rx_config& cfg);
int prs_len(const prs_rx_config& cfg);
int frame_len(const prs_rx_config& cfg);
std::vector<gr_complex> coarse_sync_sequence(int root);
double coarse_sync_energy(const std::vector<gr_complex>& sequence);
std::vector<gr_complex> prs_pilots(const prs_rx_config& cfg);
std::vector<float> active_frequencies(const prs_rx_config& cfg);
std::vector<float> mc_ds_pilot_frequencies(const prs_rx_config& cfg);
prs_cfo_estimate estimate_prs_cp_cfo(const gr_complex* frame,
                                     size_t frame_size,
                                     const prs_rx_config& cfg,
                                     double unwrap_reference_hz);
std::vector<float> unwrap_phase(const std::vector<gr_complex>& samples);
std::vector<float> unwrap_phase(const gr_complex* samples, size_t size);
bool pdu_get_c32(const pmt::pmt_t& msg, pmt::pmt_t& meta, std::vector<gr_complex>& data);
bool pdu_get_c32_view(const pmt::pmt_t& msg,
                      pmt::pmt_t& meta,
                      const gr_complex*& data,
                      size_t& size);
pmt::pmt_t dict_add_double(pmt::pmt_t dict, const std::string& key, double value);
pmt::pmt_t dict_add_int(pmt::pmt_t dict, const std::string& key, int64_t value);
double dict_ref_double(const pmt::pmt_t& dict, const std::string& key, double fallback);
uint64_t
dict_ref_uint64(const pmt::pmt_t& dict, const std::string& key, uint64_t fallback);

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_RECEIVER_UTILS_H */
