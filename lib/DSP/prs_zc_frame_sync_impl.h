/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/ofdm_prs_ranging/prs_zc_frame_sync.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

class prs_zc_frame_sync_impl : public prs_zc_frame_sync
{
public:
    prs_zc_frame_sync_impl(double samp_rate,
                           int fft_len,
                           int cp_len,
                           int active_bins,
                           int prs_symbols,
                           int preamble_len,
                           int preamble_repeats,
                           int coarse_sync_len,
                           float preamble_threshold,
                           float zc_threshold,
                           int coarse_zc_root,
                           int channel_id,
                           bool enable_profiling);
    ~prs_zc_frame_sync_impl() override = default;

private:
    struct release_guard {
        prs_zc_frame_sync_impl* owner;
        uint64_t candidate_id;
        bool valid;
        ~release_guard() noexcept;
    };

    struct cfo_reference {
        bool valid = false;
        size_t index = 0;
        gr_complex correlation = gr_complex(0.0f, 0.0f);
        float magnitude = 0.0f;
        float metric = 0.0f;
        double cfo_hz = 0.0;
    };

    prs_rx_config d_cfg;
    float d_preamble_threshold;
    float d_zc_threshold;
    bool d_enable_profiling;
    uint64_t d_next_recv_id;
    std::vector<gr_complex> d_coarse;
    std::vector<gr_complex> d_scratch;

    void handle_candidate(const pmt::pmt_t& message);
    void publish_slot_release(uint64_t candidate_id) noexcept;
    bool metadata_matches_geometry(const pmt::pmt_t& meta) const;
    cfo_reference select_cfo_reference(const gr_complex* samples, size_t size) const;
    void correct_cfo(const gr_complex* samples, size_t size, double cfo_hz);
    bool search_zc(size_t search_start,
                   size_t search_end,
                   size_t& selected_index,
                   float& selected_metric) const;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_IMPL_H */
