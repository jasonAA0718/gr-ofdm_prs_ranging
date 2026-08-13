/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_BURST_SOURCE_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_BURST_SOURCE_IMPL_H

#include "prs_frame_builder.h"
#include <gnuradio/ofdm_prs_ranging/pluto_prs_burst_source.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

class pluto_prs_burst_source_impl : public pluto_prs_burst_source
{
private:
    struct pending_burst {
        uint8_t packet_type;
        uint32_t poll_frame_id;
        uint32_t response_frame_id;
        uint32_t reply_delay_samples;
        uint64_t guard_samples;
    };

    double d_samp_rate;
    int d_fft_len;
    int d_cp_len;
    int d_active_bins;
    int d_prs_symbols;
    int d_preamble_len;
    int d_preamble_repeats;
    int d_coarse_sync_len;
    int d_zero_guard_len;
    int d_tail_guard_len;
    float d_tx_amp;
    uint32_t d_seed;
    uint8_t d_default_packet_type;
    uint64_t d_default_guard_samples;
    int d_coarse_zc_root;

    std::vector<gr_complex> d_frame;
    std::vector<gr_complex> d_burst_frame;
    int d_prs_start = 0;
    int d_prs_len = 0;
    int d_payload_start = 0;
    int d_payload_len = 0;

    std::mutex d_queue_mutex;
    std::deque<pending_burst> d_pending;
    uint32_t d_next_frame_id = 1;
    bool d_active = false;
    pending_burst d_current{};
    uint64_t d_guard_remaining = 0;
    size_t d_burst_offset = 0;

    void validate_parameters() const;
    prs_frame_config frame_config() const;
    void build_frame();
    void prepare_burst_frame();
    void handle_trigger(pmt::pmt_t msg);
    bool start_next_burst(uint64_t absolute_offset);
    void add_burst_tags(uint64_t absolute_offset);
    void publish_tx_event(uint64_t absolute_offset);

public:
    pluto_prs_burst_source_impl(double samp_rate,
                                int fft_len,
                                int cp_len,
                                int active_bins,
                                int prs_symbols,
                                int preamble_len,
                                int preamble_repeats,
                                int coarse_sync_len,
                                int zero_guard_len,
                                int tail_guard_len,
                                float tx_amp,
                                uint32_t seed,
                                int default_packet_type,
                                uint64_t guard_samples,
                                int coarse_zc_root);

    int frame_len() const override { return static_cast<int>(d_frame.size()); }
    int prs_start() const override { return d_prs_start; }
    int prs_len() const override { return d_prs_len; }
    std::vector<gr_complex> frame_samples() const override { return d_frame; }

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_BURST_SOURCE_IMPL_H */
