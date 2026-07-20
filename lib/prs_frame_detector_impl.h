/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_DETECTOR_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_DETECTOR_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/ofdm_prs_ranging/prs_frame_detector.h>

namespace gr {
namespace ofdm_prs_ranging {

class prs_frame_detector_impl : public prs_frame_detector
{
public:
    prs_frame_detector_impl(double samp_rate,
                            int fft_len,
                            int cp_len,
                            int active_bins,
                            int prs_symbols,
                            int preamble_len,
                            int preamble_repeats,
                            int coarse_sync_len,
                            int zero_guard_len,
                            int tail_guard_len,
                            float threshold,
                            int min_frame_gap,
                            int coarse_zc_root,
                            int channel_id);
    ~prs_frame_detector_impl() override = default;

    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;

private:
    prs_rx_config d_cfg;
    float d_threshold;
    int d_min_frame_gap;
    std::vector<gr_complex> d_coarse;
    std::vector<gr_complex> d_buffer;
    size_t d_buffer_head;
    size_t d_next_scan_index;
    uint64_t d_buffer_abs_start;
    uint64_t d_total_seen;
    uint64_t d_next_frame_id;
    int64_t d_last_frame_start;
    bool d_have_rx_time;
    uint64_t d_rx_time_tag_offset;
    uint64_t d_rx_time_secs;
    double d_rx_time_frac;

    void update_rx_time_tags(uint64_t abs_start, uint64_t abs_stop);
    size_t buffered_size() const { return d_buffer.size() - d_buffer_head; }
    const gr_complex& buffered_sample(size_t index) const
    {
        return d_buffer[d_buffer_head + index];
    }
    void drop_buffer_prefix(size_t count);
    void compact_buffer(size_t threshold);
    float coarse_sync_metric(size_t coarse_index) const;
    bool find_frame(size_t& frame_start_index,
                    size_t& coarse_index,
                    float& preamble_metric,
                    float& coarse_metric);
    void publish_frame(size_t frame_start_index,
                       size_t coarse_index,
                       float preamble_metric,
                       float coarse_metric);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_DETECTOR_IMPL_H */
