/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_FFT_ZC_FRAME_DETECTOR_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_FFT_ZC_FRAME_DETECTOR_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/fft/fft.h>
#include <gnuradio/ofdm_prs_ranging/prs_fft_zc_frame_detector.h>
#include <memory>
#include <mutex>

namespace gr {
namespace ofdm_prs_ranging {

class prs_fft_zc_frame_detector_impl : public prs_fft_zc_frame_detector
{
public:
    prs_fft_zc_frame_detector_impl(double samp_rate,
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
                                   int channel_id,
                                   bool time_gating,
                                   double reply_delay_s,
                                   double window_before_s,
                                   double window_after_s,
                                   float zc_threshold,
                                   int correlation_fft_len,
                                   int zc_search_before,
                                   int zc_search_after,
                                   bool cfo_compensation,
                                   float peak_ratio_threshold,
                                   bool enable_profiling);
    ~prs_fft_zc_frame_detector_impl() override = default;

    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;

private:
    prs_rx_config d_cfg;
    float d_preamble_threshold;
    float d_zc_threshold;
    int d_min_frame_gap;
    std::vector<gr_complex> d_coarse;
    int d_correlation_fft_len;
    int d_zc_search_before;
    int d_zc_search_after;
    bool d_cfo_compensation;
    float d_peak_ratio_threshold;
    std::unique_ptr<gr::fft::fft_complex_fwd> d_zc_fft;
    std::unique_ptr<gr::fft::fft_complex_rev> d_zc_ifft;
    std::vector<gr_complex> d_zc_spectrum;
    std::vector<gr_complex> d_buffer;
    size_t d_buffer_head;
    size_t d_next_gate_index;
    size_t d_next_coarse_index;
    uint64_t d_buffer_abs_start;
    uint64_t d_total_seen;
    uint64_t d_next_frame_id;
    int64_t d_last_frame_start;
    bool d_have_rx_time;
    uint64_t d_rx_time_tag_offset;
    uint64_t d_rx_time_secs;
    double d_rx_time_frac;
    bool d_time_gating;
    double d_reply_delay_s;
    double d_window_before_s;
    double d_window_after_s;
    bool d_enable_profiling;
    uint64_t d_gate_scan_duration_ns;
    uint64_t d_zc_input_duration_ns;
    uint64_t d_zc_fft_duration_ns;
    uint64_t d_zc_multiply_duration_ns;
    uint64_t d_zc_ifft_duration_ns;
    uint64_t d_zc_peak_duration_ns;
    bool d_gate_armed;
    size_t d_armed_preamble_index;
    float d_armed_preamble_metric;
    float d_last_zc_peak_ratio;
    int64_t d_last_zc_gate_offset_samples;
    std::mutex d_cfo_mutex;
    bool d_have_tracked_cfo;
    double d_tracked_cfo_hz;
    double d_last_detection_cfo_hz;
    bool d_last_cfo_was_tracked;

    struct correlation_window {
        double start;
        double end;
        uint64_t attempt_id;
        bool saw_preamble;
        bool saw_coarse;
        bool completed;
        float preamble_metric;
        float coarse_metric;
    };
    std::mutex d_window_mutex;
    std::vector<correlation_window> d_windows;

    void handle_tx_time(const pmt::pmt_t& msg);
    void handle_prs_cfo(const pmt::pmt_t& msg);
    void update_rx_time_tags(uint64_t abs_start, uint64_t abs_stop);
    double sample_time(uint64_t abs_offset) const;
    size_t buffered_size() const { return d_buffer.size() - d_buffer_head; }
    const gr_complex& buffered_sample(size_t index) const
    {
        return d_buffer[d_buffer_head + index];
    }
    void reset_buffer(uint64_t abs_start);
    void drop_buffer_prefix(size_t count);
    void compact_buffer(size_t threshold);
    void process_samples(const gr_complex* samples, size_t count, uint64_t abs_start);
    bool find_preamble_gate(size_t& preamble_index,
                            float& metric,
                            bool require_threshold);
    bool fft_zc_search(size_t first_coarse,
                       size_t last_coarse,
                       double cfo_hz,
                       size_t& best_coarse,
                       float& best_metric,
                       float& peak_ratio);
    bool fft_zc_cfo_search(size_t first_coarse,
                           size_t last_coarse,
                           size_t& best_coarse,
                           float& best_metric,
                           float& peak_ratio,
                           double& selected_cfo_hz,
                           bool& used_tracked_cfo);
    void record_candidate(uint64_t abs_start, float preamble_metric, float coarse_metric);
    bool complete_attempt(double frame_time, uint64_t& attempt_id);
    void publish_failed_attempt(const correlation_window& window);
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

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_FFT_ZC_FRAME_DETECTOR_IMPL_H */
