/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_WINDOW_CAPTURE_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_WINDOW_CAPTURE_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/ofdm_prs_ranging/prs_window_capture.h>
#include <pmt/pmt.h>
#include <unordered_set>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

class prs_window_capture_impl : public prs_window_capture
{
public:
    prs_window_capture_impl(double samp_rate,
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
                            int candidate_margin_before,
                            int candidate_margin_after,
                            int queue_depth,
                            bool acquisition_enabled,
                            bool enable_profiling);
    ~prs_window_capture_impl() override = default;

    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;

private:
    enum class acquisition_state { idle, preamble_active, capturing, lockout };

    struct rx_time_anchor {
        uint64_t offset;
        double time;
    };

    struct window_spec {
        bool by_time;
        uint64_t start_sample;
        uint64_t end_sample;
        double start_time;
        double end_time;
        uint64_t window_id;
        uint64_t attempt_id;
    };

    struct active_interval {
        uint64_t start;
        uint64_t end;
        uint64_t window_id;
        uint64_t attempt_id;
    };

    prs_rx_config d_cfg;
    float d_threshold;
    int d_min_frame_gap;
    int d_margin_before;
    int d_margin_after;
    int d_queue_depth;
    std::atomic<bool> d_acquisition_enabled;
    bool d_enable_profiling;

    std::vector<gr_complex> d_ring;
    size_t d_ring_mask;
    size_t d_ring_head;
    size_t d_ring_count;
    uint64_t d_ring_abs_start;
    bool d_have_stream_position;
    uint64_t d_next_abs_sample;

    bool d_metric_valid;
    uint64_t d_metric_start;
    gr_complex d_metric_corr;
    double d_metric_first_power;
    double d_metric_second_power;
    uint64_t d_metric_updates_since_rebuild;

    acquisition_state d_state;
    uint64_t d_next_candidate_id;
    uint64_t d_first_crossing;
    uint64_t d_preamble_monitor_end;
    uint64_t d_candidate_start;
    uint64_t d_candidate_end;
    uint64_t d_lockout_until;
    uint64_t d_threshold_crossing_count;
    float d_max_preamble_metric;
    bool d_candidate_window_enabled;
    uint64_t d_candidate_window_id;
    uint64_t d_candidate_attempt_id;
    std::vector<gr_complex> d_candidate_samples;

    std::deque<rx_time_anchor> d_rx_time_anchors;
    std::mutex d_window_mutex;
    std::vector<window_spec> d_windows;

    std::mutex d_credit_mutex;
    std::unordered_set<uint64_t> d_outstanding_candidates;
    uint64_t d_window_queue_full_count;

    uint64_t d_total_samples_consumed;
    uint64_t d_work_calls;
    uint64_t d_work_duration_ns;
    bool d_inside_work;
    uint64_t d_current_work_samples;
    std::chrono::steady_clock::time_point d_current_work_start;

    void handle_window(const pmt::pmt_t& message);
    void handle_slot_release(const pmt::pmt_t& message);
    void update_rx_time_tags(uint64_t abs_start, uint64_t abs_stop);
    bool sample_time(uint64_t abs_offset, double& result) const;
    bool sample_offset(double time, uint64_t& result) const;
    std::vector<active_interval> active_intervals(uint64_t abs_start, uint64_t abs_stop);

    void reset_stream(uint64_t abs_start);
    void append_ring_chunk(const gr_complex* samples, size_t count, uint64_t abs_start);
    bool ring_contains(uint64_t abs_start, uint64_t abs_end) const;
    const gr_complex* ring_data(uint64_t abs_index) const;
    void copy_ring_range(uint64_t abs_start,
                         uint64_t abs_end,
                         std::vector<gr_complex>& destination) const;
    void append_candidate_range(uint64_t abs_start, uint64_t abs_end);

    void invalidate_metric();
    bool update_metric(uint64_t metric_start, float& metric, bool& threshold_passed);
    void rebuild_metric(uint64_t metric_start);
    void advance_metric(uint64_t metric_start);
    bool scan_idle(uint64_t newest_start,
                   uint64_t newest_end,
                   const active_interval* interval,
                   uint64_t& consumed_until);
    void monitor_candidate(uint64_t newest_start, uint64_t newest_end);

    bool start_candidate(uint64_t crossing_index,
                         float metric,
                         const active_interval* interval,
                         uint64_t newest_abs);
    void finish_candidate();
    void publish_queue_full(uint64_t candidate_id, pmt::pmt_t metadata);
    pmt::pmt_t candidate_metadata(uint64_t candidate_id,
                                  uint64_t queue_depth,
                                  const char* event_name) const;
    bool reserve_credit(uint64_t candidate_id, uint64_t& queue_depth);
    static const char* state_name(acquisition_state state);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_WINDOW_CAPTURE_IMPL_H */
