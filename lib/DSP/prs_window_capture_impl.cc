/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_timing_helper.h"
#include "prs_window_capture_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

namespace {

double pmt_time_value(const pmt::pmt_t& value)
{
    if (pmt::is_real(value)) {
        return pmt::to_double(value);
    }
    if (pmt::is_uint64(value)) {
        return static_cast<double>(pmt::to_uint64(value));
    }
    if (pmt::is_integer(value)) {
        return static_cast<double>(pmt::to_long(value));
    }
    if (pmt::is_tuple(value) && pmt::length(value) >= 2) {
        const auto secs = pmt::tuple_ref(value, 0);
        const auto frac = pmt::tuple_ref(value, 1);
        if ((pmt::is_integer(secs) || pmt::is_uint64(secs)) && pmt::is_real(frac)) {
            const double whole = pmt::is_uint64(secs)
                                     ? static_cast<double>(pmt::to_uint64(secs))
                                     : static_cast<double>(pmt::to_long(secs));
            return whole + pmt::to_double(frac);
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

pmt::pmt_t time_tuple(double time)
{
    const double whole = std::floor(time);
    return pmt::make_tuple(pmt::from_uint64(static_cast<uint64_t>(whole)),
                           pmt::from_double(time - whole));
}

} // namespace

prs_window_capture::sptr prs_window_capture::make(double samp_rate,
                                                  int fft_len,
                                                  int cp_len,
                                                  int active_bins,
                                                  int prs_symbols,
                                                  int preamble_len,
                                                  int preamble_repeats,
                                                  int coarse_sync_len,
                                                  float threshold,
                                                  int min_frame_gap,
                                                  int candidate_margin_before,
                                                  int candidate_margin_after,
                                                  int queue_depth,
                                                  bool acquisition_enabled,
                                                  bool enable_profiling)
{
    return gnuradio::make_block_sptr<prs_window_capture_impl>(samp_rate,
                                                              fft_len,
                                                              cp_len,
                                                              active_bins,
                                                              prs_symbols,
                                                              preamble_len,
                                                              preamble_repeats,
                                                              coarse_sync_len,
                                                              threshold,
                                                              min_frame_gap,
                                                              candidate_margin_before,
                                                              candidate_margin_after,
                                                              queue_depth,
                                                              acquisition_enabled,
                                                              enable_profiling);
}

prs_window_capture_impl::prs_window_capture_impl(double samp_rate,
                                                 int fft_len,
                                                 int cp_len,
                                                 int active_bins,
                                                 int prs_symbols,
                                                 int preamble_len,
                                                 int preamble_repeats,
                                                 int coarse_sync_len,
                                                 float threshold,
                                                 int min_frame_gap,
                                                 int candidate_margin_before,
                                                 int candidate_margin_after,
                                                 int queue_depth,
                                                 bool acquisition_enabled,
                                                 bool enable_profiling)
    : gr::block("prs_window_capture",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(0, 0, 0)),
      d_threshold(threshold),
      d_min_frame_gap(min_frame_gap),
      d_margin_before(candidate_margin_before),
      d_margin_after(candidate_margin_after),
      d_queue_depth(queue_depth),
      d_acquisition_enabled(acquisition_enabled),
      d_enable_profiling(enable_profiling),
      d_ring_mask(0),
      d_ring_head(0),
      d_ring_count(0),
      d_ring_abs_start(0),
      d_have_stream_position(false),
      d_next_abs_sample(0),
      d_metric_valid(false),
      d_metric_start(0),
      d_metric_corr(0.0f, 0.0f),
      d_metric_first_power(0.0),
      d_metric_second_power(0.0),
      d_metric_updates_since_rebuild(0),
      d_state(acquisition_state::idle),
      d_next_candidate_id(0),
      d_first_crossing(0),
      d_preamble_monitor_end(0),
      d_candidate_start(0),
      d_candidate_end(0),
      d_lockout_until(0),
      d_threshold_crossing_count(0),
      d_max_preamble_metric(0.0f),
      d_candidate_window_enabled(false),
      d_candidate_window_id(0),
      d_candidate_attempt_id(0),
      d_window_queue_full_count(0),
      d_total_samples_consumed(0),
      d_work_calls(0),
      d_work_duration_ns(0),
      d_inside_work(false),
      d_current_work_samples(0)
{
    if (samp_rate <= 0.0 || fft_len <= 0 || cp_len < 0 || active_bins <= 0 ||
        prs_symbols <= 0 || preamble_len <= 0 || preamble_repeats < 2 ||
        coarse_sync_len <= 0) {
        throw std::invalid_argument("invalid PRS window-capture geometry");
    }
    if (threshold <= 0.0f || threshold > 1.0f) {
        throw std::invalid_argument("preamble threshold must be in (0, 1]");
    }
    if (min_frame_gap < 0 || candidate_margin_before < 0 || candidate_margin_after < 0 ||
        queue_depth <= 0) {
        throw std::invalid_argument("capture margins, gap and queue depth must be valid");
    }

    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.cp_len = cp_len;
    d_cfg.active_bins = active_bins;
    d_cfg.prs_symbols = prs_symbols;
    d_cfg.preamble_len = preamble_len;
    d_cfg.preamble_repeats = preamble_repeats;
    d_cfg.coarse_sync_len = coarse_sync_len;

    const size_t preamble_total =
        static_cast<size_t>(preamble_len) * static_cast<size_t>(preamble_repeats);
    const size_t history = static_cast<size_t>(phy::leading_guard_samples) + preamble_total +
                           static_cast<size_t>(candidate_margin_before);
    const size_t required_ring_size = history + preamble_total + 64U;
    size_t ring_size = 1U;
    while (ring_size < required_ring_size) {
        ring_size <<= 1U;
    }
    // Mirror the circular storage so every retained range can be addressed as
    // one contiguous span, including ranges that cross the physical wrap.
    d_ring.resize(2U * ring_size);
    d_ring_mask = ring_size - 1U;

    message_port_register_in(pmt::mp("window_in"));
    message_port_register_in(pmt::mp("slot_release"));
    message_port_register_out(pmt::mp("candidate_out"));
    message_port_register_out(pmt::mp("event_out"));
    message_port_register_out(pmt::mp("timing_out"));
    set_msg_handler(pmt::mp("window_in"),
                    [this](const pmt::pmt_t& msg) { handle_window(msg); });
    set_msg_handler(pmt::mp("slot_release"),
                    [this](const pmt::pmt_t& msg) { handle_slot_release(msg); });
}

void prs_window_capture_impl::forecast(int noutput_items,
                                       gr_vector_int& ninput_items_required)
{
    (void)noutput_items;
    ninput_items_required[0] = 1;
}

void prs_window_capture_impl::handle_window(const pmt::pmt_t& message)
{
    const pmt::pmt_t msg = pmt::is_dict(message)
                               ? message
                               : (pmt::is_pair(message) ? pmt::car(message) : message);
    if (!pmt::is_dict(msg)) {
        return;
    }

    const auto enabled = pmt::dict_ref(msg, pmt::mp("enabled"), pmt::PMT_NIL);
    const bool has_sample_bounds =
        !pmt::is_null(pmt::dict_ref(msg, pmt::mp("start_sample"), pmt::PMT_NIL)) &&
        !pmt::is_null(pmt::dict_ref(msg, pmt::mp("end_sample"), pmt::PMT_NIL));
    const bool has_time_bounds =
        !pmt::is_null(pmt::dict_ref(msg, pmt::mp("start_time"), pmt::PMT_NIL)) &&
        !pmt::is_null(pmt::dict_ref(msg, pmt::mp("end_time"), pmt::PMT_NIL));

    if (!pmt::is_null(enabled) && !has_sample_bounds && !has_time_bounds) {
        d_acquisition_enabled.store(pmt::to_bool(enabled), std::memory_order_relaxed);
        return;
    }

    window_spec window{};
    window.window_id = dict_ref_uint64(msg, "window_id", 0);
    window.attempt_id = dict_ref_uint64(msg, "attempt_id", window.window_id);
    if (has_sample_bounds) {
        window.by_time = false;
        window.start_sample = dict_ref_uint64(msg, "start_sample", 0);
        window.end_sample = dict_ref_uint64(msg, "end_sample", 0);
        if (window.start_sample >= window.end_sample) {
            return;
        }
    } else if (has_time_bounds) {
        window.by_time = true;
        window.start_time =
            pmt_time_value(pmt::dict_ref(msg, pmt::mp("start_time"), pmt::PMT_NIL));
        window.end_time =
            pmt_time_value(pmt::dict_ref(msg, pmt::mp("end_time"), pmt::PMT_NIL));
        if (!std::isfinite(window.start_time) || !std::isfinite(window.end_time) ||
            window.start_time >= window.end_time) {
            return;
        }
    } else {
        return;
    }

    std::lock_guard<std::mutex> lock(d_window_mutex);
    d_windows.push_back(window);
}

void prs_window_capture_impl::handle_slot_release(const pmt::pmt_t& message)
{
    const pmt::pmt_t msg = pmt::is_dict(message)
                               ? message
                               : (pmt::is_pair(message) ? pmt::car(message) : message);
    uint64_t candidate_id = std::numeric_limits<uint64_t>::max();
    if (pmt::is_dict(msg)) {
        candidate_id = dict_ref_uint64(msg, "candidate_id", candidate_id);
    } else if (pmt::is_uint64(msg)) {
        candidate_id = pmt::to_uint64(msg);
    } else if (pmt::is_integer(msg)) {
        const auto value = pmt::to_long(msg);
        if (value >= 0) {
            candidate_id = static_cast<uint64_t>(value);
        }
    }
    if (candidate_id == std::numeric_limits<uint64_t>::max()) {
        return;
    }

    std::lock_guard<std::mutex> lock(d_credit_mutex);
    d_outstanding_candidates.erase(candidate_id);
}

void prs_window_capture_impl::update_rx_time_tags(uint64_t abs_start, uint64_t abs_stop)
{
    std::vector<tag_t> tags;
    get_tags_in_range(tags, 0, abs_start, abs_stop, pmt::mp("rx_time"));
    for (const auto& tag : tags) {
        const double time = pmt_time_value(tag.value);
        if (!std::isfinite(time)) {
            continue;
        }
        if (!d_rx_time_anchors.empty() && d_rx_time_anchors.back().offset == tag.offset) {
            d_rx_time_anchors.back().time = time;
        } else {
            d_rx_time_anchors.push_back({ tag.offset, time });
        }
    }
    while (d_rx_time_anchors.size() > 1 &&
           d_rx_time_anchors[1].offset <= d_ring_abs_start) {
        d_rx_time_anchors.pop_front();
    }
}

bool prs_window_capture_impl::sample_time(uint64_t abs_offset, double& result) const
{
    for (auto it = d_rx_time_anchors.rbegin(); it != d_rx_time_anchors.rend(); ++it) {
        if (it->offset <= abs_offset) {
            result =
                it->time + static_cast<double>(abs_offset - it->offset) / d_cfg.samp_rate;
            return true;
        }
    }
    return false;
}

bool prs_window_capture_impl::sample_offset(double time, uint64_t& result) const
{
    if (!std::isfinite(time) || d_rx_time_anchors.empty()) {
        return false;
    }
    const rx_time_anchor* selected = &d_rx_time_anchors.front();
    for (const auto& anchor : d_rx_time_anchors) {
        if (anchor.time > time) {
            break;
        }
        selected = &anchor;
    }
    const double offset =
        static_cast<double>(selected->offset) + (time - selected->time) * d_cfg.samp_rate;
    if (offset < 0.0 ||
        offset >= static_cast<double>(std::numeric_limits<uint64_t>::max())) {
        return false;
    }
    // Preserve first-sample-at-or-after semantics without advancing an exactly
    // sample-aligned boundary because of double-precision roundoff.
    result = static_cast<uint64_t>(std::ceil(offset - 1e-6));
    return true;
}

std::vector<prs_window_capture_impl::active_interval>
prs_window_capture_impl::active_intervals(uint64_t abs_start, uint64_t abs_stop)
{
    if (d_acquisition_enabled.load(std::memory_order_relaxed)) {
        return { { abs_start, abs_stop, 0, 0 } };
    }

    std::vector<window_spec> windows;
    {
        std::lock_guard<std::mutex> lock(d_window_mutex);
        windows = d_windows;
    }

    std::vector<active_interval> intervals;
    for (const auto& window : windows) {
        uint64_t start = window.start_sample;
        uint64_t end = window.end_sample;
        if (window.by_time) {
            if (!sample_offset(window.start_time, start) ||
                !sample_offset(window.end_time, end)) {
                continue;
            }
        }
        const uint64_t clipped_start = std::max(abs_start, start);
        const uint64_t clipped_end = std::min(abs_stop, end);
        if (clipped_start < clipped_end) {
            intervals.push_back(
                { clipped_start, clipped_end, window.window_id, window.attempt_id });
        }
    }
    std::sort(intervals.begin(),
              intervals.end(),
              [](const active_interval& lhs, const active_interval& rhs) {
                  return lhs.start < rhs.start;
              });

    double chunk_start_time = 0.0;
    const bool have_chunk_time = sample_time(abs_start, chunk_start_time);
    {
        std::lock_guard<std::mutex> lock(d_window_mutex);
        d_windows.erase(std::remove_if(d_windows.begin(),
                                       d_windows.end(),
                                       [abs_start, chunk_start_time, have_chunk_time](
                                           const window_spec& window) {
                                           return window.by_time
                                                      ? have_chunk_time &&
                                                            window.end_time <=
                                                                chunk_start_time
                                                      : window.end_sample <= abs_start;
                                       }),
                        d_windows.end());
    }
    return intervals;
}

void prs_window_capture_impl::reset_stream(uint64_t abs_start)
{
    d_ring_head = 0;
    d_ring_count = 0;
    d_ring_abs_start = abs_start;
    d_have_stream_position = true;
    d_next_abs_sample = abs_start;
    invalidate_metric();
    d_state = acquisition_state::idle;
    d_candidate_samples.clear();
}

void prs_window_capture_impl::append_ring_chunk(const gr_complex* samples,
                                                size_t count,
                                                uint64_t abs_start)
{
    if (count == 0U) {
        return;
    }
    if (!d_have_stream_position || abs_start != d_next_abs_sample) {
        reset_stream(abs_start);
    }

    const size_t capacity = d_ring_mask + 1U;
    if (count > capacity) {
        throw std::runtime_error("window-capture input segment exceeds history capacity");
    }
    const size_t write_index = (d_ring_head + d_ring_count) & d_ring_mask;
    // first write
    // capacity - write_index is the remaining space from the write index to the end of the ring buffer.
    const size_t first = std::min(count, capacity - write_index);
    std::memcpy(d_ring.data() + write_index, samples, first * sizeof(gr_complex));
    std::memcpy(
        d_ring.data() + write_index + capacity, samples, first * sizeof(gr_complex));
    // second write (when first write touch the end of the ring buffer)
        const size_t second = count - first;
    if (second != 0U) {
        std::memcpy(d_ring.data(), samples + first, second * sizeof(gr_complex));
        std::memcpy(
            d_ring.data() + capacity, samples + first, second * sizeof(gr_complex));
    }

    const size_t combined = d_ring_count + count;
    if (combined > capacity) {
        const size_t discarded = combined - capacity;
        d_ring_head = (d_ring_head + discarded) & d_ring_mask;
        d_ring_abs_start += discarded;
        d_ring_count = capacity;
    } else {
        d_ring_count = combined;
    }
    d_next_abs_sample = abs_start + count;
}

bool prs_window_capture_impl::ring_contains(uint64_t abs_start, uint64_t abs_end) const
{
    return abs_start <= abs_end && abs_start >= d_ring_abs_start &&
           abs_end <= d_ring_abs_start + d_ring_count;
}

const gr_complex* prs_window_capture_impl::ring_data(uint64_t abs_index) const
{
    const size_t offset = static_cast<size_t>(abs_index - d_ring_abs_start);
    return d_ring.data() + ((d_ring_head + offset) & d_ring_mask);
}

void prs_window_capture_impl::copy_ring_range(uint64_t abs_start,
                                              uint64_t abs_end,
                                              std::vector<gr_complex>& destination) const
{
    destination.clear();
    const size_t count = static_cast<size_t>(abs_end - abs_start);
    destination.reserve(count);
    if (count != 0U) {
        const auto* begin = ring_data(abs_start);
        destination.insert(destination.end(), begin, begin + count);
    }
}

void prs_window_capture_impl::append_candidate_range(uint64_t abs_start, uint64_t abs_end)
{
    if (abs_start >= abs_end) {
        return;
    }
    const auto* begin = ring_data(abs_start);
    d_candidate_samples.insert(
        d_candidate_samples.end(), begin, begin + (abs_end - abs_start));
}

void prs_window_capture_impl::invalidate_metric()
{
    d_metric_valid = false;
    d_metric_updates_since_rebuild = 0;
}

void prs_window_capture_impl::rebuild_metric(uint64_t metric_start)
{
    const size_t lag = static_cast<size_t>(d_cfg.preamble_len);
    const size_t span = lag * static_cast<size_t>(d_cfg.preamble_repeats - 1);
    const auto* first = ring_data(metric_start);
    const auto* second = first + lag;
    float corr_real = 0.0f;
    float corr_imag = 0.0f;
    d_metric_first_power = 0.0;
    d_metric_second_power = 0.0;
    for (size_t i = 0; i < span; ++i) {
        const float ar = first[i].real();
        const float ai = first[i].imag();
        const float br = second[i].real();
        const float bi = second[i].imag();
        corr_real += ar * br + ai * bi;
        corr_imag += ar * bi - ai * br;
        const float first_power = ar * ar + ai * ai;
        const float second_power = br * br + bi * bi;
        d_metric_first_power += first_power;
        d_metric_second_power += second_power;
    }
    d_metric_corr = gr_complex(corr_real, corr_imag);
    d_metric_start = metric_start;
    d_metric_valid = true;
    d_metric_updates_since_rebuild = 0;
}

void prs_window_capture_impl::advance_metric(uint64_t metric_start)
{
    const uint64_t lag = static_cast<uint64_t>(d_cfg.preamble_len);
    const uint64_t span = lag * static_cast<uint64_t>(d_cfg.preamble_repeats - 1);
    const auto* removed = ring_data(d_metric_start);
    const auto* added = removed + span;
    const auto removed_product = std::conj(removed[0]) * removed[lag];
    const auto added_product = std::conj(added[0]) * added[lag];
    const float removed_first_power = std::norm(removed[0]);
    const float removed_second_power = std::norm(removed[lag]);
    const float added_first_power = std::norm(added[0]);
    const float added_second_power = std::norm(added[lag]);
    d_metric_corr += added_product - removed_product;
    d_metric_first_power += static_cast<double>(added_first_power) - removed_first_power;
    d_metric_second_power +=
        static_cast<double>(added_second_power) - removed_second_power;
    d_metric_start = metric_start;
    ++d_metric_updates_since_rebuild;
}

bool prs_window_capture_impl::update_metric(uint64_t metric_start,
                                            float& metric,
                                            bool& threshold_passed)
{
    if (!d_metric_valid || metric_start != d_metric_start + 1U ||
        d_metric_updates_since_rebuild >= 16384U) {
        rebuild_metric(metric_start);
    } else {
        advance_metric(metric_start);
    }

    const double first_power = std::max(0.0, d_metric_first_power);
    const double second_power = std::max(0.0, d_metric_second_power);
    const double power_product = first_power * second_power;
    const double corr_power = std::norm(d_metric_corr);
    const double threshold_power =
        static_cast<double>(d_threshold) * d_threshold * power_product;
    threshold_passed = power_product > 0.0 && corr_power >= threshold_power;
    // The metric value is only consumed for threshold-passing positions. Avoid a
    // square root on every sample in the continuous 30 MS/s acquisition path.
    metric = threshold_passed ? static_cast<float>(std::sqrt(corr_power / power_product))
                              : 0.0f;
    return true;
}

bool prs_window_capture_impl::scan_idle(uint64_t newest_start,
                                        uint64_t newest_end,
                                        const active_interval* interval,
                                        uint64_t& consumed_until)
{
    const uint64_t total = static_cast<uint64_t>(d_cfg.preamble_len) *
                           static_cast<uint64_t>(d_cfg.preamble_repeats);
    consumed_until = newest_end;
    if (newest_start >= newest_end || newest_end < total) {
        return false;
    }
    const uint64_t first_newest = std::max(newest_start, total - 1U);
    const uint64_t first_metric = std::max(first_newest + 1U - total, d_ring_abs_start);
    const uint64_t metric_end = newest_end - total + 1U;
    for (uint64_t metric_start = first_metric; metric_start < metric_end;
         ++metric_start) {
        float metric = 0.0f;
        bool threshold_passed = false;
        if (!update_metric(metric_start, metric, threshold_passed)) {
            continue;
        }
        if (threshold_passed &&
            start_candidate(metric_start, metric, interval, metric_start + total - 1U)) {
            consumed_until = metric_start + total;
            return true;
        }
    }
    return false;
}

void prs_window_capture_impl::monitor_candidate(uint64_t newest_start,
                                                uint64_t newest_end)
{
    const uint64_t total = static_cast<uint64_t>(d_cfg.preamble_len) *
                           static_cast<uint64_t>(d_cfg.preamble_repeats);
    const uint64_t monitor_newest_end =
        std::min(newest_end, d_preamble_monitor_end + total);
    if (newest_start >= monitor_newest_end) {
        invalidate_metric();
        return;
    }
    const uint64_t first_newest = std::max(newest_start, total - 1U);
    for (uint64_t newest = first_newest; newest < monitor_newest_end; ++newest) {
        float metric = 0.0f;
        bool threshold_passed = false;
        if (!update_metric(newest + 1U - total, metric, threshold_passed)) {
            continue;
        }
        if (threshold_passed) {
            ++d_threshold_crossing_count;
            d_max_preamble_metric = std::max(d_max_preamble_metric, metric);
        } else if (d_state == acquisition_state::preamble_active) {
            d_state = acquisition_state::capturing;
        }
    }
    if (monitor_newest_end < newest_end ||
        (d_metric_valid && d_metric_start >= d_preamble_monitor_end)) {
        invalidate_metric();
    }
}

bool prs_window_capture_impl::start_candidate(uint64_t crossing_index,
                                              float metric,
                                              const active_interval* interval,
                                              uint64_t newest_abs)
{
    const uint64_t preamble_total = static_cast<uint64_t>(d_cfg.preamble_len) *
                                    static_cast<uint64_t>(d_cfg.preamble_repeats);
    const uint64_t history = static_cast<uint64_t>(phy::leading_guard_samples) +
                             preamble_total + static_cast<uint64_t>(d_margin_before);
    if (crossing_index < history) {
        return false;
    }

    const uint64_t candidate_start = crossing_index - history;
    const uint64_t future = preamble_total +
                            static_cast<uint64_t>(d_cfg.coarse_sync_len) +
                            static_cast<uint64_t>(prs_len(d_cfg)) +
                            static_cast<uint64_t>(phy::tail_guard_samples) +
                            static_cast<uint64_t>(d_margin_after);
    if (crossing_index > std::numeric_limits<uint64_t>::max() - future) {
        return false;
    }
    const uint64_t candidate_end = crossing_index + future;
    if (!ring_contains(candidate_start, newest_abs + 1U)) {
        return false;
    }

    d_first_crossing = crossing_index;
    d_preamble_monitor_end = crossing_index + preamble_total;
    d_candidate_start = candidate_start;
    d_candidate_end = candidate_end;
    d_threshold_crossing_count = 1;
    d_max_preamble_metric = metric;
    d_candidate_window_enabled = interval != nullptr;
    d_candidate_window_id = interval != nullptr ? interval->window_id : 0;
    d_candidate_attempt_id = interval != nullptr ? interval->attempt_id : 0;
    copy_ring_range(candidate_start, newest_abs + 1U, d_candidate_samples);
    d_candidate_samples.reserve(static_cast<size_t>(candidate_end - candidate_start));
    d_state = acquisition_state::preamble_active;
    return true;
}

bool prs_window_capture_impl::reserve_credit(uint64_t candidate_id, uint64_t& queue_depth)
{
    std::lock_guard<std::mutex> lock(d_credit_mutex);
    if (d_outstanding_candidates.size() >= static_cast<size_t>(d_queue_depth)) {
        queue_depth = d_outstanding_candidates.size();
        return false;
    }
    d_outstanding_candidates.insert(candidate_id);
    queue_depth = d_outstanding_candidates.size();
    return true;
}

pmt::pmt_t prs_window_capture_impl::candidate_metadata(uint64_t candidate_id,
                                                       uint64_t queue_depth,
                                                       const char* event_name) const
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("candidate_id"), pmt::from_uint64(candidate_id));
    meta = pmt::dict_add(
        meta, pmt::mp("acquisition_state"), pmt::intern(state_name(d_state)));
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::intern(event_name));
    meta = pmt::dict_add(
        meta, pmt::mp("absolute_sample_index"), pmt::from_uint64(d_candidate_start));
    meta = pmt::dict_add(meta,
                         pmt::mp("first_preamble_crossing_index"),
                         pmt::from_uint64(d_first_crossing));
    meta = pmt::dict_add(meta,
                         pmt::mp("first_preamble_crossing_offset"),
                         pmt::from_uint64(d_first_crossing - d_candidate_start));
    meta = pmt::dict_add(
        meta, pmt::mp("max_preamble_metric"), pmt::from_double(d_max_preamble_metric));
    meta = pmt::dict_add(
        meta, pmt::mp("preamble_metric"), pmt::from_double(d_max_preamble_metric));
    meta = pmt::dict_add(meta,
                         pmt::mp("preamble_threshold_crossings"),
                         pmt::from_uint64(d_threshold_crossing_count));
    meta = pmt::dict_add(
        meta, pmt::mp("candidate_window_start"), pmt::from_uint64(d_candidate_start));
    meta = pmt::dict_add(
        meta, pmt::mp("candidate_window_end"), pmt::from_uint64(d_candidate_end));
    meta = pmt::dict_add(meta,
                         pmt::mp("candidate_window_length"),
                         pmt::from_uint64(d_candidate_samples.size()));
    meta = pmt::dict_add(
        meta, pmt::mp("candidate_queue_depth"), pmt::from_uint64(queue_depth));
    meta = pmt::dict_add(meta,
                         pmt::mp("window_queue_full_count"),
                         pmt::from_uint64(d_window_queue_full_count));
    meta = pmt::dict_add(meta,
                         pmt::mp("receive_window_enabled"),
                         d_candidate_window_enabled ? pmt::PMT_T : pmt::PMT_F);
    meta = pmt::dict_add(
        meta, pmt::mp("window_id"), pmt::from_uint64(d_candidate_window_id));
    meta = pmt::dict_add(
        meta, pmt::mp("attempt_id"), pmt::from_uint64(d_candidate_attempt_id));
    meta = pmt::dict_add(meta, pmt::mp("samp_rate"), pmt::from_double(d_cfg.samp_rate));
    meta = pmt::dict_add(meta, pmt::mp("fft_len"), pmt::from_long(d_cfg.fft_len));
    meta = pmt::dict_add(meta, pmt::mp("cp_len"), pmt::from_long(d_cfg.cp_len));
    meta = pmt::dict_add(meta, pmt::mp("active_bins"), pmt::from_long(d_cfg.active_bins));
    meta = pmt::dict_add(meta, pmt::mp("prs_symbols"), pmt::from_long(d_cfg.prs_symbols));
    meta =
        pmt::dict_add(meta, pmt::mp("preamble_len"), pmt::from_long(d_cfg.preamble_len));
    meta = pmt::dict_add(
        meta, pmt::mp("preamble_repeats"), pmt::from_long(d_cfg.preamble_repeats));
    meta = pmt::dict_add(
        meta, pmt::mp("coarse_sync_len"), pmt::from_long(d_cfg.coarse_sync_len));
    meta = pmt::dict_add(
        meta, pmt::mp("zero_guard_len"), pmt::from_long(phy::leading_guard_samples));
    meta = pmt::dict_add(
        meta, pmt::mp("tail_guard_len"), pmt::from_long(phy::tail_guard_samples));

    const uint64_t coarse_rel = static_cast<uint64_t>(phy::leading_guard_samples) +
                                static_cast<uint64_t>(d_cfg.preamble_len) *
                                    static_cast<uint64_t>(d_cfg.preamble_repeats);
    const uint64_t samples_after_zc = static_cast<uint64_t>(d_cfg.coarse_sync_len) +
                                      static_cast<uint64_t>(prs_len(d_cfg)) +
                                      static_cast<uint64_t>(phy::tail_guard_samples);
    const uint64_t search_end = d_candidate_samples.size() >= samples_after_zc
                                    ? d_candidate_samples.size() - samples_after_zc
                                    : 0;
    meta = pmt::dict_add(
        meta, pmt::mp("zc_search_start_offset"), pmt::from_uint64(coarse_rel));
    meta = pmt::dict_add(
        meta, pmt::mp("zc_search_end_offset"), pmt::from_uint64(search_end));
    uint64_t samples_consumed = d_total_samples_consumed;
    uint64_t work_calls = d_work_calls;
    uint64_t handler_time_ns = d_work_duration_ns;
    if (d_inside_work) {
        samples_consumed += d_current_work_samples;
        ++work_calls;
        if (d_enable_profiling) {
            handler_time_ns += static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - d_current_work_start)
                    .count());
        }
    }
    meta = pmt::dict_add(
        meta, pmt::mp("block_a_samples_consumed"), pmt::from_uint64(samples_consumed));
    meta =
        pmt::dict_add(meta, pmt::mp("block_a_work_calls"), pmt::from_uint64(work_calls));
    meta = pmt::dict_add(
        meta, pmt::mp("block_a_handler_time_ns"), pmt::from_uint64(handler_time_ns));

    double value = 0.0;
    if (sample_time(d_candidate_start, value)) {
        meta = pmt::dict_add(meta, pmt::mp("candidate_rx_time"), time_tuple(value));
        meta = pmt::dict_add(meta, pmt::mp("rx_time"), time_tuple(value));
        for (auto it = d_rx_time_anchors.rbegin(); it != d_rx_time_anchors.rend(); ++it) {
            if (it->offset <= d_candidate_start) {
                meta = pmt::dict_add(
                    meta, pmt::mp("rx_time_tag_offset"), pmt::from_uint64(it->offset));
                break;
            }
        }
    }
    if (sample_time(d_first_crossing, value)) {
        meta = pmt::dict_add(
            meta, pmt::mp("first_preamble_crossing_rx_time"), time_tuple(value));
    }
    return meta;
}

void prs_window_capture_impl::publish_queue_full(uint64_t candidate_id,
                                                 pmt::pmt_t metadata)
{
    metadata =
        pmt::dict_add(metadata, pmt::mp("failure_reason"), pmt::mp("WINDOW_QUEUE_FULL"));
    metadata = pmt::dict_add(metadata, pmt::mp("candidate_dropped"), pmt::PMT_T);
    message_port_pub(pmt::mp("event_out"), pmt::cons(metadata, pmt::PMT_NIL));
    (void)candidate_id;
}

void prs_window_capture_impl::finish_candidate()
{
    const uint64_t candidate_id = d_next_candidate_id++;
    d_lockout_until = d_first_crossing + static_cast<uint64_t>(d_min_frame_gap);
    d_state = acquisition_state::lockout;
    invalidate_metric();

    uint64_t queue_depth = 0;
    if (!reserve_credit(candidate_id, queue_depth)) {
        ++d_window_queue_full_count;
        auto meta = candidate_metadata(candidate_id, queue_depth, "WINDOW_QUEUE_FULL");
        publish_queue_full(candidate_id, meta);
    } else {
        profiling::timing_report report(d_enable_profiling, "window_capture");
        auto meta = candidate_metadata(candidate_id, queue_depth, "CANDIDATE_READY");
        const auto pdu = pmt::cons(
            meta, pmt::init_c32vector(d_candidate_samples.size(), d_candidate_samples));
        report.checkpoint("candidate_pdu_build");
        message_port_pub(pmt::mp("candidate_out"), pdu);
        report.checkpoint("candidate_pdu_publish");
        message_port_pub(pmt::mp("event_out"), pmt::cons(meta, pmt::PMT_NIL));
        if (report.enabled()) {
            message_port_pub(pmt::mp("timing_out"), report.finish(meta));
        }
    }

    d_candidate_samples.clear();
}

const char* prs_window_capture_impl::state_name(acquisition_state state)
{
    switch (state) {
    case acquisition_state::idle:
        return "IDLE";
    case acquisition_state::preamble_active:
        return "PREAMBLE_ACTIVE";
    case acquisition_state::capturing:
        return "CAPTURING";
    case acquisition_state::lockout:
        return "LOCKOUT";
    }
    return "UNKNOWN";
}

int prs_window_capture_impl::general_work(int noutput_items,
                                          gr_vector_int& ninput_items,
                                          gr_vector_const_void_star& input_items,
                                          gr_vector_void_star& output_items)
{
    (void)noutput_items;
    (void)output_items;
    if (d_enable_profiling) {
        d_current_work_start = std::chrono::steady_clock::now();
    }
    d_inside_work = true;
    d_current_work_samples = 0;
    const auto* input = static_cast<const gr_complex*>(input_items[0]);
    const int ninput = ninput_items[0];
    const uint64_t abs_start = nitems_read(0);
    const uint64_t abs_stop = abs_start + static_cast<uint64_t>(ninput);
    update_rx_time_tags(abs_start, abs_stop);

    const bool continuous_acquisition =
        d_acquisition_enabled.load(std::memory_order_relaxed);
    const active_interval continuous_interval{ abs_start, abs_stop, 0, 0 };

    // If continuous_acquisition is true, the entire input is treated as a single interval
    // Otherwise, the segment is split into multiple intervals based on the active windows
    std::vector<active_interval> intervals = continuous_acquisition
                                                 ? std::vector<active_interval>()
                                                 : active_intervals(abs_start, abs_stop);
    size_t interval_index = 0;

    const uint64_t preamble_total = static_cast<uint64_t>(d_cfg.preamble_len) *
                                    static_cast<uint64_t>(d_cfg.preamble_repeats);
    const uint64_t candidate_history = static_cast<uint64_t>(phy::leading_guard_samples) +
                                       preamble_total +
                                       static_cast<uint64_t>(d_margin_before);
    const size_t ring_capacity = d_ring_mask + 1U;
    const size_t max_segment =
        ring_capacity - static_cast<size_t>(candidate_history + preamble_total);

    uint64_t segment_start = abs_start;
    // The input is processed in segments
    // Each segment is appended to the ring buffer and then processed for candidate detection
    // The segment size is limited to ensure that the ring buffer does not overflow
    while (segment_start < abs_stop) {
        const uint64_t segment_end =
            std::min(abs_stop, segment_start + static_cast<uint64_t>(max_segment));
        append_ring_chunk(input + (segment_start - abs_start),
                          static_cast<size_t>(segment_end - segment_start),
                          segment_start);

        uint64_t cursor = segment_start;
        while (cursor < segment_end) {
            // state machine for candidate detection and capture
            // LOCKOUT: skip samples until lockout period ends
            // PREAMBLE_ACTIVE: monitor for additional threshold crossings
            // CAPTURING: continue capturing until candidate end is reached
            // IDLE: scan for new threshold crossings
            d_current_work_samples = cursor - abs_start;

            if (d_state == acquisition_state::lockout) {
                if (cursor < d_lockout_until) {
                    cursor = std::min(segment_end, d_lockout_until);
                    invalidate_metric();
                    continue;
                }
                d_state = acquisition_state::idle;
                invalidate_metric();
            }

            if (d_state == acquisition_state::preamble_active ||
                d_state == acquisition_state::capturing) {
                const uint64_t capture_end = std::min(segment_end, d_candidate_end);
                const uint64_t captured_until =
                    d_candidate_start + d_candidate_samples.size();
                if (captured_until < capture_end) {
                    append_candidate_range(std::max(cursor, captured_until), capture_end);
                }
                monitor_candidate(cursor, capture_end);
                cursor = capture_end;
                d_current_work_samples = cursor - abs_start;
                if (cursor >= d_candidate_end) {
                    finish_candidate();
                }
                continue;
            }

            const active_interval* interval = nullptr;
            if (continuous_acquisition) {
                interval = &continuous_interval;
            } else {
                while (interval_index < intervals.size() &&
                       intervals[interval_index].end <= cursor) {
                    ++interval_index;
                }
                if (interval_index < intervals.size() &&
                    intervals[interval_index].start <= cursor &&
                    cursor < intervals[interval_index].end) {
                    interval = &intervals[interval_index];
                }
            }
            if (interval == nullptr) {
                const uint64_t disabled_end =
                    interval_index < intervals.size()
                        ? std::min(segment_end, intervals[interval_index].start)
                        : segment_end;
                invalidate_metric();
                cursor = disabled_end;
                continue;
            }

            const uint64_t enabled_end = std::min(segment_end, interval->end);
            uint64_t consumed_until = enabled_end;
            scan_idle(cursor, enabled_end, interval, consumed_until);
            cursor = consumed_until;
        }
        segment_start = segment_end;
    }

    consume_each(ninput);
    d_total_samples_consumed += static_cast<uint64_t>(ninput);
    ++d_work_calls;
    if (d_enable_profiling) {
        d_work_duration_ns += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - d_current_work_start)
                .count());
    }
    d_inside_work = false;
    d_current_work_samples = 0;
    return 0;
}

} // namespace ofdm_prs_ranging
} // namespace gr
