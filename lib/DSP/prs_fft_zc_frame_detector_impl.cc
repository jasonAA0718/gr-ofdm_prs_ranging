/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_fft_zc_frame_detector_impl.h"
#include "prs_timing_helper.h"
#include <gnuradio/fft/fft.h>
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

prs_fft_zc_frame_detector::sptr prs_fft_zc_frame_detector::make(double samp_rate,
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
                                                                bool enable_profiling)
{
    return gnuradio::make_block_sptr<prs_fft_zc_frame_detector_impl>(samp_rate,
                                                                     fft_len,
                                                                     cp_len,
                                                                     active_bins,
                                                                     prs_symbols,
                                                                     preamble_len,
                                                                     preamble_repeats,
                                                                     coarse_sync_len,
                                                                     zero_guard_len,
                                                                     tail_guard_len,
                                                                     threshold,
                                                                     min_frame_gap,
                                                                     coarse_zc_root,
                                                                     channel_id,
                                                                     time_gating,
                                                                     reply_delay_s,
                                                                     window_before_s,
                                                                     window_after_s,
                                                                     zc_threshold,
                                                                     correlation_fft_len,
                                                                     zc_search_before,
                                                                     zc_search_after,
                                                                     cfo_compensation,
                                                                     enable_profiling);
}

prs_fft_zc_frame_detector_impl::prs_fft_zc_frame_detector_impl(double samp_rate,
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
                                                               bool enable_profiling)
    : gr::block("prs_fft_zc_frame_detector",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(0, 0, 0)),
      d_preamble_threshold(threshold),
      d_zc_threshold(zc_threshold),
      d_min_frame_gap(min_frame_gap),
      d_correlation_fft_len(correlation_fft_len),
      d_zc_search_before(zc_search_before),
      d_zc_search_after(zc_search_after),
      d_cfo_compensation(cfo_compensation),
      d_buffer_head(0),
      d_next_gate_index(0),
      d_next_coarse_index(0),
      d_buffer_abs_start(0),
      d_total_seen(0),
      d_next_frame_id(0),
      d_last_frame_start(-min_frame_gap),
      d_have_rx_time(false),
      d_rx_time_tag_offset(0),
      d_rx_time_secs(0),
      d_rx_time_frac(0.0),
      d_time_gating(time_gating),
      d_reply_delay_s(reply_delay_s),
      d_window_before_s(window_before_s),
      d_window_after_s(window_after_s),
      d_enable_profiling(enable_profiling),
      d_gate_scan_duration_ns(0),
      d_zc_input_duration_ns(0),
      d_zc_fft_duration_ns(0),
      d_zc_multiply_duration_ns(0),
      d_zc_ifft_duration_ns(0),
      d_zc_peak_duration_ns(0),
      d_gate_armed(false),
      d_armed_preamble_index(0),
      d_armed_preamble_metric(0.0f),
      d_last_zc_gate_offset_samples(0),
      d_have_tracked_cfo(false),
      d_tracked_cfo_hz(0.0),
      d_last_detection_cfo_hz(0.0),
      d_last_cfo_was_tracked(false)
{
    if (samp_rate <= 0.0) {
        throw std::invalid_argument("samp_rate must be positive");
    }
    if (threshold <= 0.0f || threshold > 1.0f || zc_threshold <= 0.0f ||
        zc_threshold > 1.0f) {
        throw std::invalid_argument("preamble and ZC thresholds must be in (0, 1]");
    }
    if (reply_delay_s < 0.0 || window_before_s < 0.0 || window_after_s <= 0.0) {
        throw std::invalid_argument("correlation window timing must be nonnegative");
    }
    if (correlation_fft_len < coarse_sync_len ||
        (correlation_fft_len & (correlation_fft_len - 1)) != 0) {
        throw std::invalid_argument(
            "correlation_fft_len must be a power of two and at least coarse_sync_len");
    }
    if (zc_search_before < 0 || zc_search_after < 0) {
        throw std::invalid_argument("ZC search extents must be nonnegative");
    }

    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.cp_len = cp_len;
    d_cfg.active_bins = active_bins;
    d_cfg.prs_symbols = prs_symbols;
    d_cfg.preamble_len = preamble_len;
    d_cfg.preamble_repeats = preamble_repeats;
    d_cfg.coarse_sync_len = coarse_sync_len;
    d_cfg.zero_guard_len = zero_guard_len;
    d_cfg.tail_guard_len = tail_guard_len;
    d_cfg.coarse_zc_root = coarse_zc_root;
    d_cfg.channel_id = channel_id;
    d_coarse = coarse_sync_sequence(d_cfg.coarse_sync_len, d_cfg.coarse_zc_root);
    d_zc_fft = std::make_unique<gr::fft::fft_complex_fwd>(d_correlation_fft_len, 1);
    d_zc_ifft = std::make_unique<gr::fft::fft_complex_rev>(d_correlation_fft_len, 1);
    auto* matched_filter = d_zc_fft->get_inbuf();
    std::fill(
        matched_filter, matched_filter + d_correlation_fft_len, gr_complex(0.0f, 0.0f));
    for (int i = 0; i < d_cfg.coarse_sync_len; ++i) {
        matched_filter[i] = std::conj(d_coarse[d_cfg.coarse_sync_len - 1 - i]);
    }
    d_zc_fft->execute();
    d_zc_spectrum.assign(d_zc_fft->get_outbuf(),
                         d_zc_fft->get_outbuf() + d_correlation_fft_len);
    d_buffer.reserve(static_cast<size_t>(frame_len(d_cfg) + d_cfg.coarse_sync_len) * 2U);
    message_port_register_in(pmt::mp("tx_time_in"));
    message_port_register_in(pmt::mp("prs_cfo_in"));
    message_port_register_out(pmt::mp("frame_out"));
    message_port_register_out(pmt::mp("event_out"));
    message_port_register_out(pmt::mp("timing_out"));
    set_msg_handler(pmt::mp("tx_time_in"),
                    [this](pmt::pmt_t msg) { handle_tx_time(msg); });
    set_msg_handler(pmt::mp("prs_cfo_in"),
                    [this](pmt::pmt_t msg) { handle_prs_cfo(msg); });
}

void prs_fft_zc_frame_detector_impl::forecast(int noutput_items,
                                              gr_vector_int& ninput_items_required)
{
    (void)noutput_items;
    ninput_items_required[0] = 1;
}

void prs_fft_zc_frame_detector_impl::handle_tx_time(const pmt::pmt_t& message)
{
    if (!d_time_gating) {
        return;
    }

    const pmt::pmt_t msg = pmt::is_dict(message)
                               ? message
                               : (pmt::is_pair(message) ? pmt::car(message) : message);
    if (!pmt::is_dict(msg)) {
        return;
    }

    double tx_time =
        dict_ref_double(msg, "tx_time", std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(tx_time)) {
        const double secs = dict_ref_double(
            msg, "tx_time_secs", std::numeric_limits<double>::quiet_NaN());
        const double frac = dict_ref_double(msg, "tx_time_frac", 0.0);
        if (std::isfinite(secs)) {
            tx_time = secs + frac;
        }
    }
    if (!std::isfinite(tx_time)) {
        return;
    }

    const uint64_t attempt_id = dict_ref_uint64(
        msg,
        "attempt_id",
        dict_ref_uint64(msg, "poll_frame_id", dict_ref_uint64(msg, "frame_id", 0)));
    correlation_window window{ tx_time + d_reply_delay_s - d_window_before_s,
                               tx_time + d_reply_delay_s + d_window_after_s,
                               attempt_id,
                               false,
                               false,
                               false,
                               0.0f,
                               0.0f };
    std::lock_guard<std::mutex> lock(d_window_mutex);
    d_windows.push_back(window);
    std::sort(d_windows.begin(),
              d_windows.end(),
              [](const correlation_window& a, const correlation_window& b) {
                  return a.start < b.start;
              });
}

void prs_fft_zc_frame_detector_impl::handle_prs_cfo(const pmt::pmt_t& message)
{
    const pmt::pmt_t meta =
        pmt::is_dict(message)
            ? message
            : (pmt::is_pair(message) ? pmt::car(message) : pmt::PMT_NIL);
    if (!pmt::is_dict(meta)) {
        return;
    }
    const double cfo_hz = dict_ref_double(
        meta, "prs_channel_cfo_hz", std::numeric_limits<double>::quiet_NaN());
    const double coherence = dict_ref_double(meta, "channel_coherence", 0.0);
    if (!std::isfinite(cfo_hz) || !std::isfinite(coherence) || coherence < 0.2) {
        return;
    }
    std::lock_guard<std::mutex> lock(d_cfo_mutex);
    d_tracked_cfo_hz = cfo_hz;
    d_have_tracked_cfo = true;
}

void prs_fft_zc_frame_detector_impl::update_rx_time_tags(uint64_t abs_start,
                                                         uint64_t abs_stop)
{
    std::vector<tag_t> tags;
    get_tags_in_range(tags, 0, abs_start, abs_stop, pmt::mp("rx_time"));
    for (const auto& tag : tags) {
        if (pmt::is_tuple(tag.value) && pmt::length(tag.value) >= 2) {
            d_rx_time_tag_offset = tag.offset;
            const auto secs = pmt::tuple_ref(tag.value, 0);
            const auto frac = pmt::tuple_ref(tag.value, 1);
            d_rx_time_secs = pmt::is_uint64(secs)
                                 ? pmt::to_uint64(secs)
                                 : static_cast<uint64_t>(pmt::to_long(secs));
            d_rx_time_frac = pmt::to_double(frac);
            d_have_rx_time = true;
        }
    }
}

double prs_fft_zc_frame_detector_impl::sample_time(uint64_t abs_offset) const
{
    const double sample_delta =
        static_cast<double>(abs_offset) - static_cast<double>(d_rx_time_tag_offset);
    return static_cast<double>(d_rx_time_secs) + d_rx_time_frac +
           sample_delta / d_cfg.samp_rate;
}

void prs_fft_zc_frame_detector_impl::reset_buffer(uint64_t abs_start)
{
    d_buffer.clear();
    d_buffer_head = 0;
    d_next_gate_index = 0;
    d_next_coarse_index = 0;
    d_gate_armed = false;
    d_last_zc_gate_offset_samples = 0;
    d_buffer_abs_start = abs_start;
}

void prs_fft_zc_frame_detector_impl::drop_buffer_prefix(size_t count)
{
    d_buffer_head += count;
    d_buffer_abs_start += count;
    d_next_gate_index = d_next_gate_index > count ? d_next_gate_index - count : 0;
    d_next_coarse_index = d_next_coarse_index > count ? d_next_coarse_index - count : 0;
    if (d_gate_armed) {
        d_armed_preamble_index =
            d_armed_preamble_index > count ? d_armed_preamble_index - count : 0;
    }
}

void prs_fft_zc_frame_detector_impl::compact_buffer(size_t threshold)
{
    if (d_buffer_head == d_buffer.size()) {
        d_buffer.clear();
        d_buffer_head = 0;
    } else if (d_buffer_head >= threshold) {
        d_buffer.erase(d_buffer.begin(), d_buffer.begin() + d_buffer_head);
        d_buffer_head = 0;
    }
}

void prs_fft_zc_frame_detector_impl::process_samples(const gr_complex* samples,
                                                     size_t count,
                                                     uint64_t abs_start)
{
    if (count == 0) {
        return;
    }
    if (buffered_size() == 0) {
        reset_buffer(abs_start);
    } else if (d_buffer_abs_start + buffered_size() != abs_start) {
        reset_buffer(abs_start);
    }

    d_buffer.insert(d_buffer.end(), samples, samples + count);

    size_t frame_start = 0;
    size_t coarse = 0;
    float preamble_metric = 0.0f;
    float coarse_metric = 0.0f;
    while (find_frame(frame_start, coarse, preamble_metric, coarse_metric)) {
        publish_frame(frame_start, coarse, preamble_metric, coarse_metric);
        const size_t drop = frame_start + static_cast<size_t>(frame_len(d_cfg));
        drop_buffer_prefix(drop);
    }

    const size_t keep = static_cast<size_t>(frame_len(d_cfg)) +
                        static_cast<size_t>(d_zc_search_before + d_zc_search_after) +
                        static_cast<size_t>(d_correlation_fft_len);
    if (buffered_size() > keep) {
        drop_buffer_prefix(buffered_size() - keep);
    }
    compact_buffer(keep);
}

bool prs_fft_zc_frame_detector_impl::find_preamble_gate(size_t& preamble_index,
                                                        float& metric_out,
                                                        bool require_threshold)
{
    profiling::accumulated_timer gate_timer(d_enable_profiling, d_gate_scan_duration_ns);
    const size_t preamble_total =
        static_cast<size_t>(d_cfg.preamble_len * d_cfg.preamble_repeats);
    if (buffered_size() < preamble_total) {
        return false;
    }

    const size_t max_preamble = buffered_size() - preamble_total;
    if (d_next_gate_index > max_preamble) {
        return false;
    }

    const int span = d_cfg.preamble_len * (d_cfg.preamble_repeats - 1);
    gr_complex corr(0.0f, 0.0f);
    double first_power = 0.0;
    double second_power = 0.0;
    const auto add_pair =
        [this](size_t index, gr_complex& value, double& first, double& second) {
            const auto x0 = buffered_sample(index);
            const auto x1 =
                buffered_sample(index + static_cast<size_t>(d_cfg.preamble_len));
            value += std::conj(x0) * x1;
            first += std::norm(x0);
            second += std::norm(x1);
        };
    const auto remove_pair =
        [this](size_t index, gr_complex& value, double& first, double& second) {
            const auto x0 = buffered_sample(index);
            const auto x1 =
                buffered_sample(index + static_cast<size_t>(d_cfg.preamble_len));
            value -= std::conj(x0) * x1;
            first -= std::norm(x0);
            second -= std::norm(x1);
        };

    for (int i = 0; i < span; ++i) {
        add_pair(
            d_next_gate_index + static_cast<size_t>(i), corr, first_power, second_power);
    }

    const double threshold_sq =
        static_cast<double>(d_preamble_threshold) * d_preamble_threshold;
    double best_metric_sq = -1.0;
    size_t best_index = d_next_gate_index;
    for (size_t p = d_next_gate_index; p <= max_preamble; ++p) {
        const double power_product = std::max(0.0, first_power * second_power);
        const double corr_power = std::norm(corr);
        const double metric_sq = power_product > 0.0 ? corr_power / power_product : 0.0;
        if (metric_sq > best_metric_sq) {
            best_metric_sq = metric_sq;
            best_index = p;
        }
        if (require_threshold && power_product > 0.0 &&
            corr_power >= threshold_sq * power_product) {
            preamble_index = p;
            metric_out = static_cast<float>(std::sqrt(metric_sq));
            return true;
        }
        if (p < max_preamble) {
            remove_pair(p, corr, first_power, second_power);
            add_pair(p + static_cast<size_t>(span), corr, first_power, second_power);
        }
    }

    d_next_gate_index = max_preamble + 1;
    if (require_threshold || best_metric_sq < 0.0) {
        return false;
    }
    preamble_index = best_index;
    metric_out = static_cast<float>(std::sqrt(best_metric_sq));
    return true;
}

bool prs_fft_zc_frame_detector_impl::fft_zc_search(size_t first_coarse,
                                                   size_t last_coarse,
                                                   double cfo_hz,
                                                   size_t& best_coarse,
                                                   float& best_metric,
                                                   bool& found_threshold_peak)
{
    if (first_coarse > last_coarse) {
        return false;
    }
    const size_t zc_len = static_cast<size_t>(d_cfg.coarse_sync_len);
    const size_t fft_len = static_cast<size_t>(d_correlation_fft_len);
    const size_t candidates_per_block = fft_len - zc_len + 1;
    const double phase_increment =
        d_cfo_compensation
            ? -2.0 * 3.141592653589793238462643383279502884 * cfo_hz / d_cfg.samp_rate
            : 0.0;
    const float ifft_scale = 1.0f / static_cast<float>(d_correlation_fft_len);
    best_metric = 0.0f;
    best_coarse = first_coarse;
    found_threshold_peak = false;
    bool inside_threshold_lobe = false;
    size_t lobe_best_coarse = first_coarse;
    float lobe_best_metric = 0.0f;
    const auto mark = [this]() {
        return d_enable_profiling ? profiling::now() : profiling::timing_point();
    };

    for (size_t block_start = first_coarse; block_start <= last_coarse;) {
        const size_t candidate_count =
            std::min(candidates_per_block, last_coarse - block_start + 1);
        const size_t needed = candidate_count + zc_len - 1;
        if (block_start + needed > buffered_size()) {
            return false;
        }

        auto stage_start = mark();
        auto* input = d_zc_fft->get_inbuf();
        std::fill(input, input + d_correlation_fft_len, gr_complex(0.0f, 0.0f));
        // Adjacent blocks overlap by ZC length minus one; only these linear-
        // convolution outputs correspond to complete ZC candidate windows.
        const gr_complex cfo_step(static_cast<float>(std::cos(phase_increment)),
                                  static_cast<float>(std::sin(phase_increment)));
        gr_complex cfo_phase(1.0f, 0.0f);
        for (size_t i = 0; i < needed; ++i) {
            const auto sample = buffered_sample(block_start + i);
            if (d_cfo_compensation && cfo_hz != 0.0) {
                input[i] = sample * cfo_phase;
                cfo_phase *= cfo_step;
            } else {
                input[i] = sample;
            }
        }
        auto stage_stop = mark();
        if (d_enable_profiling) {
            d_zc_input_duration_ns += profiling::elapsed_ns(stage_start, stage_stop);
        }

        stage_start = stage_stop;
        d_zc_fft->execute();
        stage_stop = mark();
        if (d_enable_profiling) {
            d_zc_fft_duration_ns += profiling::elapsed_ns(stage_start, stage_stop);
        }

        stage_start = stage_stop;
        auto* product = d_zc_ifft->get_inbuf();
        const auto* spectrum = d_zc_fft->get_outbuf();
        for (size_t k = 0; k < fft_len; ++k) {
            product[k] = spectrum[k] * d_zc_spectrum[k];
        }
        stage_stop = mark();
        if (d_enable_profiling) {
            d_zc_multiply_duration_ns += profiling::elapsed_ns(stage_start, stage_stop);
        }

        stage_start = stage_stop;
        d_zc_ifft->execute();
        stage_stop = mark();
        if (d_enable_profiling) {
            d_zc_ifft_duration_ns += profiling::elapsed_ns(stage_start, stage_stop);
        }

        stage_start = stage_stop;
        const auto* correlation = d_zc_ifft->get_outbuf();
        double energy = 0.0;
        for (size_t i = 0; i < zc_len; ++i) {
            energy += std::norm(input[i]);
        }
        for (size_t i = 0; i < candidate_count; ++i) {
            const float magnitude = std::abs(correlation[zc_len - 1 + i]) * ifft_scale;
            const double denom = std::sqrt(
                std::max(0.0, energy * static_cast<double>(d_cfg.coarse_sync_len)));
            const float metric =
                denom > 0.0 ? magnitude / static_cast<float>(denom) : 0.0f;
            const size_t coarse_index = block_start + i;
            if (!inside_threshold_lobe && metric > best_metric) {
                best_metric = metric;
                best_coarse = coarse_index;
            }
            if (metric >= d_zc_threshold) {
                if (!inside_threshold_lobe) {
                    inside_threshold_lobe = true;
                    lobe_best_coarse = coarse_index;
                    lobe_best_metric = metric;
                } else if (metric > lobe_best_metric) {
                    lobe_best_coarse = coarse_index;
                    lobe_best_metric = metric;
                }
            } else if (inside_threshold_lobe) {
                best_coarse = lobe_best_coarse;
                best_metric = lobe_best_metric;
                found_threshold_peak = true;
                if (d_enable_profiling) {
                    d_zc_peak_duration_ns += profiling::elapsed_ns(stage_start, mark());
                }
                return true;
            }
            if (i + 1 < candidate_count) {
                energy -= std::norm(input[i]);
                energy += std::norm(input[i + zc_len]);
            }
        }
        stage_stop = mark();
        if (d_enable_profiling) {
            d_zc_peak_duration_ns += profiling::elapsed_ns(stage_start, stage_stop);
        }
        block_start += candidate_count;
    }

    if (inside_threshold_lobe) {
        best_coarse = lobe_best_coarse;
        best_metric = lobe_best_metric;
        found_threshold_peak = true;
    }
    return true;
}

bool prs_fft_zc_frame_detector_impl::fft_zc_cfo_search(size_t first_coarse,
                                                       size_t last_coarse,
                                                       size_t& best_coarse,
                                                       float& best_metric,
                                                       double& selected_cfo_hz,
                                                       bool& used_tracked_cfo,
                                                       bool& found_threshold_peak)
{
    double tracked_cfo_hz = 0.0;
    bool have_tracked_cfo = false;
    if (d_cfo_compensation) {
        std::lock_guard<std::mutex> lock(d_cfo_mutex);
        have_tracked_cfo = d_have_tracked_cfo;
        tracked_cfo_hz = d_tracked_cfo_hz;
    }

    if (have_tracked_cfo || !d_cfo_compensation) {
        selected_cfo_hz = have_tracked_cfo ? tracked_cfo_hz : 0.0;
        used_tracked_cfo = have_tracked_cfo;
        return fft_zc_search(first_coarse,
                             last_coarse,
                             selected_cfo_hz,
                             best_coarse,
                             best_metric,
                             found_threshold_peak);
    }

    // Bootstrap the FLL with a small acquisition bank. Later frames use the
    // PRS channel-CFO feedback and require only one matched-filter search.
    best_metric = -1.0f;
    used_tracked_cfo = false;
    found_threshold_peak = false;
    for (int cfo_hz = -300; cfo_hz <= 300; cfo_hz += 100) {
        size_t candidate_coarse = first_coarse;
        float candidate_metric = 0.0f;
        bool candidate_found_peak = false;
        if (!fft_zc_search(first_coarse,
                           last_coarse,
                           static_cast<double>(cfo_hz),
                           candidate_coarse,
                           candidate_metric,
                           candidate_found_peak)) {
            return false;
        }
        const bool prefer_candidate =
            (candidate_found_peak && !found_threshold_peak) ||
            (candidate_found_peak == found_threshold_peak &&
             (candidate_coarse < best_coarse ||
              (candidate_coarse == best_coarse && candidate_metric > best_metric)));
        if (prefer_candidate || best_metric < 0.0f) {
            best_coarse = candidate_coarse;
            best_metric = candidate_metric;
            selected_cfo_hz = static_cast<double>(cfo_hz);
            found_threshold_peak = candidate_found_peak;
        }
    }
    return best_metric >= 0.0f;
}

void prs_fft_zc_frame_detector_impl::record_candidate(uint64_t abs_start,
                                                      float preamble_metric,
                                                      float coarse_metric)
{
    if (!d_time_gating || !d_have_rx_time) {
        return;
    }
    const double candidate_time = sample_time(abs_start);
    std::lock_guard<std::mutex> lock(d_window_mutex);
    for (auto& window : d_windows) {
        if (candidate_time < window.start) {
            break;
        }
        if (candidate_time >= window.end || window.completed) {
            continue;
        }
        window.saw_preamble =
            window.saw_preamble || preamble_metric >= d_preamble_threshold;
        window.preamble_metric = std::max(window.preamble_metric, preamble_metric);
        window.coarse_metric = std::max(window.coarse_metric, coarse_metric);
        window.saw_coarse = window.saw_coarse || coarse_metric >= d_zc_threshold;
        break;
    }
}

bool prs_fft_zc_frame_detector_impl::complete_attempt(double frame_time,
                                                      uint64_t& attempt_id)
{
    if (!d_time_gating || !std::isfinite(frame_time)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(d_window_mutex);
    for (auto& window : d_windows) {
        if (frame_time < window.start) {
            break;
        }
        if (frame_time >= window.end || window.completed) {
            continue;
        }
        window.completed = true;
        attempt_id = window.attempt_id;
        return true;
    }
    return false;
}

void prs_fft_zc_frame_detector_impl::publish_failed_attempt(
    const correlation_window& window)
{
    profiling::timing_report report(d_enable_profiling, "fft_zc_frame_detector");
    const uint64_t scan_duration_ns = d_gate_scan_duration_ns + d_zc_input_duration_ns +
                                      d_zc_fft_duration_ns + d_zc_multiply_duration_ns +
                                      d_zc_ifft_duration_ns + d_zc_peak_duration_ns;
    report.add("preamble_gate_scan", d_gate_scan_duration_ns);
    report.add("zc_input_prepare", d_zc_input_duration_ns);
    report.add("zc_fft_forward", d_zc_fft_duration_ns);
    report.add("zc_spectrum_multiply", d_zc_multiply_duration_ns);
    report.add("zc_ifft", d_zc_ifft_duration_ns);
    report.add("zc_normalize_peak", d_zc_peak_duration_ns);
    report.add("preamble_zc_scan", scan_duration_ns);
    d_gate_scan_duration_ns = 0;
    d_zc_input_duration_ns = 0;
    d_zc_fft_duration_ns = 0;
    d_zc_multiply_duration_ns = 0;
    d_zc_ifft_duration_ns = 0;
    d_zc_peak_duration_ns = 0;
    const char* failure_reason = window.saw_coarse
                                     ? "FRAME_BOUNDARY"
                                     : (window.saw_preamble ? "ZC_SYNC" : "NO_PREAMBLE");
    pmt::pmt_t meta = pmt::make_dict();
    meta =
        pmt::dict_add(meta, pmt::mp("attempt_id"), pmt::from_uint64(window.attempt_id));
    meta = pmt::dict_add(meta, pmt::mp("failure_reason"), pmt::mp(failure_reason));
    meta = pmt::dict_add(meta, pmt::mp("frame_id_valid"), pmt::PMT_F);
    meta = pmt::dict_add(
        meta, pmt::mp("preamble_metric"), pmt::from_double(window.preamble_metric));
    meta = pmt::dict_add(
        meta, pmt::mp("coarse_metric"), pmt::from_double(window.coarse_metric));
    meta = pmt::dict_add(
        meta, pmt::mp("coarse_zc_root"), pmt::from_long(d_cfg.coarse_zc_root));
    meta = pmt::dict_add(meta, pmt::mp("channel_id"), pmt::from_long(d_cfg.channel_id));
    meta = pmt::dict_add(meta, pmt::mp("samp_rate"), pmt::from_double(d_cfg.samp_rate));
    meta = pmt::dict_add(meta, pmt::mp("fft_len"), pmt::from_long(d_cfg.fft_len));
    meta = pmt::dict_add(meta, pmt::mp("cp_len"), pmt::from_long(d_cfg.cp_len));
    meta = pmt::dict_add(meta, pmt::mp("active_bins"), pmt::from_long(d_cfg.active_bins));
    meta = pmt::dict_add(meta, pmt::mp("prs_symbols"), pmt::from_long(d_cfg.prs_symbols));
    meta = pmt::dict_add(
        meta, pmt::mp("prs_start_rel"), pmt::from_long(prs_start_offset(d_cfg)));
    meta = pmt::dict_add(meta, pmt::mp("prs_len"), pmt::from_long(prs_len(d_cfg)));
    auto stage_start = report.mark();
    const auto event = pmt::cons(meta, pmt::PMT_NIL);
    auto stage_stop = report.mark();
    report.add("failure_output_build", stage_start, stage_stop);
    stage_start = stage_stop;
    message_port_pub(pmt::mp("event_out"), event);
    stage_stop = report.mark();
    report.add("failure_output_publish", stage_start, stage_stop);
    report.add("acquisition_total", scan_duration_ns + report.handler_elapsed_ns());
    if (report.enabled()) {
        message_port_pub(pmt::mp("timing_out"), report.finish(meta));
    }
}


bool prs_fft_zc_frame_detector_impl::find_frame(size_t& frame_start_index,
                                                size_t& coarse_index,
                                                float& preamble_metric_out,
                                                float& coarse_metric_out)
{
    const size_t flen = static_cast<size_t>(frame_len(d_cfg));
    const size_t preamble_total =
        static_cast<size_t>(d_cfg.preamble_len * d_cfg.preamble_repeats);
    const size_t coarse_rel = static_cast<size_t>(d_cfg.zero_guard_len) + preamble_total;
    if (buffered_size() < flen) {
        return false;
    }

    const size_t max_frame_start = buffered_size() - flen;
    const size_t max_coarse = max_frame_start + coarse_rel;
    size_t first_coarse = 0;
    size_t last_coarse = 0;
    float preamble_metric = 0.0f;
    int64_t predicted_coarse = -1;

    if (d_time_gating) {
        if (d_next_coarse_index < coarse_rel) {
            d_next_coarse_index = coarse_rel;
        }
        if (d_next_coarse_index > max_coarse) {
            return false;
        }
        first_coarse = d_next_coarse_index;
        last_coarse = max_coarse;
        // Retain the full scheduled ZC search, but restore the global repeated-
        // preamble maximum as a boundary prediction for experimental comparison.
        // It does not provide CFO correction or constrain the ZC search interval.
        size_t gate_index = 0;
        if (find_preamble_gate(gate_index, preamble_metric, false)) {
            predicted_coarse = static_cast<int64_t>(gate_index + preamble_total);
        }
        d_next_coarse_index = max_coarse + 1;
    } else {
        if (!d_gate_armed) {
            size_t gate_index = 0;
            if (!find_preamble_gate(gate_index, preamble_metric, true)) {
                return false;
            }
            d_gate_armed = true;
            d_armed_preamble_index = gate_index;
            d_armed_preamble_metric = preamble_metric;
        }

        const size_t gate_predicted_coarse = d_armed_preamble_index + preamble_total;
        predicted_coarse = static_cast<int64_t>(gate_predicted_coarse);
        first_coarse =
            gate_predicted_coarse > static_cast<size_t>(d_zc_search_before)
                ? gate_predicted_coarse - static_cast<size_t>(d_zc_search_before)
                : coarse_rel;
        first_coarse = std::max(first_coarse, coarse_rel);
        last_coarse = gate_predicted_coarse + static_cast<size_t>(d_zc_search_after);
        if (max_coarse < last_coarse) {
            return false;
        }
        preamble_metric = d_armed_preamble_metric;
    }

    size_t best_coarse = first_coarse;
    float best_metric = 0.0f;
    double selected_cfo_hz = 0.0;
    bool used_tracked_cfo = false;
    bool found_threshold_peak = false;
    if (!fft_zc_cfo_search(first_coarse,
                           last_coarse,
                           best_coarse,
                           best_metric,
                           selected_cfo_hz,
                           used_tracked_cfo,
                           found_threshold_peak)) {
        return false;
    }
    d_last_detection_cfo_hz = selected_cfo_hz;
    d_last_cfo_was_tracked = used_tracked_cfo;
    d_last_zc_gate_offset_samples =
        predicted_coarse >= 0 ? static_cast<int64_t>(best_coarse) - predicted_coarse : 0;

    if (!d_time_gating) {
        d_next_gate_index =
            std::max(d_next_gate_index, d_armed_preamble_index + preamble_total);
        d_gate_armed = false;
    }

    const size_t best_start = best_coarse - coarse_rel;
    const uint64_t abs_start = d_buffer_abs_start + best_start;
    record_candidate(abs_start, preamble_metric, best_metric);
    if (!found_threshold_peak ||
        static_cast<int64_t>(abs_start) - d_last_frame_start < d_min_frame_gap) {
        return false;
    }

    frame_start_index = best_start;
    coarse_index = best_coarse;
    preamble_metric_out = preamble_metric;
    coarse_metric_out = best_metric;
    return true;
}

void prs_fft_zc_frame_detector_impl::publish_frame(size_t frame_start_index,
                                                   size_t coarse_index,
                                                   float preamble_metric,
                                                   float coarse_metric)
{
    profiling::timing_report report(d_enable_profiling, "fft_zc_frame_detector");
    const uint64_t scan_duration_ns = d_gate_scan_duration_ns + d_zc_input_duration_ns +
                                      d_zc_fft_duration_ns + d_zc_multiply_duration_ns +
                                      d_zc_ifft_duration_ns + d_zc_peak_duration_ns;
    report.add("preamble_gate_scan", d_gate_scan_duration_ns);
    report.add("zc_input_prepare", d_zc_input_duration_ns);
    report.add("zc_fft_forward", d_zc_fft_duration_ns);
    report.add("zc_spectrum_multiply", d_zc_multiply_duration_ns);
    report.add("zc_ifft", d_zc_ifft_duration_ns);
    report.add("zc_normalize_peak", d_zc_peak_duration_ns);
    report.add("preamble_zc_scan", scan_duration_ns);
    d_gate_scan_duration_ns = 0;
    d_zc_input_duration_ns = 0;
    d_zc_fft_duration_ns = 0;
    d_zc_multiply_duration_ns = 0;
    d_zc_ifft_duration_ns = 0;
    d_zc_peak_duration_ns = 0;
    const int flen = frame_len(d_cfg);
    const uint64_t abs_start = d_buffer_abs_start + frame_start_index;
    const uint64_t coarse_abs = d_buffer_abs_start + coarse_index;
    const auto frame_begin = d_buffer.begin() + d_buffer_head + frame_start_index;
    auto stage_start = report.mark();
    std::vector<gr_complex> frame(frame_begin, frame_begin + flen);
    auto stage_stop = report.mark();
    report.add("frame_extract", stage_start, stage_stop);
    stage_start = stage_stop;
    const auto prs_cp_cfo =
        estimate_prs_cp_cfo(frame.data(), frame.size(), d_cfg, d_last_detection_cfo_hz);
    stage_stop = report.mark();
    report.add("prs_cp_cfo", stage_start, stage_stop);

    stage_start = stage_stop;
    const uint64_t recv_id = d_next_frame_id++;
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("recv_id"), pmt::from_uint64(recv_id));
    meta = pmt::dict_add(
        meta, pmt::mp("absolute_sample_index"), pmt::from_uint64(abs_start));
    meta = pmt::dict_add(meta, pmt::mp("frame_start"), pmt::from_uint64(abs_start));
    meta = pmt::dict_add(meta, pmt::mp("coarse_peak"), pmt::from_uint64(coarse_abs));
    meta = pmt::dict_add(
        meta, pmt::mp("preamble_metric"), pmt::from_double(preamble_metric));
    meta = pmt::dict_add(meta, pmt::mp("coarse_metric"), pmt::from_double(coarse_metric));
    meta = pmt::dict_add(meta,
                         pmt::mp("zc_gate_offset_samples"),
                         pmt::from_long(d_last_zc_gate_offset_samples));
    meta = pmt::dict_add(
        meta, pmt::mp("coarse_zc_root"), pmt::from_long(d_cfg.coarse_zc_root));
    meta = pmt::dict_add(meta, pmt::mp("channel_id"), pmt::from_long(d_cfg.channel_id));
    meta = pmt::dict_add(
        meta, pmt::mp("detection_cfo_hz"), pmt::from_double(d_last_detection_cfo_hz));
    meta =
        pmt::dict_add(meta,
                      pmt::mp("cfo_source"),
                      pmt::mp(d_last_cfo_was_tracked
                                  ? "PRS_FLL"
                                  : (d_cfo_compensation ? "INITIAL_BIN" : "DISABLED")));
    meta = pmt::dict_add(meta,
                         pmt::mp("prs_cp_cfo_hz"),
                         pmt::from_double(prs_cp_cfo.valid ? prs_cp_cfo.hz : 0.0));
    meta = pmt::dict_add(
        meta, pmt::mp("prs_cp_cfo_coherence"), pmt::from_double(prs_cp_cfo.coherence));
    meta = pmt::dict_add(
        meta, pmt::mp("selected_cfo_hz"), pmt::from_double(d_last_detection_cfo_hz));
    meta = pmt::dict_add(meta, pmt::mp("cfo"), pmt::from_double(d_last_detection_cfo_hz));
    meta = pmt::dict_add(meta, pmt::mp("samp_rate"), pmt::from_double(d_cfg.samp_rate));
    meta = pmt::dict_add(meta, pmt::mp("fft_len"), pmt::from_long(d_cfg.fft_len));
    meta = pmt::dict_add(meta, pmt::mp("cp_len"), pmt::from_long(d_cfg.cp_len));
    meta = pmt::dict_add(meta, pmt::mp("active_bins"), pmt::from_long(d_cfg.active_bins));
    meta = pmt::dict_add(meta, pmt::mp("prs_symbols"), pmt::from_long(d_cfg.prs_symbols));
    meta = pmt::dict_add(
        meta, pmt::mp("prs_start_rel"), pmt::from_long(prs_start_offset(d_cfg)));
    meta = pmt::dict_add(meta, pmt::mp("prs_len"), pmt::from_long(prs_len(d_cfg)));
    const double frame_time = d_have_rx_time ? sample_time(abs_start)
                                             : std::numeric_limits<double>::quiet_NaN();
    uint64_t attempt_id = 0;
    const bool have_gated_attempt = complete_attempt(frame_time, attempt_id);
    if (have_gated_attempt) {
        meta = pmt::dict_add(meta, pmt::mp("attempt_id"), pmt::from_uint64(attempt_id));
    }
    if (d_have_rx_time) {
        const double rx_time = frame_time;
        meta = pmt::dict_add(
            meta,
            pmt::mp("rx_time"),
            pmt::make_tuple(pmt::from_uint64(static_cast<uint64_t>(std::floor(rx_time))),
                            pmt::from_double(rx_time - std::floor(rx_time))));
        meta = pmt::dict_add(
            meta, pmt::mp("rx_time_tag_offset"), pmt::from_uint64(d_rx_time_tag_offset));
    }
    stage_stop = report.mark();
    report.add("metadata_build", stage_start, stage_stop);

    stage_start = stage_stop;
    const auto data = pmt::init_c32vector(frame.size(), frame);
    const auto pdu = pmt::cons(meta, data);
    stage_stop = report.mark();
    report.add("frame_pdu_build", stage_start, stage_stop);
    stage_start = stage_stop;
    message_port_pub(pmt::mp("frame_out"), pdu);
    stage_stop = report.mark();
    report.add("frame_pdu_publish", stage_start, stage_stop);
    d_last_frame_start = static_cast<int64_t>(abs_start);
    report.add("acquisition_total", scan_duration_ns + report.handler_elapsed_ns());
    if (report.enabled()) {
        message_port_pub(pmt::mp("timing_out"), report.finish(meta));
    }
}

int prs_fft_zc_frame_detector_impl::general_work(int noutput_items,
                                                 gr_vector_int& ninput_items,
                                                 gr_vector_const_void_star& input_items,
                                                 gr_vector_void_star& output_items)
{
    (void)noutput_items;
    (void)output_items;
    const auto in = static_cast<const gr_complex*>(input_items[0]);
    const int ninput = ninput_items[0];
    const uint64_t abs_start = nitems_read(0);
    const uint64_t abs_stop = abs_start + static_cast<uint64_t>(ninput);
    update_rx_time_tags(abs_start, abs_stop);
    d_total_seen += ninput;

    if (!d_time_gating) {
        process_samples(in, static_cast<size_t>(ninput), abs_start);
    } else if (!d_have_rx_time) {
        reset_buffer(abs_stop);
    } else {
        const double chunk_start_time = sample_time(abs_start);
        const double chunk_stop_time = sample_time(abs_stop);
        std::vector<std::pair<uint64_t, uint64_t>> segments;
        std::vector<correlation_window> expired_windows;
        {
            std::lock_guard<std::mutex> lock(d_window_mutex);
            for (const auto& window : d_windows) {
                if (window.end <= chunk_start_time && !window.completed) {
                    expired_windows.push_back(window);
                }
            }
            d_windows.erase(
                std::remove_if(d_windows.begin(),
                               d_windows.end(),
                               [chunk_start_time](const correlation_window& window) {
                                   return window.end <= chunk_start_time;
                               }),
                d_windows.end());
            for (const auto& window : d_windows) {
                if (window.start >= chunk_stop_time) {
                    break;
                }
                if (window.end <= chunk_start_time) {
                    continue;
                }
                const double start_offset =
                    static_cast<double>(d_rx_time_tag_offset) +
                    (window.start -
                     (static_cast<double>(d_rx_time_secs) + d_rx_time_frac)) *
                        d_cfg.samp_rate;
                const double stop_offset =
                    static_cast<double>(d_rx_time_tag_offset) +
                    (window.end -
                     (static_cast<double>(d_rx_time_secs) + d_rx_time_frac)) *
                        d_cfg.samp_rate;
                const uint64_t segment_start = std::max(
                    abs_start,
                    static_cast<uint64_t>(std::max(0.0, std::ceil(start_offset))));
                const uint64_t segment_stop = std::min(
                    abs_stop,
                    static_cast<uint64_t>(std::max(0.0, std::ceil(stop_offset))));
                if (segment_start < segment_stop) {
                    segments.emplace_back(segment_start, segment_stop);
                }
            }
        }
        for (const auto& window : expired_windows) {
            publish_failed_attempt(window);
        }

        uint64_t processed_until = abs_start;
        for (const auto& segment : segments) {
            if (segment.first != processed_until) {
                reset_buffer(segment.first);
            }
            process_samples(in + (segment.first - abs_start),
                            static_cast<size_t>(segment.second - segment.first),
                            segment.first);
            processed_until = segment.second;
        }
        if (processed_until != abs_stop) {
            reset_buffer(abs_stop);
        }
    }

    consume_each(ninput);
    return 0;
}

} // namespace ofdm_prs_ranging
} // namespace gr
