/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_frame_detector_impl.h"
#include "prs_payload_codec.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

prs_frame_detector::sptr prs_frame_detector::make(double samp_rate,
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
                                                  float zc_threshold)
{
    return gnuradio::make_block_sptr<prs_frame_detector_impl>(samp_rate,
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
                                                              zc_threshold);
}

prs_frame_detector_impl::prs_frame_detector_impl(double samp_rate,
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
                                                 float zc_threshold)
    : gr::block("prs_frame_detector",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(0, 0, 0)),
      d_preamble_threshold(threshold),
      d_zc_threshold(zc_threshold),
      d_min_frame_gap(min_frame_gap),
      d_buffer_head(0),
      d_next_scan_index(0),
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
      d_window_after_s(window_after_s)
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
    d_buffer.reserve(static_cast<size_t>(frame_len(d_cfg) + d_cfg.coarse_sync_len) * 2U);
    message_port_register_in(pmt::mp("tx_time_in"));
    message_port_register_out(pmt::mp("frame_out"));
    message_port_register_out(pmt::mp("event_out"));
    set_msg_handler(pmt::mp("tx_time_in"),
                    [this](pmt::pmt_t msg) { handle_tx_time(msg); });
}

void prs_frame_detector_impl::forecast(int noutput_items,
                                       gr_vector_int& ninput_items_required)
{
    (void)noutput_items;
    ninput_items_required[0] = 1;
}

void prs_frame_detector_impl::handle_tx_time(const pmt::pmt_t& message)
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

void prs_frame_detector_impl::update_rx_time_tags(uint64_t abs_start, uint64_t abs_stop)
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

double prs_frame_detector_impl::sample_time(uint64_t abs_offset) const
{
    const double sample_delta =
        static_cast<double>(abs_offset) - static_cast<double>(d_rx_time_tag_offset);
    return static_cast<double>(d_rx_time_secs) + d_rx_time_frac +
           sample_delta / d_cfg.samp_rate;
}

void prs_frame_detector_impl::reset_buffer(uint64_t abs_start)
{
    d_buffer.clear();
    d_buffer_head = 0;
    d_next_scan_index = 0;
    d_buffer_abs_start = abs_start;
}

void prs_frame_detector_impl::drop_buffer_prefix(size_t count)
{
    d_buffer_head += count;
    d_buffer_abs_start += count;
    d_next_scan_index = d_next_scan_index > count ? d_next_scan_index - count : 0;
}

void prs_frame_detector_impl::compact_buffer(size_t threshold)
{
    if (d_buffer_head == d_buffer.size()) {
        d_buffer.clear();
        d_buffer_head = 0;
    } else if (d_buffer_head >= threshold) {
        d_buffer.erase(d_buffer.begin(), d_buffer.begin() + d_buffer_head);
        d_buffer_head = 0;
    }
}

void prs_frame_detector_impl::process_samples(const gr_complex* samples,
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

    const size_t keep = static_cast<size_t>(frame_len(d_cfg) + d_cfg.coarse_sync_len);
    if (buffered_size() > keep) {
        drop_buffer_prefix(buffered_size() - keep);
    }
    compact_buffer(keep);
}

float prs_frame_detector_impl::coarse_sync_metric(size_t coarse_index) const
{
    gr_complex corr(0.0f, 0.0f);
    double power = 0.0;
    for (int i = 0; i < d_cfg.coarse_sync_len; ++i) {
        const auto sample = buffered_sample(coarse_index + static_cast<size_t>(i));
        corr += sample * std::conj(d_coarse[i]);
        power += std::norm(sample);
    }
    const double denom = std::sqrt(power * static_cast<double>(d_cfg.coarse_sync_len));
    return denom > 0.0 ? static_cast<float>(std::abs(corr) / denom) : 0.0f;
}

void prs_frame_detector_impl::record_candidate(uint64_t abs_start,
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
        window.saw_preamble = true;
        window.preamble_metric = std::max(window.preamble_metric, preamble_metric);
        window.coarse_metric = std::max(window.coarse_metric, coarse_metric);
        window.saw_coarse = window.saw_coarse || coarse_metric >= d_zc_threshold;
        break;
    }
}

bool prs_frame_detector_impl::complete_attempt(double frame_time, uint64_t& attempt_id)
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

void prs_frame_detector_impl::publish_failed_attempt(const correlation_window& window)
{
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
    message_port_pub(pmt::mp("event_out"), pmt::cons(meta, pmt::PMT_NIL));
}

bool prs_frame_detector_impl::find_frame(size_t& frame_start_index,
                                         size_t& coarse_index,
                                         float& preamble_metric_out,
                                         float& coarse_metric_out)
{
    const int flen = frame_len(d_cfg);
    const int coarse_rel =
        d_cfg.zero_guard_len + d_cfg.preamble_len * d_cfg.preamble_repeats;
    const int preamble_rel = d_cfg.zero_guard_len;
    if (buffered_size() < static_cast<size_t>(flen)) {
        return false;
    }

    const size_t first_preamble = static_cast<size_t>(preamble_rel);
    if (d_next_scan_index < first_preamble) {
        d_next_scan_index = first_preamble;
    }
    const size_t max_preamble =
        buffered_size() - static_cast<size_t>(flen) + static_cast<size_t>(preamble_rel);
    if (d_next_scan_index > max_preamble) {
        return false;
    }

    const int span = d_cfg.preamble_len * (d_cfg.preamble_repeats - 1);
    const auto add_pair = [this](size_t index,
                                 gr_complex& corr,
                                 double& first_power,
                                 double& second_power) {
        const auto first = buffered_sample(index);
        const auto second =
            buffered_sample(index + static_cast<size_t>(d_cfg.preamble_len));
        corr += std::conj(first) * second;
        first_power += std::norm(first);
        second_power += std::norm(second);
    };
    const auto remove_pair = [this](size_t index,
                                    gr_complex& corr,
                                    double& first_power,
                                    double& second_power) {
        const auto first = buffered_sample(index);
        const auto second =
            buffered_sample(index + static_cast<size_t>(d_cfg.preamble_len));
        corr -= std::conj(first) * second;
        first_power -= std::norm(first);
        second_power -= std::norm(second);
    };

    gr_complex preamble_corr(0.0f, 0.0f);
    double first_power = 0.0;
    double second_power = 0.0;
    for (int i = 0; i < span; ++i) {
        add_pair(d_next_scan_index + static_cast<size_t>(i),
                 preamble_corr,
                 first_power,
                 second_power);
    }

    std::deque<std::pair<size_t, float>> zc_metric_cache;
    const auto cached_coarse_metric = [&](size_t coarse_index) {
        const auto cached = std::find_if(
            zc_metric_cache.begin(),
            zc_metric_cache.end(),
            [coarse_index](const auto& entry) { return entry.first == coarse_index; });
        if (cached != zc_metric_cache.end()) {
            return cached->second;
        }
        const float metric = coarse_sync_metric(coarse_index);
        zc_metric_cache.emplace_back(coarse_index, metric);
        if (zc_metric_cache.size() > 16) {
            zc_metric_cache.pop_front();
        }
        return metric;
    };

    for (size_t p = d_next_scan_index; p <= max_preamble; ++p) {
        const size_t start = p - static_cast<size_t>(preamble_rel);
        const int64_t abs_start = static_cast<int64_t>(d_buffer_abs_start + start);
        if (abs_start - d_last_frame_start < d_min_frame_gap) {
        } else {
            float corr_power = std::norm(preamble_corr);
            double threshold_power = d_preamble_threshold * d_preamble_threshold *
                                     first_power * second_power;

            if (corr_power >= threshold_power) {
                float denom = std::sqrt(first_power * second_power);
                const float preamble_metric =
                    denom > 0.0 ? static_cast<float>(std::sqrt(corr_power) / denom)
                                : 0.0f;
                float best_metric = -1.0f;
                size_t best_start = start;
                size_t best_coarse = start + static_cast<size_t>(coarse_rel);
                const auto refine_zc = [&](size_t center) {
                    best_metric = -1.0f;
                    best_start = center;
                    best_coarse = center + static_cast<size_t>(coarse_rel);
                    for (int offset = -2; offset <= 2; ++offset) {
                        const int64_t refined_start_signed =
                            static_cast<int64_t>(center) + offset;
                        if (refined_start_signed < 0) {
                            continue;
                        }
                        const size_t refined_start =
                            static_cast<size_t>(refined_start_signed);
                        if (refined_start + static_cast<size_t>(flen) >
                            buffered_size()) {
                            continue;
                        }
                        const size_t refined_coarse =
                            refined_start + static_cast<size_t>(coarse_rel);
                        const float metric = cached_coarse_metric(refined_coarse);
                        if (metric > best_metric) {
                            best_metric = metric;
                            best_start = refined_start;
                            best_coarse = refined_coarse;
                        }
                    }
                };

                refine_zc(start);
                const int64_t first_offset =
                    static_cast<int64_t>(best_start) - static_cast<int64_t>(start);
                if (best_metric >= d_zc_threshold &&
                    (first_offset == -2 || first_offset == 2)) {
                    refine_zc(best_start);
                }

                const uint64_t refined_abs_start = d_buffer_abs_start + best_start;
                record_candidate(refined_abs_start, preamble_metric, best_metric);
                if (best_metric >= d_zc_threshold) {
                    const size_t refined_preamble =
                        best_start + static_cast<size_t>(preamble_rel);
                    d_next_scan_index =
                        refined_preamble + static_cast<size_t>(d_min_frame_gap);
                    frame_start_index = best_start;
                    coarse_index = best_coarse;
                    preamble_metric_out = preamble_metric;
                    coarse_metric_out = best_metric;
                    return true;
                }
            }
        }

        if (p < max_preamble) {
            remove_pair(p, preamble_corr, first_power, second_power);
            add_pair(
                p + static_cast<size_t>(span), preamble_corr, first_power, second_power);
        }
    }

    d_next_scan_index = max_preamble + 1;
    return false;
}

void prs_frame_detector_impl::publish_frame(size_t frame_start_index,
                                            size_t coarse_index,
                                            float preamble_metric,
                                            float coarse_metric)
{
    const int flen = frame_len(d_cfg);
    const uint64_t abs_start = d_buffer_abs_start + frame_start_index;
    const uint64_t coarse_abs = d_buffer_abs_start + coarse_index;
    const auto frame_begin = d_buffer.begin() + d_buffer_head + frame_start_index;
    std::vector<gr_complex> frame(frame_begin, frame_begin + flen);
    gr_complex cfo_corr(0.0f, 0.0f);
    const int preamble_start = d_cfg.zero_guard_len;
    const int preamble_span = d_cfg.preamble_len * (d_cfg.preamble_repeats - 1);
    for (int i = 0; i < preamble_span; ++i) {
        cfo_corr += std::conj(frame[preamble_start + i]) *
                    frame[preamble_start + i + d_cfg.preamble_len];
    }
    const double cfo_phase_increment =
        std::atan2(cfo_corr.imag(), cfo_corr.real()) /
        static_cast<double>(d_cfg.preamble_len);
    const double preamble_cfo_hz =
        cfo_phase_increment * d_cfg.samp_rate /
        (2.0 * 3.141592653589793238462643383279502884);

    const auto prs_cp_cfo = estimate_prs_cp_cfo(
        frame.data(), frame.size(), d_cfg, preamble_cfo_hz);

    prs_payload_info payload_info;
    float payload_metric = 0.0f;
    const int payload_start = d_cfg.zero_guard_len +
                              d_cfg.preamble_len * d_cfg.preamble_repeats +
                              d_cfg.coarse_sync_len;
    bool frame_id_valid = decode_packet_payload(
        frame.data() + payload_start,
        d_cfg.payload_len,
        payload_info,
        payload_metric,
        cfo_phase_increment);
    bool payload_retry_used = false;
    double selected_cfo_hz = preamble_cfo_hz;
    if (!frame_id_valid && prs_cp_cfo.valid && prs_cp_cfo.coherence >= 0.2) {
        prs_payload_info retry_info;
        float retry_metric = 0.0f;
        const double retry_phase_increment =
            2.0 * 3.141592653589793238462643383279502884 * prs_cp_cfo.hz /
            d_cfg.samp_rate;
        payload_retry_used = true;
        const bool retry_valid = decode_packet_payload(frame.data() + payload_start,
                                                       d_cfg.payload_len,
                                                       retry_info,
                                                       retry_metric,
                                                       retry_phase_increment);
        if (retry_valid || retry_metric > payload_metric) {
            payload_info = retry_info;
            payload_metric = retry_metric;
            selected_cfo_hz = prs_cp_cfo.hz;
        }
        frame_id_valid = retry_valid;
    }
    const uint64_t tx_frame_id = payload_info.packet_type == prs_packet_type_response
                                     ? payload_info.response_frame_id
                                     : payload_info.poll_frame_id;
    const uint64_t recv_id = d_next_frame_id++;
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("recv_id"), pmt::from_uint64(recv_id));
    meta = pmt::dict_add(meta, pmt::mp("frame_id"), pmt::from_uint64(tx_frame_id));
    meta = pmt::dict_add(
        meta, pmt::mp("packet_type"), pmt::from_long(payload_info.packet_type));
    meta = pmt::dict_add(
        meta, pmt::mp("poll_frame_id"), pmt::from_uint64(payload_info.poll_frame_id));
    meta = pmt::dict_add(meta,
                         pmt::mp("response_frame_id"),
                         pmt::from_uint64(payload_info.response_frame_id));
    meta = pmt::dict_add(meta,
                         pmt::mp("reply_delay_samples"),
                         pmt::from_uint64(payload_info.reply_delay_samples));
    meta = pmt::dict_add(
        meta, pmt::mp("frame_id_valid"), frame_id_valid ? pmt::PMT_T : pmt::PMT_F);
    meta =
        pmt::dict_add(meta, pmt::mp("payload_metric"), pmt::from_double(payload_metric));
    meta = pmt::dict_add(
        meta, pmt::mp("absolute_sample_index"), pmt::from_uint64(abs_start));
    meta = pmt::dict_add(meta, pmt::mp("frame_start"), pmt::from_uint64(abs_start));
    meta = pmt::dict_add(meta, pmt::mp("coarse_peak"), pmt::from_uint64(coarse_abs));
    meta = pmt::dict_add(
        meta, pmt::mp("preamble_metric"), pmt::from_double(preamble_metric));
    meta = pmt::dict_add(meta, pmt::mp("coarse_metric"), pmt::from_double(coarse_metric));
    meta = pmt::dict_add(
        meta, pmt::mp("coarse_zc_root"), pmt::from_long(d_cfg.coarse_zc_root));
    meta = pmt::dict_add(meta, pmt::mp("channel_id"), pmt::from_long(d_cfg.channel_id));
    meta = pmt::dict_add(
        meta, pmt::mp("preamble_cfo_hz"), pmt::from_double(preamble_cfo_hz));
    meta = pmt::dict_add(meta,
                         pmt::mp("prs_cp_cfo_hz"),
                         pmt::from_double(prs_cp_cfo.valid ? prs_cp_cfo.hz : 0.0));
    meta = pmt::dict_add(meta,
                         pmt::mp("prs_cp_cfo_coherence"),
                         pmt::from_double(prs_cp_cfo.coherence));
    meta = pmt::dict_add(meta,
                         pmt::mp("payload_retry_used"),
                         payload_retry_used ? pmt::PMT_T : pmt::PMT_F);
    meta = pmt::dict_add(
        meta, pmt::mp("selected_cfo_hz"), pmt::from_double(selected_cfo_hz));
    meta = pmt::dict_add(meta, pmt::mp("cfo"), pmt::from_double(selected_cfo_hz));
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
    if (!have_gated_attempt && frame_id_valid) {
        attempt_id = payload_info.poll_frame_id;
    }
    if (have_gated_attempt || frame_id_valid) {
        meta = pmt::dict_add(meta, pmt::mp("attempt_id"), pmt::from_uint64(attempt_id));
    }
    meta = pmt::dict_add(meta,
                         pmt::mp("failure_reason"),
                         pmt::mp(frame_id_valid ? "NONE" : "PAYLOAD_CRC"));
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

    const auto data = pmt::init_c32vector(frame.size(), frame);
    const auto pdu = pmt::cons(meta, data);
    message_port_pub(pmt::mp("event_out"), pdu);
    message_port_pub(pmt::mp("frame_out"), pdu);
    d_last_frame_start = static_cast<int64_t>(abs_start);
}

int prs_frame_detector_impl::general_work(int noutput_items,
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
