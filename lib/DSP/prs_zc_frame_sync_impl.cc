/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_timing_helper.h"
#include "prs_zc_frame_sync_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr double pi = 3.141592653589793238462643383279502884;

double pmt_time_value(const pmt::pmt_t& value)
{
    if (pmt::is_real(value)) {
        return pmt::to_double(value);
    }
    if (pmt::is_tuple(value) && pmt::length(value) >= 2) {
        const auto seconds = pmt::tuple_ref(value, 0);
        const auto fraction = pmt::tuple_ref(value, 1);
        if ((pmt::is_uint64(seconds) || pmt::is_integer(seconds)) &&
            pmt::is_real(fraction)) {
            const double whole = pmt::is_uint64(seconds)
                                     ? static_cast<double>(pmt::to_uint64(seconds))
                                     : static_cast<double>(pmt::to_long(seconds));
            return whole + pmt::to_double(fraction);
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

pmt::pmt_t time_tuple(double value)
{
    const double whole = std::floor(value);
    return pmt::make_tuple(pmt::from_uint64(static_cast<uint64_t>(whole)),
                           pmt::from_double(value - whole));
}

bool metadata_int_matches(const pmt::pmt_t& meta, const char* key_name, int expected)
{
    const auto value = pmt::dict_ref(meta, pmt::mp(key_name), pmt::PMT_NIL);
    if (pmt::is_null(value)) {
        return true;
    }
    if (pmt::is_uint64(value)) {
        return pmt::to_uint64(value) == static_cast<uint64_t>(expected);
    }
    return pmt::is_integer(value) && pmt::to_long(value) == expected;
}

uint64_t elapsed_ns(profiling::timing_point start, profiling::timing_point stop)
{
    return profiling::elapsed_ns(start, stop);
}
} // namespace

prs_zc_frame_sync::sptr prs_zc_frame_sync::make(double samp_rate,
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
                                                bool enable_profiling)
{
    return gnuradio::make_block_sptr<prs_zc_frame_sync_impl>(samp_rate,
                                                             fft_len,
                                                             cp_len,
                                                             active_bins,
                                                             prs_symbols,
                                                             preamble_len,
                                                             preamble_repeats,
                                                             coarse_sync_len,
                                                             preamble_threshold,
                                                             zc_threshold,
                                                             coarse_zc_root,
                                                             channel_id,
                                                             enable_profiling);
}

prs_zc_frame_sync_impl::prs_zc_frame_sync_impl(double samp_rate,
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
                                               bool enable_profiling)
    : gr::block("prs_zc_frame_sync",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_preamble_threshold(preamble_threshold),
      d_zc_threshold(zc_threshold),
      d_enable_profiling(enable_profiling),
      d_next_recv_id(0)
{
    if (samp_rate <= 0.0 || fft_len <= 0 || cp_len < 0 || active_bins <= 0 ||
        prs_symbols <= 0 || preamble_len <= 0 || preamble_repeats < 2 ||
        coarse_sync_len <= 0) {
        throw std::invalid_argument("invalid PRS ZC frame-sync geometry");
    }
    if (preamble_threshold <= 0.0f || preamble_threshold > 1.0f || zc_threshold <= 0.0f ||
        zc_threshold > 1.0f) {
        throw std::invalid_argument("preamble and ZC thresholds must be in (0, 1]");
    }

    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.cp_len = cp_len;
    d_cfg.active_bins = active_bins;
    d_cfg.prs_symbols = prs_symbols;
    d_cfg.preamble_len = preamble_len;
    d_cfg.preamble_repeats = preamble_repeats;
    d_cfg.coarse_sync_len = coarse_sync_len;
    d_cfg.coarse_zc_root = coarse_zc_root;
    d_cfg.channel_id = channel_id;
    d_coarse = coarse_sync_sequence(coarse_sync_len, coarse_zc_root);

    message_port_register_in(pmt::mp("candidate_in"));
    message_port_register_out(pmt::mp("synchronized_frame_out"));
    message_port_register_out(pmt::mp("diagnostics_out"));
    message_port_register_out(pmt::mp("slot_release"));
    message_port_register_out(pmt::mp("timing_out"));
    set_msg_handler(pmt::mp("candidate_in"),
                    [this](const pmt::pmt_t& msg) { handle_candidate(msg); });
}

prs_zc_frame_sync_impl::release_guard::~release_guard() noexcept
{
    if (valid) {
        owner->publish_slot_release(candidate_id);
    }
}

void prs_zc_frame_sync_impl::publish_slot_release(uint64_t candidate_id) noexcept
{
    try {
        auto message = pmt::make_dict();
        message = pmt::dict_add(
            message, pmt::mp("candidate_id"), pmt::from_uint64(candidate_id));
        message_port_pub(pmt::mp("slot_release"), message);
    } catch (...) {
        // Message publication must never allow an exception to leave a handler guard.
    }
}

bool prs_zc_frame_sync_impl::metadata_matches_geometry(const pmt::pmt_t& meta) const
{
    return metadata_int_matches(meta, "fft_len", d_cfg.fft_len) &&
           metadata_int_matches(meta, "cp_len", d_cfg.cp_len) &&
           metadata_int_matches(meta, "active_bins", d_cfg.active_bins) &&
           metadata_int_matches(meta, "prs_symbols", d_cfg.prs_symbols) &&
           metadata_int_matches(meta, "preamble_len", d_cfg.preamble_len) &&
           metadata_int_matches(meta, "preamble_repeats", d_cfg.preamble_repeats) &&
           metadata_int_matches(meta, "coarse_sync_len", d_cfg.coarse_sync_len) &&
           metadata_int_matches(meta, "zero_guard_len", phy::leading_guard_samples) &&
           metadata_int_matches(meta, "tail_guard_len", phy::tail_guard_samples);
}

prs_zc_frame_sync_impl::cfo_reference
prs_zc_frame_sync_impl::select_cfo_reference(const gr_complex* samples, size_t size) const
{
    cfo_reference selected;
    const size_t lag = static_cast<size_t>(d_cfg.preamble_len);
    const size_t span = lag * static_cast<size_t>(d_cfg.preamble_repeats - 1);
    const size_t total = lag * static_cast<size_t>(d_cfg.preamble_repeats);
    if (samples == nullptr || size < total) {
        return selected;
    }

    gr_complex correlation(0.0f, 0.0f);
    double first_power = 0.0;
    double second_power = 0.0;
    const auto add_pair = [&](size_t index) {
        const auto first = samples[index];
        const auto second = samples[index + lag];
        correlation += std::conj(first) * second;
        first_power += std::norm(first);
        second_power += std::norm(second);
    };
    const auto remove_pair = [&](size_t index) {
        const auto first = samples[index];
        const auto second = samples[index + lag];
        correlation -= std::conj(first) * second;
        first_power -= std::norm(first);
        second_power -= std::norm(second);
    };

    for (size_t i = 0; i < span; ++i) {
        add_pair(i);
    }
    const size_t last_start = size - total;
    for (size_t start = 0; start <= last_start; ++start) {
        const double p0 = std::max(0.0, first_power);
        const double p1 = std::max(0.0, second_power);
        const double product = p0 * p1;
        const double corr_power = std::norm(correlation);
        const double threshold_power =
            static_cast<double>(d_preamble_threshold) * d_preamble_threshold * product;
        if (product > 0.0 && corr_power >= threshold_power) {
            const float magnitude = std::abs(correlation);
            if (!selected.valid || magnitude > selected.magnitude) {
                selected.valid = true;
                selected.index = start;
                selected.correlation = correlation;
                selected.magnitude = magnitude;
                selected.metric = static_cast<float>(std::sqrt(corr_power / product));
            }
        }

        if (start < last_start) {
            remove_pair(start);
            add_pair(start + span);
        }
    }

    if (selected.valid) {
        selected.cfo_hz = d_cfg.samp_rate /
                          (2.0 * pi * static_cast<double>(d_cfg.preamble_len)) *
                          std::arg(selected.correlation);
    }
    return selected;
}

void prs_zc_frame_sync_impl::correct_cfo(const gr_complex* samples,
                                         size_t size,
                                         double cfo_hz)
{
    d_scratch.resize(size);
    const double radians_per_sample = -2.0 * pi * cfo_hz / d_cfg.samp_rate;
    const gr_complex step(static_cast<float>(std::cos(radians_per_sample)),
                          static_cast<float>(std::sin(radians_per_sample)));
    gr_complex rotation(1.0f, 0.0f);
    for (size_t i = 0; i < size; ++i) {
        d_scratch[i] = samples[i] * rotation;
        rotation *= step;
    }
}

bool prs_zc_frame_sync_impl::search_zc(size_t search_start,
                                       size_t search_end,
                                       size_t& selected_index,
                                       float& selected_metric) const
{
    selected_index = search_start;
    selected_metric = 0.0f;
    const size_t zc_len = static_cast<size_t>(d_cfg.coarse_sync_len);
    double power = 0.0;
    for (size_t i = 0; i < zc_len; ++i) {
        power += std::norm(d_scratch[search_start + i]);
    }

    double best_metric_squared = -1.0;
    for (size_t start = search_start; start <= search_end; ++start) {
        gr_complex correlation(0.0f, 0.0f);
        volk_32fc_x2_conjugate_dot_prod_32fc(
            reinterpret_cast<lv_32fc_t*>(&correlation),
            reinterpret_cast<const lv_32fc_t*>(d_scratch.data() + start),
            reinterpret_cast<const lv_32fc_t*>(d_coarse.data()),
            static_cast<unsigned int>(zc_len));

        const double denominator_squared =
            std::max(0.0, power) * static_cast<double>(zc_len);
        const double metric_squared =
            denominator_squared > 0.0 ? std::norm(correlation) / denominator_squared : 0.0;
        if (metric_squared > best_metric_squared) {
            best_metric_squared = metric_squared;
            selected_index = start;
        }

        if (start < search_end) {
            power -= std::norm(d_scratch[start]);
            power += std::norm(d_scratch[start + zc_len]);
        }
    }
    selected_metric = best_metric_squared > 0.0
                          ? static_cast<float>(std::sqrt(best_metric_squared))
                          : 0.0f;
    const double threshold_squared =
        static_cast<double>(d_zc_threshold) * static_cast<double>(d_zc_threshold);
    return best_metric_squared >= threshold_squared;
}

void prs_zc_frame_sync_impl::handle_candidate(const pmt::pmt_t& message)
{
    const auto total_start = profiling::now();
    pmt::pmt_t input_meta = pmt::is_pair(message) ? pmt::car(message) : pmt::PMT_NIL;
    const uint64_t missing_id = std::numeric_limits<uint64_t>::max();
    const uint64_t candidate_id =
        pmt::is_dict(input_meta) ? dict_ref_uint64(input_meta, "candidate_id", missing_id)
                                 : missing_id;
    release_guard release{ this, candidate_id, candidate_id != missing_id };
    profiling::timing_report timing(d_enable_profiling, "zc_frame_sync");

    pmt::pmt_t diagnostics = pmt::is_dict(input_meta) ? input_meta : pmt::make_dict();
    const auto finish_diagnostics = [&](const char* reason,
                                        bool success,
                                        uint64_t cfo_ns,
                                        uint64_t zc_ns,
                                        uint64_t extraction_ns) {
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("success"), success ? pmt::PMT_T : pmt::PMT_F);
        diagnostics =
            pmt::dict_add(diagnostics, pmt::mp("failure_reason"), pmt::mp(reason));
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("cfo_processing_time_ns"), pmt::from_uint64(cfo_ns));
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("zc_processing_time_ns"), pmt::from_uint64(zc_ns));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("frame_extraction_time_ns"),
                                    pmt::from_uint64(extraction_ns));
        diagnostics =
            pmt::dict_add(diagnostics,
                          pmt::mp("block_b_total_time_ns"),
                          pmt::from_uint64(elapsed_ns(total_start, profiling::now())));
        message_port_pub(pmt::mp("diagnostics_out"),
                         pmt::cons(diagnostics, pmt::PMT_NIL));
        if (timing.enabled()) {
            auto timing_message = timing.finish(diagnostics);
            for (const char* key_name : { "candidate_id", "window_id" }) {
                const auto key = pmt::mp(key_name);
                const auto value = pmt::dict_ref(diagnostics, key, pmt::PMT_NIL);
                if (!pmt::is_null(value)) {
                    timing_message = pmt::dict_add(timing_message, key, value);
                }
            }
            message_port_pub(pmt::mp("timing_out"), timing_message);
        }
    };

    try {
        const gr_complex* samples = nullptr;
        size_t sample_count = 0;
        if (!pdu_get_c32_view(message, input_meta, samples, sample_count) ||
            !pmt::is_dict(input_meta)) {
            finish_diagnostics("INVALID_CANDIDATE", false, 0, 0, 0);
            return;
        }
        diagnostics = input_meta;
        if (!metadata_matches_geometry(input_meta)) {
            finish_diagnostics("INVALID_GEOMETRY", false, 0, 0, 0);
            return;
        }

        const uint64_t candidate_start =
            dict_ref_uint64(input_meta,
                            "candidate_window_start",
                            dict_ref_uint64(input_meta, "absolute_sample_index", 0));
        const uint64_t candidate_end = dict_ref_uint64(
            input_meta, "candidate_window_end", candidate_start + sample_count);
        const uint64_t first_crossing =
            dict_ref_uint64(input_meta, "first_preamble_crossing_index", candidate_start);
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("candidate_window_start"),
                                    pmt::from_uint64(candidate_start));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("candidate_window_end"),
                                    pmt::from_uint64(candidate_end));

        const uint64_t missing_bound = std::numeric_limits<uint64_t>::max();
        const uint64_t search_start_u64 =
            dict_ref_uint64(input_meta, "zc_search_start_offset", missing_bound);
        const uint64_t search_end_u64 =
            dict_ref_uint64(input_meta, "zc_search_end_offset", missing_bound);
        const size_t coarse_rel = static_cast<size_t>(phy::leading_guard_samples) +
                                  static_cast<size_t>(d_cfg.preamble_len) *
                                      static_cast<size_t>(d_cfg.preamble_repeats);
        const size_t flen = static_cast<size_t>(frame_len(d_cfg));
        const size_t samples_from_zc_to_frame_end = flen - coarse_rel;
        const bool candidate_end_valid =
            candidate_start <= std::numeric_limits<uint64_t>::max() - sample_count &&
            candidate_end == candidate_start + sample_count;
        const bool bounds_fit =
            sample_count >= samples_from_zc_to_frame_end &&
            search_end_u64 <= sample_count - samples_from_zc_to_frame_end;
        if (search_start_u64 == missing_bound || search_end_u64 == missing_bound ||
            search_start_u64 > search_end_u64 || search_start_u64 < coarse_rel ||
            !candidate_end_valid || !bounds_fit || first_crossing < candidate_start ||
            first_crossing >= candidate_end) {
            finish_diagnostics("INVALID_GEOMETRY", false, 0, 0, 0);
            return;
        }
        const size_t search_start = static_cast<size_t>(search_start_u64);
        const size_t search_end = static_cast<size_t>(search_end_u64);
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("zc_search_start_offset"),
                                    pmt::from_uint64(search_start));
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("zc_search_end_offset"), pmt::from_uint64(search_end));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("zc_search_width"),
                                    pmt::from_uint64(search_end - search_start + 1U));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("zc_search_start_absolute_index"),
                                    pmt::from_uint64(candidate_start + search_start));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("zc_search_end_absolute_index"),
                                    pmt::from_uint64(candidate_start + search_end));

        const auto cfo_start = profiling::now();
        const auto cfo = select_cfo_reference(samples, sample_count);
        if (!cfo.valid) {
            const uint64_t cfo_ns = elapsed_ns(cfo_start, profiling::now());
            timing.add("preamble_cfo", cfo_ns);
            finish_diagnostics("PREAMBLE_INVALID", false, cfo_ns, 0, 0);
            return;
        }
        correct_cfo(samples, sample_count, cfo.cfo_hz);
        const uint64_t cfo_ns = elapsed_ns(cfo_start, profiling::now());
        timing.add("preamble_cfo", cfo_ns);
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("cfo_reference_index"), pmt::from_uint64(cfo.index));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("cfo_reference_absolute_index"),
                                    pmt::from_uint64(candidate_start + cfo.index));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("cfo_reference_p_magnitude"),
                                    pmt::from_double(cfo.magnitude));
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("cfo_reference_metric"), pmt::from_double(cfo.metric));
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("preamble_cfo_hz"), pmt::from_double(cfo.cfo_hz));

        const auto zc_start = profiling::now();
        size_t selected_zc = 0;
        float zc_metric = 0.0f;
        const bool zc_valid = search_zc(search_start, search_end, selected_zc, zc_metric);
        const uint64_t zc_ns = elapsed_ns(zc_start, profiling::now());
        timing.add("direct_zc_search", zc_ns);
        const uint64_t selected_zc_abs = candidate_start + selected_zc;
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("coarse_metric"), pmt::from_double(zc_metric));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("selected_local_zc_index"),
                                    pmt::from_uint64(selected_zc));
        diagnostics = pmt::dict_add(diagnostics,
                                    pmt::mp("selected_absolute_zc_index"),
                                    pmt::from_uint64(selected_zc_abs));
        diagnostics = dict_add_int(diagnostics,
                                   "zc_selected_minus_first_preamble_crossing_samples",
                                   static_cast<int64_t>(selected_zc_abs) -
                                       static_cast<int64_t>(first_crossing));
        const size_t frame_start_local = selected_zc - coarse_rel;
        const uint64_t frame_start_abs = candidate_start + frame_start_local;
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("frame_start"), pmt::from_uint64(frame_start_abs));
        if (!zc_valid) {
            finish_diagnostics("ZC_SYNC", false, cfo_ns, zc_ns, 0);
            return;
        }

        if (frame_start_local > sample_count || flen > sample_count - frame_start_local) {
            finish_diagnostics("INVALID_GEOMETRY", false, cfo_ns, zc_ns, 0);
            return;
        }

        const auto extraction_start = profiling::now();
        std::vector<gr_complex> frame(samples + frame_start_local,
                                      samples + frame_start_local + flen);
        const uint64_t extraction_ns = elapsed_ns(extraction_start, profiling::now());
        timing.add("frame_extract", extraction_ns);

        const auto cp_cfo_start = profiling::now();
        const auto prs_cp_cfo =
            estimate_prs_cp_cfo(frame.data(), frame.size(), d_cfg, 0.0);
        timing.add("prs_cp_cfo", elapsed_ns(cp_cfo_start, profiling::now()));

        pmt::pmt_t frame_meta = input_meta;
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("recv_id"), pmt::from_uint64(d_next_recv_id++));
        frame_meta = pmt::dict_add(frame_meta,
                                   pmt::mp("absolute_sample_index"),
                                   pmt::from_uint64(frame_start_abs));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("frame_start"), pmt::from_uint64(frame_start_abs));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("coarse_peak"), pmt::from_uint64(selected_zc_abs));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("preamble_metric"), pmt::from_double(cfo.metric));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("coarse_metric"), pmt::from_double(zc_metric));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("coarse_zc_root"), pmt::from_long(d_cfg.coarse_zc_root));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("channel_id"), pmt::from_long(d_cfg.channel_id));
        frame_meta =
            pmt::dict_add(frame_meta,
                          pmt::mp("prs_cp_cfo_hz"),
                          pmt::from_double(prs_cp_cfo.valid ? prs_cp_cfo.hz : 0.0));
        frame_meta = pmt::dict_add(frame_meta,
                                   pmt::mp("prs_cp_cfo_coherence"),
                                   pmt::from_double(prs_cp_cfo.coherence));
        frame_meta =
            pmt::dict_add(frame_meta,
                          pmt::mp("selected_cfo_hz"),
                          pmt::from_double(prs_cp_cfo.valid ? prs_cp_cfo.hz : 0.0));
        frame_meta =
            pmt::dict_add(frame_meta,
                          pmt::mp("cfo"),
                          pmt::from_double(prs_cp_cfo.valid ? prs_cp_cfo.hz : 0.0));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("samp_rate"), pmt::from_double(d_cfg.samp_rate));
        frame_meta =
            pmt::dict_add(frame_meta, pmt::mp("fft_len"), pmt::from_long(d_cfg.fft_len));
        frame_meta =
            pmt::dict_add(frame_meta, pmt::mp("cp_len"), pmt::from_long(d_cfg.cp_len));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("active_bins"), pmt::from_long(d_cfg.active_bins));
        frame_meta = pmt::dict_add(
            frame_meta, pmt::mp("prs_symbols"), pmt::from_long(d_cfg.prs_symbols));
        frame_meta = pmt::dict_add(frame_meta,
                                   pmt::mp("prs_start_rel"),
                                   pmt::from_long(prs_start_offset(d_cfg)));
        frame_meta =
            pmt::dict_add(frame_meta, pmt::mp("prs_len"), pmt::from_long(prs_len(d_cfg)));
        const double candidate_time = pmt_time_value(
            pmt::dict_ref(input_meta, pmt::mp("candidate_rx_time"), pmt::PMT_NIL));
        if (std::isfinite(candidate_time)) {
            frame_meta = pmt::dict_add(
                frame_meta,
                pmt::mp("rx_time"),
                time_tuple(candidate_time +
                           static_cast<double>(frame_start_local) / d_cfg.samp_rate));
        }

        message_port_pub(pmt::mp("synchronized_frame_out"),
                         pmt::cons(frame_meta, pmt::init_c32vector(frame.size(), frame)));
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("preamble_metric"), pmt::from_double(cfo.metric));
        finish_diagnostics("NONE", true, cfo_ns, zc_ns, extraction_ns);
    } catch (const std::exception& error) {
        diagnostics = pmt::dict_add(
            diagnostics, pmt::mp("error"), pmt::string_to_symbol(error.what()));
        finish_diagnostics("PROCESSING_ERROR", false, 0, 0, 0);
    } catch (...) {
        finish_diagnostics("PROCESSING_ERROR", false, 0, 0, 0);
    }
}

} // namespace ofdm_prs_ranging
} // namespace gr
