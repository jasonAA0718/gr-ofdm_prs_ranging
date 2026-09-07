/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "mc_ds_prs.h"
#include "prs_channel_estimator_impl.h"
#include "prs_payload_codec.h"
#include "prs_timing_helper.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
}

prs_channel_estimator::sptr prs_channel_estimator::make(double samp_rate,
                                                        int fft_len,
                                                        int active_bins,
                                                        int prs_symbols,
                                                        uint32_t seed,
                                                        bool enable_profiling)
{
    return gnuradio::make_block_sptr<prs_channel_estimator_impl>(
        samp_rate, fft_len, active_bins, prs_symbols, seed, enable_profiling);
}

prs_channel_estimator_impl::prs_channel_estimator_impl(double samp_rate,
                                                       int fft_len,
                                                       int active_bins,
                                                       int prs_symbols,
                                                       uint32_t seed,
                                                       bool enable_profiling)
    : gr::block("prs_channel_estimator",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_enable_profiling(enable_profiling)
{
    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.active_bins = active_bins;
    d_cfg.prs_symbols = prs_symbols;
    (void)seed;
    if (fft_len != mc_ds_fft_len || active_bins != mc_ds_fft_len ||
        prs_symbols != mc_ds_symbol_count) {
        throw std::invalid_argument(
            "MC-DS channel estimator requires 1024 bins and 8 symbols");
    }
    d_symbol_channels.resize(static_cast<size_t>(prs_symbols * active_bins));
    d_full_channel.resize(static_cast<size_t>(active_bins));
    d_channel.resize(mc_ds_pilot_bin_count);
    d_channel_energy.resize(static_cast<size_t>(active_bins));
    message_port_register_in(pmt::mp("symbols_in"));
    message_port_register_out(pmt::mp("channel_out"));
    message_port_register_out(pmt::mp("event_out"));
    message_port_register_out(pmt::mp("timing_out"));
    set_msg_handler(pmt::mp("symbols_in"),
                    [this](pmt::pmt_t msg) { handle_symbols(msg); });
}

void prs_channel_estimator_impl::handle_symbols(pmt::pmt_t msg)
{
    profiling::timing_report report(d_enable_profiling, "channel_estimator");
    pmt::pmt_t meta;
    const gr_complex* symbols = nullptr;
    size_t symbols_size = 0;
    if (!pdu_get_c32_view(msg, meta, symbols, symbols_size)) {
        return;
    }
    report.checkpoint("input_decode");
    const size_t expected = static_cast<size_t>(d_cfg.prs_symbols * d_cfg.active_bins);
    if (symbols_size < expected) {
        meta = pmt::dict_add(meta, pmt::mp("channel_error"), pmt::mp("short_symbols"));
        meta = pmt::dict_add(meta, pmt::mp("frame_id_valid"), pmt::PMT_F);
        meta = pmt::dict_add(meta, pmt::mp("failure_reason"), pmt::mp("PAYLOAD_CRC"));
        message_port_pub(pmt::mp("event_out"), pmt::cons(meta, pmt::PMT_NIL));
        message_port_pub(
            pmt::mp("channel_out"),
            pmt::cons(meta, pmt::init_c32vector(0, std::vector<gr_complex>())));
        if (report.enabled()) {
            message_port_pub(pmt::mp("timing_out"), report.finish(meta));
        }
        return;
    }

    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        for (int ordered_bin = 0; ordered_bin < d_cfg.active_bins; ++ordered_bin) {
            const size_t idx = static_cast<size_t>(sym * d_cfg.active_bins + ordered_bin);
            const int native_bin = mc_ds_ordered_to_native_bin(ordered_bin);
            d_symbol_channels[idx] = mc_ds_is_data_bin(native_bin)
                                         ? gr_complex(0.0f, 0.0f)
                                         : symbols[idx] / mc_ds_pilot(sym, native_bin);
        }
    }
    report.checkpoint("pilot_removal");

    std::complex<double> symbol_corr(0.0, 0.0);
    double previous_power = 0.0;
    double next_power = 0.0;
    for (int sym = 0; sym + 1 < d_cfg.prs_symbols; ++sym) {
        for (int ordered_bin = 0; ordered_bin < d_cfg.active_bins; ++ordered_bin) {
            const int native_bin = mc_ds_ordered_to_native_bin(ordered_bin);
            if (mc_ds_is_data_bin(native_bin)) {
                continue;
            }
            const size_t first =
                static_cast<size_t>(sym * d_cfg.active_bins + ordered_bin);
            const size_t second = first + static_cast<size_t>(d_cfg.active_bins);
            const auto h0 = std::complex<double>(d_symbol_channels[first]);
            const auto h1 = std::complex<double>(d_symbol_channels[second]);
            symbol_corr += std::conj(h0) * h1;
            previous_power += std::norm(h0);
            next_power += std::norm(h1);
        }
    }

    const double symbol_period =
        static_cast<double>(d_cfg.fft_len + d_cfg.cp_len) / d_cfg.samp_rate;
    double phase = std::atan2(symbol_corr.imag(), symbol_corr.real());
    double unwrap_reference_hz =
        dict_ref_double(meta, "prs_cp_cfo_hz", std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(unwrap_reference_hz)) {
        unwrap_reference_hz = dict_ref_double(meta, "detection_cfo_hz", 0.0);
    }
    const double reference_phase = 2.0 * pi * unwrap_reference_hz * symbol_period;
    phase += 2.0 * pi * std::round((reference_phase - phase) / (2.0 * pi));
    const double prs_channel_cfo_hz = phase / (2.0 * pi * symbol_period);
    const double coherence_denom = std::sqrt(previous_power * next_power);
    const double channel_coherence =
        coherence_denom > 0.0 ? std::min(1.0, std::abs(symbol_corr) / coherence_denom)
                              : 0.0;
    report.checkpoint("channel_cfo_estimation");

    std::array<gr_complex, mc_ds_symbol_count> symbol_rotations{};
    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        const double angle = -2.0 * pi * prs_channel_cfo_hz * symbol_period * sym;
        symbol_rotations[static_cast<size_t>(sym)] = gr_complex(
            static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
    }

    std::fill(d_full_channel.begin(), d_full_channel.end(), gr_complex(0.0f, 0.0f));
    std::fill(d_channel_energy.begin(), d_channel_energy.end(), 0.0);
    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        const auto rotation = symbol_rotations[static_cast<size_t>(sym)];
        for (int ordered_bin = 0; ordered_bin < d_cfg.active_bins; ++ordered_bin) {
            const int native_bin = mc_ds_ordered_to_native_bin(ordered_bin);
            if (mc_ds_is_data_bin(native_bin)) {
                continue;
            }
            const size_t idx = static_cast<size_t>(sym * d_cfg.active_bins + ordered_bin);
            const auto h = d_symbol_channels[idx] * rotation;
            d_symbol_channels[idx] = h;
            d_full_channel[static_cast<size_t>(ordered_bin)] += h;
            d_channel_energy[static_cast<size_t>(ordered_bin)] += std::norm(h);
        }
    }

    double signal_power = 0.0;
    double error_power = 0.0;
    size_t compact_index = 0;
    for (int ordered_bin = 0; ordered_bin < d_cfg.active_bins; ++ordered_bin) {
        const int native_bin = mc_ds_ordered_to_native_bin(ordered_bin);
        if (mc_ds_is_data_bin(native_bin)) {
            continue;
        }
        auto& h = d_full_channel[static_cast<size_t>(ordered_bin)];
        h /= static_cast<float>(d_cfg.prs_symbols);
        d_channel[compact_index++] = h;
        signal_power += std::norm(h);
        const double residual = d_channel_energy[static_cast<size_t>(ordered_bin)] -
                                d_cfg.prs_symbols * std::norm(h);
        error_power += std::max(0.0, residual);
    }
    signal_power /= mc_ds_pilot_bin_count;
    error_power /= static_cast<double>(mc_ds_pilot_bin_count * d_cfg.prs_symbols);
    const double snr = 10.0 * std::log10((signal_power + 1e-12) / (error_power + 1e-12));
    report.checkpoint("cfo_rotation_average");

    std::array<uint8_t, prs_payload_data_bits> decoded_bits{};
    double margin_sum = 0.0;
    for (int q = 0; q < mc_ds_data_bin_count; ++q) {
        const int native_bin = q * mc_ds_bin_period + mc_ds_data_bin_remainder;
        const int ordered_bin = mc_ds_native_to_ordered_bin(native_bin);
        gr_complex combined(0.0f, 0.0f);
        for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
            const size_t idx = static_cast<size_t>(sym * d_cfg.active_bins + ordered_bin);
            combined += symbols[idx] * mc_ds_prn_code[static_cast<size_t>(sym)] *
                        symbol_rotations[static_cast<size_t>(sym)];
        }
        combined /= static_cast<float>(d_cfg.prs_symbols);

        const int left_ordered = mc_ds_native_to_ordered_bin(native_bin - 1);
        gr_complex h = d_full_channel[static_cast<size_t>(left_ordered)];
        if (native_bin != mc_ds_fft_len - 1) {
            const int right_ordered = mc_ds_native_to_ordered_bin(native_bin + 1);
            h = 0.5f * (h + d_full_channel[static_cast<size_t>(right_ordered)]);
        }
        const float h_power = std::norm(h);
        const gr_complex equalized =
            h_power > 1e-12f ? combined * std::conj(h) / h_power : gr_complex(0.0f, 0.0f);
        if (q < prs_payload_data_bits) {
            decoded_bits[static_cast<size_t>(q)] = equalized.real() >= 0.0f ? 1U : 0U;
            margin_sum +=
                std::abs(equalized.real()) / std::max(1e-12f, std::abs(equalized));
        }
    }
    prs_payload_info payload_info;
    const bool frame_id_valid = deserialize_packet_payload(
        decoded_bits.data(), decoded_bits.size(), payload_info);
    const double payload_metric = margin_sum / prs_payload_data_bits;
    report.checkpoint("payload_equalize_decode");

    const uint64_t frame_id = payload_info.packet_type == prs_packet_type_response
                                  ? payload_info.response_frame_id
                                  : payload_info.poll_frame_id;
    meta = pmt::dict_add(meta, pmt::mp("frame_id"), pmt::from_uint64(frame_id));
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
        meta, pmt::mp("payload_initial_valid"), frame_id_valid ? pmt::PMT_T : pmt::PMT_F);
    meta = pmt::dict_add(
        meta, pmt::mp("payload_initial_metric"), pmt::from_double(payload_metric));
    meta = pmt::dict_add(meta, pmt::mp("payload_retry_valid"), pmt::PMT_F);
    meta = pmt::dict_add(meta, pmt::mp("payload_retry_metric"), pmt::from_double(0.0));
    meta = pmt::dict_add(meta, pmt::mp("payload_retry_used"), pmt::PMT_F);
    meta = pmt::dict_add(meta,
                         pmt::mp("failure_reason"),
                         pmt::mp(frame_id_valid ? "NONE" : "PAYLOAD_CRC"));
    if (pmt::is_null(pmt::dict_ref(meta, pmt::mp("attempt_id"), pmt::PMT_NIL))) {
        meta = pmt::dict_add(
            meta, pmt::mp("attempt_id"), pmt::from_uint64(payload_info.poll_frame_id));
    }
    meta = pmt::dict_add(meta, pmt::mp("snr"), pmt::from_double(snr));
    meta = pmt::dict_add(
        meta, pmt::mp("prs_channel_cfo_hz"), pmt::from_double(prs_channel_cfo_hz));
    meta = pmt::dict_add(meta,
                         pmt::mp("residual_cfo_hz"),
                         pmt::from_double(prs_channel_cfo_hz - unwrap_reference_hz));
    meta = pmt::dict_add(
        meta, pmt::mp("channel_coherence"), pmt::from_double(channel_coherence));
    meta = pmt::dict_add(
        meta, pmt::mp("channel_bins"), pmt::from_long(mc_ds_pilot_bin_count));
    meta = pmt::dict_add(meta, pmt::mp("mc_ds_enabled"), pmt::PMT_T);
    meta = pmt::dict_add(
        meta, pmt::mp("mc_ds_code_length"), pmt::from_long(mc_ds_code_length));
    meta = pmt::dict_add(meta, pmt::mp("mc_ds_code_id"), pmt::from_long(mc_ds_code_id));
    meta = pmt::dict_add(
        meta, pmt::mp("mc_ds_despread_metric"), pmt::from_double(payload_metric));
    report.checkpoint("metadata_build");

    const auto output = pmt::cons(meta, pmt::init_c32vector(d_channel.size(), d_channel));
    message_port_pub(pmt::mp("event_out"), pmt::cons(meta, pmt::PMT_NIL));
    message_port_pub(pmt::mp("channel_out"), output);
    if (report.enabled()) {
        message_port_pub(pmt::mp("timing_out"), report.finish(meta));
    }
}

} // namespace ofdm_prs_ranging
} // namespace gr
