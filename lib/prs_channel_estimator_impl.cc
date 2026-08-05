/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_channel_estimator_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
}

prs_channel_estimator::sptr prs_channel_estimator::make(double samp_rate,
                                                        int fft_len,
                                                        int active_bins,
                                                        int prs_symbols,
                                                        uint32_t seed)
{
    return gnuradio::make_block_sptr<prs_channel_estimator_impl>(
        samp_rate, fft_len, active_bins, prs_symbols, seed);
}

prs_channel_estimator_impl::prs_channel_estimator_impl(double samp_rate,
                                                       int fft_len,
                                                       int active_bins,
                                                       int prs_symbols,
                                                       uint32_t seed)
    : gr::block("prs_channel_estimator",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0))
{
    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.active_bins = active_bins;
    d_cfg.prs_symbols = prs_symbols;
    (void)seed; // Retained in the public constructor for compatibility.
    d_pilot_reciprocals = prs_pilots(d_cfg);
    for (auto& pilot : d_pilot_reciprocals) {
        pilot = gr_complex(1.0f, 0.0f) / pilot;
    }
    d_symbol_channels.resize(
        static_cast<size_t>(d_cfg.prs_symbols * d_cfg.active_bins));
    d_channel.resize(static_cast<size_t>(d_cfg.active_bins));
    d_channel_energy.resize(static_cast<size_t>(d_cfg.active_bins));
    message_port_register_in(pmt::mp("symbols_in"));
    message_port_register_out(pmt::mp("channel_out"));
    set_msg_handler(pmt::mp("symbols_in"), [this](pmt::pmt_t msg) { handle_symbols(msg); });
}

void prs_channel_estimator_impl::handle_symbols(pmt::pmt_t msg)
{
    pmt::pmt_t meta;
    const gr_complex* symbols = nullptr;
    size_t symbols_size = 0;
    if (!pdu_get_c32_view(msg, meta, symbols, symbols_size)) {
        return;
    }
    const size_t expected = static_cast<size_t>(d_cfg.prs_symbols * d_cfg.active_bins);
    if (symbols_size < expected || d_pilot_reciprocals.size() < expected) {
        meta = pmt::dict_add(meta, pmt::mp("channel_error"), pmt::mp("short_symbols"));
        message_port_pub(pmt::mp("channel_out"), pmt::cons(meta, pmt::init_c32vector(0, std::vector<gr_complex>())));
        return;
    }

    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        for (int k = 0; k < d_cfg.active_bins; ++k) {
            const size_t idx = static_cast<size_t>(sym * d_cfg.active_bins + k);
            d_symbol_channels[idx] = symbols[idx] * d_pilot_reciprocals[idx];
        }
    }

    std::complex<double> symbol_corr(0.0, 0.0);
    double previous_power = 0.0;
    double next_power = 0.0;
    for (int sym = 0; sym + 1 < d_cfg.prs_symbols; ++sym) {
        for (int k = 0; k < d_cfg.active_bins; ++k) {
            const size_t first = static_cast<size_t>(sym * d_cfg.active_bins + k);
            const size_t second = first + static_cast<size_t>(d_cfg.active_bins);
            const auto& h0 = d_symbol_channels[first];
            const auto& h1 = d_symbol_channels[second];
            symbol_corr += std::conj(std::complex<double>(h0.real(), h0.imag())) *
                           std::complex<double>(h1.real(), h1.imag());
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
        unwrap_reference_hz = dict_ref_double(meta, "preamble_cfo_hz", 0.0);
    }
    const double reference_phase = 2.0 * pi * unwrap_reference_hz * symbol_period;
    phase += 2.0 * pi * std::round((reference_phase - phase) / (2.0 * pi));
    const double prs_channel_cfo_hz = phase / (2.0 * pi * symbol_period);
    const double coherence_denom = std::sqrt(previous_power * next_power);
    const double channel_coherence =
        coherence_denom > 0.0
            ? std::min(1.0, std::abs(symbol_corr) / coherence_denom)
            : 0.0;

    std::fill(d_channel.begin(), d_channel.end(), gr_complex(0.0f, 0.0f));
    std::fill(d_channel_energy.begin(), d_channel_energy.end(), 0.0);
    double signal_power = 0.0;
    double error_power = 0.0;
    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        const double rotation_phase =
            -2.0 * pi * prs_channel_cfo_hz * symbol_period * sym;
        const gr_complex rotation(static_cast<float>(std::cos(rotation_phase)),
                                  static_cast<float>(std::sin(rotation_phase)));
        for (int k = 0; k < d_cfg.active_bins; ++k) {
            const size_t idx = static_cast<size_t>(sym * d_cfg.active_bins + k);
            const auto h = d_symbol_channels[idx] * rotation;
            d_symbol_channels[idx] = h;
            d_channel[static_cast<size_t>(k)] += h;
            d_channel_energy[static_cast<size_t>(k)] += std::norm(h);
        }
    }
    for (int k = 0; k < d_cfg.active_bins; ++k) {
        auto& h = d_channel[static_cast<size_t>(k)];
        h /= static_cast<float>(d_cfg.prs_symbols);
        signal_power += std::norm(h);
        const double residual =
            d_channel_energy[static_cast<size_t>(k)] -
            static_cast<double>(d_cfg.prs_symbols) * std::norm(h);
        error_power += std::max(0.0, residual);
    }
    signal_power /= static_cast<double>(d_cfg.active_bins);
    error_power /= static_cast<double>(expected);
    const double snr = 10.0 * std::log10((signal_power + 1e-12) / (error_power + 1e-12));

    meta = pmt::dict_add(meta, pmt::mp("snr"), pmt::from_double(snr));
    meta = pmt::dict_add(meta,
                         pmt::mp("prs_channel_cfo_hz"),
                         pmt::from_double(prs_channel_cfo_hz));
    meta = pmt::dict_add(meta,
                         pmt::mp("residual_cfo_hz"),
                         pmt::from_double(prs_channel_cfo_hz - unwrap_reference_hz));
    meta = pmt::dict_add(meta,
                         pmt::mp("channel_coherence"),
                         pmt::from_double(channel_coherence));
    meta = pmt::dict_add(meta, pmt::mp("channel_bins"), pmt::from_long(d_cfg.active_bins));
    message_port_pub(pmt::mp("channel_out"),
                     pmt::cons(meta, pmt::init_c32vector(d_channel.size(), d_channel)));
}

} // namespace ofdm_prs_ranging
} // namespace gr
