/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_channel_estimator_impl.h"
#include <gnuradio/io_signature.h>
#include <cmath>

namespace gr {
namespace ofdm_prs_ranging {

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
    d_cfg.seed = seed;
    d_pilots = qpsk_pilots(d_cfg);
    message_port_register_in(pmt::mp("symbols_in"));
    message_port_register_out(pmt::mp("channel_out"));
    set_msg_handler(pmt::mp("symbols_in"), [this](pmt::pmt_t msg) { handle_symbols(msg); });
}

void prs_channel_estimator_impl::handle_symbols(pmt::pmt_t msg)
{
    pmt::pmt_t meta;
    std::vector<gr_complex> symbols;
    if (!pdu_get_c32(msg, meta, symbols)) {
        return;
    }
    const size_t expected = static_cast<size_t>(d_cfg.prs_symbols * d_cfg.active_bins);
    if (symbols.size() < expected || d_pilots.size() < expected) {
        meta = pmt::dict_add(meta, pmt::mp("channel_error"), pmt::mp("short_symbols"));
        message_port_pub(pmt::mp("channel_out"), pmt::cons(meta, pmt::init_c32vector(0, std::vector<gr_complex>())));
        return;
    }

    std::vector<gr_complex> channel(d_cfg.active_bins, gr_complex(0.0f, 0.0f));
    double signal_power = 0.0;
    double error_power = 0.0;
    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        for (int k = 0; k < d_cfg.active_bins; ++k) {
            const size_t idx = static_cast<size_t>(sym * d_cfg.active_bins + k);
            const auto h = symbols[idx] / d_pilots[idx];
            channel[k] += h;
        }
    }
    for (auto& h : channel) {
        h /= static_cast<float>(d_cfg.prs_symbols);
        signal_power += std::norm(h);
    }
    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        for (int k = 0; k < d_cfg.active_bins; ++k) {
            const size_t idx = static_cast<size_t>(sym * d_cfg.active_bins + k);
            const auto h = symbols[idx] / d_pilots[idx];
            error_power += std::norm(h - channel[k]);
        }
    }
    signal_power /= static_cast<double>(d_cfg.active_bins);
    error_power /= static_cast<double>(expected);
    const double snr = 10.0 * std::log10((signal_power + 1e-12) / (error_power + 1e-12));

    meta = pmt::dict_add(meta, pmt::mp("snr"), pmt::from_double(snr));
    meta = pmt::dict_add(meta, pmt::mp("channel_bins"), pmt::from_long(d_cfg.active_bins));
    message_port_pub(pmt::mp("channel_out"),
                     pmt::cons(meta, pmt::init_c32vector(channel.size(), channel)));
}

} // namespace ofdm_prs_ranging
} // namespace gr
