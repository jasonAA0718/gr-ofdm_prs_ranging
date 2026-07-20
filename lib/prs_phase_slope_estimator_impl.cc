/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_phase_slope_estimator_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
}

prs_phase_slope_estimator::sptr prs_phase_slope_estimator::make(double samp_rate,
                                                                int fft_len,
                                                                int active_bins,
                                                                float max_residual_rms)
{
    return gnuradio::make_block_sptr<prs_phase_slope_estimator_impl>(
        samp_rate, fft_len, active_bins, max_residual_rms);
}

prs_phase_slope_estimator_impl::prs_phase_slope_estimator_impl(double samp_rate,
                                                               int fft_len,
                                                               int active_bins,
                                                               float max_residual_rms)
    : gr::block("prs_phase_slope_estimator",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_max_residual_rms(max_residual_rms)
{
    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.active_bins = active_bins;
    d_freq = active_frequencies(d_cfg);
    message_port_register_in(pmt::mp("channel_in"));
    message_port_register_out(pmt::mp("measurement_out"));
    set_msg_handler(pmt::mp("channel_in"), [this](pmt::pmt_t msg) { handle_channel(msg); });
}

void prs_phase_slope_estimator_impl::handle_channel(pmt::pmt_t msg)
{
    pmt::pmt_t meta;
    std::vector<gr_complex> channel;
    if (!pdu_get_c32(msg, meta, channel)) {
        return;
    }
    if (channel.size() < static_cast<size_t>(d_cfg.active_bins)) {
        meta = pmt::dict_add(meta, pmt::mp("valid"), pmt::PMT_F);
        meta = pmt::dict_add(meta, pmt::mp("error_reason"), pmt::mp("short_channel"));
        message_port_pub(pmt::mp("measurement_out"), pmt::cons(meta, pmt::PMT_NIL));
        return;
    }

    const auto phase = unwrap_phase(channel);
    double wsum = 0.0;
    double fsum = 0.0;
    double psum = 0.0;
    for (int k = 0; k < d_cfg.active_bins; ++k) {
        const double w = std::max(1e-12, static_cast<double>(std::norm(channel[k])));
        wsum += w;
        fsum += w * d_freq[k];
        psum += w * phase[k];
    }
    const double fbar = fsum / wsum;
    const double pbar = psum / wsum;

    double num = 0.0;
    double den = 0.0;
    for (int k = 0; k < d_cfg.active_bins; ++k) {
        const double w = std::max(1e-12, static_cast<double>(std::norm(channel[k])));
        const double df = d_freq[k] - fbar;
        const double dp = phase[k] - pbar;
        num += w * df * dp;
        den += w * df * df;
    }

    const double slope = den > 0.0 ? num / den : 0.0;
    const double tau = -slope / (2.0 * pi);
    double residual_power = 0.0;
    for (int k = 0; k < d_cfg.active_bins; ++k) {
        const double predicted = pbar + slope * (d_freq[k] - fbar);
        const double err = phase[k] - predicted;
        residual_power += err * err;
    }
    const double residual_rms = std::sqrt(residual_power / d_cfg.active_bins);
    const double snr = dict_ref_double(meta, "snr", 0.0);
    const double peak_metric = dict_ref_double(meta, "peak_metric", 0.0);
    const double quality = std::max(0.0, peak_metric) *
                           std::max(0.0, 1.0 - residual_rms / std::max(1e-6f, d_max_residual_rms)) *
                           std::max(0.0, std::min(1.0, (snr + 10.0) / 40.0));

    const uint64_t frame_start = dict_ref_uint64(meta, "frame_start", 0);
    const uint64_t coarse_peak = dict_ref_uint64(meta, "coarse_peak", frame_start);
    const double coarse_delay = static_cast<double>(coarse_peak - frame_start) / d_cfg.samp_rate;

    meta = pmt::dict_add(meta, pmt::mp("coarse_delay"), pmt::from_double(coarse_delay));
    meta = pmt::dict_add(meta, pmt::mp("fine_delay"), pmt::from_double(tau));
    meta = pmt::dict_add(meta, pmt::mp("fine_delay_samples"), pmt::from_double(tau * d_cfg.samp_rate));
    meta = pmt::dict_add(meta, pmt::mp("cfo"), pmt::from_double(dict_ref_double(meta, "cfo", 0.0)));
    meta = pmt::dict_add(meta, pmt::mp("phase_slope"), pmt::from_double(slope));
    meta = pmt::dict_add(meta, pmt::mp("phase_residual"), pmt::from_double(residual_rms));
    meta = pmt::dict_add(meta, pmt::mp("quality"), pmt::from_double(quality));
    meta = pmt::dict_add(meta, pmt::mp("valid"), residual_rms <= d_max_residual_rms ? pmt::PMT_T : pmt::PMT_F);
    message_port_pub(pmt::mp("measurement_out"), pmt::cons(meta, pmt::PMT_NIL));
}

} // namespace ofdm_prs_ranging
} // namespace gr
