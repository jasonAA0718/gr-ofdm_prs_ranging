/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_frame_builder.h"
#include "golay_prs_table.h"
#include "prs_payload_codec.h"
#include "prs_receiver_utils.h"
#include <gnuradio/fft/fft.h>
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
gr_complex deterministic_qpsk(std::mt19937& gen)
{
    const uint32_t bits = gen();
    const float scale = static_cast<float>(1.0 / std::sqrt(2.0));
    const float re = (bits & 0x1U) ? scale : -scale;
    const float im = (bits & 0x2U) ? scale : -scale;
    return gr_complex(re, im);
}

void append_short_preamble(std::vector<gr_complex>& frame, const prs_frame_config& cfg)
{
    std::mt19937 gen(cfg.seed ^ 0x5a17U);
    std::vector<gr_complex> one_period;
    one_period.reserve(cfg.preamble_len);
    for (int i = 0; i < cfg.preamble_len; ++i) {
        one_period.push_back(deterministic_qpsk(gen));
    }
    for (int r = 0; r < cfg.preamble_repeats; ++r) {
        frame.insert(frame.end(), one_period.begin(), one_period.end());
    }
}

void append_coarse_sync(std::vector<gr_complex>& frame, const prs_frame_config& cfg)
{
    const auto seq = coarse_sync_sequence(cfg.coarse_sync_len, cfg.coarse_zc_root);
    frame.insert(frame.end(), seq.begin(), seq.end());
}

void append_prs_symbols(std::vector<gr_complex>& frame, const prs_frame_config& cfg)
{
    if (cfg.fft_len != static_cast<int>(golay_prs_fft_len) ||
        cfg.active_bins != static_cast<int>(golay_prs_fft_len) ||
        cfg.prs_symbols != static_cast<int>(golay_prs_symbol_count)) {
        throw std::invalid_argument(
            "Golay PRS requires fft_len=1024, active_bins=1024, prs_symbols=16");
    }

    gr::fft::fft_complex_rev ifft(cfg.fft_len, 1);
    const float scale = 1.0f / static_cast<float>(cfg.fft_len);
    for (int sym = 0; sym < cfg.prs_symbols; ++sym) {
        auto* freq = ifft.get_inbuf();
        for (int fft_bin = 0; fft_bin < cfg.fft_len; ++fft_bin) {
            const auto& pilot =
                golay_prs_at(static_cast<size_t>(sym), static_cast<size_t>(fft_bin));
            freq[fft_bin] = gr_complex(pilot.real, pilot.imag);
        }

        ifft.execute();
        const auto* time = ifft.get_outbuf();
        for (int i = cfg.fft_len - cfg.cp_len; i < cfg.fft_len; ++i) {
            frame.push_back(time[i] * scale);
        }
        for (int i = 0; i < cfg.fft_len; ++i) {
            frame.push_back(time[i] * scale);
        }
    }
}
} // namespace

void prs_frame_builder::normalize_sections(std::vector<gr_complex>& samples,
                                           const prs_frame_config& cfg,
                                           int payload_start,
                                           int payload_len,
                                           int prs_start,
                                           int prs_len)
{
    constexpr float acquisition_boost = 1.4125375446f; // 3 dB in amplitude.
    constexpr float peak_limit = 0.9f;

    const auto scale_range = [&samples](size_t start, size_t length, float target_rms) {
        if (length == 0 || start + length > samples.size()) {
            return;
        }
        double power = 0.0;
        for (size_t i = start; i < start + length; ++i) {
            power += std::norm(samples[i]);
        }
        const double rms = std::sqrt(power / static_cast<double>(length));
        if (rms <= 0.0) {
            return;
        }
        const float scale = target_rms / static_cast<float>(rms);
        for (size_t i = start; i < start + length; ++i) {
            samples[i] *= scale;
        }
    };

    const size_t preamble_start = static_cast<size_t>(cfg.zero_guard_len);
    const size_t preamble_length =
        static_cast<size_t>(cfg.preamble_len * cfg.preamble_repeats);
    const size_t coarse_start = preamble_start + preamble_length;
    scale_range(
        preamble_start, preamble_length, cfg.tx_amp * acquisition_boost);
    scale_range(coarse_start,
                static_cast<size_t>(cfg.coarse_sync_len),
                cfg.tx_amp * acquisition_boost);
    scale_range(static_cast<size_t>(payload_start),
                static_cast<size_t>(payload_len),
                cfg.tx_amp);

    const size_t ofdm_symbol_len = static_cast<size_t>(cfg.fft_len + cfg.cp_len);
    if (prs_len == cfg.prs_symbols * static_cast<int>(ofdm_symbol_len)) {
        for (int sym = 0; sym < cfg.prs_symbols; ++sym) {
            scale_range(static_cast<size_t>(prs_start) +
                            static_cast<size_t>(sym) * ofdm_symbol_len,
                        ofdm_symbol_len,
                        cfg.tx_amp);
        }
    }

    float peak = 0.0f;
    for (const auto& sample : samples) {
        peak = std::max(peak, std::abs(sample));
    }
    if (peak > peak_limit) {
        const float scale = peak_limit / peak;
        for (auto& sample : samples) {
            sample *= scale;
        }
    }
}

prs_frame prs_frame_builder::build(const prs_frame_config& cfg)
{
    prs_frame result;
    auto& frame = result.samples;
    frame.reserve(cfg.zero_guard_len + cfg.preamble_len * cfg.preamble_repeats +
                  cfg.coarse_sync_len + cfg.payload_len +
                  cfg.prs_symbols * (cfg.fft_len + cfg.cp_len) + cfg.tail_guard_len);

    frame.insert(frame.end(), cfg.zero_guard_len, gr_complex(0.0f, 0.0f));
    append_short_preamble(frame, cfg);
    append_coarse_sync(frame, cfg);
    result.payload_start = static_cast<int>(frame.size());
    result.payload_len = cfg.payload_len;
    frame.insert(frame.end(), cfg.payload_len, gr_complex(0.0f, 0.0f));
    if (cfg.payload_len >= prs_frame_id_payload_symbols) {
        encode_frame_id_payload(0, 1.0f, frame.begin() + result.payload_start);
    }
    result.prs_start = static_cast<int>(frame.size());
    append_prs_symbols(frame, cfg);
    result.prs_len = static_cast<int>(frame.size()) - result.prs_start;
    frame.insert(frame.end(), cfg.tail_guard_len, gr_complex(0.0f, 0.0f));

    normalize_sections(frame,
                       cfg,
                       result.payload_start,
                       result.payload_len,
                       result.prs_start,
                       result.prs_len);

    return result;
}

} // namespace ofdm_prs_ranging
} // namespace gr
