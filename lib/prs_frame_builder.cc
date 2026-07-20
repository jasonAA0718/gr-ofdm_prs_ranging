/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_frame_builder.h"
#include "prs_payload_codec.h"
#include "prs_receiver_utils.h"
#include <algorithm>
#include <cmath>
#include <random>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr double pi = 3.141592653589793238462643383279502884;

gr_complex deterministic_qpsk(std::mt19937& gen)
{
    const uint32_t bits = gen();
    const float scale = static_cast<float>(1.0 / std::sqrt(2.0));
    const float re = (bits & 0x1U) ? scale : -scale;
    const float im = (bits & 0x2U) ? scale : -scale;
    return gr_complex(re, im);
}

std::vector<gr_complex> ifft(const std::vector<gr_complex>& freq)
{
    std::vector<gr_complex> time(freq.size(), gr_complex(0.0f, 0.0f));
    const double n = static_cast<double>(freq.size());
    for (size_t t = 0; t < freq.size(); ++t) {
        gr_complex acc(0.0f, 0.0f);
        for (size_t k = 0; k < freq.size(); ++k) {
            const double phase = 2.0 * pi * static_cast<double>(k * t) / n;
            acc += freq[k] * gr_complex(std::cos(phase), std::sin(phase));
        }
        time[t] = acc / static_cast<float>(n);
    }
    return time;
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
    std::mt19937 gen(cfg.seed);
    const int half_active = cfg.active_bins / 2;
    for (int sym = 0; sym < cfg.prs_symbols; ++sym) {
        std::vector<gr_complex> freq(cfg.fft_len, gr_complex(0.0f, 0.0f));

        for (int b = -half_active; b < 0; ++b) {
            const int index = (b + cfg.fft_len) % cfg.fft_len;
            freq[index] = deterministic_qpsk(gen);
        }
        for (int b = 1; b <= half_active; ++b) {
            freq[b] = deterministic_qpsk(gen);
        }

        const auto time = ifft(freq);
        frame.insert(frame.end(), time.end() - cfg.cp_len, time.end());
        frame.insert(frame.end(), time.begin(), time.end());
    }
}
} // namespace

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

    double power = 0.0;
    float peak = 0.0f;
    for (const auto& sample : frame) {
        const float mag = std::abs(sample);
        power += static_cast<double>(mag) * static_cast<double>(mag);
        peak = std::max(peak, mag);
    }
    const double rms = std::sqrt(power / static_cast<double>(frame.size()));
    if (rms > 0.0) {
        const float scale = cfg.tx_amp / static_cast<float>(rms);
        for (auto& sample : frame) {
            sample *= scale;
        }
    }

    peak = 0.0f;
    for (const auto& sample : frame) {
        peak = std::max(peak, std::abs(sample));
    }
    if (peak > 0.8f) {
        const float scale = 0.8f / peak;
        for (auto& sample : frame) {
            sample *= scale;
        }
    }

    return result;
}

} // namespace ofdm_prs_ranging
} // namespace gr
