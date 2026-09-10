/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "mc_ds_prs.h"
#include "prs_frame_builder.h"
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

void append_prs_symbols(std::vector<gr_complex>& frame,
                        const prs_frame_config& cfg,
                        const prs_payload_info& payload)
{
    if (cfg.fft_len != mc_ds_fft_len || cfg.active_bins != mc_ds_fft_len ||
        cfg.cp_len != mc_ds_cp_len || cfg.prs_symbols != mc_ds_symbol_count) {
        throw std::invalid_argument(
            "MC-DS PRS requires fft_len=512, cp_len=64, active_bins=512, "
            "prs_symbols=127");
    }
    if (cfg.mc_ds_gold_code_id < 0 || cfg.mc_ds_gold_code_id >= gold127_family_size) {
        throw std::invalid_argument("mc_ds_gold_code_id must be in [0, 128]");
    }

    const auto payload_bits = serialize_packet_payload(payload);
    gr::fft::fft_complex_rev ifft(cfg.fft_len, 1);
    const float scale = 1.0f / static_cast<float>(cfg.fft_len);
    for (int sym = 0; sym < cfg.prs_symbols; ++sym) {
        auto* freq = ifft.get_inbuf();
        for (int fft_bin = 0; fft_bin < cfg.fft_len; ++fft_bin) {
            if (mc_ds_is_data_bin(fft_bin)) {
                const int data_index = mc_ds_data_index(fft_bin);
                const uint8_t logical_bit =
                    data_index < prs_payload_data_bits
                        ? payload_bits[static_cast<size_t>(data_index)]
                        : 0U;
                const bool one = (logical_bit ^ mc_ds_scrambler_bit(data_index)) != 0U;
                const float bpsk = one ? 1.0f : -1.0f;
                freq[fft_bin] = mc_ds_gold_chip(cfg.mc_ds_gold_code_id, sym) * bpsk;
            } else {
                freq[fft_bin] = mc_ds_pilot(sym, fft_bin, cfg.mc_ds_gold_code_id);
            }
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
    /*
    Normalize the OFDM sections to the target amplitude, 
    and then limit the peak amplitude to avoid clipping.
    */
    constexpr float peak_limit = 0.9f;
    const float preamble_amp = 0.95;

    // Lambda to scale a range of samples to a target RMS amplitude
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
    // Scale the preamble and coarse sync sections to the target amplitude
    const size_t preamble_start = static_cast<size_t>(cfg.zero_guard_len);
    const size_t preamble_length = static_cast<size_t>(cfg.preamble_len * cfg.preamble_repeats);
    const size_t coarse_start = preamble_start + preamble_length;
    const size_t coarse_sync_len = static_cast<size_t>(cfg.coarse_sync_len);

    for (size_t i = preamble_start; i < coarse_start + coarse_sync_len; ++i) 
        samples[i] *= preamble_amp;

    // Scale the PRS section to the target amplitude
    const size_t ofdm_symbol_len = static_cast<size_t>(cfg.fft_len + cfg.cp_len);
    if (prs_len == cfg.prs_symbols * static_cast<int>(ofdm_symbol_len)) {
        for (int sym = 0; sym < cfg.prs_symbols; ++sym) {
            scale_range(static_cast<size_t>(prs_start) + static_cast<size_t>(sym) * ofdm_symbol_len,
                        ofdm_symbol_len,
                        cfg.tx_amp);
        }
    }
    // Ensure the PRS peak  is limited to the target amplitude
    float peak = 0.0f;
    // Find the peak amplitude in the PRS section
    if (prs_start >= 0 && prs_len > 0) {
        const size_t start  = static_cast<size_t>(prs_start);
        const size_t length = static_cast<size_t>(prs_len);
        if(start < samples.size() && length <= samples.size() - start)
            for (size_t i = start; i < start+length; ++i)
                peak = std::max(peak,std::abs(samples[i])); 
        if (peak > peak_limit) {
            const float scale = peak_limit / peak;
            for (size_t i = start; i < start+length; ++i) {
                samples[i] *= scale;
            }
        }
    }
}

prs_frame prs_frame_builder::build(const prs_frame_config& cfg)
{
    return build(cfg, prs_payload_info{});
}

prs_frame prs_frame_builder::build(const prs_frame_config& cfg,
                                   const prs_payload_info& payload)
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
    result.payload_len = 0;
    result.prs_start = static_cast<int>(frame.size());
    append_prs_symbols(frame, cfg, payload);
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
