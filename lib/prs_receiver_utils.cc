/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

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
    return gr_complex((bits & 0x1U) ? scale : -scale, (bits & 0x2U) ? scale : -scale);
}
} // namespace

int prs_start_offset(const prs_rx_config& cfg)
{
    return cfg.zero_guard_len + cfg.preamble_len * cfg.preamble_repeats +
           cfg.coarse_sync_len + cfg.payload_len;
}

int prs_len(const prs_rx_config& cfg) { return cfg.prs_symbols * (cfg.fft_len + cfg.cp_len); }

int frame_len(const prs_rx_config& cfg)
{
    return prs_start_offset(cfg) + prs_len(cfg) + cfg.tail_guard_len;
}

std::vector<gr_complex> coarse_sync_sequence(int len, int root)
{
    std::vector<gr_complex> seq;
    seq.reserve(len);
    int effective_root = root % len;
    if (effective_root <= 0) {
        effective_root += len;
    }
    for (int n = 0; n < len; ++n) {
        const double phase =
            -pi * effective_root * n * (n + 1) / static_cast<double>(len);
        seq.emplace_back(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return seq;
}

std::vector<gr_complex> qpsk_pilots(const prs_rx_config& cfg)
{
    std::mt19937 gen(cfg.seed);
    std::vector<gr_complex> pilots;
    pilots.reserve(static_cast<size_t>(cfg.prs_symbols * cfg.active_bins));
    const int half_active = cfg.active_bins / 2;
    for (int sym = 0; sym < cfg.prs_symbols; ++sym) {
        for (int b = -half_active; b < 0; ++b) {
            (void)b;
            pilots.push_back(deterministic_qpsk(gen));
        }
        for (int b = 1; b <= half_active; ++b) {
            (void)b;
            pilots.push_back(deterministic_qpsk(gen));
        }
    }
    return pilots;
}

std::vector<float> active_frequencies(const prs_rx_config& cfg)
{
    std::vector<float> freq;
    freq.reserve(cfg.active_bins);
    const int half_active = cfg.active_bins / 2;
    const double spacing = cfg.samp_rate / static_cast<double>(cfg.fft_len);
    for (int b = -half_active; b < 0; ++b) {
        freq.push_back(static_cast<float>(b * spacing));
    }
    for (int b = 1; b <= half_active; ++b) {
        freq.push_back(static_cast<float>(b * spacing));
    }
    return freq;
}

std::vector<float> unwrap_phase(const std::vector<gr_complex>& samples)
{
    return unwrap_phase(samples.data(), samples.size());
}

std::vector<float> unwrap_phase(const gr_complex* samples, size_t size)
{
    std::vector<float> phase;
    phase.reserve(size);
    float offset = 0.0f;
    float prev = 0.0f;
    bool have_prev = false;
    for (size_t i = 0; i < size; ++i) {
        const auto& sample = samples[i];
        float p = std::atan2(sample.imag(), sample.real());
        if (have_prev) {
            const float delta = p + offset - prev;
            if (delta > static_cast<float>(pi)) {
                offset -= static_cast<float>(2.0 * pi);
            } else if (delta < static_cast<float>(-pi)) {
                offset += static_cast<float>(2.0 * pi);
            }
        }
        p += offset;
        phase.push_back(p);
        prev = p;
        have_prev = true;
    }
    return phase;
}

bool pdu_get_c32(const pmt::pmt_t& msg, pmt::pmt_t& meta, std::vector<gr_complex>& data)
{
    if (!pmt::is_pair(msg)) {
        return false;
    }
    meta = pmt::car(msg);
    const auto vec = pmt::cdr(msg);
    if (!pmt::is_c32vector(vec)) {
        return false;
    }
    data = pmt::c32vector_elements(vec);
    return true;
}

bool pdu_get_c32_view(const pmt::pmt_t& msg,
                      pmt::pmt_t& meta,
                      const gr_complex*& data,
                      size_t& size)
{
    if (!pmt::is_pair(msg)) {
        return false;
    }
    meta = pmt::car(msg);
    const auto vec = pmt::cdr(msg);
    if (!pmt::is_c32vector(vec)) {
        return false;
    }
    data = pmt::c32vector_elements(vec, size);
    return true;
}

pmt::pmt_t dict_add_double(pmt::pmt_t dict, const std::string& key, double value)
{
    return pmt::dict_add(dict, pmt::mp(key), pmt::from_double(value));
}

pmt::pmt_t dict_add_int(pmt::pmt_t dict, const std::string& key, int64_t value)
{
    return pmt::dict_add(dict, pmt::mp(key), pmt::from_long(value));
}

double dict_ref_double(const pmt::pmt_t& dict, const std::string& key, double fallback)
{
    const auto value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_real(value)) {
        return pmt::to_double(value);
    }
    if (pmt::is_integer(value)) {
        return static_cast<double>(pmt::to_long(value));
    }
    if (pmt::is_uint64(value)) {
        return static_cast<double>(pmt::to_uint64(value));
    }
    return fallback;
}

uint64_t dict_ref_uint64(const pmt::pmt_t& dict, const std::string& key, uint64_t fallback)
{
    const auto value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_uint64(value)) {
        return pmt::to_uint64(value);
    }
    if (pmt::is_integer(value)) {
        return static_cast<uint64_t>(pmt::to_long(value));
    }
    return fallback;
}

} // namespace ofdm_prs_ranging
} // namespace gr
