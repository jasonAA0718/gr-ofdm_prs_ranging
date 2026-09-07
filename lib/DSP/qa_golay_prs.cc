/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "golay_prs_table.h"
#include "mc_ds_prs.h"
#include "prs_payload_codec.h"
#include <gnuradio/fft/fft.h>
#include <gnuradio/ofdm_prs_ranging/prs_timed_burst_source.h>
#include <boost/test/unit_test.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr int fft_len = 1024;
constexpr int cp_len = 128;
constexpr int prs_symbols = mc_ds_symbol_count;

std::vector<gr_complex> ifft_symbol(const std::vector<gr_complex>& freq)
{
    gr::fft::fft_complex_rev ifft(fft_len, 1);
    std::copy(freq.begin(), freq.end(), ifft.get_inbuf());
    ifft.execute();
    const float scale = 1.0f / static_cast<float>(fft_len);
    std::vector<gr_complex> time(static_cast<size_t>(fft_len));
    for (int i = 0; i < fft_len; ++i) {
        time[static_cast<size_t>(i)] = ifft.get_outbuf()[i] * scale;
    }
    return time;
}

double papr_db(const std::vector<gr_complex>& time)
{
    double power = 0.0;
    double peak_power = 0.0;
    for (const auto& sample : time) {
        const double sample_power = std::norm(sample);
        power += sample_power;
        peak_power = std::max(peak_power, sample_power);
    }
    return 10.0 * std::log10(peak_power / (power / time.size()));
}

std::vector<gr_complex> mc_ds_symbol(int symbol, const prs_payload_bits& bits)
{
    std::vector<gr_complex> freq(static_cast<size_t>(fft_len));
    for (int fft_bin = 0; fft_bin < fft_len; ++fft_bin) {
        if (mc_ds_is_data_bin(fft_bin)) {
            const int q = mc_ds_data_index(fft_bin);
            const float bpsk =
                q < prs_payload_data_bits && bits[static_cast<size_t>(q)] ? 1.0f : -1.0f;
            freq[static_cast<size_t>(fft_bin)] =
                mc_ds_prn_code[static_cast<size_t>(symbol)] * bpsk;
        } else {
            freq[static_cast<size_t>(fft_bin)] = mc_ds_pilot(symbol, fft_bin);
        }
    }
    return freq;
}

std::array<double, 3> summarize(std::vector<double> values)
{
    const auto bounds = std::minmax_element(values.begin(), values.end());
    const double mean =
        std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    return { *bounds.first, mean, *bounds.second };
}

double rms(const std::vector<gr_complex>& samples, size_t start, size_t length)
{
    double power = 0.0;
    for (size_t i = start; i < start + length; ++i) {
        power += std::norm(samples[i]);
    }
    return std::sqrt(power / static_cast<double>(length));
}
} // namespace

BOOST_AUTO_TEST_CASE(test_golay_table_dimensions_and_csv_entries)
{
    static_assert(golay_prs_symbol_count == 16);
    static_assert(golay_prs_fft_len == 1024);
    static_assert(golay_prs_table.size() == 16 * 1024);

    const auto check = [](size_t symbol, size_t fft_bin, float real, float imag) {
        const auto& value = golay_prs_at(symbol, fft_bin);
        BOOST_CHECK_EQUAL(value.real, real);
        BOOST_CHECK_EQUAL(value.imag, imag);
    };
    check(0, 0, 1.0f, 0.0f);
    check(0, 3, -1.0f, 0.0f);
    check(0, 512, 1.0f, 0.0f);
    check(0, 1023, -1.0f, 0.0f);
    check(1, 512, -1.0f, 0.0f);
    check(1, 1023, 1.0f, 0.0f);
    check(6, 777, -1.0f, 0.0f);
    check(7, 777, 1.0f, 0.0f);
    check(14, 1000, 1.0f, 0.0f);
    check(15, 1022, -1.0f, 0.0f);
}

BOOST_AUTO_TEST_CASE(test_mc_ds_tx_fft_mapping)
{
    auto source = prs_timed_burst_source::make();
    const auto frame = source->frame_samples();
    const size_t useful_start = static_cast<size_t>(source->prs_start() + cp_len);

    const auto bits = serialize_packet_payload(prs_payload_info{});
    gr::fft::fft_complex_fwd fft(fft_len, 1);
    for (int symbol = 0; symbol < prs_symbols; ++symbol) {
        const size_t start = useful_start + static_cast<size_t>(symbol) *
                                                static_cast<size_t>(fft_len + cp_len);
        std::copy(
            frame.begin() + start, frame.begin() + start + fft_len, fft.get_inbuf());
        fft.execute();
        const auto expected = mc_ds_symbol(symbol, bits);
        const gr_complex gain = fft.get_outbuf()[0] / expected[0];
        for (int fft_bin = 0; fft_bin < fft_len; ++fft_bin) {
            BOOST_CHECK_SMALL(std::abs(fft.get_outbuf()[fft_bin] -
                                       gain * expected[static_cast<size_t>(fft_bin)]),
                              2.0e-4f);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_mc_ds_papr_and_section_levels)
{
    const auto bits = serialize_packet_payload(prs_payload_info{});
    std::vector<double> papr;
    papr.reserve(prs_symbols);
    for (int symbol = 0; symbol < prs_symbols; ++symbol) {
        papr.push_back(papr_db(ifft_symbol(mc_ds_symbol(symbol, bits))));
    }

    const auto summary = summarize(papr);
    BOOST_TEST_MESSAGE("MC-DS PAPR dB min/mean/max: " << summary[0] << " / " << summary[1]
                                                      << " / " << summary[2]);
    BOOST_CHECK(std::isfinite(summary[0]));
    BOOST_CHECK_LE(summary[2], 15.0);

    auto source = prs_timed_burst_source::make();
    const auto frame = source->frame_samples();
    const size_t preamble_start = 1000;
    const size_t preamble_length = 128 * 16;
    const size_t coarse_start = preamble_start + preamble_length;
    const size_t prs_start = static_cast<size_t>(source->prs_start());
    const double preamble_rms = rms(frame, preamble_start, preamble_length);
    const double coarse_rms = rms(frame, coarse_start, 839);
    const double ofdm_rms = rms(frame, prs_start, fft_len + cp_len);
    const auto peak =
        std::max_element(frame.begin(), frame.end(), [](const auto& a, const auto& b) {
            return std::abs(a) < std::abs(b);
        });

    BOOST_REQUIRE(peak != frame.end());
    BOOST_CHECK_LE(std::abs(*peak), 0.900001f);
    BOOST_CHECK_SMALL(20.0 * std::log10(preamble_rms / coarse_rms), 0.01);
    BOOST_CHECK_CLOSE(20.0 * std::log10(preamble_rms / ofdm_rms), 3.0, 0.1);
}

} // namespace ofdm_prs_ranging
} // namespace gr
