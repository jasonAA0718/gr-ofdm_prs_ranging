/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "golay_prs_table.h"
#include "prs_payload_codec.h"
#include <gnuradio/fft/fft.h>
#include <gnuradio/ofdm_prs_ranging/prs_timed_burst_source.h>
#include <boost/test/unit_test.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr int fft_len = 1024;
constexpr int cp_len = 128;
constexpr int prs_symbols = 16;

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

std::vector<gr_complex> golay_symbol(int symbol)
{
    std::vector<gr_complex> freq(static_cast<size_t>(fft_len));
    for (int fft_bin = 0; fft_bin < fft_len; ++fft_bin) {
        const auto& pilot =
            golay_prs_at(static_cast<size_t>(symbol), static_cast<size_t>(fft_bin));
        freq[static_cast<size_t>(fft_bin)] = gr_complex(pilot.real, pilot.imag);
    }
    return freq;
}

gr_complex old_random_qpsk(std::mt19937& gen)
{
    const uint32_t bits = gen();
    const float scale = static_cast<float>(1.0 / std::sqrt(2.0));
    return gr_complex((bits & 0x1U) ? scale : -scale,
                      (bits & 0x2U) ? scale : -scale);
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

BOOST_AUTO_TEST_CASE(test_golay_tx_fft_mapping)
{
    auto source = prs_timed_burst_source::make();
    const auto frame = source->frame_samples();
    const size_t useful_start =
        static_cast<size_t>(source->prs_start() + cp_len);

    gr::fft::fft_complex_fwd fft(fft_len, 1);
    std::copy(frame.begin() + useful_start,
              frame.begin() + useful_start + fft_len,
              fft.get_inbuf());
    fft.execute();

    gr_complex gain(0.0f, 0.0f);
    for (int fft_bin = 0; fft_bin < fft_len; ++fft_bin) {
        const auto& pilot = golay_prs_at(0, static_cast<size_t>(fft_bin));
        const gr_complex expected(pilot.real, pilot.imag);
        gain += fft.get_outbuf()[fft_bin] / expected;
    }
    gain /= static_cast<float>(fft_len);

    for (int fft_bin = 0; fft_bin < fft_len; ++fft_bin) {
        const auto& pilot = golay_prs_at(0, static_cast<size_t>(fft_bin));
        const gr_complex expected(pilot.real, pilot.imag);
        BOOST_CHECK_SMALL(std::abs(fft.get_outbuf()[fft_bin] - gain * expected),
                          2.0e-4f);
    }
}

BOOST_AUTO_TEST_CASE(test_golay_papr_and_section_levels)
{
    std::vector<double> golay_papr;
    golay_papr.reserve(prs_symbols);
    for (int symbol = 0; symbol < prs_symbols; ++symbol) {
        golay_papr.push_back(papr_db(ifft_symbol(golay_symbol(symbol))));
    }

    std::mt19937 gen(13990001U);
    std::vector<double> random_papr;
    random_papr.reserve(prs_symbols);
    for (int symbol = 0; symbol < prs_symbols; ++symbol) {
        std::vector<gr_complex> freq(static_cast<size_t>(fft_len),
                                     gr_complex(0.0f, 0.0f));
        for (int bin = -300; bin < 0; ++bin) {
            freq[static_cast<size_t>(bin + fft_len)] = old_random_qpsk(gen);
        }
        for (int bin = 1; bin <= 300; ++bin) {
            freq[static_cast<size_t>(bin)] = old_random_qpsk(gen);
        }
        random_papr.push_back(papr_db(ifft_symbol(freq)));
    }

    const auto golay = summarize(golay_papr);
    const auto random = summarize(random_papr);
    BOOST_TEST_MESSAGE("Golay PAPR dB min/mean/max: "
                       << golay[0] << " / " << golay[1] << " / " << golay[2]);
    BOOST_TEST_MESSAGE("Old Random-QPSK PAPR dB min/mean/max: "
                       << random[0] << " / " << random[1] << " / " << random[2]);
    BOOST_CHECK_LE(golay[2], 3.02);
    BOOST_CHECK_LT(golay[2], random[0]);

    auto source = prs_timed_burst_source::make();
    const auto frame = source->frame_samples();
    const size_t preamble_start = 1000;
    const size_t preamble_length = 128 * 16;
    const size_t coarse_start = preamble_start + preamble_length;
    const size_t payload_start = static_cast<size_t>(
        source->prs_start() - prs_frame_id_payload_symbols);
    const size_t prs_start = static_cast<size_t>(source->prs_start());
    const double preamble_rms = rms(frame, preamble_start, preamble_length);
    const double coarse_rms = rms(frame, coarse_start, 839);
    const double payload_rms =
        rms(frame, payload_start, prs_frame_id_payload_symbols);
    const double ofdm_rms = rms(frame, prs_start, fft_len + cp_len);
    const auto peak = std::max_element(
        frame.begin(), frame.end(), [](const auto& a, const auto& b) {
            return std::abs(a) < std::abs(b);
        });

    BOOST_REQUIRE(peak != frame.end());
    BOOST_CHECK_LE(std::abs(*peak), 0.900001f);
    BOOST_CHECK_SMALL(20.0 * std::log10(preamble_rms / coarse_rms), 0.01);
    BOOST_CHECK_LE(20.0 * std::log10(preamble_rms / payload_rms), 3.001);
    BOOST_CHECK_SMALL(20.0 * std::log10(payload_rms / ofdm_rms), 0.01);
}

} // namespace ofdm_prs_ranging
} // namespace gr
