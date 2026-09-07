/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "mc_ds_prs.h"
#include "prs_payload_codec.h"
#include <gnuradio/fft/fft.h>
#include <gnuradio/ofdm_prs_ranging/prs_timed_burst_source.h>
#include <boost/test/unit_test.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
std::array<int8_t, gold127_code_length> load_csv_code(int code_id)
{
    std::string path = __FILE__;
    path.replace(
        path.find_last_of("/\\") + 1, std::string::npos, "gold127_family_bipolar.csv");
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("cannot open " + path);
    }
    std::string line;
    for (int row = 0; row <= code_id; ++row) {
        if (!std::getline(input, line)) {
            throw std::runtime_error("missing Gold code row " + std::to_string(row));
        }
    }
    std::array<int8_t, gold127_code_length> result{};
    std::stringstream stream(line);
    std::string token;
    int column = 0;
    while (std::getline(stream, token, ',')) {
        if (column >= gold127_code_length) {
            throw std::runtime_error("too many chips in Gold code row");
        }
        result[static_cast<size_t>(column++)] = static_cast<int8_t>(std::stoi(token));
    }
    if (column != gold127_code_length) {
        throw std::runtime_error("Gold code row does not contain 127 chips");
    }
    return result;
}

std::vector<gr_complex>
expected_symbol(int symbol, int code_id, const prs_payload_bits& bits)
{
    std::vector<gr_complex> freq(mc_ds_fft_len);
    const float chip = mc_ds_gold_chip(code_id, symbol);
    for (int fft_bin = 0; fft_bin < mc_ds_fft_len; ++fft_bin) {
        if (mc_ds_is_data_bin(fft_bin)) {
            const int q = mc_ds_data_index(fft_bin);
            const uint8_t logical =
                q < prs_payload_data_bits ? bits[static_cast<size_t>(q)] : 0U;
            const bool one = (logical ^ mc_ds_scrambler_bit(q)) != 0U;
            freq[static_cast<size_t>(fft_bin)] = chip * (one ? 1.0f : -1.0f);
        } else {
            freq[static_cast<size_t>(fft_bin)] = mc_ds_pilot(symbol, fft_bin, code_id);
        }
    }
    return freq;
}

double papr_db(gr::fft::fft_complex_rev& ifft, const std::vector<gr_complex>& freq)
{
    std::copy(freq.begin(), freq.end(), ifft.get_inbuf());
    ifft.execute();
    double power = 0.0;
    double peak_power = 0.0;
    for (int i = 0; i < mc_ds_fft_len; ++i) {
        const double sample_power = std::norm(ifft.get_outbuf()[i]);
        power += sample_power;
        peak_power = std::max(peak_power, sample_power);
    }
    return 10.0 * std::log10(peak_power / (power / mc_ds_fft_len));
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

BOOST_AUTO_TEST_CASE(test_gold127_table_matches_authoritative_csv)
{
    static_assert(gold127_codes.size() == 129);
    static_assert(gold127_codes[0].size() == 127);
    for (const auto& code : gold127_codes) {
        for (const auto chip : code) {
            BOOST_CHECK(chip == -1 || chip == 1);
        }
    }
    const auto csv_code = load_csv_code(mc_ds_default_gold_code_id);
    BOOST_CHECK_EQUAL_COLLECTIONS(csv_code.begin(),
                                  csv_code.end(),
                                  gold127_codes[mc_ds_default_gold_code_id].begin(),
                                  gold127_codes[mc_ds_default_gold_code_id].end());
}

BOOST_AUTO_TEST_CASE(test_scrambler_and_frequency_mapping)
{
    static_assert(sizeof(mc_ds_data_scrambler) - 1 == 128);
    int ones = 0;
    int data_bins = 0;
    int pilot_bins = 0;
    for (int q = 0; q < mc_ds_data_bin_count; ++q) {
        ones += mc_ds_scrambler_bit(q);
        const uint8_t original = static_cast<uint8_t>((q * 17 + 3) & 1);
        const uint8_t scrambled = original ^ mc_ds_scrambler_bit(q);
        BOOST_CHECK_EQUAL(scrambled ^ mc_ds_scrambler_bit(q), original);
    }
    for (int k = 0; k < mc_ds_fft_len; ++k) {
        if (mc_ds_is_data_bin(k)) {
            ++data_bins;
            BOOST_CHECK_EQUAL(k % 4, 3);
        } else {
            ++pilot_bins;
        }
    }
    BOOST_CHECK_EQUAL(ones, 64);
    BOOST_CHECK_EQUAL(data_bins, 128);
    BOOST_CHECK_EQUAL(pilot_bins, 384);

    for (int k = 0; k < mc_ds_fft_len; ++k) {
        BOOST_CHECK_EQUAL(std::abs(mc_ds_golay.a[static_cast<size_t>(k)]), 1);
        BOOST_CHECK_EQUAL(std::abs(mc_ds_golay.b[static_cast<size_t>(k)]), 1);
        BOOST_CHECK_EQUAL(mc_ds_golay_pilot(0, k).real(), mc_ds_golay.a[k]);
        BOOST_CHECK_EQUAL(mc_ds_golay_pilot(1, k).real(), mc_ds_golay.b[k]);
    }
}

BOOST_AUTO_TEST_CASE(test_mc_ds_tx_fft_mapping)
{
    auto source = prs_timed_burst_source::make();
    const auto frame = source->frame_samples();
    const auto bits = serialize_packet_payload(prs_payload_info{});
    gr::fft::fft_complex_fwd fft(mc_ds_fft_len, 1);
    for (int symbol = 0; symbol < mc_ds_symbol_count; ++symbol) {
        const size_t start = static_cast<size_t>(source->prs_start() + mc_ds_cp_len) +
                             static_cast<size_t>(symbol) *
                                 static_cast<size_t>(mc_ds_fft_len + mc_ds_cp_len);
        std::copy(frame.begin() + start,
                  frame.begin() + start + mc_ds_fft_len,
                  fft.get_inbuf());
        fft.execute();
        const auto expected = expected_symbol(symbol, mc_ds_default_gold_code_id, bits);
        const gr_complex gain = fft.get_outbuf()[0] / expected[0];
        for (int k = 0; k < mc_ds_fft_len; ++k) {
            BOOST_CHECK_SMALL(
                std::abs(fft.get_outbuf()[k] - gain * expected[static_cast<size_t>(k)]),
                2.0e-4f);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_scrambled_poll_papr_regression)
{
    constexpr int frame_count = 50000;
    gr::fft::fft_complex_rev ifft(mc_ds_fft_len, 1);
    std::vector<double> values;
    values.reserve(frame_count * 2);
    for (int frame_id = 0; frame_id < frame_count; ++frame_id) {
        prs_payload_info info;
        info.packet_type = prs_packet_type_poll;
        info.poll_frame_id = static_cast<uint32_t>(frame_id);
        const auto bits = serialize_packet_payload(info);
        values.push_back(
            papr_db(ifft, expected_symbol(0, mc_ds_default_gold_code_id, bits)));
        values.push_back(
            papr_db(ifft, expected_symbol(1, mc_ds_default_gold_code_id, bits)));
    }
    std::sort(values.begin(), values.end());
    const double mean =
        std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    const double minimum = values.front();
    const double p999 = values[static_cast<size_t>(std::ceil(0.999 * values.size())) - 1];
    const double maximum = values.back();
    BOOST_TEST_MESSAGE("Scrambled POLL PAPR dB min/mean/P99.9/max: "
                       << minimum << " / " << mean << " / " << p999 << " / " << maximum);
    BOOST_CHECK_GT(mean, 6.4);
    BOOST_CHECK_LT(mean, 7.8);
    BOOST_CHECK_LT(p999, 9.6);
    BOOST_CHECK_LT(maximum, 10.0);
}

BOOST_AUTO_TEST_CASE(test_frame_section_levels)
{
    auto source = prs_timed_burst_source::make();
    const auto frame = source->frame_samples();
    const size_t preamble_start = 1000;
    const size_t preamble_length = 128 * 16;
    const size_t coarse_start = preamble_start + preamble_length;
    const double preamble_rms = rms(frame, preamble_start, preamble_length);
    const double coarse_rms = rms(frame, coarse_start, 839);
    const double ofdm_rms = rms(
        frame, static_cast<size_t>(source->prs_start()), mc_ds_fft_len + mc_ds_cp_len);
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
