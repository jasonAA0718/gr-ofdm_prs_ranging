/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_payload_codec.h"
#include <gnuradio/ofdm_prs_ranging/prs_timed_burst_source.h>
#include <boost/test/unit_test.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

BOOST_AUTO_TEST_CASE(test_prs_timed_burst_source_frame_geometry)
{
    auto src = prs_timed_burst_source::make();
    const int expected_prs_start =
        1000 + 128 * 16 + 839 + prs_frame_id_payload_symbols;
    BOOST_CHECK_EQUAL(src->frame_len(),
                      expected_prs_start + 16 * (1024 + 128) + 1000);
    BOOST_CHECK_EQUAL(src->prs_start(), expected_prs_start);
    BOOST_CHECK_EQUAL(src->prs_len(), 16 * (1024 + 128));
}

BOOST_AUTO_TEST_CASE(test_prs_timed_burst_source_amplitude_limit)
{
    auto src = prs_timed_burst_source::make();
    const auto frame = src->frame_samples();
    const auto peak = std::max_element(frame.begin(), frame.end(), [](const auto& a, const auto& b) {
        return std::abs(a) < std::abs(b);
    });
    BOOST_REQUIRE(peak != frame.end());
    BOOST_CHECK_LE(std::abs(*peak), 0.900001f);
}

BOOST_AUTO_TEST_CASE(test_prs_frame_id_payload_crc)
{
    std::vector<gr_complex> payload(prs_frame_id_payload_symbols);
    encode_frame_id_payload(0x1234abcdU, 0.25f, payload.begin());

    uint64_t frame_id = 0;
    float metric = 0.0f;
    BOOST_CHECK(decode_frame_id_payload(
        payload.data(), static_cast<int>(payload.size()), frame_id, metric));
    BOOST_CHECK_EQUAL(frame_id, 0x1234abcdU);
    BOOST_CHECK_GT(metric, 0.99f);

    payload[prs_frame_id_ref_symbols + prs_payload_repeat * 3] *= -1.0f;
    BOOST_CHECK(decode_frame_id_payload(
        payload.data(), static_cast<int>(payload.size()), frame_id, metric));
    const float one_disputed_margin =
        static_cast<float>(prs_payload_repeat - 2) /
        static_cast<float>(prs_payload_repeat);
    const float expected_metric =
        1.0f - (1.0f - one_disputed_margin) /
                   static_cast<float>(prs_payload_data_bits);
    BOOST_CHECK_CLOSE(metric, expected_metric, 0.001f);

    for (int r = 0; r < prs_payload_repeat; ++r) {
        payload[prs_frame_id_ref_symbols + prs_payload_repeat * 7 + r] *= -1.0f;
    }
    BOOST_CHECK(!decode_frame_id_payload(
        payload.data(), static_cast<int>(payload.size()), frame_id, metric));
    BOOST_CHECK_CLOSE(metric, expected_metric, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_prs_payload_coherent_combining_uses_sample_magnitude)
{
    std::vector<gr_complex> payload(prs_frame_id_payload_symbols);
    encode_frame_id_payload(0x1234abcdU, 0.25f, payload.begin());

    const int bit_start = prs_frame_id_ref_symbols;
    const int negative_samples = prs_payload_repeat / 2 + 1;
    for (int r = 0; r < negative_samples; ++r) {
        payload[bit_start + r] = gr_complex(-0.001f, 0.0f);
    }

    uint64_t frame_id = 0;
    float metric = 0.0f;
    BOOST_CHECK(decode_frame_id_payload(
        payload.data(), static_cast<int>(payload.size()), frame_id, metric));
    BOOST_CHECK_EQUAL(frame_id, 0x1234abcdU);
    BOOST_CHECK_GT(metric, 0.95f);
}

BOOST_AUTO_TEST_CASE(test_prs_payload_cfo_derotation)
{
    constexpr double samp_rate = 30e6;
    constexpr double cfo_hz = 10000.0;
    constexpr double pi = 3.141592653589793238462643383279502884;
    const double phase_increment = 2.0 * pi * cfo_hz / samp_rate;

    std::vector<gr_complex> payload(prs_frame_id_payload_symbols);
    prs_payload_info tx_info;
    tx_info.packet_type = prs_packet_type_response;
    tx_info.poll_frame_id = 1234;
    tx_info.response_frame_id = 5678;
    tx_info.reply_delay_samples = 1500000;
    encode_packet_payload(tx_info, 0.25f, payload.begin());
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] *= gr_complex(
            static_cast<float>(std::cos(phase_increment * static_cast<double>(i))),
            static_cast<float>(std::sin(phase_increment * static_cast<double>(i))));
    }

    prs_payload_info rx_info;
    float metric = 0.0f;
    BOOST_CHECK(decode_packet_payload(payload.data(),
                                      static_cast<int>(payload.size()),
                                      rx_info,
                                      metric,
                                      phase_increment));
    BOOST_CHECK_EQUAL(rx_info.packet_type, tx_info.packet_type);
    BOOST_CHECK_EQUAL(rx_info.poll_frame_id, tx_info.poll_frame_id);
    BOOST_CHECK_EQUAL(rx_info.response_frame_id, tx_info.response_frame_id);
    BOOST_CHECK_EQUAL(rx_info.reply_delay_samples, tx_info.reply_delay_samples);
    BOOST_CHECK_GT(metric, 0.99f);
}

} /* namespace ofdm_prs_ranging */
} /* namespace gr */
