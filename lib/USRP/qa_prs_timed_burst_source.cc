/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_payload_codec.h"
#include "prs_burst_scheduler.h"
#include "prs_receiver_utils.h"
#include <gnuradio/ofdm_prs_ranging/prs_timed_burst_source.h>
#include <boost/test/unit_test.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

BOOST_AUTO_TEST_CASE(test_prs_timed_burst_source_frame_geometry)
{
    auto src = prs_timed_burst_source::make();
    const int expected_prs_start = phy::leading_guard_samples + 128 * 16 + 839;
    BOOST_CHECK_EQUAL(src->frame_len(), expected_prs_start + 127 * (512 + 64) + phy::tail_guard_samples);
    BOOST_CHECK_EQUAL(src->prs_start(), expected_prs_start);
    BOOST_CHECK_EQUAL(src->prs_len(), 127 * (512 + 64));
}

BOOST_AUTO_TEST_CASE(test_prs_payload_bit_serialization_round_trip)
{
    prs_payload_info tx;
    tx.packet_type = prs_packet_type_response;
    tx.poll_id = 0xffffU;
    tx.anchor_id = 7;
    tx.poll_fine_delay_q10 = -32768;
    tx.fine_delay_valid = true;
    tx.reply_delay_samples = 1500000U;
    const auto bits = serialize_packet_payload(tx);
    BOOST_CHECK_EQUAL(bits.size(), 80U);

    prs_payload_info rx;
    BOOST_CHECK(deserialize_packet_payload(bits.data(), bits.size(), rx));
    BOOST_CHECK_EQUAL(rx.packet_type, tx.packet_type);
    BOOST_CHECK_EQUAL(rx.poll_id, tx.poll_id);
    BOOST_CHECK_EQUAL(rx.anchor_id, tx.anchor_id);
    BOOST_CHECK_EQUAL(rx.reply_delay_samples, tx.reply_delay_samples);
    const uint64_t expected_control = 2ULL | (7ULL << 2) | (0xffffULL << 5) |
        (1500000ULL << 21) | (0x8000ULL << 45) | (1ULL << 61);
    for (int bit = 0; bit < 64; ++bit) {
        BOOST_CHECK_EQUAL(bits[bit], (expected_control >> bit) & 1);
    }

    auto corrupt = bits;
    corrupt[19] ^= 1U;
    BOOST_CHECK(!deserialize_packet_payload(corrupt.data(), corrupt.size(), rx));
}

BOOST_AUTO_TEST_CASE(test_packet_v2_bounds_crc_reserved_and_quantization)
{
    prs_payload_info info;
    for (auto anchor : {0ULL, 7ULL}) {
        info.anchor_id = anchor;
        for (auto poll : {0ULL, 65535ULL}) {
            info.poll_id = poll;
            for (auto delay : {0ULL, 0xffffffULL}) {
                info.reply_delay_samples = delay;
                for (auto q : {-32768, -32767, -256, 256, 32766, 32767}) {
                    info.poll_fine_delay_q10 = q;
                    info.fine_delay_valid = true;
                    const auto bits = serialize_packet_payload(info);
                    prs_payload_info decoded;
                    BOOST_REQUIRE(deserialize_packet_payload(bits.data(), bits.size(), decoded));
                    BOOST_CHECK_EQUAL(decoded.poll_fine_delay_q10, q);
                    BOOST_CHECK_EQUAL(decoded.anchor_id, anchor);
                    BOOST_CHECK_EQUAL(decoded.poll_id, poll);
                    BOOST_CHECK_EQUAL(decoded.reply_delay_samples, delay);
                }
            }
        }
    }
    info = prs_payload_info{};
    for (auto bad : {8ULL, 255ULL, 256ULL}) {
        info.anchor_id = bad;
        BOOST_CHECK_THROW(serialize_packet_payload(info), std::invalid_argument);
    }
    info.anchor_id = 0;
    info.poll_id = 65536;
    BOOST_CHECK_THROW(serialize_packet_payload(info), std::invalid_argument);
    info.poll_id = 0;
    info.reply_delay_samples = 0x1000000;
    BOOST_CHECK_THROW(serialize_packet_payload(info), std::invalid_argument);
    info.reply_delay_samples = 0;
    for (auto bad : {-32769, 32768}) {
        info.poll_fine_delay_q10 = bad;
        BOOST_CHECK_THROW(serialize_packet_payload(info), std::invalid_argument);
    }
    info.poll_fine_delay_q10 = 0;
    auto bits = serialize_packet_payload(info);
    prs_payload_info decoded;
    for (int bit = 0; bit < 80; ++bit) {
        auto corrupt = bits;
        corrupt[bit] ^= 1;
        BOOST_CHECK(!deserialize_packet_payload(corrupt.data(), corrupt.size(), decoded));
    }
    for (int reserved : {62, 63}) {
        auto invalid = bits;
        invalid[reserved] = 1;
        uint64_t control = 1ULL | (1ULL << reserved);
        const auto crc = crc16_control_word(control);
        for (int bit = 0; bit < 16; ++bit) invalid[64 + bit] = (crc >> bit) & 1;
        BOOST_CHECK(!deserialize_packet_payload(invalid.data(), invalid.size(), decoded));
    }
    // Independently calculated CRC-CCITT for eight packed bytes 01 00 ... 00.
    BOOST_CHECK_EQUAL(crc16_control_word(1), 0x76ed);
    int16_t q = 0;
    for (double value : {-32.0, -0.25, 0.25, 32767.0 / 1024.0}) {
        BOOST_CHECK(quantize_poll_fine_delay(value, true, q));
        BOOST_CHECK_SMALL(q / 1024.0 - value, 1e-12);
    }
    for (double value : {32.0, -32.001, static_cast<double>(NAN), static_cast<double>(INFINITY), static_cast<double>(-INFINITY)}) {
        BOOST_CHECK(!quantize_poll_fine_delay(value, true, q));
    }
    BOOST_CHECK(!quantize_poll_fine_delay(0.0, false, q));
    BOOST_CHECK(quantize_poll_fine_delay(0.5 / 1024, true, q));
    BOOST_CHECK_EQUAL(q, 1);
    BOOST_CHECK(quantize_poll_fine_delay(-0.5 / 1024, true, q));
    BOOST_CHECK_EQUAL(q, -1);
}

BOOST_AUTO_TEST_CASE(test_fixed_guards_and_poll_id_wrap)
{
    prs_burst_scheduler scheduler;
    for (unsigned id = 0; id <= 65535; ++id) BOOST_CHECK_EQUAL(scheduler.allocate_poll_id(), id);
    BOOST_CHECK_EQUAL(scheduler.allocate_poll_id(), 0);
    prs_rx_config rx;
    auto tx = prs_timed_burst_source::make();
    BOOST_CHECK_EQUAL(frame_len(rx), tx->frame_len());
    BOOST_CHECK_EQUAL(prs_start_offset(rx), tx->prs_start());
    const auto samples = tx->frame_samples();
    for (int i = 0; i < phy::leading_guard_samples; ++i) BOOST_CHECK_EQUAL(samples[i], gr_complex(0, 0));
    for (int i = 0; i < phy::tail_guard_samples; ++i) BOOST_CHECK_EQUAL(samples[samples.size() - 1 - i], gr_complex(0, 0));
    BOOST_CHECK_CLOSE(std::abs(samples[phy::leading_guard_samples]), 0.95, 0.001);
}

BOOST_AUTO_TEST_CASE(test_prs_timed_burst_source_amplitude_limit)
{
    auto src = prs_timed_burst_source::make();
    const auto frame = src->frame_samples();
    const auto peak =
        std::max_element(frame.begin(), frame.end(), [](const auto& a, const auto& b) {
            return std::abs(a) < std::abs(b);
        });
    BOOST_REQUIRE(peak != frame.end());
    BOOST_CHECK_LE(std::abs(*peak), 1.00001f);
}

BOOST_AUTO_TEST_CASE(test_prs_frame_id_payload_crc)
{
    std::vector<gr_complex> payload(prs_frame_id_payload_symbols);
    encode_frame_id_payload(0xabcdU, 0.25f, payload.begin());

    uint64_t frame_id = 0;
    float metric = 0.0f;
    BOOST_CHECK(decode_frame_id_payload(
        payload.data(), static_cast<int>(payload.size()), frame_id, metric));
    BOOST_CHECK_EQUAL(frame_id, 0xabcdU);
    BOOST_CHECK_GT(metric, 0.99f);

    payload[prs_frame_id_ref_symbols + prs_payload_repeat * 3] *= -1.0f;
    BOOST_CHECK(decode_frame_id_payload(
        payload.data(), static_cast<int>(payload.size()), frame_id, metric));
    const float one_disputed_margin = static_cast<float>(prs_payload_repeat - 2) /
                                      static_cast<float>(prs_payload_repeat);
    const float expected_metric =
        1.0f - (1.0f - one_disputed_margin) / static_cast<float>(prs_payload_data_bits);
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
    encode_frame_id_payload(0xabcdU, 0.25f, payload.begin());

    const int bit_start = prs_frame_id_ref_symbols;
    const int negative_samples = prs_payload_repeat / 2 + 1;
    for (int r = 0; r < negative_samples; ++r) {
        payload[bit_start + r] = gr_complex(-0.001f, 0.0f);
    }

    uint64_t frame_id = 0;
    float metric = 0.0f;
    BOOST_CHECK(decode_frame_id_payload(
        payload.data(), static_cast<int>(payload.size()), frame_id, metric));
    BOOST_CHECK_EQUAL(frame_id, 0xabcdU);
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
    tx_info.poll_id = 1234;
    tx_info.anchor_id = 7;
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
    BOOST_CHECK_EQUAL(rx_info.poll_id, tx_info.poll_id);
    BOOST_CHECK_EQUAL(rx_info.anchor_id, tx_info.anchor_id);
    BOOST_CHECK_EQUAL(rx_info.reply_delay_samples, tx_info.reply_delay_samples);
    BOOST_CHECK_GT(metric, 0.99f);
}

} /* namespace ofdm_prs_ranging */
} /* namespace gr */
