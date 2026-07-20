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
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

BOOST_AUTO_TEST_CASE(test_prs_timed_burst_source_frame_geometry)
{
    auto src = prs_timed_burst_source::make();
    BOOST_CHECK_EQUAL(src->frame_len(), 23935);
    BOOST_CHECK_EQUAL(src->prs_start(), 1000 + 128 * 16 + 839 + 616);
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
    BOOST_CHECK_LE(std::abs(*peak), 0.800001f);
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

    for (int r = 0; r < prs_payload_repeat; ++r) {
        payload[prs_frame_id_ref_symbols + prs_payload_repeat * 7 + r] *= -1.0f;
    }
    BOOST_CHECK(!decode_frame_id_payload(
        payload.data(), static_cast<int>(payload.size()), frame_id, metric));
}

} /* namespace ofdm_prs_ranging */
} /* namespace gr */
