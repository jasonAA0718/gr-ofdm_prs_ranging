/* -*- c++ -*- */
#include "zc_packet_utils.h"
#include <boost/test/unit_test.hpp>

namespace gr {
namespace ofdm_prs_ranging {

BOOST_AUTO_TEST_CASE(test_zc_response_packet_roundtrip)
{
    const auto bits = zc_build_response_packet(0x02, -30000, 0x0abc);
    BOOST_CHECK_EQUAL(bits.size(), static_cast<size_t>(zc_response_packet_bits));

    uint8_t responder_id = 0;
    int32_t t2_minus_t3 = 0;
    uint16_t measurement_number = 0;
    BOOST_CHECK(zc_decode_response_packet(bits, responder_id, t2_minus_t3, measurement_number));
    BOOST_CHECK_EQUAL(responder_id, 0x02);
    BOOST_CHECK_EQUAL(t2_minus_t3, -30000);
    BOOST_CHECK_EQUAL(measurement_number, 0x0abc);
}

BOOST_AUTO_TEST_CASE(test_zc_response_packet_crc_rejects_error)
{
    auto bits = zc_build_response_packet(0x02, -30000, 0x0012);
    bits[zc_payload_sync_bits + 3] ^= 1U;

    uint8_t responder_id = 0;
    int32_t t2_minus_t3 = 0;
    uint16_t measurement_number = 0;
    BOOST_CHECK(!zc_decode_response_packet(bits, responder_id, t2_minus_t3, measurement_number));
}

} // namespace ofdm_prs_ranging
} // namespace gr
