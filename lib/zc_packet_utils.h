/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_PACKET_UTILS_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_PACKET_UTILS_H

#include <cstdint>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

constexpr uint16_t zc_payload_sync = 0xA5C3;
constexpr int zc_payload_sync_bits = 16;
constexpr int zc_responder_id_bits = 8;
constexpr int zc_t2_minus_t3_bits = 24;
constexpr int zc_measurement_number_bits = 12;
constexpr int zc_crc_bits = 8;
constexpr int zc_response_body_bits =
    zc_responder_id_bits + zc_t2_minus_t3_bits + zc_measurement_number_bits;
constexpr int zc_response_packet_bits =
    zc_payload_sync_bits + zc_response_body_bits + zc_crc_bits;

std::vector<uint8_t> zc_int_to_bits(uint32_t value, int width);
uint32_t zc_bits_to_uint(const std::vector<uint8_t>& bits, size_t start, int width);
int32_t zc_bits_to_int_signed(const std::vector<uint8_t>& bits, size_t start, int width);
uint8_t zc_crc8(const std::vector<uint8_t>& bits, size_t start, int width);
std::vector<uint8_t> zc_build_response_packet(uint8_t responder_id,
                                              int32_t t2_minus_t3,
                                              uint16_t measurement_number);
bool zc_decode_response_packet(const std::vector<uint8_t>& bits,
                               uint8_t& responder_id,
                               int32_t& t2_minus_t3,
                               uint16_t& measurement_number);

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
