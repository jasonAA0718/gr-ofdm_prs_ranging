/* -*- c++ -*- */
#include "zc_packet_utils.h"
#include <algorithm>

namespace gr {
namespace ofdm_prs_ranging {

std::vector<uint8_t> zc_int_to_bits(uint32_t value, int width)
{
    const uint32_t mask = width >= 32 ? 0xffffffffU : ((uint32_t{ 1 } << width) - 1U);
    value &= mask;
    std::vector<uint8_t> bits;
    bits.reserve(width);
    for (int i = width - 1; i >= 0; --i) {
        bits.push_back(static_cast<uint8_t>((value >> i) & 0x1U));
    }
    return bits;
}

uint32_t zc_bits_to_uint(const std::vector<uint8_t>& bits, size_t start, int width)
{
    uint32_t value = 0;
    for (int i = 0; i < width; ++i) {
        value = (value << 1) | (bits[start + i] ? 1U : 0U);
    }
    return value;
}

int32_t zc_bits_to_int_signed(const std::vector<uint8_t>& bits, size_t start, int width)
{
    uint32_t value = zc_bits_to_uint(bits, start, width);
    const uint32_t sign = uint32_t{ 1 } << (width - 1);
    if (value & sign) {
        value -= uint32_t{ 1 } << width;
    }
    return static_cast<int32_t>(value);
}

uint8_t zc_crc8(const std::vector<uint8_t>& bits, size_t start, int width)
{
    uint8_t crc = 0;
    for (int i = 0; i < width; ++i) {
        const uint8_t feedback = static_cast<uint8_t>(((crc >> 7) & 1U) ^ (bits[start + i] & 1U));
        crc = static_cast<uint8_t>(crc << 1);
        if (feedback) {
            crc ^= 0x07U;
        }
    }
    return crc;
}

std::vector<uint8_t> zc_build_response_packet(uint8_t responder_id,
                                              int32_t t2_minus_t3,
                                              uint16_t measurement_number)
{
    std::vector<uint8_t> bits = zc_int_to_bits(zc_payload_sync, zc_payload_sync_bits);
    const auto rid = zc_int_to_bits(responder_id, zc_responder_id_bits);
    bits.insert(bits.end(), rid.begin(), rid.end());
    const auto dt = zc_int_to_bits(static_cast<uint32_t>(t2_minus_t3), zc_t2_minus_t3_bits);
    bits.insert(bits.end(), dt.begin(), dt.end());
    const auto meas = zc_int_to_bits(measurement_number, zc_measurement_number_bits);
    bits.insert(bits.end(), meas.begin(), meas.end());
    const uint8_t crc = zc_crc8(bits, zc_payload_sync_bits, zc_response_body_bits);
    const auto crc_bits = zc_int_to_bits(crc, zc_crc_bits);
    bits.insert(bits.end(), crc_bits.begin(), crc_bits.end());
    return bits;
}

bool zc_decode_response_packet(const std::vector<uint8_t>& bits,
                               uint8_t& responder_id,
                               int32_t& t2_minus_t3,
                               uint16_t& measurement_number)
{
    const auto sync = zc_int_to_bits(zc_payload_sync, zc_payload_sync_bits);
    const int limit = static_cast<int>(bits.size()) - zc_response_packet_bits;
    int sync_start = -1;
    for (int i = 0; i <= limit; ++i) {
        if (std::equal(sync.begin(), sync.end(), bits.begin() + i)) {
            sync_start = i;
            break;
        }
    }
    if (sync_start < 0) {
        return false;
    }
    const size_t body_start = static_cast<size_t>(sync_start + zc_payload_sync_bits);
    const uint8_t expected = zc_crc8(bits, body_start, zc_response_body_bits);
    const uint8_t received = static_cast<uint8_t>(
        zc_bits_to_uint(bits, body_start + zc_response_body_bits, zc_crc_bits));
    if (received != expected) {
        return false;
    }
    responder_id = static_cast<uint8_t>(zc_bits_to_uint(bits, body_start, zc_responder_id_bits));
    size_t pos = body_start + zc_responder_id_bits;
    t2_minus_t3 = zc_bits_to_int_signed(bits, pos, zc_t2_minus_t3_bits);
    pos += zc_t2_minus_t3_bits;
    measurement_number =
        static_cast<uint16_t>(zc_bits_to_uint(bits, pos, zc_measurement_number_bits));
    return true;
}

} // namespace ofdm_prs_ranging
} // namespace gr
