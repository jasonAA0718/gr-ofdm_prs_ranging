/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_payload_codec.h"
#include <algorithm>
#include <cmath>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
float ref_sign(int index)
{
    static constexpr int signs[prs_frame_id_ref_symbols] = {
        1, 1, 1, -1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1
    };
    return static_cast<float>(signs[index]);
}

void crc16_update(uint16_t& crc, uint8_t byte)
{
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                              : static_cast<uint16_t>(crc << 1);
    }
}

void crc16_update_u32(uint16_t& crc, uint32_t value)
{
    for (int byte = 0; byte < 4; ++byte) {
        crc16_update(crc, static_cast<uint8_t>((value >> (8 * byte)) & 0xffU));
    }
}

uint16_t crc16_ccitt(const prs_payload_info& info)
{
    uint16_t crc = 0xffffU;
    crc16_update(crc, info.packet_type);
    crc16_update_u32(crc, info.poll_frame_id);
    crc16_update_u32(crc, info.response_frame_id);
    crc16_update_u32(crc, info.reply_delay_samples);
    return crc;
}

bool payload_bit(const prs_payload_info& info, uint16_t crc, int bit_index)
{
    if (bit_index < prs_payload_packet_type_bits) {
        return ((info.packet_type >> bit_index) & 0x1U) != 0;
    }
    bit_index -= prs_payload_packet_type_bits;
    if (bit_index < prs_payload_frame_id_bits) {
        return ((info.poll_frame_id >> bit_index) & 0x1U) != 0;
    }
    bit_index -= prs_payload_frame_id_bits;
    if (bit_index < prs_payload_frame_id_bits) {
        return ((info.response_frame_id >> bit_index) & 0x1U) != 0;
    }
    bit_index -= prs_payload_frame_id_bits;
    if (bit_index < prs_payload_reply_delay_bits) {
        return ((info.reply_delay_samples >> bit_index) & 0x1U) != 0;
    }
    bit_index -= prs_payload_reply_delay_bits;
    return ((crc >> bit_index) & 0x1U) != 0;
}
} // namespace

void encode_packet_payload(const prs_payload_info& info,
                           float amplitude,
                           std::vector<gr_complex>::iterator out)
{
    const uint16_t crc = crc16_ccitt(info);

    for (int i = 0; i < prs_frame_id_ref_symbols; ++i) {
        *(out++) = gr_complex(ref_sign(i) * amplitude, 0.0f);
    }

    for (int bit = 0; bit < prs_payload_data_bits; ++bit) {
        const float sign = payload_bit(info, crc, bit) ? 1.0f : -1.0f;
        for (int r = 0; r < prs_payload_repeat; ++r) {
            *(out++) = gr_complex(sign * amplitude, 0.0f);
        }
    }
}

void encode_frame_id_payload(uint64_t frame_id,
                             float amplitude,
                             std::vector<gr_complex>::iterator out)
{
    prs_payload_info info;
    info.packet_type = prs_packet_type_poll;
    info.poll_frame_id = static_cast<uint32_t>(frame_id & 0xffffffffU);
    info.response_frame_id = 0;
    info.reply_delay_samples = 0;
    encode_packet_payload(info, amplitude, out);
}

bool decode_packet_payload(const gr_complex* payload,
                           int payload_len,
                           prs_payload_info& info,
                           float& metric,
                           double phase_increment_rad)
{
    info = prs_payload_info{};
    metric = 0.0f;
    if (payload == nullptr || payload_len < prs_frame_id_payload_symbols) {
        return false;
    }

    const gr_complex phase_step(
        static_cast<float>(std::cos(-phase_increment_rad)),
        static_cast<float>(std::sin(-phase_increment_rad)));
    gr_complex phase_rotation(1.0f, 0.0f);
    gr_complex ref_corr(0.0f, 0.0f);
    double ref_power = 0.0;
    for (int i = 0; i < prs_frame_id_ref_symbols; ++i) {
        const auto expected = gr_complex(ref_sign(i), 0.0f);
        const gr_complex derotated = payload[i] * phase_rotation;
        ref_corr += derotated * expected;
        ref_power += std::norm(derotated);
        phase_rotation *= phase_step;
    }

    const float ref_mag = std::abs(ref_corr);
    if (ref_mag <= 0.0f || ref_power <= 0.0) {
        return false;
    }

    const gr_complex correction = std::conj(ref_corr) / ref_mag;
    const float ref_metric = static_cast<float>(
        std::min(1.0, ref_mag / std::sqrt(ref_power * prs_frame_id_ref_symbols)));

    uint8_t packet_type = 0;
    uint32_t poll_frame_id = 0;
    uint32_t response_frame_id = 0;
    uint32_t reply_delay_samples = 0;
    uint16_t rx_crc = 0;
    double decision_margin_sum = 0.0;
    for (int bit = 0; bit < prs_payload_data_bits; ++bit) {
        double decision_sum = 0.0;
        double magnitude_sum = 0.0;
        for (int r = 0; r < prs_payload_repeat; ++r) {
            const int index = prs_frame_id_ref_symbols + bit * prs_payload_repeat + r;
            const gr_complex corrected =
                payload[index] * phase_rotation * correction;
            decision_sum += corrected.real();
            magnitude_sum += std::abs(corrected.real());
            phase_rotation *= phase_step;
        }
        const bool one = decision_sum >= 0.0;
        if (magnitude_sum > 0.0) {
            decision_margin_sum +=
                std::min(1.0, std::abs(decision_sum) / magnitude_sum);
        }

        int field_bit = bit;
        if (field_bit < prs_payload_packet_type_bits) {
            if (one) {
                packet_type |= static_cast<uint8_t>(uint8_t{ 1 } << field_bit);
            }
            continue;
        }
        field_bit -= prs_payload_packet_type_bits;
        if (field_bit < prs_payload_frame_id_bits) {
            if (one) {
                poll_frame_id |= (uint32_t{ 1 } << field_bit);
            }
            continue;
        }
        field_bit -= prs_payload_frame_id_bits;
        if (field_bit < prs_payload_frame_id_bits) {
            if (one) {
                response_frame_id |= (uint32_t{ 1 } << field_bit);
            }
            continue;
        }
        field_bit -= prs_payload_frame_id_bits;
        if (field_bit < prs_payload_reply_delay_bits) {
            if (one) {
                reply_delay_samples |= (uint32_t{ 1 } << field_bit);
            }
            continue;
        }
        field_bit -= prs_payload_reply_delay_bits;
        if (one) {
            rx_crc |= static_cast<uint16_t>(uint16_t{ 1 } << field_bit);
        }
    }

    info.packet_type = packet_type;
    info.poll_frame_id = poll_frame_id;
    info.response_frame_id = response_frame_id;
    info.reply_delay_samples = reply_delay_samples;
    const float decision_metric = static_cast<float>(
        decision_margin_sum / static_cast<double>(prs_payload_data_bits));
    metric = std::min(ref_metric, decision_metric);
    return rx_crc == crc16_ccitt(info);
}

bool decode_frame_id_payload(const gr_complex* payload,
                             int payload_len,
                             uint64_t& frame_id,
                             float& metric)
{
    prs_payload_info info;
    const bool valid = decode_packet_payload(payload, payload_len, info, metric);
    frame_id = info.packet_type == prs_packet_type_response ? info.response_frame_id
                                                            : info.poll_frame_id;
    return valid;
}

} // namespace ofdm_prs_ranging
} // namespace gr
