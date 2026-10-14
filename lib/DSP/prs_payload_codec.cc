/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_payload_codec.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

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

} // namespace

bool payload_fields_valid(const prs_payload_info& info)
{
    return info.packet_type <= 3 && info.anchor_id <= 7 && info.poll_id <= 65535 &&
           info.reply_delay_samples <= 0xffffff && info.poll_fine_delay_q10 >= -32768 &&
           info.poll_fine_delay_q10 <= 32767;
}

uint16_t crc16_control_word(uint64_t control)
{
    uint16_t crc = 0xffffU;
    for (int byte = 0; byte < 8; ++byte) {
        crc16_update(crc, static_cast<uint8_t>(control >> (8 * byte)));
    }
    return crc;
}

bool quantize_poll_fine_delay(double samples, bool phase_valid, int16_t& q)
{
    q = 0;
    if (!phase_valid || !std::isfinite(samples)) {
        return false;
    }
    const double rounded = std::round(samples * 1024.0);
    if (!std::isfinite(rounded) || rounded < -32768.0 || rounded > 32767.0) {
        return false;
    }
    q = static_cast<int16_t>(rounded);
    return true;
}

prs_payload_bits serialize_packet_payload(const prs_payload_info& info)
{
    if (!payload_fields_valid(info)) {
        throw std::invalid_argument("packet v2 field out of range");
    }
    const uint64_t control = info.packet_type | (info.anchor_id << 2) |
        (info.poll_id << 5) | (info.reply_delay_samples << 21) |
        (static_cast<uint64_t>(static_cast<uint16_t>(info.poll_fine_delay_q10)) << 45) |
        (static_cast<uint64_t>(info.fine_delay_valid) << 61);
    const uint16_t crc = crc16_control_word(control);
    prs_payload_bits bits{};
    for (int bit = 0; bit < 64; ++bit) {
        bits[bit] = (control >> bit) & 1U;
    }
    for (int bit = 0; bit < 16; ++bit) {
        bits[64 + bit] = (crc >> bit) & 1U;
    }
    return bits;
}

bool deserialize_packet_payload(const uint8_t* bits, size_t bit_count,
                                prs_payload_info& info)
{
    info = prs_payload_info{};
    info.packet_type = 0;
    if (!bits || bit_count < prs_payload_data_bits) {
        return false;
    }
    uint64_t control = 0;
    uint16_t crc = 0;
    for (int bit = 0; bit < 64; ++bit) {
        control |= static_cast<uint64_t>(bits[bit] & 1U) << bit;
    }
    for (int bit = 0; bit < 16; ++bit) {
        crc |= static_cast<uint16_t>(bits[64 + bit] & 1U) << bit;
    }
    info.packet_type = control & 3U;
    info.anchor_id = (control >> 2) & 7U;
    info.poll_id = (control >> 5) & 0xffffU;
    info.reply_delay_samples = (control >> 21) & 0xffffffU;
    const int64_t raw_q = (control >> 45) & 0xffffU;
    info.poll_fine_delay_q10 = raw_q >= 32768 ? raw_q - 65536 : raw_q;
    info.fine_delay_valid = ((control >> 61) & 1U) != 0;
    return (control >> 62) == 0 && crc == crc16_control_word(control);
}

void encode_packet_payload(const prs_payload_info& info,
                           float amplitude,
                           std::vector<gr_complex>::iterator out)
{
    const auto bits = serialize_packet_payload(info);

    for (int i = 0; i < prs_frame_id_ref_symbols; ++i) {
        *(out++) = gr_complex(ref_sign(i) * amplitude, 0.0f);
    }

    for (int bit = 0; bit < prs_payload_data_bits; ++bit) {
        const float sign = bits[static_cast<size_t>(bit)] != 0U ? 1.0f : -1.0f;
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
    info.poll_id = frame_id;
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

    const gr_complex phase_step(static_cast<float>(std::cos(-phase_increment_rad)),
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

    prs_payload_bits bits{};
    double decision_margin_sum = 0.0;
    for (int bit = 0; bit < prs_payload_data_bits; ++bit) {
        double decision_sum = 0.0;
        double magnitude_sum = 0.0;
        for (int r = 0; r < prs_payload_repeat; ++r) {
            const int index = prs_frame_id_ref_symbols + bit * prs_payload_repeat + r;
            const gr_complex corrected = payload[index] * phase_rotation * correction;
            decision_sum += corrected.real();
            magnitude_sum += std::abs(corrected.real());
            phase_rotation *= phase_step;
        }
        const bool one = decision_sum >= 0.0;
        if (magnitude_sum > 0.0) {
            decision_margin_sum += std::min(1.0, std::abs(decision_sum) / magnitude_sum);
        }

        bits[bit] = one ? 1U : 0U;
    }

    const float decision_metric = static_cast<float>(
        decision_margin_sum / static_cast<double>(prs_payload_data_bits));
    metric = std::min(ref_metric, decision_metric);
    return deserialize_packet_payload(bits.data(), bits.size(), info);
}

bool decode_frame_id_payload(const gr_complex* payload,
                             int payload_len,
                             uint64_t& frame_id,
                             float& metric)
{
    prs_payload_info info;
    const bool valid = decode_packet_payload(payload, payload_len, info, metric);
    frame_id = info.poll_id;
    return valid;
}

} // namespace ofdm_prs_ranging
} // namespace gr
