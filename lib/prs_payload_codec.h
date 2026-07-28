/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_PAYLOAD_CODEC_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_PAYLOAD_CODEC_H

#include <gnuradio/gr_complex.h>
#include <cstdint>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

constexpr uint8_t prs_packet_type_poll = 1;
constexpr uint8_t prs_packet_type_response = 2;
constexpr int prs_frame_id_ref_symbols = 16;
constexpr int prs_payload_packet_type_bits = 8;
constexpr int prs_payload_frame_id_bits = 32;
constexpr int prs_payload_reply_delay_bits = 32;
constexpr int prs_payload_crc_bits = 16;
constexpr int prs_payload_repeat = 280;
constexpr int prs_payload_data_bits = prs_payload_packet_type_bits +
                                      prs_payload_frame_id_bits +
                                      prs_payload_frame_id_bits +
                                      prs_payload_reply_delay_bits +
                                      prs_payload_crc_bits;
constexpr int prs_frame_id_data_symbols = prs_payload_data_bits * prs_payload_repeat;
constexpr int prs_frame_id_payload_symbols =
    prs_frame_id_ref_symbols + prs_frame_id_data_symbols;

struct prs_payload_info {
    uint8_t packet_type = prs_packet_type_poll;
    uint32_t poll_frame_id = 0;
    uint32_t response_frame_id = 0;
    uint32_t reply_delay_samples = 0;
};

void encode_packet_payload(const prs_payload_info& info,
                           float amplitude,
                           std::vector<gr_complex>::iterator out);
bool decode_packet_payload(const gr_complex* payload,
                           int payload_len,
                           prs_payload_info& info,
                           float& metric,
                           double phase_increment_rad = 0.0);
void encode_frame_id_payload(uint64_t frame_id,
                             float amplitude,
                             std::vector<gr_complex>::iterator out);
bool decode_frame_id_payload(const gr_complex* payload,
                             int payload_len,
                             uint64_t& frame_id,
                             float& metric);

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_PAYLOAD_CODEC_H */
