/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_MC_DS_PRS_H
#define INCLUDED_OFDM_PRS_RANGING_MC_DS_PRS_H

#include "gold127_codes.h"
#include <gnuradio/gr_complex.h>
#include <array>
#include <cstddef>
#include <cstdint>

namespace gr {
namespace ofdm_prs_ranging {

constexpr int mc_ds_fft_len = 512;
constexpr int mc_ds_cp_len = 64;
constexpr int mc_ds_symbol_count = gold127_code_length;
constexpr int mc_ds_bin_period = 4;
constexpr int mc_ds_data_bin_remainder = 3;
constexpr int mc_ds_data_bin_count = 128;
constexpr int mc_ds_pilot_bin_count = 384;
constexpr int mc_ds_default_gold_code_id = 2;

constexpr char mc_ds_data_scrambler[] =
    "0010010100101001110011001111011001010110010000001001011110011011"
    "1111010100001000001100001101101100111100000111111110011100100011";
static_assert(sizeof(mc_ds_data_scrambler) - 1 == mc_ds_data_bin_count,
              "MC-DS data scrambler must contain 128 bits");

struct mc_ds_golay_pair {
    std::array<int8_t, mc_ds_fft_len> a{};
    std::array<int8_t, mc_ds_fft_len> b{};
};

constexpr mc_ds_golay_pair make_mc_ds_golay_pair()
{
    mc_ds_golay_pair pair{};
    pair.a[0] = 1;
    pair.b[0] = 1;
    for (int length = 1; length < mc_ds_fft_len; length *= 2) {
        for (int i = 0; i < length; ++i) {
            const int8_t old_a = pair.a[static_cast<size_t>(i)];
            const int8_t old_b = pair.b[static_cast<size_t>(i)];
            pair.a[static_cast<size_t>(length + i)] = old_b;
            pair.b[static_cast<size_t>(i)] = old_a;
            pair.b[static_cast<size_t>(length + i)] = -old_b;
        }
    }
    return pair;
}

constexpr auto mc_ds_golay = make_mc_ds_golay_pair();

inline bool mc_ds_is_data_bin(int native_fft_bin)
{
    return native_fft_bin % mc_ds_bin_period == mc_ds_data_bin_remainder;
}

inline int mc_ds_data_index(int native_fft_bin)
{
    return native_fft_bin / mc_ds_bin_period;
}

inline int mc_ds_ordered_to_native_bin(int ordered_bin)
{
    return (ordered_bin + mc_ds_fft_len / 2) % mc_ds_fft_len;
}

inline int mc_ds_native_to_ordered_bin(int native_fft_bin)
{
    return (native_fft_bin + mc_ds_fft_len / 2) % mc_ds_fft_len;
}

inline uint8_t mc_ds_scrambler_bit(int data_index)
{
    return static_cast<uint8_t>(mc_ds_data_scrambler[data_index] - '0');
}

inline float mc_ds_gold_chip(int code_id, int symbol_index)
{
    return static_cast<float>(
        gold127_codes[static_cast<size_t>(code_id)][static_cast<size_t>(symbol_index)]);
}

inline gr_complex mc_ds_golay_pilot(int symbol_index, int native_fft_bin)
{
    const auto& sequence = symbol_index % 2 == 0 ? mc_ds_golay.a : mc_ds_golay.b;
    return gr_complex(static_cast<float>(sequence[static_cast<size_t>(native_fft_bin)]),
                      0.0f);
}

inline gr_complex mc_ds_pilot(int symbol_index, int native_fft_bin, int code_id)
{
    return mc_ds_gold_chip(code_id, symbol_index) *
           mc_ds_golay_pilot(symbol_index, native_fft_bin);
}

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_MC_DS_PRS_H */
