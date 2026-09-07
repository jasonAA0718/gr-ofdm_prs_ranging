/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_MC_DS_PRS_H
#define INCLUDED_OFDM_PRS_RANGING_MC_DS_PRS_H

#include "golay_prs_table.h"
#include <gnuradio/gr_complex.h>
#include <array>
#include <cstddef>

namespace gr {
namespace ofdm_prs_ranging {

constexpr int mc_ds_fft_len = 1024;
constexpr int mc_ds_symbol_count = 8;
constexpr int mc_ds_code_length = 8;
constexpr int mc_ds_bin_period = 8;
constexpr int mc_ds_data_bin_remainder = 7;
constexpr int mc_ds_data_bin_count = 128;
constexpr int mc_ds_pilot_bin_count = 896;
constexpr int mc_ds_code_id = 0;

// Fixed balanced prototype code. This is not represented as a Gold code.
constexpr std::array<float, mc_ds_code_length> mc_ds_prn_code = { 1.0f,  1.0f,  1.0f,
                                                                  -1.0f, -1.0f, 1.0f,
                                                                  -1.0f, -1.0f };

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

inline gr_complex mc_ds_pilot(int symbol_index, int native_fft_bin)
{
    const auto& pilot = golay_prs_at(static_cast<size_t>(symbol_index),
                                     static_cast<size_t>(native_fft_bin));
    return mc_ds_prn_code[static_cast<size_t>(symbol_index)] *
           gr_complex(pilot.real, pilot.imag);
}

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_MC_DS_PRS_H */
