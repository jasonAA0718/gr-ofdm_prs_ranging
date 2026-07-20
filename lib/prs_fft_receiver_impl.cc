/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_fft_receiver_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>

namespace gr {
namespace ofdm_prs_ranging {

prs_fft_receiver::sptr prs_fft_receiver::make(double samp_rate,
                                              int fft_len,
                                              int cp_len,
                                              int active_bins,
                                              int prs_symbols)
{
    return gnuradio::make_block_sptr<prs_fft_receiver_impl>(
        samp_rate, fft_len, cp_len, active_bins, prs_symbols);
}

prs_fft_receiver_impl::prs_fft_receiver_impl(double samp_rate,
                                             int fft_len,
                                             int cp_len,
                                             int active_bins,
                                             int prs_symbols)
    : gr::block("prs_fft_receiver",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0))
{
    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.cp_len = cp_len;
    d_cfg.active_bins = active_bins;
    d_cfg.prs_symbols = prs_symbols;
    d_fft = std::make_unique<gr::fft::fft_complex_fwd>(d_cfg.fft_len, 1);
    message_port_register_in(pmt::mp("frame_in"));
    message_port_register_out(pmt::mp("symbols_out"));
    set_msg_handler(pmt::mp("frame_in"), [this](pmt::pmt_t msg) { handle_frame(msg); });
}

void prs_fft_receiver_impl::handle_frame(pmt::pmt_t msg)
{
    pmt::pmt_t meta;
    std::vector<gr_complex> frame;
    if (!pdu_get_c32(msg, meta, frame)) {
        return;
    }
    const int start = static_cast<int>(dict_ref_double(meta, "prs_start_rel", prs_start_offset(d_cfg)));
    const int needed = start + prs_len(d_cfg);
    if (static_cast<int>(frame.size()) < needed) {
        meta = pmt::dict_add(meta, pmt::mp("fft_error"), pmt::mp("short_frame"));
        message_port_pub(pmt::mp("symbols_out"), pmt::cons(meta, pmt::init_c32vector(0, std::vector<gr_complex>())));
        return;
    }

    const int half_active = d_cfg.active_bins / 2;
    std::vector<gr_complex> active;
    active.reserve(static_cast<size_t>(d_cfg.prs_symbols * d_cfg.active_bins));
    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        const int sym_start = start + sym * (d_cfg.fft_len + d_cfg.cp_len) + d_cfg.cp_len;
        std::copy(frame.begin() + sym_start,
                  frame.begin() + sym_start + d_cfg.fft_len,
                  d_fft->get_inbuf());
        d_fft->execute();
        const gr_complex* freq = d_fft->get_outbuf();
        for (int b = -half_active; b < 0; ++b) {
            active.push_back(freq[(b + d_cfg.fft_len) % d_cfg.fft_len]);
        }
        for (int b = 1; b <= half_active; ++b) {
            active.push_back(freq[b]);
        }
    }

    meta = pmt::dict_add(meta, pmt::mp("symbols"), pmt::from_long(d_cfg.prs_symbols));
    meta = pmt::dict_add(meta, pmt::mp("active_bins"), pmt::from_long(d_cfg.active_bins));
    message_port_pub(pmt::mp("symbols_out"),
                     pmt::cons(meta, pmt::init_c32vector(active.size(), active)));
}

} // namespace ofdm_prs_ranging
} // namespace gr
