/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_fft_receiver_impl.h"
#include "prs_timing_helper.h"
#include <gnuradio/io_signature.h>
#include <algorithm>

namespace gr {
namespace ofdm_prs_ranging {

prs_fft_receiver::sptr prs_fft_receiver::make(double samp_rate,
                                              int fft_len,
                                              int cp_len,
                                              int active_bins,
                                              int prs_symbols,
                                              bool enable_profiling)
{
    return gnuradio::make_block_sptr<prs_fft_receiver_impl>(
        samp_rate, fft_len, cp_len, active_bins, prs_symbols, enable_profiling);
}

prs_fft_receiver_impl::prs_fft_receiver_impl(double samp_rate,
                                             int fft_len,
                                             int cp_len,
                                             int active_bins,
                                             int prs_symbols,
                                             bool enable_profiling)
    : gr::block("prs_fft_receiver",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_enable_profiling(enable_profiling)
{
    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.cp_len = cp_len;
    d_cfg.active_bins = active_bins;
    d_cfg.prs_symbols = prs_symbols;
    d_fft = std::make_unique<gr::fft::fft_complex_fwd>(d_cfg.fft_len, 1);
    d_active.resize(static_cast<size_t>(d_cfg.prs_symbols * d_cfg.active_bins));
    message_port_register_in(pmt::mp("frame_in"));
    message_port_register_out(pmt::mp("symbols_out"));
    message_port_register_out(pmt::mp("timing_out"));
    set_msg_handler(pmt::mp("frame_in"), [this](pmt::pmt_t msg) { handle_frame(msg); });
}

void prs_fft_receiver_impl::handle_frame(pmt::pmt_t msg)
{
    profiling::timing_report report(d_enable_profiling, "fft_receiver");
    pmt::pmt_t meta;
    const gr_complex* frame = nullptr;
    size_t frame_size = 0;
    if (!pdu_get_c32_view(msg, meta, frame, frame_size)) {
        return;
    }
    report.checkpoint("input_decode");
    const int start =
        static_cast<int>(dict_ref_double(meta, "prs_start_rel", prs_start_offset(d_cfg)));
    const int needed = start + prs_len(d_cfg);
    if (frame_size < static_cast<size_t>(needed)) {
        meta = pmt::dict_add(meta, pmt::mp("fft_error"), pmt::mp("short_frame"));
        message_port_pub(
            pmt::mp("symbols_out"),
            pmt::cons(meta, pmt::init_c32vector(0, std::vector<gr_complex>())));
        if (report.enabled()) {
            message_port_pub(pmt::mp("timing_out"), report.finish(meta));
        }
        return;
    }

    size_t active_index = 0;
    uint64_t copy_ns = 0;
    uint64_t fft_ns = 0;
    uint64_t reorder_ns = 0;
    for (int sym = 0; sym < d_cfg.prs_symbols; ++sym) {
        const int sym_start = start + sym * (d_cfg.fft_len + d_cfg.cp_len) + d_cfg.cp_len;
        auto stage_start = report.mark();
        std::copy(
            frame + sym_start, frame + sym_start + d_cfg.fft_len, d_fft->get_inbuf());
        auto stage_stop = report.mark();
        if (report.enabled()) {
            copy_ns += profiling::elapsed_ns(stage_start, stage_stop);
        }
        stage_start = stage_stop;
        d_fft->execute();
        stage_stop = report.mark();
        if (report.enabled()) {
            fft_ns += profiling::elapsed_ns(stage_start, stage_stop);
        }
        stage_start = stage_stop;
        const gr_complex* freq = d_fft->get_outbuf();
        if (d_cfg.active_bins == d_cfg.fft_len) {
            for (int fft_bin = d_cfg.fft_len / 2; fft_bin < d_cfg.fft_len; ++fft_bin) {
                d_active[active_index++] = freq[fft_bin];
            }
            for (int fft_bin = 0; fft_bin < d_cfg.fft_len / 2; ++fft_bin) {
                d_active[active_index++] = freq[fft_bin];
            }
            if (report.enabled()) {
                reorder_ns += profiling::elapsed_ns(stage_start, report.mark());
            }
            continue;
        }

        const int half_active = d_cfg.active_bins / 2;
        for (int b = -half_active; b < 0; ++b) {
            d_active[active_index++] = freq[b + d_cfg.fft_len];
        }
        for (int b = 1; b <= half_active; ++b) {
            d_active[active_index++] = freq[b];
        }
        if (report.enabled()) {
            reorder_ns += profiling::elapsed_ns(stage_start, report.mark());
        }
    }
    report.add("cp_remove_copy", copy_ns);
    report.add("fft_execute", fft_ns);
    report.add("active_bin_reorder", reorder_ns);

    auto stage_start = report.mark();
    meta = pmt::dict_add(meta, pmt::mp("symbols"), pmt::from_long(d_cfg.prs_symbols));
    meta = pmt::dict_add(meta, pmt::mp("active_bins"), pmt::from_long(d_cfg.active_bins));
    const auto output = pmt::cons(meta, pmt::init_c32vector(d_active.size(), d_active));
    auto stage_stop = report.mark();
    report.add("output_build", stage_start, stage_stop);
    stage_start = stage_stop;
    message_port_pub(pmt::mp("symbols_out"), output);
    stage_stop = report.mark();
    report.add("output_publish", stage_start, stage_stop);
    if (report.enabled()) {
        message_port_pub(pmt::mp("timing_out"), report.finish(meta));
    }
}

} // namespace ofdm_prs_ranging
} // namespace gr
