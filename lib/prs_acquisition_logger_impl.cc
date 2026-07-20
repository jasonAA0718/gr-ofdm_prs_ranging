/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_acquisition_logger_impl.h"
#include <gnuradio/io_signature.h>
#include <chrono>
#include <cmath>
#include <iomanip>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
double dict_ref_time_tuple_local(const pmt::pmt_t& dict, const char* key, double fallback)
{
    const auto value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_tuple(value) && pmt::length(value) >= 2) {
        const auto secs = pmt::tuple_ref(value, 0);
        const auto frac = pmt::tuple_ref(value, 1);
        const double s = pmt::is_uint64(secs) ? static_cast<double>(pmt::to_uint64(secs))
                                             : static_cast<double>(pmt::to_long(secs));
        return s + pmt::to_double(frac);
    }
    return fallback;
}

uint64_t pdu_len(pmt::pmt_t msg)
{
    if (!pmt::is_pair(msg)) {
        return 0;
    }
    const auto data = pmt::cdr(msg);
    return pmt::is_uniform_vector(data) ? pmt::length(data) : 0;
}
} // namespace

prs_acquisition_logger::sptr
prs_acquisition_logger::make(const std::string& path,
                             const std::string& node,
                             bool append)
{
    return gnuradio::make_block_sptr<prs_acquisition_logger_impl>(path, node, append);
}

prs_acquisition_logger_impl::prs_acquisition_logger_impl(const std::string& path,
                                                         const std::string& node,
                                                         bool append)
    : gr::block("prs_acquisition_logger",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_file(path, append ? std::ios::app : std::ios::trunc),
      d_node(node)
{
    if (!append || d_file.tellp() == 0) {
        d_file << "log_time_unix,node,channel_id,coarse_zc_root,recv_id,packet_type,frame_id,poll_frame_id,"
                  "response_frame_id,reply_delay_samples,frame_id_valid,rx_time,"
                  "rx_time_tag_offset,absolute_sample_index,frame_start,coarse_peak,"
                  "coarse_offset_samples,peak_metric,preamble_metric,coarse_metric,"
                  "payload_metric,cfo,samp_rate,fft_len,cp_len,active_bins,prs_symbols,"
                  "prs_start_rel,prs_len,pdu_len\n";
    }
    message_port_register_in(pmt::mp("frame_in"));
    set_msg_handler(pmt::mp("frame_in"),
                    [this](pmt::pmt_t msg) { handle_frame(msg); });
}

prs_acquisition_logger_impl::~prs_acquisition_logger_impl()
{
    if (d_file.is_open()) {
        d_file.flush();
    }
}

void prs_acquisition_logger_impl::handle_frame(pmt::pmt_t msg)
{
    if (!pmt::is_pair(msg) || !d_file.is_open()) {
        return;
    }

    const auto meta = pmt::car(msg);
    const uint64_t frame_start = dict_ref_uint64(meta, "frame_start", 0);
    const uint64_t coarse_peak = dict_ref_uint64(meta, "coarse_peak", 0);
    const int64_t coarse_offset =
        static_cast<int64_t>(coarse_peak) - static_cast<int64_t>(frame_start);
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const double now_s =
        std::chrono::duration_cast<std::chrono::duration<double>>(now).count();

    d_file << std::setprecision(15)
           << now_s << ','
           << d_node << ','
           << dict_ref_uint64(meta, "channel_id", 0) << ','
           << dict_ref_uint64(meta, "coarse_zc_root", 25) << ','
           << dict_ref_uint64(meta, "recv_id", 0) << ','
           << dict_ref_uint64(meta, "packet_type", 0) << ','
           << dict_ref_uint64(meta, "frame_id", 0) << ','
           << dict_ref_uint64(meta, "poll_frame_id", 0) << ','
           << dict_ref_uint64(meta, "response_frame_id", 0) << ','
           << dict_ref_uint64(meta, "reply_delay_samples", 0) << ','
           << (pmt::to_bool(pmt::dict_ref(meta, pmt::mp("frame_id_valid"), pmt::PMT_F)) ? 1 : 0)
           << ','
           << dict_ref_time_tuple_local(meta, "rx_time", NAN) << ','
           << dict_ref_uint64(meta, "rx_time_tag_offset", 0) << ','
           << dict_ref_uint64(meta, "absolute_sample_index", 0) << ','
           << frame_start << ','
           << coarse_peak << ','
           << coarse_offset << ','
           << dict_ref_double(meta, "peak_metric", 0.0) << ','
           << dict_ref_double(meta, "preamble_metric", 0.0) << ','
           << dict_ref_double(meta, "coarse_metric", 0.0) << ','
           << dict_ref_double(meta, "payload_metric", 0.0) << ','
           << dict_ref_double(meta, "cfo", 0.0) << ','
           << dict_ref_double(meta, "samp_rate", 0.0) << ','
           << dict_ref_uint64(meta, "fft_len", 0) << ','
           << dict_ref_uint64(meta, "cp_len", 0) << ','
           << dict_ref_uint64(meta, "active_bins", 0) << ','
           << dict_ref_uint64(meta, "prs_symbols", 0) << ','
           << dict_ref_uint64(meta, "prs_start_rel", 0) << ','
           << dict_ref_uint64(meta, "prs_len", 0) << ','
           << pdu_len(msg) << '\n';
    d_file.flush();
}

} // namespace ofdm_prs_ranging
} // namespace gr
