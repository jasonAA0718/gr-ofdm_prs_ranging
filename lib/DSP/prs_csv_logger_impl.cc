/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_csv_logger_impl.h"
#include <gnuradio/io_signature.h>
#include <iomanip>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr double c_mps = 299792458.0;
}

prs_csv_logger::sptr prs_csv_logger::make(const std::string& path, bool append)
{
    return gnuradio::make_block_sptr<prs_csv_logger_impl>(path, append);
}

prs_csv_logger_impl::prs_csv_logger_impl(const std::string& path, bool append)
    : gr::block("prs_csv_logger",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_file(path, append ? std::ios::app : std::ios::trunc)
{
    if (!append || d_file.tellp() == 0) {
        d_file << "direction,poll_frame_id,response_frame_id,t1_tx_time,t4_rx_time,"
                  "reply_delay_samples,reply_delay_s,rtt_s,tof_s,range_m,"
                  "integer_tof_s,integer_range_m,fine_delay_s,"
                  "fine_delay_samples,phase_range_contribution_m,"
                  "frame_id_valid,coarse_metric,payload_metric,preamble_cfo_hz,"
                  "prs_cp_cfo_hz,prs_cp_cfo_coherence,selected_cfo_hz,"
                  "payload_retry_used,prs_channel_cfo_hz,residual_cfo_hz,"
                  "channel_coherence,phase_slope_rad_per_hz,phase_residual,snr,quality\n";
    }
    message_port_register_in(pmt::mp("measurement_in"));
    set_msg_handler(pmt::mp("measurement_in"),
                    [this](pmt::pmt_t msg) { handle_measurement(msg); });
}

prs_csv_logger_impl::~prs_csv_logger_impl()
{
    if (d_file.is_open()) {
        d_file.flush();
    }
}

void prs_csv_logger_impl::handle_measurement(pmt::pmt_t msg)
{
    if (!pmt::is_pair(msg) || !d_file.is_open()) {
        return;
    }
    const auto meta = pmt::car(msg);
    const uint64_t packet_type = dict_ref_uint64(meta, "packet_type", 0);
    const char* direction = packet_type == prs_packet_type_poll
                                ? "poll_rx"
                                : (packet_type == prs_packet_type_response
                                       ? "response_rx"
                                       : "unknown");
    const double fine_delay_s = dict_ref_double(meta, "fine_delay", 0.0);
    const double phase_range_contribution_m = 0.5 * c_mps * fine_delay_s;
    d_file << std::setprecision(15);
    d_file << direction << ','
           << dict_ref_uint64(meta, "poll_frame_id", 0) << ','
           << dict_ref_uint64(meta, "response_frame_id", 0) << ','
           << dict_ref_double(meta, "t1_tx_time", 0.0) << ','
           << dict_ref_double(meta, "t4_rx_time", 0.0) << ','
           << dict_ref_uint64(meta, "reply_delay_samples", 0) << ','
           << dict_ref_double(meta, "reply_delay_s", 0.0) << ','
           << dict_ref_double(meta, "rtt_s", 0.0) << ','
           << dict_ref_double(meta, "tof_s", 0.0) << ','
           << dict_ref_double(meta, "range_m", 0.0) << ','
           << dict_ref_double(meta, "integer_tof_s", 0.0) << ','
           << dict_ref_double(meta, "integer_range_m", 0.0) << ','
           << fine_delay_s << ','
           << dict_ref_double(meta, "fine_delay_samples", 0.0) << ','
           << phase_range_contribution_m << ','
           << (pmt::to_bool(pmt::dict_ref(meta, pmt::mp("frame_id_valid"), pmt::PMT_F)) ? 1 : 0) << ','
           << dict_ref_double(
                  meta, "coarse_metric", dict_ref_double(meta, "peak_metric", 0.0))
           << ','
           << dict_ref_double(meta, "payload_metric", 0.0) << ','
           << dict_ref_double(meta, "preamble_cfo_hz", 0.0) << ','
           << dict_ref_double(meta, "prs_cp_cfo_hz", 0.0) << ','
           << dict_ref_double(meta, "prs_cp_cfo_coherence", 0.0) << ','
           << dict_ref_double(meta, "selected_cfo_hz", 0.0) << ','
           << (pmt::to_bool(
                   pmt::dict_ref(meta, pmt::mp("payload_retry_used"), pmt::PMT_F))
                   ? 1
                   : 0)
           << ','
           << dict_ref_double(meta, "prs_channel_cfo_hz", 0.0) << ','
           << dict_ref_double(meta, "residual_cfo_hz", 0.0) << ','
           << dict_ref_double(meta, "channel_coherence", 0.0) << ','
           << dict_ref_double(meta, "phase_slope", 0.0) << ','
           << dict_ref_double(meta, "phase_residual", 0.0) << ','
           << dict_ref_double(meta, "snr", 0.0) << ','
           << dict_ref_double(meta, "quality", 0.0) << '\n';
    d_file.flush();
}

} // namespace ofdm_prs_ranging
} // namespace gr
