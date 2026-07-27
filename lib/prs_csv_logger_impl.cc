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
        d_file << "poll_frame_id,response_frame_id,t1_tx_time,t4_rx_time,reply_delay_samples,reply_delay_s,rtt_s,tof_s,range_m,frame_id_valid,coarse_metric,payload_metric,phase_residual,snr,quality\n";
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
    d_file << std::setprecision(15);
    d_file << dict_ref_uint64(meta, "poll_frame_id", 0) << ','
           << dict_ref_uint64(meta, "response_frame_id", 0) << ','
           << dict_ref_double(meta, "t1_tx_time", 0.0) << ','
           << dict_ref_double(meta, "t4_rx_time", 0.0) << ','
           << dict_ref_uint64(meta, "reply_delay_samples", 0) << ','
           << dict_ref_double(meta, "reply_delay_s", 0.0) << ','
           << dict_ref_double(meta, "rtt_s", 0.0) << ','
           << dict_ref_double(meta, "tof_s", 0.0) << ','
           << dict_ref_double(meta, "range_m", 0.0) << ','
           << (pmt::to_bool(pmt::dict_ref(meta, pmt::mp("frame_id_valid"), pmt::PMT_F)) ? 1 : 0) << ','
           << dict_ref_double(
                  meta, "coarse_metric", dict_ref_double(meta, "peak_metric", 0.0))
           << ','
           << dict_ref_double(meta, "payload_metric", 0.0) << ','
           << dict_ref_double(meta, "phase_residual", 0.0) << ','
           << dict_ref_double(meta, "snr", 0.0) << ','
           << dict_ref_double(meta, "quality", 0.0) << '\n';
    d_file.flush();
}

} // namespace ofdm_prs_ranging
} // namespace gr
