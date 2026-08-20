/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_ssrtt_solver_impl.h"
#include "prs_payload_codec.h"
#include "prs_receiver_utils.h"
#include <gnuradio/io_signature.h>
#include <cmath>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr double c_mps = 299792458.0;

double dict_ref_time_tuple(const pmt::pmt_t& dict, const char* key, double fallback)
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
} // namespace

prs_ssrtt_solver::sptr prs_ssrtt_solver::make(double samp_rate)
{
    return gnuradio::make_block_sptr<prs_ssrtt_solver_impl>(samp_rate);
}

prs_ssrtt_solver_impl::prs_ssrtt_solver_impl(double samp_rate)
    : gr::block("prs_ssrtt_solver",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_samp_rate(samp_rate)
{
    message_port_register_in(pmt::mp("tx_time_in"));
    message_port_register_in(pmt::mp("measurement_in"));
    message_port_register_out(pmt::mp("ssrtt_out"));
    set_msg_handler(pmt::mp("tx_time_in"),
                    [this](pmt::pmt_t msg) { handle_tx_time(msg); });
    set_msg_handler(pmt::mp("measurement_in"),
                    [this](pmt::pmt_t msg) { handle_measurement(msg); });
}

void prs_ssrtt_solver_impl::handle_tx_time(pmt::pmt_t msg)
{
    if (!pmt::is_dict(msg)) {
        return;
    }
    const uint64_t packet_type = dict_ref_uint64(msg, "packet_type", prs_packet_type_poll);
    if (packet_type != prs_packet_type_poll) {
        return;
    }
    const uint64_t poll_frame_id =
        dict_ref_uint64(msg, "poll_frame_id", dict_ref_uint64(msg, "frame_id", 0));
    const double t1 = dict_ref_double(msg, "tx_time_secs", 0.0) +
                      dict_ref_double(msg, "tx_time_frac", 0.0);
    d_poll_tx_times[poll_frame_id] = t1;
}

void prs_ssrtt_solver_impl::handle_measurement(pmt::pmt_t msg)
{
    if (!pmt::is_pair(msg)) {
        return;
    }
    pmt::pmt_t meta = pmt::car(msg);
    if (!pmt::to_bool(pmt::dict_ref(meta, pmt::mp("frame_id_valid"), pmt::PMT_F))) {
        return;
    }
    if (dict_ref_uint64(meta, "packet_type", 0) != prs_packet_type_response) {
        return;
    }

    const uint64_t poll_frame_id = dict_ref_uint64(meta, "poll_frame_id", 0);
    const auto found = d_poll_tx_times.find(poll_frame_id);
    if (found == d_poll_tx_times.end()) {
        return;
    }

    const double t1 = found->second;
    const double t4 = dict_ref_time_tuple(meta, "rx_time", NAN);
    if (!std::isfinite(t4)) {
        return;
    }

    const uint64_t reply_delay_samples = dict_ref_uint64(meta, "reply_delay_samples", 0);
    const double reply_delay_s = static_cast<double>(reply_delay_samples) / d_samp_rate;
    const double rtt_s = t4 - t1 - reply_delay_s;
    const double tof_s = (rtt_s-1.159952546762614e-05) / 2.0; // the processing delay of the response packet;
    const double range_m = tof_s * c_mps ;
    const double response_fine_delay_s = dict_ref_double(meta, "fine_delay", 0.0);
    const double response_phase_range_correction_m =
        0.5 * c_mps * response_fine_delay_s;

    meta = pmt::dict_add(meta, pmt::mp("t1_tx_time"), pmt::from_double(t1));
    meta = pmt::dict_add(meta, pmt::mp("t4_rx_time"), pmt::from_double(t4));
    meta = pmt::dict_add(meta, pmt::mp("reply_delay_s"), pmt::from_double(reply_delay_s));
    meta = pmt::dict_add(meta, pmt::mp("rtt_s"), pmt::from_double(rtt_s));
    meta = pmt::dict_add(meta, pmt::mp("tof_s"), pmt::from_double(tof_s));
    meta = pmt::dict_add(meta, pmt::mp("range_m"), pmt::from_double(range_m));
    meta = pmt::dict_add(meta, pmt::mp("integer_tof_s"), pmt::from_double(tof_s));
    meta = pmt::dict_add(
        meta, pmt::mp("integer_range_m"), pmt::from_double(range_m));
    meta = pmt::dict_add(meta,
                         pmt::mp("response_fine_delay_s"),
                         pmt::from_double(response_fine_delay_s));
    meta = pmt::dict_add(meta,
                         pmt::mp("response_phase_range_correction_m"),
                         pmt::from_double(response_phase_range_correction_m));

    message_port_pub(pmt::mp("ssrtt_out"), pmt::cons(meta, pmt::PMT_NIL));
}

} // namespace ofdm_prs_ranging
} // namespace gr
