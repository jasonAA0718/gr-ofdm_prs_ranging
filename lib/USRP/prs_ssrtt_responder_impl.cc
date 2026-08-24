/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_payload_codec.h"
#include "prs_receiver_utils.h"
#include "prs_ssrtt_responder_impl.h"
#include "prs_timing_helper.h"
#include <gnuradio/io_signature.h>
#include <cmath>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
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

prs_ssrtt_responder::sptr prs_ssrtt_responder::make(double samp_rate,
                                                    uint32_t reply_delay_samples,
                                                    bool enable_profiling)
{
    return gnuradio::make_block_sptr<prs_ssrtt_responder_impl>(
        samp_rate, reply_delay_samples, enable_profiling);
}

prs_ssrtt_responder_impl::prs_ssrtt_responder_impl(double samp_rate,
                                                   uint32_t reply_delay_samples,
                                                   bool enable_profiling)
    : gr::block("prs_ssrtt_responder",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_samp_rate(samp_rate),
      d_reply_delay_samples(reply_delay_samples),
      d_enable_profiling(enable_profiling)
{
    message_port_register_in(pmt::mp("measurement_in"));
    message_port_register_out(pmt::mp("trigger_out"));
    message_port_register_out(pmt::mp("timing_out"));
    set_msg_handler(pmt::mp("measurement_in"),
                    [this](pmt::pmt_t msg) { handle_measurement(msg); });
}

void prs_ssrtt_responder_impl::handle_measurement(pmt::pmt_t msg)
{
    profiling::timing_report report(d_enable_profiling, "ssrtt_responder");
    if (!pmt::is_pair(msg)) {
        return;
    }
    const auto meta = pmt::car(msg);
    if (!pmt::to_bool(pmt::dict_ref(meta, pmt::mp("frame_id_valid"), pmt::PMT_F))) {
        return;
    }
    if (dict_ref_uint64(meta, "packet_type", 0) != prs_packet_type_poll) {
        return;
    }

    const double poll_rx_time = dict_ref_time_tuple(meta, "rx_time", NAN);
    if (!std::isfinite(poll_rx_time)) {
        return;
    }

    const uint64_t poll_frame_id =
        dict_ref_uint64(meta, "poll_frame_id", dict_ref_uint64(meta, "frame_id", 0));
    const double reply_delay_s = static_cast<double>(d_reply_delay_samples) / d_samp_rate;
    const double response_tx_time = poll_rx_time + reply_delay_s;
    const double secs_floor = std::floor(response_tx_time);
    report.checkpoint("validate_and_schedule");

    auto stage_start = report.mark();
    pmt::pmt_t trigger = pmt::make_dict();
    trigger = pmt::dict_add(
        trigger, pmt::mp("packet_type"), pmt::from_long(prs_packet_type_response));
    trigger =
        pmt::dict_add(trigger, pmt::mp("poll_frame_id"), pmt::from_uint64(poll_frame_id));
    trigger = pmt::dict_add(
        trigger, pmt::mp("reply_delay_samples"), pmt::from_uint64(d_reply_delay_samples));
    trigger = pmt::dict_add(trigger,
                            pmt::mp("tx_time_secs"),
                            pmt::from_uint64(static_cast<uint64_t>(secs_floor)));
    trigger = pmt::dict_add(trigger,
                            pmt::mp("tx_time_frac"),
                            pmt::from_double(response_tx_time - secs_floor));
    auto stage_stop = report.mark();
    report.add("trigger_build", stage_start, stage_stop);
    stage_start = stage_stop;
    message_port_pub(pmt::mp("trigger_out"), trigger);
    stage_stop = report.mark();
    report.add("trigger_publish", stage_start, stage_stop);
    if (report.enabled()) {
        message_port_pub(pmt::mp("timing_out"), report.finish(meta));
    }
}

} // namespace ofdm_prs_ranging
} // namespace gr
