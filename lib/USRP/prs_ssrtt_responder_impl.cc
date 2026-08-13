/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_ssrtt_responder_impl.h"
#include "prs_payload_codec.h"
#include "prs_receiver_utils.h"
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
                                                    uint32_t reply_delay_samples)
{
    return gnuradio::make_block_sptr<prs_ssrtt_responder_impl>(samp_rate,
                                                               reply_delay_samples);
}

prs_ssrtt_responder_impl::prs_ssrtt_responder_impl(double samp_rate,
                                                   uint32_t reply_delay_samples)
    : gr::block("prs_ssrtt_responder",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_samp_rate(samp_rate),
      d_reply_delay_samples(reply_delay_samples)
{
    message_port_register_in(pmt::mp("measurement_in"));
    message_port_register_out(pmt::mp("trigger_out"));
    set_msg_handler(pmt::mp("measurement_in"),
                    [this](pmt::pmt_t msg) { handle_measurement(msg); });
}

void prs_ssrtt_responder_impl::handle_measurement(pmt::pmt_t msg)
{
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

    const uint64_t poll_frame_id = dict_ref_uint64(meta, "poll_frame_id", dict_ref_uint64(meta, "frame_id", 0));
    const double reply_delay_s = static_cast<double>(d_reply_delay_samples) / d_samp_rate;
    const double response_tx_time = poll_rx_time + reply_delay_s;
    const double secs_floor = std::floor(response_tx_time);

    pmt::pmt_t trigger = pmt::make_dict();
    trigger = pmt::dict_add(trigger, pmt::mp("packet_type"), pmt::from_long(prs_packet_type_response));
    trigger = pmt::dict_add(trigger, pmt::mp("poll_frame_id"), pmt::from_uint64(poll_frame_id));
    trigger = pmt::dict_add(trigger,
                            pmt::mp("reply_delay_samples"),
                            pmt::from_uint64(d_reply_delay_samples));
    trigger = pmt::dict_add(trigger,
                            pmt::mp("tx_time_secs"),
                            pmt::from_uint64(static_cast<uint64_t>(secs_floor)));
    trigger = pmt::dict_add(trigger,
                            pmt::mp("tx_time_frac"),
                            pmt::from_double(response_tx_time - secs_floor));
    message_port_pub(pmt::mp("trigger_out"), trigger);
}

} // namespace ofdm_prs_ranging
} // namespace gr
