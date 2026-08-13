/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "pluto_prs_responder_impl.h"
#include "prs_payload_codec.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace ofdm_prs_ranging {

pluto_prs_responder::sptr pluto_prs_responder::make(uint64_t guard_samples)
{
    return gnuradio::make_block_sptr<pluto_prs_responder_impl>(guard_samples);
}

pluto_prs_responder_impl::pluto_prs_responder_impl(uint64_t guard_samples)
    : gr::block("pluto_prs_responder",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_guard_samples(guard_samples)
{
    message_port_register_in(pmt::mp("frame_in"));
    message_port_register_out(pmt::mp("trigger_out"));
    set_msg_handler(pmt::mp("frame_in"),
                    [this](pmt::pmt_t msg) { handle_frame(std::move(msg)); });
}

void pluto_prs_responder_impl::handle_frame(pmt::pmt_t msg)
{
    const auto meta = pmt::is_pair(msg) ? pmt::car(msg) : msg;
    if (!pmt::is_dict(meta)) {
        return;
    }

    const auto valid = pmt::dict_ref(meta, pmt::mp("frame_id_valid"), pmt::PMT_F);
    const auto packet = pmt::dict_ref(meta, pmt::mp("packet_type"), pmt::from_long(0));
    if (!pmt::is_true(valid) || !pmt::is_integer(packet) ||
        pmt::to_long(packet) != prs_packet_type_poll) {
        return;
    }

    const auto poll = pmt::dict_ref(meta, pmt::mp("poll_frame_id"), pmt::PMT_NIL);
    if (!pmt::is_uint64(poll) && !pmt::is_integer(poll)) {
        return;
    }

    pmt::pmt_t trigger = pmt::make_dict();
    trigger = pmt::dict_add(
        trigger, pmt::mp("packet_type"), pmt::from_long(prs_packet_type_response));
    trigger = pmt::dict_add(trigger, pmt::mp("poll_frame_id"), poll);
    trigger = pmt::dict_add(trigger, pmt::mp("reply_delay_samples"), pmt::from_uint64(0));
    trigger = pmt::dict_add(
        trigger, pmt::mp("guard_samples"), pmt::from_uint64(d_guard_samples));
    trigger = pmt::dict_add(trigger, pmt::mp("untimed_response"), pmt::PMT_T);
    message_port_pub(pmt::mp("trigger_out"), trigger);
}

} // namespace ofdm_prs_ranging
} // namespace gr
