/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_rx_timekeeper_impl.h"
#include <gnuradio/io_signature.h>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
double rx_time_tuple_value(const pmt::pmt_t& value)
{
    if (!pmt::is_tuple(value) || pmt::length(value) < 2) {
        return NAN;
    }
    const auto secs = pmt::tuple_ref(value, 0);
    const auto frac = pmt::tuple_ref(value, 1);
    if ((!pmt::is_uint64(secs) && !pmt::is_integer(secs)) || !pmt::is_real(frac)) {
        return NAN;
    }
    const double whole = pmt::is_uint64(secs)
                             ? static_cast<double>(pmt::to_uint64(secs))
                             : static_cast<double>(pmt::to_long(secs));
    return whole + pmt::to_double(frac);
}

bool has_explicit_tx_time(const pmt::pmt_t& msg)
{
    if (!pmt::is_dict(msg)) {
        return false;
    }
    return !pmt::eq(pmt::dict_ref(msg, pmt::mp("tx_time"), pmt::PMT_NIL),
                    pmt::PMT_NIL) ||
           !pmt::eq(pmt::dict_ref(msg, pmt::mp("tx_time_secs"), pmt::PMT_NIL),
                    pmt::PMT_NIL);
}
} // namespace

prs_rx_timekeeper::sptr prs_rx_timekeeper::make(double samp_rate, double tx_lead_time)
{
    return gnuradio::make_block_sptr<prs_rx_timekeeper_impl>(samp_rate, tx_lead_time);
}

prs_rx_timekeeper_impl::prs_rx_timekeeper_impl(double samp_rate, double tx_lead_time)
    : gr::block("prs_rx_timekeeper",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(0, 0, 0)),
      d_samp_rate(samp_rate),
      d_tx_lead_time(tx_lead_time),
      d_have_rx_time(false),
      d_rx_time_tag_offset(0),
      d_rx_time_value(0.0)
{
    if (d_samp_rate <= 0.0) {
        throw std::invalid_argument("samp_rate must be positive");
    }
    if (d_tx_lead_time < 0.0) {
        throw std::invalid_argument("tx_lead_time must be nonnegative");
    }

    message_port_register_in(pmt::mp("trigger_in"));
    message_port_register_out(pmt::mp("timed_trigger_out"));
    set_msg_handler(pmt::mp("trigger_in"),
                    [this](pmt::pmt_t msg) { handle_trigger(std::move(msg)); });
}

void prs_rx_timekeeper_impl::forecast(int noutput_items,
                                      gr_vector_int& ninput_items_required)
{
    (void)noutput_items;
    ninput_items_required[0] = 1;
}

void prs_rx_timekeeper_impl::handle_trigger(pmt::pmt_t msg)
{
    std::lock_guard<std::mutex> lock(d_pending_mutex);
    d_pending_triggers.push_back(std::move(msg));
}

void prs_rx_timekeeper_impl::update_rx_time_tags(uint64_t abs_start, uint64_t abs_stop)
{
    std::vector<tag_t> tags;
    get_tags_in_range(tags, 0, abs_start, abs_stop, pmt::mp("rx_time"));
    for (const auto& tag : tags) {
        const double value = rx_time_tuple_value(tag.value);
        if (std::isfinite(value)) {
            d_rx_time_tag_offset = tag.offset;
            d_rx_time_value = value;
            d_have_rx_time = true;
        }
    }
}

void prs_rx_timekeeper_impl::publish_pending(uint64_t current_abs_offset)
{
    std::deque<pmt::pmt_t> pending;
    {
        std::lock_guard<std::mutex> lock(d_pending_mutex);
        if (!d_have_rx_time) {
            return;
        }
        pending.swap(d_pending_triggers);
    }

    const double current_time =
        d_rx_time_value +
        static_cast<double>(current_abs_offset - d_rx_time_tag_offset) / d_samp_rate;
    for (auto& request : pending) {
        pmt::pmt_t trigger = pmt::is_dict(request) ? request : pmt::make_dict();
        if (!has_explicit_tx_time(trigger)) {
            const double tx_time = current_time + d_tx_lead_time;
            const double secs_floor = std::floor(tx_time);
            trigger = pmt::dict_add(
                trigger,
                pmt::mp("tx_time_secs"),
                pmt::from_uint64(static_cast<uint64_t>(secs_floor)));
            trigger = pmt::dict_add(trigger,
                                    pmt::mp("tx_time_frac"),
                                    pmt::from_double(tx_time - secs_floor));
        }
        trigger = pmt::dict_add(trigger,
                                pmt::mp("rx_time_tag_offset"),
                                pmt::from_uint64(d_rx_time_tag_offset));
        trigger = pmt::dict_add(trigger,
                                pmt::mp("timekeeper_sample_offset"),
                                pmt::from_uint64(current_abs_offset));
        message_port_pub(pmt::mp("timed_trigger_out"), trigger);
    }
}

int prs_rx_timekeeper_impl::general_work(int noutput_items,
                                         gr_vector_int& ninput_items,
                                         gr_vector_const_void_star& input_items,
                                         gr_vector_void_star& output_items)
{
    (void)noutput_items;
    (void)input_items;
    (void)output_items;
    const int ninput = ninput_items[0];
    const uint64_t abs_start = nitems_read(0);
    const uint64_t abs_stop = abs_start + static_cast<uint64_t>(ninput);

    update_rx_time_tags(abs_start, abs_stop);
    publish_pending(abs_stop);
    consume_each(ninput);
    return 0;
}

} // namespace ofdm_prs_ranging
} // namespace gr
