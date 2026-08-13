/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_RX_TIMEKEEPER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_RX_TIMEKEEPER_IMPL_H

#include <gnuradio/ofdm_prs_ranging/prs_rx_timekeeper.h>
#include <deque>
#include <mutex>

namespace gr {
namespace ofdm_prs_ranging {

class prs_rx_timekeeper_impl : public prs_rx_timekeeper
{
public:
    prs_rx_timekeeper_impl(double samp_rate, double tx_lead_time);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;

private:
    double d_samp_rate;
    double d_tx_lead_time;
    bool d_have_rx_time;
    uint64_t d_rx_time_tag_offset;
    double d_rx_time_value;
    std::mutex d_pending_mutex;
    std::deque<pmt::pmt_t> d_pending_triggers;

    void handle_trigger(pmt::pmt_t msg);
    void update_rx_time_tags(uint64_t abs_start, uint64_t abs_stop);
    void publish_pending(uint64_t current_abs_offset);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_RX_TIMEKEEPER_IMPL_H */
