/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_RESPONDER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_RESPONDER_IMPL_H

#include <gnuradio/ofdm_prs_ranging/prs_ssrtt_responder.h>
#include <pmt/pmt.h>
#include <fstream>
#include <optional>
#include <unordered_map>

namespace gr {
namespace ofdm_prs_ranging {

class prs_ssrtt_responder_impl : public prs_ssrtt_responder
{
public:
    prs_ssrtt_responder_impl(double samp_rate,
                             uint64_t reply_delay_samples,
                             uint64_t anchor_id,
                             double poll_period_s,
                             const std::string& poll_tracking_csv,
                             double poll_window_margin_s);

private:
    double d_samp_rate;
    uint64_t d_reply_delay_samples;
    uint64_t d_anchor_id;
    std::unordered_map<uint64_t, double> d_recent_polls;
    double d_latest_poll_time = -1.0;

    struct poll_timing {
        uint64_t id;
        long double rx_time;
        long double period_s;
    };
    double d_poll_period_s;
    double d_poll_window_margin_s;
    std::ofstream d_poll_log;
    std::optional<poll_timing> d_poll_timing;

    void handle_measurement(pmt::pmt_t msg);
    void publish_response(pmt::pmt_t trigger,
                          const pmt::pmt_t& meta,
                          uint64_t poll_id,
                          long double rx_time);
    pmt::pmt_t add_poll_window(pmt::pmt_t trigger,
                               const pmt::pmt_t& meta,
                               uint64_t poll_id,
                               long double period) const;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_RESPONDER_IMPL_H */
