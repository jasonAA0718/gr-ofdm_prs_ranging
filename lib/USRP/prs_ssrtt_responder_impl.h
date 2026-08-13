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

namespace gr {
namespace ofdm_prs_ranging {

class prs_ssrtt_responder_impl : public prs_ssrtt_responder
{
public:
    prs_ssrtt_responder_impl(double samp_rate, uint32_t reply_delay_samples);

private:
    double d_samp_rate;
    uint32_t d_reply_delay_samples;

    void handle_measurement(pmt::pmt_t msg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_RESPONDER_IMPL_H */
