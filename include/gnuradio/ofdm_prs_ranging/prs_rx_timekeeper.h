/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_RX_TIMEKEEPER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_RX_TIMEKEEPER_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

/*!
 * \brief Stamp trigger messages with time derived from UHD rx_time tags.
 * \ingroup ofdm_prs_ranging
 */
class OFDM_PRS_RANGING_API prs_rx_timekeeper : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<prs_rx_timekeeper>;

    static sptr make(double samp_rate = 10e6, double tx_lead_time = 0.5);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_RX_TIMEKEEPER_H */
