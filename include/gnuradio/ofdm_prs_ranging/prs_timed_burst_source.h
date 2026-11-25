/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_TIMED_BURST_SOURCE_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_TIMED_BURST_SOURCE_H

#include <gnuradio/block.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

/*!
 * \brief Deterministic OFDM / PRS-like timed burst source.
 * \ingroup ofdm_prs_ranging
 *
 * The optional stream input supports legacy RX-clock tracking. Triggers
 * carrying explicit tx_time fields require no stream input.
 */
class OFDM_PRS_RANGING_API prs_timed_burst_source : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_timed_burst_source> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of
     * ofdm_prs_ranging::prs_timed_burst_source.
     *
     * To avoid accidental use of raw pointers, ofdm_prs_ranging::prs_timed_burst_source's
     * constructor is in a private implementation
     * class. ofdm_prs_ranging::prs_timed_burst_source::make is the public interface for
     * creating new instances.
     */
    static sptr make(double samp_rate = 10e6,
                     double tx_lead_time = 0.5,
                     double burst_period = 0.1,
                     float tx_amp = 0.6f,
                     int pings_per_trigger = 1,
                     bool attach_tx_time = true);

    virtual int frame_len() const = 0;
    virtual int prs_start() const = 0;
    virtual int prs_len() const = 0;
    virtual std::vector<gr_complex> frame_samples() const = 0;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_TIMED_BURST_SOURCE_H */
