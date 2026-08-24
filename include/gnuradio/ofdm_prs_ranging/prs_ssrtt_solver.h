/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_SOLVER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_SOLVER_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_ssrtt_solver : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_ssrtt_solver> sptr;
    static sptr make(double samp_rate = 10e6, bool enable_profiling = false);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_SOLVER_H */
