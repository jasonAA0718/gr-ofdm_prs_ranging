/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_SOLVER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_SOLVER_IMPL_H

#include <gnuradio/ofdm_prs_ranging/prs_ssrtt_solver.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <unordered_map>

namespace gr {
namespace ofdm_prs_ranging {

class prs_ssrtt_solver_impl : public prs_ssrtt_solver
{
public:
    explicit prs_ssrtt_solver_impl(double samp_rate);

private:
    double d_samp_rate;
    std::unordered_map<uint64_t, double> d_poll_tx_times;

    void handle_tx_time(pmt::pmt_t msg);
    void handle_measurement(pmt::pmt_t msg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_SSRTT_SOLVER_IMPL_H */
