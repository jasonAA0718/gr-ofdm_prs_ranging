/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_RESPONDER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_RESPONDER_IMPL_H

#include <gnuradio/ofdm_prs_ranging/pluto_prs_responder.h>
#include <pmt/pmt.h>

namespace gr {
namespace ofdm_prs_ranging {

class pluto_prs_responder_impl : public pluto_prs_responder
{
private:
    uint64_t d_guard_samples;
    void handle_frame(pmt::pmt_t msg);

public:
    explicit pluto_prs_responder_impl(uint64_t guard_samples);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_RESPONDER_IMPL_H */
