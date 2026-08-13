/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_RESPONDER_H
#define INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_RESPONDER_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API pluto_prs_responder : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<pluto_prs_responder>;
    static sptr make(uint64_t guard_samples = 0);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PLUTO_PRS_RESPONDER_H */
