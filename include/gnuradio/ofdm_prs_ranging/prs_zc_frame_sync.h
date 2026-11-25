/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

/*! \brief Message-only CFO-assisted ZC synchronization and frame extraction. */
class OFDM_PRS_RANGING_API prs_zc_frame_sync : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<prs_zc_frame_sync>;

    static sptr make(double samp_rate = 30e6,
                     float preamble_threshold = 0.09f,
                     float zc_threshold = 0.35f,
                     bool expect_response = false,
                     int channel_id = 0);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_ZC_FRAME_SYNC_H */
