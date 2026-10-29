/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_SOLVER
#define INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_SOLVER

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>
#include <memory>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_position_solver : virtual public gr::block
{
public:
    using sptr =  std::shared_ptr<prs_position_solver> ;
    static sptr make(
        const std::vector<int>& anchor_ids,
        const std::vector<std::vector<double>>& anchor_positions_m,
        double vehicle_height_m);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_FRAME_DETECTOR_H */
