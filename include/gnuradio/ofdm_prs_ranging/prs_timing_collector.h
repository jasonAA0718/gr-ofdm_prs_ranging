/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_COLLECTOR_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_COLLECTOR_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>
#include <memory>
#include <string>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_timing_collector : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<prs_timing_collector>;

    static sptr make(const std::string& raw_csv_path = "prs_timing_raw.csv",
                     const std::string& summary_csv_path = "prs_timing_summary.csv",
                     const std::string& role = "unknown");
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_COLLECTOR_H */
