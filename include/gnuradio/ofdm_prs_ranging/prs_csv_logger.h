/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_CSV_LOGGER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_CSV_LOGGER_H

#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_csv_logger : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_csv_logger> sptr;
    static sptr make(const std::string& path = "prs_measurements.csv", bool append = true);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_CSV_LOGGER_H */
