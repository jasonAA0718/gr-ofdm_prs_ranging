/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_ACQUISITION_LOGGER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_ACQUISITION_LOGGER_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API prs_acquisition_logger : virtual public gr::block
{
public:
    typedef std::shared_ptr<prs_acquisition_logger> sptr;
    static sptr make(const std::string& path = "prs_acquisition.csv",
                     const std::string& node = "rx",
                     bool append = true);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_ACQUISITION_LOGGER_H */
