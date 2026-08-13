/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_ACQUISITION_LOGGER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_ACQUISITION_LOGGER_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/ofdm_prs_ranging/prs_acquisition_logger.h>
#include <fstream>

namespace gr {
namespace ofdm_prs_ranging {

class prs_acquisition_logger_impl : public prs_acquisition_logger
{
public:
    prs_acquisition_logger_impl(const std::string& path,
                                const std::string& node,
                                bool append);
    ~prs_acquisition_logger_impl() override;

private:
    std::ofstream d_file;
    std::string d_node;
    void handle_frame(pmt::pmt_t msg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_ACQUISITION_LOGGER_IMPL_H */
