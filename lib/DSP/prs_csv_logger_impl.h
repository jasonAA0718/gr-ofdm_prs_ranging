/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_CSV_LOGGER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_CSV_LOGGER_IMPL_H

#include "prs_receiver_utils.h"
#include <gnuradio/ofdm_prs_ranging/prs_csv_logger.h>
#include <fstream>

namespace gr {
namespace ofdm_prs_ranging {

class prs_csv_logger_impl : public prs_csv_logger
{
public:
    prs_csv_logger_impl(const std::string& path, bool append);
    ~prs_csv_logger_impl() override;

private:
    std::ofstream d_file;
    void handle_measurement(pmt::pmt_t msg);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_CSV_LOGGER_IMPL_H */
