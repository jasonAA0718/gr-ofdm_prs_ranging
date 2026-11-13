// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_CSV_LOGGER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_CSV_LOGGER_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>
#include <string>

namespace gr {
namespace ofdm_prs_ranging {

//! One CSV row per position_out or diagnostic_out PDU from prs_position_solver.
class OFDM_PRS_RANGING_API prs_position_csv_logger : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<prs_position_csv_logger>;
    static sptr make(const std::string& path = "prs_positions.csv", bool append = true);
};

} // namespace ofdm_prs_ranging
} // namespace gr
#endif
