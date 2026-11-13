// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_CSV_LOGGER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_POSITION_CSV_LOGGER_IMPL_H

#include <gnuradio/ofdm_prs_ranging/prs_position_csv_logger.h>
#include <fstream>
#include <mutex>

namespace gr {
namespace ofdm_prs_ranging {

class prs_position_csv_logger_impl : public prs_position_csv_logger
{
public:
    prs_position_csv_logger_impl(const std::string& path, bool append);

private:
    std::ofstream d_file;
    std::mutex d_mutex;
    void handle_result(pmt::pmt_t msg, bool success);
};

} // namespace ofdm_prs_ranging
} // namespace gr
#endif
