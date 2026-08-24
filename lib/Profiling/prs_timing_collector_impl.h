/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_COLLECTOR_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_COLLECTOR_IMPL_H

#include <gnuradio/ofdm_prs_ranging/prs_timing_collector.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

class prs_timing_collector_impl : public prs_timing_collector
{
public:
    prs_timing_collector_impl(std::string raw_csv_path,
                              std::string summary_csv_path,
                              std::string role);
    ~prs_timing_collector_impl() override;

    bool stop() override;

private:
    struct timing_row {
        std::string block;
        std::string stage;
        uint64_t attempt_id;
        uint64_t poll_frame_id;
        uint64_t response_frame_id;
        uint64_t frame_id;
        uint64_t packet_type;
        bool valid;
        uint64_t handler_start_ns;
        uint64_t handler_end_ns;
        uint64_t duration_ns;
    };

    std::string d_raw_csv_path;
    std::string d_summary_csv_path;
    std::string d_role;
    std::mutex d_mutex;
    std::vector<timing_row> d_rows;

    void handle_timing(const pmt::pmt_t& message);
    void finalize();
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_TIMING_COLLECTOR_IMPL_H */
