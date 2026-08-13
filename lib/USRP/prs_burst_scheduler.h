/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_BURST_SCHEDULER_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_BURST_SCHEDULER_H

#include <cstdint>
#include <deque>

namespace gr {
namespace ofdm_prs_ranging {

struct prs_pending_burst {
    uint64_t frame_id;
    double tx_time;
    double sequence_delay;
    uint8_t packet_type;
    uint32_t poll_frame_id;
    uint32_t response_frame_id;
    uint32_t reply_delay_samples;
};

class prs_burst_scheduler
{
public:
    void queue_trigger(int count, double base_tx_time, double burst_period);
    uint64_t allocate_frame_id();
    void queue_burst(const prs_pending_burst& burst);
    bool empty() const;
    const prs_pending_burst& front() const;
    prs_pending_burst pop_front();

private:
    uint64_t d_next_frame_id = 0;
    std::deque<prs_pending_burst> d_pending;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_PRS_BURST_SCHEDULER_H */
