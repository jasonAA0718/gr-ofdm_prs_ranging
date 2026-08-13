/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_burst_scheduler.h"
#include "prs_payload_codec.h"

namespace gr {
namespace ofdm_prs_ranging {

void prs_burst_scheduler::queue_trigger(int count, double base_tx_time, double burst_period)
{
    for (int i = 0; i < count; ++i) {
        const uint64_t frame_id = allocate_frame_id();
        const double sequence_delay = i * burst_period;
        d_pending.push_back(prs_pending_burst{ frame_id,
                                               base_tx_time + sequence_delay,
                                               sequence_delay,
                                               prs_packet_type_poll,
                                               static_cast<uint32_t>(frame_id),
                                               0,
                                               0 });
    }
}

uint64_t prs_burst_scheduler::allocate_frame_id() { return d_next_frame_id++; }

void prs_burst_scheduler::queue_burst(const prs_pending_burst& burst)
{
    d_pending.push_back(burst);
}

bool prs_burst_scheduler::empty() const { return d_pending.empty(); }

const prs_pending_burst& prs_burst_scheduler::front() const { return d_pending.front(); }

prs_pending_burst prs_burst_scheduler::pop_front()
{
    const auto burst = d_pending.front();
    d_pending.pop_front();
    return burst;
}

} // namespace ofdm_prs_ranging
} // namespace gr
