/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_MULTI_ANCHOR_CYCLE_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_MULTI_ANCHOR_CYCLE_H
#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>
#include <cstddef>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {
// Static 1..8-anchor ranging cycles. Delays/margins are seconds in the initiator
// UHD clock domain. window_after_s is the margin AFTER the complete response.
// epoch_out publishes a (dictionary, NIL) PDU only after normal completion of
// a cycle with all configured ranges and at least three anchors. Epoch arrays
// and full measurement metadata follow configured anchor_ids order.
class OFDM_PRS_RANGING_API prs_multi_anchor_cycle : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<prs_multi_anchor_cycle>;
    static sptr make(double samp_rate = 30e6,
                     const std::vector<int>& anchor_ids = {0, 1},
                     const std::vector<double>& time_delays = {0.031, 0.081},
                     int preamble_len = 128, int preamble_repeats = 17,
                     int coarse_sync_len = 819, double window_before_s = 0.002,
                     double window_after_s = 0.005, double processing_grace_s = 0.1);
    virtual size_t active_cycle_count() const = 0;
};
} // namespace ofdm_prs_ranging
} // namespace gr
#endif
