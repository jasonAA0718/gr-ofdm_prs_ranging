/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_MULTI_ANCHOR_CYCLE_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_MULTI_ANCHOR_CYCLE_IMPL_H
#include <gnuradio/ofdm_prs_ranging/prs_multi_anchor_cycle.h>
#include <map>
#include <mutex>
#include <set>
#include <cstdint>

namespace gr {
namespace ofdm_prs_ranging {
class prs_multi_anchor_cycle_impl : public prs_multi_anchor_cycle
{
public:
    prs_multi_anchor_cycle_impl(double samp_rate, const std::vector<int>& anchor_ids,
        const std::vector<double>& time_delays, int preamble_len, int preamble_repeats,
        int coarse_sync_len, double window_before_s, double window_after_s,
        double processing_grace_s);
    size_t active_cycle_count() const override;
    bool start() override;
    bool stop() override;
private:
    struct slot { uint32_t anchor; uint32_t delay; double start; double end; };
    struct cycle {
        uint64_t sequence;
        uint64_t poll;
        double t1;
        double end;
        std::vector<slot> slots;
        std::set<uint32_t> pending;
        std::set<uint32_t> received;
        // Complete accepted metadata, keyed by decoded packet anchor identity.
        std::map<uint32_t, pmt::pmt_t> measurements;
        uint64_t duplicates = 0, identity_mismatches = 0, delay_mismatches = 0, crc_failures = 0;
    };
    double d_samp_rate, d_frame_s, d_before, d_after, d_grace;
    double d_clock = -1.0;
    double d_last_tx = -1.0;
    uint64_t d_sequence = 0;
    std::vector<uint32_t> d_anchors, d_delays;
    std::map<uint64_t, cycle> d_cycles;
    mutable std::mutex d_mutex;
    void handle_tx(pmt::pmt_t msg);
    void handle_clock(pmt::pmt_t msg);
    void handle_measurement(pmt::pmt_t msg);
    void handle_range(pmt::pmt_t msg);
    void finish(const cycle& c, const char* reason);
    void publish_epoch(const cycle& c);
    void diagnostic(pmt::pmt_t meta, const char* reason);
    cycle* locate(pmt::pmt_t meta);
};
} // namespace ofdm_prs_ranging
} // namespace gr
#endif
