/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "prs_multi_anchor_cycle_impl.h"
#include "prs_payload_codec.h"
#include "prs_protocol_metadata.h"
#include "prs_receiver_utils.h"
#include "prs_phy_geometry.h"
#include "mc_ds_prs.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {
namespace {
double receive_time(pmt::pmt_t meta)
{
    const auto value = pmt::dict_ref(meta, pmt::mp("rx_time"), pmt::PMT_NIL);
    if (!pmt::is_tuple(value) || pmt::length(value) != 2) return NAN;
    const auto secs = pmt::tuple_ref(value, 0);
    const auto frac = pmt::tuple_ref(value, 1);
    if ((!pmt::is_uint64(secs) && !pmt::is_integer(secs)) || !pmt::is_real(frac)) return NAN;
    return (pmt::is_uint64(secs) ? static_cast<double>(pmt::to_uint64(secs))
                               : static_cast<double>(pmt::to_long(secs))) + pmt::to_double(frac);
}
pmt::pmt_t metadata(pmt::pmt_t msg)
{
    return pmt::is_dict(msg) ? msg : (pmt::is_pair(msg) ? pmt::car(msg) : pmt::PMT_NIL);
}
}
prs_multi_anchor_cycle::sptr prs_multi_anchor_cycle::make(double fs,
    const std::vector<int>& ids, const std::vector<double>& delays, int l, int r, int nz,
    double before, double after, double grace)
{
    return gnuradio::make_block_sptr<prs_multi_anchor_cycle_impl>(
        fs, ids, delays, l, r, nz, before, after, grace);
}
prs_multi_anchor_cycle_impl::prs_multi_anchor_cycle_impl(double fs,
    const std::vector<int>& ids, const std::vector<double>& delays, int l, int r, int nz,
    double before, double after, double grace)
    : gr::block("prs_multi_anchor_cycle", gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)), d_samp_rate(fs),
      d_frame_s(0), d_before(before), d_after(after), d_grace(grace)
{
    if (!std::isfinite(fs) || fs <= 0 || ids.size() != 2 || ids.size() != delays.size() ||
        l <= 0 || r < 2 || nz <= 0 || !std::isfinite(before) || before < 0 ||
        !std::isfinite(after) || after < 0 || !std::isfinite(grace) || grace < 0) {
        throw std::invalid_argument("two anchors and finite positive geometry/delays required");
    }
    d_frame_s = phy::frame_samples(l, r, nz, mc_ds_fft_len, mc_ds_cp_len, mc_ds_symbol_count) / fs;
    std::set<int> unique;
    for (size_t i = 0; i < ids.size(); ++i) {
        const double raw = std::round(delays[i] * fs);
        if (ids[i] < 0 || ids[i] > 7 || !unique.insert(ids[i]).second ||
            !std::isfinite(delays[i]) || delays[i] <= 0 || !std::isfinite(raw) ||
            raw < 1 || raw > 0xffffff || raw / fs <= before) {
            throw std::invalid_argument("invalid anchor identity or reply delay");
        }
        d_anchors.push_back(ids[i]);
        d_delays.push_back(static_cast<uint32_t>(raw));
    }
    std::vector<double> starts;
    for (auto delay : d_delays) starts.push_back(delay / fs);
    std::sort(starts.begin(), starts.end());
    for (size_t i = 1; i < starts.size(); ++i) {
        if (starts[i] - before <= starts[i - 1] + d_frame_s + after) {
            throw std::invalid_argument("complete response receive windows must not overlap");
        }
    }
    for (const auto* port : {"tx_time_in", "clock_in", "measurement_in", "range_in"})
        message_port_register_in(pmt::mp(port));
    for (const auto* port : {"window_out", "solver_tx_out", "solver_measurement_out",
                             "measurement_out", "cycle_out", "diagnostic_out"})
        message_port_register_out(pmt::mp(port));
    set_msg_handler(pmt::mp("tx_time_in"), [this](pmt::pmt_t m) { handle_tx(m); });
    set_msg_handler(pmt::mp("clock_in"), [this](pmt::pmt_t m) { handle_clock(m); });
    set_msg_handler(pmt::mp("measurement_in"), [this](pmt::pmt_t m) { handle_measurement(m); });
    set_msg_handler(pmt::mp("range_in"), [this](pmt::pmt_t m) { handle_range(m); });
}
size_t prs_multi_anchor_cycle_impl::active_cycle_count() const
{
    std::lock_guard<std::mutex> lock(d_mutex);
    return d_cycles.size();
}
bool prs_multi_anchor_cycle_impl::start()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_clock = -1.0;
    d_last_tx = -1.0;
    d_cycles.clear();
    // Sequence stays monotonic across stop/start of this block instance.
    return true;
}
void prs_multi_anchor_cycle_impl::diagnostic(pmt::pmt_t meta, const char* reason)
{
    meta = pmt::dict_add(meta, pmt::mp("cycle_diagnostic"), pmt::mp(reason));
    message_port_pub(pmt::mp("diagnostic_out"), pmt::cons(meta, pmt::PMT_NIL));
}
void prs_multi_anchor_cycle_impl::handle_tx(pmt::pmt_t msg)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    const auto meta = metadata(msg);
    if (!pmt::is_dict(meta) || dict_ref_uint64(meta, "packet_type", 0) != prs_packet_type_poll) return;
    uint64_t poll = 0;
    if (!bounded_uint(meta, "poll_id", 65536, 65535, poll)) return;
    const double t1 = dict_ref_double(meta, "tx_time_secs", NAN) + dict_ref_double(meta, "tx_time_frac", 0);
    if (!std::isfinite(t1) || t1 < 0) return;
    if (t1 <= d_last_tx || d_cycles.size() >= 16) { diagnostic(meta, "TX_ORDER_OR_CAPACITY"); return; }
    cycle c{};
    c.sequence = ++d_sequence; c.poll = poll; c.t1 = t1; c.end = t1;
    for (size_t i = 0; i < d_anchors.size(); ++i) {
        const double expected = t1 + d_delays[i] / d_samp_rate;
        c.slots.push_back({d_anchors[i], d_delays[i], expected - d_before,
                           expected + d_frame_s + d_after});
        c.end = std::max(c.end, c.slots.back().end);
    }
    for (const auto& entry : d_cycles) {
        if (entry.second.poll == poll || std::min_element(c.slots.begin(), c.slots.end(), [](const slot& a, const slot& b) { return a.start < b.start; })->start < entry.second.end) {
            diagnostic(meta, "ACTIVE_CYCLE_OVERLAP_OR_POLL_REUSE"); return;
        }
    }
    if (d_clock >= c.end + d_grace) { diagnostic(meta, "LATE_TX_NOTIFICATION"); return; }
    d_last_tx = t1;
    d_cycles.emplace(c.sequence, c);
    auto tx = pmt::dict_add(meta, pmt::mp("cycle_sequence"), pmt::from_uint64(c.sequence));
    tx = pmt::dict_add(tx, pmt::mp("cycle_managed"), pmt::PMT_T);
    message_port_pub(pmt::mp("solver_tx_out"), tx);
    for (size_t i = 0; i < c.slots.size(); ++i) {
        const auto& slot = c.slots[i];
        auto window = tx;
        window = pmt::dict_add(window, pmt::mp("window_start"), pmt::from_double(slot.start));
        window = pmt::dict_add(window, pmt::mp("window_end"), pmt::from_double(slot.end));
        window = pmt::dict_add(window, pmt::mp("expected_anchor_id"), pmt::from_uint64(slot.anchor));
        window = pmt::dict_add(window, pmt::mp("response_slot_index"), pmt::from_uint64(i));
        window = pmt::dict_add(window, pmt::mp("expected_reply_delay_samples"), pmt::from_uint64(slot.delay));
        message_port_pub(pmt::mp("window_out"), window);
    }
}
prs_multi_anchor_cycle_impl::cycle* prs_multi_anchor_cycle_impl::locate(pmt::pmt_t meta)
{
    const uint64_t seq = dict_ref_uint64(meta, "cycle_sequence", 0);
    const auto found = d_cycles.find(seq);
    if (found == d_cycles.end() || dict_ref_uint64(meta, "poll_id", 65536) != found->second.poll)
        return nullptr;
    return &found->second;
}
void prs_multi_anchor_cycle_impl::handle_measurement(pmt::pmt_t msg)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    auto meta = metadata(msg);
    if (!pmt::is_dict(meta)) return;
    // CRC-failed decoded IDs are untrusted: only count by local cycle sequence.
    if (!metadata_true(meta, "frame_id_valid")) {
        auto found = d_cycles.find(dict_ref_uint64(meta, "cycle_sequence", 0));
        if (found != d_cycles.end()) ++found->second.crc_failures;
        diagnostic(meta, "INVALID_PAYLOAD"); return;
    }
    if (dict_ref_uint64(meta, "packet_type", 0) != prs_packet_type_response) return;
    auto* c = locate(meta);
    if (!c) { diagnostic(meta, "STALE_OR_UNKNOWN_CYCLE"); return; }
    const auto index = dict_ref_uint64(meta, "response_slot_index", c->slots.size());
    const double t4 = receive_time(meta);
    if (index >= c->slots.size() || !std::isfinite(t4) || t4 < c->slots[index].start ||
        t4 + d_frame_s > c->slots[index].end + 1e-9) {
        diagnostic(meta, "OUTSIDE_RESPONSE_WINDOW"); return;
    }
    const auto& slot = c->slots[index];
    const uint64_t anchor = dict_ref_uint64(meta, "anchor_id", 8);
    meta = pmt::dict_add(meta, pmt::mp("expected_anchor_id"), pmt::from_uint64(slot.anchor));
    meta = pmt::dict_add(meta, pmt::mp("expected_reply_delay_samples"), pmt::from_uint64(slot.delay));
    if (anchor != slot.anchor) { ++c->identity_mismatches; diagnostic(meta, "SLOT_IDENTITY_MISMATCH"); return; }
    if (dict_ref_uint64(meta, "reply_delay_samples", 0x1000000) != slot.delay) {
        ++c->delay_mismatches; diagnostic(meta, "REPLY_DELAY_MISMATCH"); return;
    }
    if (c->pending.count(anchor) || c->received.count(anchor)) {
        ++c->duplicates; diagnostic(meta, "DUPLICATE_RESPONSE"); return;
    }
    c->pending.insert(anchor);
    meta = pmt::dict_add(meta, pmt::mp("reply_delay_mismatch"), pmt::PMT_F);
    meta = pmt::dict_add(meta, pmt::mp("slot_identity_mismatch"), pmt::PMT_F);
    message_port_pub(pmt::mp("solver_measurement_out"), pmt::cons(meta, pmt::PMT_NIL));
}
void prs_multi_anchor_cycle_impl::handle_range(pmt::pmt_t msg)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    const auto meta = metadata(msg);
    if (!pmt::is_dict(meta)) return;
    auto* c = locate(meta);
    if (!c) { diagnostic(meta, "LATE_SOLVER_RESULT"); return; }
    const auto anchor = dict_ref_uint64(meta, "anchor_id", 8);
    if (anchor > 7) { diagnostic(meta, "UNEXPECTED_SOLVER_RESULT"); return; }
    if (!c->pending.erase(anchor) || !std::isfinite(dict_ref_double(meta, "integer_range_m", NAN))) {
        diagnostic(meta, "UNEXPECTED_SOLVER_RESULT"); return;
    }
    c->received.insert(anchor);
    message_port_pub(pmt::mp("measurement_out"), msg);
}
void prs_multi_anchor_cycle_impl::finish(const cycle& c, const char* reason)
{
    auto meta = pmt::make_dict();
    std::vector<uint32_t> received, missing;
    for (auto anchor : d_anchors) (c.received.count(anchor) ? received : missing).push_back(anchor);
    meta = pmt::dict_add(meta, pmt::mp("crc_failure_count"), pmt::from_uint64(c.crc_failures));
    meta = pmt::dict_add(meta, pmt::mp("poll_tx_timestamp"), pmt::from_double(c.t1));
    meta = pmt::dict_add(meta, pmt::mp("completion_reason"), pmt::mp(reason));
    meta = pmt::dict_add(meta, pmt::mp("accepted_measurement_count"), pmt::from_uint64(received.size()));
    meta = pmt::dict_add(meta, pmt::mp("missing_anchor_ids"), pmt::init_u32vector(missing.size(), missing));
    meta = pmt::dict_add(meta, pmt::mp("poll_id"), pmt::from_uint64(c.poll));
    // meta = pmt::dict_add(meta, pmt::mp("cycle_sequence"), pmt::from_uint64(c.sequence));
    // meta = pmt::dict_add(meta, pmt::mp("cycle_completion_timestamp"), pmt::from_double(c.end));
    // meta = pmt::dict_add(meta, pmt::mp("summary_emission_timestamp"), pmt::from_double(d_clock));
    // meta = pmt::dict_add(meta, pmt::mp("expected_anchor_ids"), pmt::init_u32vector(d_anchors.size(), d_anchors));
    // meta = pmt::dict_add(meta, pmt::mp("received_anchor_ids"), pmt::init_u32vector(received.size(), received));
    // meta = pmt::dict_add(meta, pmt::mp("duplicate_response_count"), pmt::from_uint64(c.duplicates));
    // meta = pmt::dict_add(meta, pmt::mp("slot_identity_mismatch_count"), pmt::from_uint64(c.identity_mismatches));
    // meta = pmt::dict_add(meta, pmt::mp("reply_delay_mismatch_count"), pmt::from_uint64(c.delay_mismatches));
    
    auto expire = pmt::dict_add(meta, pmt::mp("expire_cycle"), pmt::PMT_T);
    message_port_pub(pmt::mp("solver_tx_out"), expire);
    message_port_pub(pmt::mp("cycle_out"), pmt::cons(meta, pmt::PMT_NIL));
}
void prs_multi_anchor_cycle_impl::handle_clock(pmt::pmt_t msg)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    const auto meta = metadata(msg);
    const double now = dict_ref_double(meta, "clock_time", NAN);
    d_clock = now;
    for (auto it = d_cycles.begin(); it != d_cycles.end();) {
        if (now >= it->second.end + d_grace) { finish(it->second, "WINDOWS_EXPIRED"); it = d_cycles.erase(it); }
        else ++it;
    }
}
bool prs_multi_anchor_cycle_impl::stop()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    for (const auto& entry : d_cycles) finish(entry.second, "STOPPED_INCOMPLETE");
    d_cycles.clear();
    return true;
}
} // namespace ofdm_prs_ranging
} // namespace gr
