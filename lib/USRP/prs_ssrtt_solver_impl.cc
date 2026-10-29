/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_payload_codec.h"
#include "prs_protocol_metadata.h"
#include "prs_receiver_utils.h"
#include "prs_ssrtt_solver_impl.h"
#include "prs_timing_helper.h"
#include <gnuradio/io_signature.h>
#include <cmath>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
constexpr double c_mps = 299792458.0;

double dict_ref_time_tuple(const pmt::pmt_t& dict, const char* key, double fallback)
{
    const auto value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_tuple(value) && pmt::length(value) >= 2) {
        const auto secs = pmt::tuple_ref(value, 0);
        const auto frac = pmt::tuple_ref(value, 1);
        const double s = pmt::is_uint64(secs) ? static_cast<double>(pmt::to_uint64(secs))
                                              : static_cast<double>(pmt::to_long(secs));
        return s + pmt::to_double(frac);
    }
    return fallback;
}
} // namespace

prs_ssrtt_solver::sptr prs_ssrtt_solver::make(double samp_rate, bool enable_profiling)
{
    return gnuradio::make_block_sptr<prs_ssrtt_solver_impl>(samp_rate, enable_profiling);
}

prs_ssrtt_solver_impl::prs_ssrtt_solver_impl(double samp_rate, bool enable_profiling)
    : gr::block("prs_ssrtt_solver",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_samp_rate(samp_rate),
      d_enable_profiling(enable_profiling)
{
    message_port_register_in(pmt::mp("tx_time_in"));
    message_port_register_in(pmt::mp("measurement_in"));
    message_port_register_out(pmt::mp("ssrtt_out"));
    message_port_register_out(pmt::mp("timing_out"));
    set_msg_handler(pmt::mp("tx_time_in"),
                    [this](pmt::pmt_t msg) { handle_tx_time(msg); });
    set_msg_handler(pmt::mp("measurement_in"),
                    [this](pmt::pmt_t msg) { handle_measurement(msg); });
}

void prs_ssrtt_solver_impl::handle_tx_time(pmt::pmt_t msg)
{
    profiling::timing_report report(d_enable_profiling, "ssrtt_solver");
    if (!pmt::is_dict(msg)) {
        return;
    }
    // Expire cycle erase
    if (metadata_true(msg, "expire_cycle")) {
        const auto poll = dict_ref_uint64(msg, "poll_id", 65536);
        const auto found = d_poll_tx_times.find(poll);
        // Check the d_poll_tx_times with entry msg poll_id is the same cycle_sequence as the msg, if so, erase it.
        if (found != d_poll_tx_times.end() && // poll in d_poll_tx_times
            found->second.managed &&          // only erase if the poll is managed
            found->second.sequence == dict_ref_uint64(msg, "cycle_sequence", 0)){// same cycle_sequence
            d_poll_tx_times.erase(found);
        }
        return;
    }
    const uint64_t packet_type = dict_ref_uint64(msg, "packet_type", prs_packet_type_poll);
    // Response packet type do not calculate measurement.
    if (packet_type != prs_packet_type_poll) {
        return;
    }
    uint64_t poll_id = 0;
    if (!bounded_uint(msg, "poll_id", 65536, 65535, poll_id)) return;
    const double t1 = dict_ref_double(msg, "tx_time_secs", 0.0) +
                      dict_ref_double(msg, "tx_time_frac", 0.0);
    if (!std::isfinite(t1)) return;
    // Create a poll tx event.
    d_poll_tx_times[poll_id] = poll_tx{t1, dict_ref_uint64(msg, "cycle_sequence", 0),
                                               metadata_true(msg, "cycle_managed")};
    report.checkpoint("tx_time_store");
    if (report.enabled()) {
        message_port_pub(pmt::mp("timing_out"), report.finish(msg));
    }
}

void prs_ssrtt_solver_impl::handle_measurement(pmt::pmt_t msg)
{
    profiling::timing_report report(d_enable_profiling, "ssrtt_solver");
    if (!pmt::is_pair(msg)) {
        return;
    }
    pmt::pmt_t meta = pmt::car(msg);
    // check if the frame_id is valid, if not, ignore the measurement.
    if (!pmt::to_bool(pmt::dict_ref(meta, pmt::mp("frame_id_valid"), pmt::PMT_F))) {
        return;
    }
    // check if the packet_type is response, if not, ignore the measurement.
    if (dict_ref_uint64(meta, "packet_type", 0) != prs_packet_type_response) {
        return;
    }

    uint64_t poll_id = 0, anchor_id = 0, reply_delay_samples = 0;
    if (!bounded_uint(meta, "poll_id", 65536, 65535, poll_id) ||
        !bounded_uint(meta, "anchor_id", 8, 7, anchor_id) ||
        !bounded_uint(meta, "reply_delay_samples", 0x1000000, 0xffffff, reply_delay_samples)) return;
    const auto found = d_poll_tx_times.find(poll_id);
    if (found == d_poll_tx_times.end()) {
        return;
    }
    // Check cycle manage is operated or not
    // if the cycle is managed, check the cycle_sequence is the same as the stored one, 
    // if not, for old version single link RTT, keep the RTT calculation.
    if (found->second.managed && found->second.sequence != dict_ref_uint64(meta, "cycle_sequence", 0)) return;
    const double t1 = found->second.t1;
    const double t4 = dict_ref_time_tuple(meta, "rx_time", NAN);
    if (!std::isfinite(t4)) {
        return;
    }
    // The range calculation. RTT = t4 - t1 - reply_delay
    const double reply_delay_s = static_cast<double>(reply_delay_samples) / d_samp_rate;
    const double rtt_s = t4 - t1 - reply_delay_s;
    const double tof_s = rtt_s/ 2.0 - 5.808278798530661e-06; // the processing delay of the response packet;
    const double range_m = tof_s * c_mps;
    const double poll_fine_samples = dict_ref_double(meta, "poll_fine_delay_samples", NAN);
    const double response_fine_samples = dict_ref_double(meta, "fine_delay_samples", NAN);
    const bool fine_valid = metadata_true(meta, "fine_delay_valid") &&
        metadata_true(meta, "valid") && std::isfinite(poll_fine_samples) &&
        std::isfinite(response_fine_samples);
    const double correction_m = fine_valid
        ? c_mps / (2.0 * d_samp_rate) * (poll_fine_samples + response_fine_samples) : NAN;
    const double corrected_range_m = fine_valid ? range_m + correction_m : NAN;
    const double response_fine_delay_s = response_fine_samples / d_samp_rate;
    report.checkpoint("ssrtt_calculation");

    auto stage_start = report.mark();
    meta = pmt::dict_add(meta, pmt::mp("t1_tx_time"), pmt::from_double(t1));
    meta = pmt::dict_add(meta, pmt::mp("t4_rx_time"), pmt::from_double(t4));
    meta = pmt::dict_add(meta, pmt::mp("calibration_time"), pmt::from_double(5.808278798530661e-06));
    meta = pmt::dict_add(meta, pmt::mp("integer_range_m"), pmt::from_double(range_m));
    meta = pmt::dict_add(meta, pmt::mp("corrected_range_m"), pmt::from_double(corrected_range_m));
    meta = pmt::dict_add(meta, pmt::mp("poll_fine_delay_samples"), pmt::from_double(poll_fine_samples));
    meta = pmt::dict_add(meta, pmt::mp("response_fine_delay_samples"), pmt::from_double(response_fine_samples));
    meta = pmt::dict_add(meta, pmt::mp("fine_range_correction_m"), pmt::from_double(correction_m));
    meta = pmt::dict_add(meta, pmt::mp("fine_correction_valid"), fine_valid ? pmt::PMT_T : pmt::PMT_F);
    auto stage_stop = report.mark();
    report.add("metadata_build", stage_start, stage_stop);

    stage_start = stage_stop;
    const auto output = pmt::cons(meta, pmt::PMT_NIL);
    message_port_pub(pmt::mp("ssrtt_out"), output);
    stage_stop = report.mark();
    report.add("output_publish", stage_start, stage_stop);
    if (report.enabled()) {
        message_port_pub(pmt::mp("timing_out"), report.finish(meta));
    }
}

} // namespace ofdm_prs_ranging
} // namespace gr
