/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_payload_codec.h"
#include "prs_phy_geometry.h"
#include "prs_protocol_metadata.h"
#include "prs_receiver_utils.h"
#include "prs_ssrtt_responder_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
long double poll_rx_timestamp(const pmt::pmt_t& dict)
{
    const auto value = pmt::dict_ref(dict, pmt::mp("rx_time"), pmt::PMT_NIL);
    if (pmt::is_tuple(value) && pmt::length(value) >= 2) {
        const auto secs = pmt::tuple_ref(value, 0);
        const auto frac = pmt::tuple_ref(value, 1);
        const long double s = pmt::is_uint64(secs)
                                  ? static_cast<long double>(pmt::to_uint64(secs))
                                  : static_cast<long double>(pmt::to_long(secs));
        return s + pmt::to_double(frac);
    }
    return NAN;
}
} // namespace

prs_ssrtt_responder::sptr prs_ssrtt_responder::make(double samp_rate,
                                                    uint64_t reply_delay_samples,
                                                    uint64_t anchor_id,
                                                    double poll_period_s,
                                                    const std::string& poll_tracking_csv,
                                                    double poll_window_margin_s)
{
    return gnuradio::make_block_sptr<prs_ssrtt_responder_impl>(
        samp_rate, reply_delay_samples, anchor_id, poll_period_s, poll_tracking_csv,
        poll_window_margin_s);
}

prs_ssrtt_responder_impl::prs_ssrtt_responder_impl(double samp_rate,
                                                   uint64_t reply_delay_samples,
                                                   uint64_t anchor_id,
                                                   double poll_period_s,
                                                   const std::string& poll_tracking_csv,
                                                   double poll_window_margin_s)
    : gr::block("prs_ssrtt_responder",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_samp_rate(samp_rate),
      d_reply_delay_samples(reply_delay_samples),
      d_anchor_id(anchor_id),
      d_poll_period_s(poll_period_s),
      d_poll_window_margin_s(poll_window_margin_s)
{
    if (!std::isfinite(samp_rate) || samp_rate <= 0 || reply_delay_samples > 0xffffff || anchor_id > 7) {
        throw std::invalid_argument("invalid responder sample rate, delay or anchor_id");
    }
    if (!std::isfinite(poll_window_margin_s) || poll_window_margin_s < 0.0) {
        throw std::invalid_argument("POLL window margin must be finite and nonnegative");
    }
    if ((!poll_tracking_csv.empty() || poll_window_margin_s > 0.0) &&
        (!std::isfinite(poll_period_s) || poll_period_s <= 0.0)) {
        throw std::invalid_argument("POLL tracking requires a positive POLL period");
    }
    if (!poll_tracking_csv.empty()) {
        d_poll_log.imbue(std::locale::classic());
        d_poll_log.open(poll_tracking_csv, std::ios::app);
        if (d_poll_log.tellp() == 0) {
            d_poll_log << "anchor_id,poll_id,poll_rx_time_s,nominal_period_s,status,"
                          "poll_id_step,predicted_rx_time_s,residual_samples,"
                          "nominal_residual_samples,observed_period_s,"
                          "estimated_period_s,apparent_skew_ppm,next_poll_rx_time_s\n";
            d_poll_log.flush();
        }
        if (!d_poll_log) {
            throw std::runtime_error("cannot open POLL tracking CSV: " + poll_tracking_csv);
        }
        d_poll_log << std::fixed;
    }
    message_port_register_in(pmt::mp("measurement_in"));
    message_port_register_out(pmt::mp("trigger_out"));
    message_port_register_out(pmt::mp("diagnostic_out"));
    set_msg_handler(pmt::mp("measurement_in"),
                    [this](pmt::pmt_t msg) { handle_measurement(msg); });
}

pmt::pmt_t prs_ssrtt_responder_impl::add_poll_window(pmt::pmt_t trigger,
                                                     const pmt::pmt_t& meta,
                                                     uint64_t poll_id,
                                                     long double period) const
{
    constexpr auto max_sample = std::numeric_limits<uint64_t>::max();
    uint64_t frame_start = 0;
    if (!bounded_uint(meta, "frame_start", max_sample, max_sample - 1, frame_start)) {
        return trigger;
    }

    const long double next_frame = frame_start + period * d_samp_rate;
    const long double margin = static_cast<long double>(d_poll_window_margin_s) * d_samp_rate;
    // The window bounds the newest sample used by the rolling preamble metric.
    // Include the leading guard and the complete preamble at the latest arrival.
    constexpr int guard_and_preamble =
        phy::coarse_start_samples(phy::preamble_len, phy::preamble_repeats);
    const long double start = std::floor(std::max(0.0L, next_frame - margin));
    const long double end = std::ceil(next_frame + margin + guard_and_preamble);
    if (!std::isfinite(end) || end >= static_cast<long double>(max_sample)) {
        return trigger;
    }
    trigger = pmt::dict_add(trigger, pmt::mp("poll_tracking_window"), pmt::PMT_T);
    trigger = pmt::dict_add(trigger, pmt::mp("start_sample"),
                            pmt::from_uint64(static_cast<uint64_t>(start)));
    trigger = pmt::dict_add(trigger, pmt::mp("end_sample"),
                            pmt::from_uint64(static_cast<uint64_t>(end)));
    return pmt::dict_add(trigger, pmt::mp("window_id"),
                         pmt::from_uint64((poll_id + 1) & 0xffff));
}

void prs_ssrtt_responder_impl::publish_response(pmt::pmt_t trigger,
                                               const pmt::pmt_t& meta,
                                               uint64_t poll_id,
                                               long double rx_time)
{
    if ((!d_poll_log.is_open() && d_poll_window_margin_s == 0.0) ||
        (d_poll_timing && rx_time <= d_poll_timing->rx_time)) {
        message_port_pub(pmt::mp("trigger_out"), trigger);
        return;
    }

    const char* status = "START";
    long double period = d_poll_period_s;
    uint64_t step = 0;
    if (d_poll_timing) {
        // Half-range comparison distinguishes forward progress (including wrap)
        // from an ID discontinuity.
        step = (poll_id - d_poll_timing->id) & 0xffff;
        status = "ID_RESET";
    }

    const bool has_prediction = step > 0 && step < 0x8000;
    long double expected_interval = 0.0L;
    long double elapsed = 0.0L;
    long double observed = 0.0L;
    if (has_prediction) {
        elapsed = rx_time - d_poll_timing->rx_time;
        expected_interval = step * d_poll_timing->period_s;
        observed = elapsed / step;
        // Keep the residual visible even when a cadence jump restarts the filter.
        if (std::abs(observed - d_poll_period_s) <= 0.5L * d_poll_period_s) {
            constexpr long double gain = 0.125L;
            period = d_poll_timing->period_s + gain * (observed - d_poll_timing->period_s);
            status = "TRACKING";
            if (d_poll_window_margin_s > 0.0) {
                trigger = add_poll_window(trigger, meta, poll_id, period);
            }
        } else {
            status = "CADENCE_RESET";
        }
    }

    d_poll_timing = poll_timing{poll_id, rx_time, period};
    message_port_pub(pmt::mp("trigger_out"), trigger);
    if (!d_poll_log.is_open()) {
        return;
    }

    constexpr int seconds_precision = 12;
    constexpr int residual_precision = 3;
    d_poll_log << std::setprecision(seconds_precision) << d_anchor_id << ',' << poll_id
               << ',' << rx_time << ',' << d_poll_period_s << ',' << status << ',';
    if (has_prediction) {
        d_poll_log << step << ',' << d_poll_timing->rx_time + expected_interval
                   << ',' << std::setprecision(residual_precision)
                   << (elapsed - expected_interval) * d_samp_rate
                   << ',' << (elapsed - step * d_poll_period_s) * d_samp_rate
                   << ',' << std::setprecision(seconds_precision) << observed;
    } else {
        d_poll_log << ",,,,";
    }
    d_poll_log << ',' << period << ',' << std::setprecision(residual_precision)
               << 1e6L * (period / d_poll_period_s - 1.0L) << ','
               << std::setprecision(seconds_precision) << rx_time + period << '\n';
    d_poll_log.flush();
    if (!d_poll_log) {
        d_logger->warn("POLL tracking CSV write failed; disabling tracking log");
        d_poll_log.close();
    }
}

void prs_ssrtt_responder_impl::handle_measurement(pmt::pmt_t msg)
{
    if (!pmt::is_pair(msg)) {
        return;
    }
    const auto meta = pmt::car(msg);
    if (!pmt::to_bool(pmt::dict_ref(meta, pmt::mp("frame_id_valid"), pmt::PMT_F))) {
        return;
    }
    if (dict_ref_uint64(meta, "packet_type", 0) != prs_packet_type_poll) {
        return;
    }

    const long double poll_rx_timestamp_s = poll_rx_timestamp(meta);
    const double poll_rx_time = static_cast<double>(poll_rx_timestamp_s);
    if (!std::isfinite(poll_rx_time)) {
        return;
    }

    uint64_t poll_id = 0;
    if (!bounded_uint(meta, "poll_id", 65536, 65535, poll_id)) return;
    const double duplicate_hold_s = std::max(0.2, d_reply_delay_samples / d_samp_rate + 0.05);
    d_latest_poll_time = std::max(d_latest_poll_time, poll_rx_time);
    for (auto it = d_recent_polls.begin(); it != d_recent_polls.end();) {
        if (d_latest_poll_time - it->second > duplicate_hold_s) it = d_recent_polls.erase(it);
        else ++it;
    }
    const auto prior = d_recent_polls.find(poll_id);
    if (poll_rx_time < d_latest_poll_time - duplicate_hold_s ||
        (prior != d_recent_polls.end() && poll_rx_time <= prior->second + duplicate_hold_s)) {
        auto diagnostic = pmt::dict_add(meta, pmt::mp("duplicate_poll"), pmt::PMT_T);
        message_port_pub(pmt::mp("diagnostic_out"), pmt::cons(diagnostic, pmt::PMT_NIL));
        return;
    }
    // At most 65536 protocol IDs; age-based cleanup normally keeps only recent POLLs.
    d_recent_polls[poll_id] = poll_rx_time;
    int16_t fine_q = 0;
    const bool fine_valid = quantize_poll_fine_delay(
        dict_ref_double(meta, "fine_delay_samples", NAN), metadata_true(meta, "valid"), fine_q);
    const double reply_delay_s = static_cast<double>(d_reply_delay_samples) / d_samp_rate;
    const double response_tx_time = poll_rx_time + reply_delay_s;
    const double secs_floor = std::floor(response_tx_time);

    pmt::pmt_t trigger = pmt::make_dict();
    trigger = pmt::dict_add(
        trigger, pmt::mp("packet_type"), pmt::from_long(prs_packet_type_response));
    trigger =
        pmt::dict_add(trigger, pmt::mp("poll_id"), pmt::from_uint64(poll_id));
    trigger = pmt::dict_add(trigger, pmt::mp("anchor_id"), pmt::from_uint64(d_anchor_id));
    trigger = pmt::dict_add(trigger, pmt::mp("poll_fine_delay_q10"), pmt::from_long(fine_q));
    trigger = pmt::dict_add(trigger, pmt::mp("fine_delay_valid"), fine_valid ? pmt::PMT_T : pmt::PMT_F);
    trigger = pmt::dict_add(
        trigger, pmt::mp("reply_delay_samples"), pmt::from_uint64(d_reply_delay_samples));
    trigger = pmt::dict_add(trigger,
                            pmt::mp("tx_time_secs"),
                            pmt::from_uint64(static_cast<uint64_t>(secs_floor)));
    trigger = pmt::dict_add(trigger,
                            pmt::mp("tx_time_frac"),
                            pmt::from_double(response_tx_time - secs_floor));
    publish_response(trigger, meta, poll_id, poll_rx_timestamp_s);
}

} // namespace ofdm_prs_ranging
} // namespace gr
