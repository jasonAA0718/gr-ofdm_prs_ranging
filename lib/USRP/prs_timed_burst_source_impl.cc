/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_frame_builder.h"
#include "prs_payload_codec.h"
#include "prs_protocol_metadata.h"
#include "prs_timed_burst_source_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
uint64_t pmt_to_uint64(const pmt::pmt_t& value)
{
    if (pmt::is_uint64(value)) {
        return pmt::to_uint64(value);
    }
    if (pmt::is_integer(value)) {
        return static_cast<uint64_t>(pmt::to_long(value));
    }
    return 0;
}

double dict_ref_double_local(const pmt::pmt_t& dict, const char* key, double fallback)
{
    const auto value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_real(value)) {
        return pmt::to_double(value);
    }
    if (pmt::is_integer(value)) {
        return static_cast<double>(pmt::to_long(value));
    }
    if (pmt::is_uint64(value)) {
        return static_cast<double>(pmt::to_uint64(value));
    }
    return fallback;
}

bool dict_has_key(const pmt::pmt_t& dict, const char* key)
{
    return !pmt::eq(pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL), pmt::PMT_NIL);
}
} // namespace

#ifdef OFDM_PRS_RANGING_ENABLE_DEBUG_LOGS
#define PRS_TBS_DEBUG(...) d_logger->debug(__VA_ARGS__)
#else
#define PRS_TBS_DEBUG(...) \
    do {                   \
    } while (0)
#endif

prs_timed_burst_source::sptr prs_timed_burst_source::make(double samp_rate,
                                                          int fft_len,
                                                          int cp_len,
                                                          int active_bins,
                                                          int prs_symbols,
                                                          int preamble_len,
                                                          int preamble_repeats,
                                                          int coarse_sync_len,
                                                          double tx_lead_time,
                                                          double burst_period,
                                                          float tx_amp,
                                                          uint32_t seed,
                                                          int pings_per_trigger,
                                                          bool attach_tx_time,
                                                          int coarse_zc_root,
                                                          int mc_ds_gold_code_id)
{
    return gnuradio::make_block_sptr<prs_timed_burst_source_impl>(samp_rate,
                                                                  fft_len,
                                                                  cp_len,
                                                                  active_bins,
                                                                  prs_symbols,
                                                                  preamble_len,
                                                                  preamble_repeats,
                                                                  coarse_sync_len,
                                                                  tx_lead_time,
                                                                  burst_period,
                                                                  tx_amp,
                                                                  seed,
                                                                  pings_per_trigger,
                                                                  attach_tx_time,
                                                                  coarse_zc_root,
                                                                  mc_ds_gold_code_id);
}

prs_timed_burst_source_impl::prs_timed_burst_source_impl(double samp_rate,
                                                         int fft_len,
                                                         int cp_len,
                                                         int active_bins,
                                                         int prs_symbols,
                                                         int preamble_len,
                                                         int preamble_repeats,
                                                         int coarse_sync_len,
                                                         double tx_lead_time,
                                                         double burst_period,
                                                         float tx_amp,
                                                         uint32_t seed,
                                                         int pings_per_trigger,
                                                         bool attach_tx_time,
                                                         int coarse_zc_root,
                                                         int mc_ds_gold_code_id)
    : gr::block("prs_timed_burst_source",
                gr::io_signature::make(0, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_samp_rate(samp_rate),
      d_fft_len(fft_len),
      d_cp_len(cp_len),
      d_active_bins(active_bins),
      d_prs_symbols(prs_symbols),
      d_preamble_len(preamble_len),
      d_preamble_repeats(preamble_repeats),
      d_coarse_sync_len(coarse_sync_len),
      d_tx_lead_time(tx_lead_time),
      d_burst_period(burst_period),
      d_tx_amp(tx_amp),
      d_seed(seed),
      d_pings_per_trigger(pings_per_trigger),
      d_attach_tx_time(attach_tx_time),
      d_coarse_zc_root(coarse_zc_root),
      d_mc_ds_gold_code_id(mc_ds_gold_code_id),
      d_prs_start(0),
      d_prs_len(0),
      d_payload_start(0),
      d_payload_len(0),
      d_in_burst(false),
      d_burst_offset(0),
      d_have_rx_time(false),
      d_rx_time_abs_sample(0),
      d_rx_time_value(0.0)
{
    validate_parameters();
    build_frame();

    message_port_register_in(pmt::mp("trigger"));
    set_msg_handler(pmt::mp("trigger"),
                    [this](pmt::pmt_t msg) { this->handle_trigger(msg); });
    message_port_register_out(pmt::mp("tx_time_out"));
}

prs_timed_burst_source_impl::~prs_timed_burst_source_impl() {}

void prs_timed_burst_source_impl::validate_parameters() const
{
    if (d_samp_rate <= 0.0) {
        throw std::invalid_argument("samp_rate must be positive");
    }
    if (d_fft_len <= 0 || d_cp_len < 0 || d_cp_len > d_fft_len) {
        throw std::invalid_argument("invalid fft_len/cp_len");
    }
    if (d_fft_len != 512 || d_active_bins != 512 || d_prs_symbols != 127 ||
        d_cp_len != 64) {
        throw std::invalid_argument("MC-DS PRS requires fft_len=512, active_bins=512, "
                                    "prs_symbols=127, cp_len=64");
    }
    if (d_prs_symbols <= 0 || d_preamble_len <= 0 || d_preamble_repeats <= 0 ||
        d_coarse_sync_len <= 0) {
        throw std::invalid_argument("frame lengths must be positive");
    }
    if (d_tx_amp <= 0.0f || d_tx_amp > 1.0f) {
        throw std::invalid_argument("tx_amp must be in (0, 1.0f]");
    }
    if (d_pings_per_trigger <= 0) {
        throw std::invalid_argument("pings_per_trigger must be positive");
    }
    if (d_mc_ds_gold_code_id < 0 || d_mc_ds_gold_code_id >= 129) {
        throw std::invalid_argument("mc_ds_gold_code_id must be in [0, 128]");
    }
}

prs_frame_config prs_timed_burst_source_impl::frame_config() const
{
    return prs_frame_config{ d_samp_rate,
                             d_fft_len,
                             d_cp_len,
                             d_active_bins,
                             d_prs_symbols,
                             d_preamble_len,
                             d_preamble_repeats,
                             d_coarse_sync_len,
                             0,
                             d_tx_amp,
                             d_seed,
                             d_coarse_zc_root,
                             d_mc_ds_gold_code_id };
}

void prs_timed_burst_source_impl::build_frame()
{
    const auto cfg = frame_config();
    const auto frame = prs_frame_builder::build(cfg);
    d_frame = frame.samples;
    d_burst_frame = d_frame;
    d_prs_start = frame.prs_start;
    d_prs_len = frame.prs_len;
    d_payload_start = frame.payload_start;
    d_payload_len = frame.payload_len;

    PRS_TBS_DEBUG("PRS frame generated: frame_len={} prs_start={} prs_len={}",
                  d_frame.size(),
                  d_prs_start,
                  d_prs_len);
}

void prs_timed_burst_source_impl::prepare_burst_frame(uint64_t frame_id)
{
    (void)frame_id;
    prs_payload_info info;
    info.packet_type = d_current_burst.packet_type;
    info.poll_id = d_current_burst.poll_id;
    info.anchor_id = d_current_burst.anchor_id;
    info.reply_delay_samples = d_current_burst.reply_delay_samples;
    info.poll_fine_delay_q10 = d_current_burst.poll_fine_delay_q10;
    info.fine_delay_valid = d_current_burst.fine_delay_valid;
    d_burst_frame = prs_frame_builder::build(frame_config(), info).samples;
}

void prs_timed_burst_source_impl::forecast(int noutput_items,
                                           gr_vector_int& ninput_items_required)
{
    (void)noutput_items;
    if (ninput_items_required.empty()) {
        return;
    }
    if (d_in_burst) {
        ninput_items_required[0] = 0;
        return;
    }
    if (!d_scheduler.empty()) {
        const bool waiting_for_rx_time = d_attach_tx_time && !d_have_rx_time &&
                                         !std::isfinite(d_scheduler.front().tx_time);
        ninput_items_required[0] = waiting_for_rx_time ? 1 : 0;
        return;
    }
    ninput_items_required[0] = 1;
}

double prs_timed_burst_source_impl::current_time_estimate()
{
    if (!d_have_rx_time) {
        return static_cast<double>(nitems_read(0)) / d_samp_rate;
    }
    const uint64_t delta = nitems_read(0) - d_rx_time_abs_sample;
    return d_rx_time_value + static_cast<double>(delta) / d_samp_rate;
}

void prs_timed_burst_source_impl::handle_trigger(pmt::pmt_t msg)
{
    if (pmt::is_dict(msg)) {
        prs_payload_info info;
        if (!bounded_uint(msg, "packet_type", prs_packet_type_poll, 3, info.packet_type) ||
            (info.packet_type != prs_packet_type_poll && info.packet_type != prs_packet_type_response) ||
            !bounded_uint(msg, "anchor_id", 0, 7, info.anchor_id) ||
            !bounded_uint(msg, "poll_id", 0, 65535, info.poll_id) ||
            !bounded_uint(msg, "reply_delay_samples", 0, 0xffffff, info.reply_delay_samples) ||
            !bounded_q10(msg, info.poll_fine_delay_q10)) {
            d_logger->warn("Rejected out-of-range packet v2 trigger");
            return;
        }
        const bool have_fine_value = dict_has_key(msg, "poll_fine_delay_q10") ||
            dict_has_key(msg, "poll_fine_delay_samples") ||
            dict_has_key(msg, "fine_delay_samples") || dict_has_key(msg, "fine_delay");
        info.fine_delay_valid = metadata_true(msg, "fine_delay_valid") && have_fine_value;
        const char* float_key = dict_has_key(msg, "poll_fine_delay_samples")
                                    ? "poll_fine_delay_samples"
                                    : (dict_has_key(msg, "fine_delay_samples") ? "fine_delay_samples" : "fine_delay");
        if (dict_has_key(msg, float_key)) {
            double samples = dict_ref_double_local(msg, float_key, NAN);
            if (std::string(float_key) == "fine_delay") samples *= d_samp_rate;
            int16_t q = 0;
            info.fine_delay_valid = quantize_poll_fine_delay(samples, info.fine_delay_valid, q);
            info.poll_fine_delay_q10 = q;
        }
        if (!info.fine_delay_valid) info.poll_fine_delay_q10 = 0;
        // Allocate only after validating arbitrary caller-supplied values.
        if (!dict_has_key(msg, "poll_id")) {
            if (info.packet_type == prs_packet_type_response) return;
            info.poll_id = d_scheduler.allocate_poll_id();
        }
        const uint64_t allocated = d_scheduler.allocate_frame_id();

        double tx_time = std::numeric_limits<double>::quiet_NaN();
        if (dict_has_key(msg, "tx_time")) {
            tx_time = dict_ref_double_local(msg, "tx_time", tx_time);
        } else if (dict_has_key(msg, "tx_time_secs") ||
                   dict_has_key(msg, "tx_time_frac")) {
            tx_time = dict_ref_double_local(msg, "tx_time_secs", 0.0) +
                      dict_ref_double_local(msg, "tx_time_frac", 0.0);
        } else if (d_have_rx_time) {
            tx_time = current_time_estimate() + d_tx_lead_time;
        }

        d_scheduler.queue_burst(prs_pending_burst{ allocated, tx_time, 0.0,
            info.packet_type, info.poll_id, info.anchor_id, info.reply_delay_samples,
            info.poll_fine_delay_q10, info.fine_delay_valid });
        return;
    }

    int count = d_pings_per_trigger;
    if (pmt::is_integer(msg)) {
        count = std::max(1, static_cast<int>(pmt::to_long(msg)));
    }

    const double base_time = d_have_rx_time ? current_time_estimate() + d_tx_lead_time
                                            : std::numeric_limits<double>::quiet_NaN();
    d_scheduler.queue_trigger(count, base_time, d_burst_period);

    PRS_TBS_DEBUG(
        "PRS trigger queued: count={} base_tx_time={} burst_period={} frame_len={}",
        count,
        base_time,
        d_burst_period,
        d_frame.size());
}

void prs_timed_burst_source_impl::add_burst_tags(uint64_t abs_offset,
                                                 const prs_pending_burst& burst)
{
    const double secs_floor = std::floor(burst.tx_time);
    const uint64_t secs = static_cast<uint64_t>(secs_floor);
    const double frac = burst.tx_time - secs_floor;

    if (d_attach_tx_time) {
        add_item_tag(0,
                     abs_offset,
                     pmt::mp("tx_time"),
                     pmt::make_tuple(pmt::from_uint64(secs), pmt::from_double(frac)),
                     pmt::mp("prs_timed_burst_source"));
    }
    add_item_tag(0, abs_offset, pmt::mp("tx_sob"), pmt::PMT_T);
    add_item_tag(0, abs_offset, pmt::mp("frame_id"), pmt::from_uint64(burst.frame_id));
    add_item_tag(
        0, abs_offset, pmt::mp("attempt_id"), pmt::from_uint64(burst.poll_id));
    add_item_tag(
        0, abs_offset, pmt::mp("packet_type"), pmt::from_long(burst.packet_type));
    add_item_tag(
        0, abs_offset, pmt::mp("poll_id"), pmt::from_uint64(burst.poll_id));
    add_item_tag(0,
                 abs_offset,
                 pmt::mp("anchor_id"),
                 pmt::from_uint64(burst.anchor_id));
    add_item_tag(0,
                 abs_offset,
                 pmt::mp("reply_delay_samples"),
                 pmt::from_uint64(burst.reply_delay_samples));
    add_item_tag(0, abs_offset, pmt::mp("poll_fine_delay_q10"), pmt::from_long(burst.poll_fine_delay_q10));
    add_item_tag(0, abs_offset, pmt::mp("fine_delay_valid"), burst.fine_delay_valid ? pmt::PMT_T : pmt::PMT_F);
    add_item_tag(0, abs_offset, pmt::mp("burst_len"), pmt::from_long(frame_len()));
    add_item_tag(0, abs_offset, pmt::mp("prs_start"), pmt::from_long(d_prs_start));
    add_item_tag(0, abs_offset, pmt::mp("prs_len"), pmt::from_long(d_prs_len));
    add_item_tag(0, abs_offset, pmt::mp("fft_len"), pmt::from_long(d_fft_len));
    add_item_tag(0, abs_offset, pmt::mp("cp_len"), pmt::from_long(d_cp_len));
    add_item_tag(0, abs_offset, pmt::mp("active_bins"), pmt::from_long(d_active_bins));
    add_item_tag(0, abs_offset, pmt::mp("prs_symbols"), pmt::from_long(d_prs_symbols));
    add_item_tag(0,
                 abs_offset,
                 pmt::mp("mc_ds_gold_code_id"),
                 pmt::from_long(d_mc_ds_gold_code_id));
    add_item_tag(0, abs_offset, pmt::mp("samp_rate"), pmt::from_double(d_samp_rate));

    PRS_TBS_DEBUG("PRS burst tags: frame_id={} start_abs={} tx_time={} attach_tx_time={}",
                  burst.frame_id,
                  abs_offset,
                  burst.tx_time,
                  d_attach_tx_time);
}

void prs_timed_burst_source_impl::publish_tx_time(const prs_pending_burst& burst)
{
    const double secs_floor = std::floor(burst.tx_time);
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("frame_id"), pmt::from_uint64(burst.frame_id));
    meta =
        pmt::dict_add(meta, pmt::mp("attempt_id"), pmt::from_uint64(burst.poll_id));
    meta = pmt::dict_add(meta, pmt::mp("packet_type"), pmt::from_long(burst.packet_type));
    meta = pmt::dict_add(
        meta, pmt::mp("poll_id"), pmt::from_uint64(burst.poll_id));
    meta = pmt::dict_add(
        meta, pmt::mp("anchor_id"), pmt::from_uint64(burst.anchor_id));
    meta = pmt::dict_add(meta,
                         pmt::mp("reply_delay_samples"),
                         pmt::from_uint64(burst.reply_delay_samples));
    meta = pmt::dict_add(meta,
                         pmt::mp("tx_time_secs"),
                         pmt::from_uint64(static_cast<uint64_t>(secs_floor)));
    meta = pmt::dict_add(
        meta, pmt::mp("tx_time_frac"), pmt::from_double(burst.tx_time - secs_floor));
    meta = pmt::dict_add(meta, pmt::mp("poll_fine_delay_q10"), pmt::from_long(burst.poll_fine_delay_q10));
    meta = pmt::dict_add(meta, pmt::mp("fine_delay_valid"), burst.fine_delay_valid ? pmt::PMT_T : pmt::PMT_F);
    meta = pmt::dict_add(meta, pmt::mp("burst_len"), pmt::from_long(frame_len()));
    meta = pmt::dict_add(meta, pmt::mp("prs_start"), pmt::from_long(d_prs_start));
    meta = pmt::dict_add(meta, pmt::mp("prs_len"), pmt::from_long(d_prs_len));
    meta = pmt::dict_add(meta, pmt::mp("fft_len"), pmt::from_long(d_fft_len));
    meta = pmt::dict_add(meta, pmt::mp("cp_len"), pmt::from_long(d_cp_len));
    meta = pmt::dict_add(meta, pmt::mp("active_bins"), pmt::from_long(d_active_bins));
    meta = pmt::dict_add(meta, pmt::mp("prs_symbols"), pmt::from_long(d_prs_symbols));
    meta = pmt::dict_add(
        meta, pmt::mp("mc_ds_gold_code_id"), pmt::from_long(d_mc_ds_gold_code_id));
    message_port_pub(pmt::mp("tx_time_out"), meta);
}

int prs_timed_burst_source_impl::general_work(int noutput_items,
                                              gr_vector_int& ninput_items,
                                              gr_vector_const_void_star& input_items,
                                              gr_vector_void_star& output_items)
{
    auto out = static_cast<gr_complex*>(output_items[0]);
    (void)input_items;

    const bool have_stream_input = !ninput_items.empty();
    if (have_stream_input) {
        const uint64_t abs_in_start = nitems_read(0);
        std::vector<tag_t> tags;
        get_tags_in_range(tags,
                          0,
                          abs_in_start,
                          abs_in_start + static_cast<uint64_t>(ninput_items[0]),
                          pmt::mp("rx_time"));
        for (const auto& tag : tags) {
            if (pmt::is_tuple(tag.value) && pmt::length(tag.value) >= 2) {
                const pmt::pmt_t secs_pmt = pmt::tuple_ref(tag.value, 0);
                const pmt::pmt_t frac_pmt = pmt::tuple_ref(tag.value, 1);
                d_rx_time_abs_sample = tag.offset;
                d_rx_time_value = static_cast<double>(pmt_to_uint64(secs_pmt)) +
                                  pmt::to_double(frac_pmt);
                d_have_rx_time = true;
            }
        }
    }

    const int consumed = ninput_items.empty() ? 0 : ninput_items[0];

    int produced = 0;
    while (produced < noutput_items) {
        if (!d_in_burst) {
            if (d_scheduler.empty()) {
                break;
            }
            if (!std::isfinite(d_scheduler.front().tx_time) && !d_have_rx_time &&
                !have_stream_input) {
                break;
            }
            d_current_burst = d_scheduler.pop_front();
            if (!std::isfinite(d_current_burst.tx_time)) {
                d_current_burst.tx_time = current_time_estimate() + d_tx_lead_time +
                                          d_current_burst.sequence_delay;
            }
            d_in_burst = true;
            d_burst_offset = 0;
            prepare_burst_frame(d_current_burst.frame_id);
            const uint64_t abs_out = nitems_written(0) + produced;
            add_burst_tags(abs_out, d_current_burst);
            publish_tx_time(d_current_burst);
            PRS_TBS_DEBUG(
                "PRS burst start: frame_id={} tx_time={} out_abs={} frame_len={}",
                d_current_burst.frame_id,
                d_current_burst.tx_time,
                abs_out,
                d_frame.size());
        }

        const size_t remaining_burst = d_burst_frame.size() - d_burst_offset;
        const int remaining_output = noutput_items - produced;
        const size_t ncopy =
            std::min(remaining_burst, static_cast<size_t>(remaining_output));
        std::copy(d_burst_frame.begin() + d_burst_offset,
                  d_burst_frame.begin() + d_burst_offset + ncopy,
                  out + produced);
        d_burst_offset += ncopy;
        produced += static_cast<int>(ncopy);
        PRS_TBS_DEBUG("PRS produced samples: frame_id={} ncopy={} burst_offset={} "
                      "produced_this_call={}",
                      d_current_burst.frame_id,
                      ncopy,
                      d_burst_offset,
                      produced);

        if (d_burst_offset == d_burst_frame.size()) {
            const uint64_t eob_offset = nitems_written(0) + produced - 1;
            add_item_tag(0, eob_offset, pmt::mp("tx_eob"), pmt::PMT_T);
            PRS_TBS_DEBUG("PRS burst complete: frame_id={} eob_abs={} total_samples={}",
                          d_current_burst.frame_id,
                          eob_offset,
                          d_frame.size());
            d_in_burst = false;
            d_burst_offset = 0;
        }
    }

    consume_each(consumed);
    return produced;
}

} /* namespace ofdm_prs_ranging */
} /* namespace gr */
