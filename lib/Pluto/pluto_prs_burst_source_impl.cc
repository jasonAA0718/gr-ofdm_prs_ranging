/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "pluto_prs_burst_source_impl.h"
#include "prs_payload_codec.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <stdexcept>

namespace gr {
namespace ofdm_prs_ranging {

namespace {
uint64_t dict_u64(const pmt::pmt_t& dict, const char* key, uint64_t fallback)
{
    const auto value = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_uint64(value)) {
        return pmt::to_uint64(value);
    }
    if (pmt::is_integer(value)) {
        return static_cast<uint64_t>(pmt::to_long(value));
    }
    return fallback;
}
} // namespace

pluto_prs_burst_source::sptr pluto_prs_burst_source::make(double samp_rate,
                                                          int fft_len,
                                                          int cp_len,
                                                          int active_bins,
                                                          int prs_symbols,
                                                          int preamble_len,
                                                          int preamble_repeats,
                                                          int coarse_sync_len,
                                                          int zero_guard_len,
                                                          int tail_guard_len,
                                                          float tx_amp,
                                                          uint32_t seed,
                                                          int default_packet_type,
                                                          uint64_t guard_samples,
                                                          int coarse_zc_root)
{
    return gnuradio::make_block_sptr<pluto_prs_burst_source_impl>(samp_rate,
                                                                  fft_len,
                                                                  cp_len,
                                                                  active_bins,
                                                                  prs_symbols,
                                                                  preamble_len,
                                                                  preamble_repeats,
                                                                  coarse_sync_len,
                                                                  zero_guard_len,
                                                                  tail_guard_len,
                                                                  tx_amp,
                                                                  seed,
                                                                  default_packet_type,
                                                                  guard_samples,
                                                                  coarse_zc_root);
}

pluto_prs_burst_source_impl::pluto_prs_burst_source_impl(double samp_rate,
                                                         int fft_len,
                                                         int cp_len,
                                                         int active_bins,
                                                         int prs_symbols,
                                                         int preamble_len,
                                                         int preamble_repeats,
                                                         int coarse_sync_len,
                                                         int zero_guard_len,
                                                         int tail_guard_len,
                                                         float tx_amp,
                                                         uint32_t seed,
                                                         int default_packet_type,
                                                         uint64_t guard_samples,
                                                         int coarse_zc_root)
    : gr::sync_block("pluto_prs_burst_source",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_samp_rate(samp_rate),
      d_fft_len(fft_len),
      d_cp_len(cp_len),
      d_active_bins(active_bins),
      d_prs_symbols(prs_symbols),
      d_preamble_len(preamble_len),
      d_preamble_repeats(preamble_repeats),
      d_coarse_sync_len(coarse_sync_len),
      d_zero_guard_len(zero_guard_len),
      d_tail_guard_len(tail_guard_len),
      d_tx_amp(tx_amp),
      d_seed(seed),
      d_default_packet_type(static_cast<uint8_t>(default_packet_type)),
      d_default_guard_samples(guard_samples),
      d_coarse_zc_root(coarse_zc_root)
{
    validate_parameters();
    build_frame();
    message_port_register_in(pmt::mp("trigger"));
    message_port_register_out(pmt::mp("tx_event_out"));
    set_msg_handler(pmt::mp("trigger"),
                    [this](pmt::pmt_t msg) { handle_trigger(std::move(msg)); });
}

void pluto_prs_burst_source_impl::validate_parameters() const
{
    if (d_samp_rate <= 0.0) {
        throw std::invalid_argument("samp_rate must be positive");
    }
    if (d_fft_len != 1024 || d_cp_len != 128 || d_active_bins != 1024 ||
        d_prs_symbols != 16) {
        throw std::invalid_argument(
            "Golay PRS requires fft_len=1024, cp_len=128, active_bins=1024, "
            "prs_symbols=16");
    }
    if (d_preamble_len <= 0 || d_preamble_repeats <= 0 || d_coarse_sync_len <= 0 ||
        d_zero_guard_len < 0 || d_tail_guard_len < 0) {
        throw std::invalid_argument("frame lengths are invalid");
    }
    if (d_tx_amp <= 0.0f || d_tx_amp > 0.8f) {
        throw std::invalid_argument("tx_amp must be in (0, 0.8]");
    }
    if (d_default_packet_type != prs_packet_type_poll &&
        d_default_packet_type != prs_packet_type_response) {
        throw std::invalid_argument(
            "default_packet_type must be poll (1) or response (2)");
    }
}

prs_frame_config pluto_prs_burst_source_impl::frame_config() const
{
    return prs_frame_config{ d_samp_rate,
                             d_fft_len,
                             d_cp_len,
                             d_active_bins,
                             d_prs_symbols,
                             d_preamble_len,
                             d_preamble_repeats,
                             d_coarse_sync_len,
                             prs_frame_id_payload_symbols,
                             d_zero_guard_len,
                             d_tail_guard_len,
                             d_tx_amp,
                             d_seed,
                             d_coarse_zc_root };
}

void pluto_prs_burst_source_impl::build_frame()
{
    const auto frame = prs_frame_builder::build(frame_config());
    d_frame = frame.samples;
    d_burst_frame = d_frame;
    d_prs_start = frame.prs_start;
    d_prs_len = frame.prs_len;
    d_payload_start = frame.payload_start;
    d_payload_len = frame.payload_len;
}

void pluto_prs_burst_source_impl::prepare_burst_frame()
{
    d_burst_frame = d_frame;
    prs_payload_info info;
    info.packet_type = d_current.packet_type;
    info.poll_frame_id = d_current.poll_frame_id;
    info.response_frame_id = d_current.response_frame_id;
    info.reply_delay_samples = d_current.reply_delay_samples;
    encode_packet_payload(info, 1.0f, d_burst_frame.begin() + d_payload_start);
    prs_frame_builder::normalize_sections(d_burst_frame,
                                          frame_config(),
                                          d_payload_start,
                                          d_payload_len,
                                          d_prs_start,
                                          d_prs_len);
}

void pluto_prs_burst_source_impl::handle_trigger(pmt::pmt_t msg)
{
    std::lock_guard<std::mutex> lock(d_queue_mutex);
    const bool is_dict = pmt::is_dict(msg);
    const uint8_t packet_type =
        static_cast<uint8_t>(is_dict ? dict_u64(msg, "packet_type", d_default_packet_type)
                                     : d_default_packet_type);
    if (packet_type != prs_packet_type_poll && packet_type != prs_packet_type_response) {
        return;
    }

    const uint32_t allocated = d_next_frame_id++;
    const uint32_t response_id = static_cast<uint32_t>(
        is_dict ? dict_u64(msg, "response_frame_id", allocated) : allocated);
    const uint32_t poll_id = static_cast<uint32_t>(
        is_dict ? dict_u64(msg,
                           "poll_frame_id",
                           packet_type == prs_packet_type_poll ? allocated : 0)
                : allocated);
    if (packet_type == prs_packet_type_response && poll_id == 0) {
        return;
    }

    d_pending.push_back(pending_burst{
        packet_type,
        poll_id,
        response_id,
        static_cast<uint32_t>(is_dict ? dict_u64(msg, "reply_delay_samples", 0) : 0),
        is_dict ? dict_u64(msg, "guard_samples", d_default_guard_samples)
                : d_default_guard_samples });
}

bool pluto_prs_burst_source_impl::start_next_burst(uint64_t absolute_offset)
{
    (void)absolute_offset;
    {
        std::lock_guard<std::mutex> lock(d_queue_mutex);
        if (d_pending.empty()) {
            return false;
        }
        d_current = d_pending.front();
        d_pending.pop_front();
    }
    d_active = true;
    d_guard_remaining = d_current.guard_samples;
    d_burst_offset = 0;
    prepare_burst_frame();
    return true;
}

void pluto_prs_burst_source_impl::add_burst_tags(uint64_t absolute_offset)
{
    const uint64_t frame_id = d_current.packet_type == prs_packet_type_response
                                  ? d_current.response_frame_id
                                  : d_current.poll_frame_id;
    add_item_tag(0, absolute_offset, pmt::mp("tx_sob"), pmt::PMT_T);
    add_item_tag(0, absolute_offset, pmt::mp("frame_id"), pmt::from_uint64(frame_id));
    add_item_tag(0,
                 absolute_offset,
                 pmt::mp("attempt_id"),
                 pmt::from_uint64(d_current.poll_frame_id));
    add_item_tag(0,
                 absolute_offset,
                 pmt::mp("packet_type"),
                 pmt::from_long(d_current.packet_type));
    add_item_tag(0,
                 absolute_offset,
                 pmt::mp("poll_frame_id"),
                 pmt::from_uint64(d_current.poll_frame_id));
    add_item_tag(0,
                 absolute_offset,
                 pmt::mp("response_frame_id"),
                 pmt::from_uint64(d_current.response_frame_id));
    add_item_tag(0, absolute_offset, pmt::mp("burst_len"), pmt::from_long(frame_len()));
}

void pluto_prs_burst_source_impl::publish_tx_event(uint64_t absolute_offset)
{
    const uint64_t frame_id = d_current.packet_type == prs_packet_type_response
                                  ? d_current.response_frame_id
                                  : d_current.poll_frame_id;
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("frame_id"), pmt::from_uint64(frame_id));
    meta = pmt::dict_add(
        meta, pmt::mp("attempt_id"), pmt::from_uint64(d_current.poll_frame_id));
    meta = pmt::dict_add(
        meta, pmt::mp("packet_type"), pmt::from_long(d_current.packet_type));
    meta = pmt::dict_add(
        meta, pmt::mp("poll_frame_id"), pmt::from_uint64(d_current.poll_frame_id));
    meta = pmt::dict_add(meta,
                         pmt::mp("response_frame_id"),
                         pmt::from_uint64(d_current.response_frame_id));
    meta = pmt::dict_add(
        meta, pmt::mp("local_tx_sample_index"), pmt::from_uint64(absolute_offset));
    meta = pmt::dict_add(meta, pmt::mp("burst_len"), pmt::from_long(frame_len()));
    meta = pmt::dict_add(meta, pmt::mp("untimed_tx"), pmt::PMT_T);
    message_port_pub(pmt::mp("tx_event_out"), meta);
}

int pluto_prs_burst_source_impl::work(int noutput_items,
                                      gr_vector_const_void_star& input_items,
                                      gr_vector_void_star& output_items)
{
    (void)input_items;
    auto* out = static_cast<gr_complex*>(output_items[0]);
    std::fill(out, out + noutput_items, gr_complex(0.0f, 0.0f));

    int produced = 0;
    while (produced < noutput_items) {
        const uint64_t absolute_offset = nitems_written(0) + produced;
        if (!d_active && !start_next_burst(absolute_offset)) {
            break;
        }

        if (d_guard_remaining > 0) {
            const uint64_t available = static_cast<uint64_t>(noutput_items - produced);
            const uint64_t nskip = std::min(d_guard_remaining, available);
            d_guard_remaining -= nskip;
            produced += static_cast<int>(nskip);
            continue;
        }

        if (d_burst_offset == 0) {
            const uint64_t start = nitems_written(0) + produced;
            add_burst_tags(start);
            publish_tx_event(start);
        }
        const size_t remaining = d_burst_frame.size() - d_burst_offset;
        const size_t ncopy =
            std::min(remaining, static_cast<size_t>(noutput_items - produced));
        std::copy(d_burst_frame.begin() + d_burst_offset,
                  d_burst_frame.begin() + d_burst_offset + ncopy,
                  out + produced);
        d_burst_offset += ncopy;
        produced += static_cast<int>(ncopy);
        if (d_burst_offset == d_burst_frame.size()) {
            add_item_tag(
                0, nitems_written(0) + produced - 1, pmt::mp("tx_eob"), pmt::PMT_T);
            d_active = false;
            d_burst_offset = 0;
        }
    }
    return noutput_items;
}

} // namespace ofdm_prs_ranging
} // namespace gr
