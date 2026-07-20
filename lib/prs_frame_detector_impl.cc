/* -*- c++ -*- */
/*
 * Copyright 2026 GNU Radio ZC TWR contributors.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "prs_frame_detector_impl.h"
#include "prs_payload_codec.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>

namespace gr {
namespace ofdm_prs_ranging {

prs_frame_detector::sptr prs_frame_detector::make(double samp_rate,
                                                  int fft_len,
                                                  int cp_len,
                                                  int active_bins,
                                                  int prs_symbols,
                                                  int preamble_len,
                                                  int preamble_repeats,
                                                  int coarse_sync_len,
                                                  int zero_guard_len,
                                                  int tail_guard_len,
                                                  float threshold,
                                                  int min_frame_gap,
                                                  int coarse_zc_root,
                                                  int channel_id)
{
    return gnuradio::make_block_sptr<prs_frame_detector_impl>(samp_rate,
                                                              fft_len,
                                                              cp_len,
                                                              active_bins,
                                                              prs_symbols,
                                                              preamble_len,
                                                              preamble_repeats,
                                                              coarse_sync_len,
                                                              zero_guard_len,
                                                              tail_guard_len,
                                                              threshold,
                                                              min_frame_gap,
                                                              coarse_zc_root,
                                                              channel_id);
}

prs_frame_detector_impl::prs_frame_detector_impl(double samp_rate,
                                                 int fft_len,
                                                 int cp_len,
                                                 int active_bins,
                                                 int prs_symbols,
                                                 int preamble_len,
                                                 int preamble_repeats,
                                                 int coarse_sync_len,
                                                 int zero_guard_len,
                                                 int tail_guard_len,
                                                 float threshold,
                                                 int min_frame_gap,
                                                 int coarse_zc_root,
                                                 int channel_id)
    : gr::block("prs_frame_detector",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(0, 0, 0)),
      d_threshold(threshold),
      d_min_frame_gap(min_frame_gap),
      d_next_scan_index(0),
      d_buffer_abs_start(0),
      d_total_seen(0),
      d_next_frame_id(0),
      d_last_frame_start(-min_frame_gap),
      d_have_rx_time(false),
      d_rx_time_tag_offset(0),
      d_rx_time_secs(0),
      d_rx_time_frac(0.0)
{
    d_cfg.samp_rate = samp_rate;
    d_cfg.fft_len = fft_len;
    d_cfg.cp_len = cp_len;
    d_cfg.active_bins = active_bins;
    d_cfg.prs_symbols = prs_symbols;
    d_cfg.preamble_len = preamble_len;
    d_cfg.preamble_repeats = preamble_repeats;
    d_cfg.coarse_sync_len = coarse_sync_len;
    d_cfg.zero_guard_len = zero_guard_len;
    d_cfg.tail_guard_len = tail_guard_len;
    d_cfg.coarse_zc_root = coarse_zc_root;
    d_cfg.channel_id = channel_id;
    d_coarse = coarse_sync_sequence(d_cfg.coarse_sync_len, d_cfg.coarse_zc_root);
    message_port_register_out(pmt::mp("frame_out"));
}

void prs_frame_detector_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    (void)noutput_items;
    ninput_items_required[0] = 1;
}

void prs_frame_detector_impl::update_rx_time_tags(uint64_t abs_start, uint64_t abs_stop)
{
    std::vector<tag_t> tags;
    get_tags_in_range(tags, 0, abs_start, abs_stop, pmt::mp("rx_time"));
    for (const auto& tag : tags) {
        if (pmt::is_tuple(tag.value) && pmt::length(tag.value) >= 2) {
            d_rx_time_tag_offset = tag.offset;
            const auto secs = pmt::tuple_ref(tag.value, 0);
            const auto frac = pmt::tuple_ref(tag.value, 1);
            d_rx_time_secs = pmt::is_uint64(secs) ? pmt::to_uint64(secs)
                                                  : static_cast<uint64_t>(pmt::to_long(secs));
            d_rx_time_frac = pmt::to_double(frac);
            d_have_rx_time = true;
        }
    }
}

float prs_frame_detector_impl::coarse_sync_metric(size_t coarse_index) const
{
    gr_complex corr(0.0f, 0.0f);
    double power = 0.0;
    for (int i = 0; i < d_cfg.coarse_sync_len; ++i) {
        const auto sample = d_buffer[coarse_index + i];
        corr += sample * std::conj(d_coarse[i]);
        power += std::norm(sample);
    }
    const double denom = std::sqrt(power * static_cast<double>(d_cfg.coarse_sync_len));
    return denom > 0.0 ? static_cast<float>(std::abs(corr) / denom) : 0.0f;
}

bool prs_frame_detector_impl::find_frame(size_t& frame_start_index,
                                         size_t& coarse_index,
                                         float& preamble_metric_out,
                                         float& coarse_metric_out)
{
    const int flen = frame_len(d_cfg);
    const int coarse_rel = d_cfg.zero_guard_len + d_cfg.preamble_len * d_cfg.preamble_repeats;
    const int preamble_rel = d_cfg.zero_guard_len;
    if (d_buffer.size() < static_cast<size_t>(flen)) {
        return false;
    }

    const size_t first_preamble = static_cast<size_t>(preamble_rel);
    if (d_next_scan_index < first_preamble) {
        d_next_scan_index = first_preamble;
    }
    const size_t max_preamble = d_buffer.size() - static_cast<size_t>(flen) +
                                static_cast<size_t>(preamble_rel);
    if (d_next_scan_index > max_preamble) {
        return false;
    }

    const int span = d_cfg.preamble_len * (d_cfg.preamble_repeats - 1);
    const auto add_pair = [this](size_t index,
                                 gr_complex& corr,
                                 double& first_power,
                                 double& second_power) {
        const auto first = d_buffer[index];
        const auto second = d_buffer[index + d_cfg.preamble_len];
        corr += std::conj(first) * second;
        first_power += std::norm(first);
        second_power += std::norm(second);
    };
    const auto remove_pair = [this](size_t index,
                                    gr_complex& corr,
                                    double& first_power,
                                    double& second_power) {
        const auto first = d_buffer[index];
        const auto second = d_buffer[index + d_cfg.preamble_len];
        corr -= std::conj(first) * second;
        first_power -= std::norm(first);
        second_power -= std::norm(second);
    };

    gr_complex preamble_corr(0.0f, 0.0f);
    double first_power = 0.0;
    double second_power = 0.0;
    for (int i = 0; i < span; ++i) {
        add_pair(d_next_scan_index + static_cast<size_t>(i),
                 preamble_corr,
                 first_power,
                 second_power);
    }

    for (size_t p = d_next_scan_index; p <= max_preamble; ++p) {
        const size_t start = p - static_cast<size_t>(preamble_rel);
        const int64_t abs_start = static_cast<int64_t>(d_buffer_abs_start + start);
        if (abs_start - d_last_frame_start < d_min_frame_gap) {
        } else {
            const double denom = std::sqrt(std::max(0.0, first_power) *
                                           std::max(0.0, second_power));
            const float preamble_metric =
                denom > 0.0 ? static_cast<float>(std::abs(preamble_corr) / denom) : 0.0f;
            if (preamble_metric >= d_threshold) {
                const size_t c = start + static_cast<size_t>(coarse_rel);
                const float m = coarse_sync_metric(c);
                if (m >= d_threshold) {
                    d_next_scan_index = p + static_cast<size_t>(d_min_frame_gap);
                    frame_start_index = start;
                    coarse_index = c;
                    preamble_metric_out = preamble_metric;
                    coarse_metric_out = m;
                    return true;
                }
            }
        }

        if (p < max_preamble) {
            remove_pair(p, preamble_corr, first_power, second_power);
            add_pair(p + static_cast<size_t>(span),
                     preamble_corr,
                     first_power,
                     second_power);
        }
    }

    d_next_scan_index = max_preamble + 1;
    return false;
}

void prs_frame_detector_impl::publish_frame(size_t frame_start_index,
                                            size_t coarse_index,
                                            float preamble_metric,
                                            float coarse_metric)
{
    const int flen = frame_len(d_cfg);
    const uint64_t abs_start = d_buffer_abs_start + frame_start_index;
    const uint64_t coarse_abs = d_buffer_abs_start + coarse_index;
    std::vector<gr_complex> frame(d_buffer.begin() + frame_start_index,
                                  d_buffer.begin() + frame_start_index + flen);
    prs_payload_info payload_info;
    float payload_metric = 0.0f;
    const int payload_start =
        d_cfg.zero_guard_len + d_cfg.preamble_len * d_cfg.preamble_repeats +
        d_cfg.coarse_sync_len;
    const bool frame_id_valid =
        decode_packet_payload(frame.data() + payload_start,
                              d_cfg.payload_len,
                              payload_info,
                              payload_metric);
    const uint64_t tx_frame_id =
        payload_info.packet_type == prs_packet_type_response ? payload_info.response_frame_id
                                                             : payload_info.poll_frame_id;
    gr_complex cfo_corr(0.0f, 0.0f);
    const int preamble_start = d_cfg.zero_guard_len;
    const int preamble_span = d_cfg.preamble_len * (d_cfg.preamble_repeats - 1);
    for (int i = 0; i < preamble_span; ++i) {
        cfo_corr += std::conj(frame[preamble_start + i]) *
                    frame[preamble_start + i + d_cfg.preamble_len];
    }
    const double cfo_hz = std::atan2(cfo_corr.imag(), cfo_corr.real()) *
                          d_cfg.samp_rate /
                          (2.0 * 3.141592653589793238462643383279502884 *
                           d_cfg.preamble_len);

    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("recv_id"), pmt::from_uint64(d_next_frame_id++));
    meta = pmt::dict_add(meta, pmt::mp("frame_id"), pmt::from_uint64(tx_frame_id));
    meta = pmt::dict_add(meta, pmt::mp("packet_type"), pmt::from_long(payload_info.packet_type));
    meta = pmt::dict_add(meta, pmt::mp("poll_frame_id"), pmt::from_uint64(payload_info.poll_frame_id));
    meta = pmt::dict_add(meta, pmt::mp("response_frame_id"), pmt::from_uint64(payload_info.response_frame_id));
    meta = pmt::dict_add(meta, pmt::mp("reply_delay_samples"), pmt::from_uint64(payload_info.reply_delay_samples));
    meta = pmt::dict_add(meta, pmt::mp("frame_id_valid"), frame_id_valid ? pmt::PMT_T : pmt::PMT_F);
    meta = pmt::dict_add(meta, pmt::mp("payload_metric"), pmt::from_double(payload_metric));
    meta = pmt::dict_add(meta, pmt::mp("absolute_sample_index"), pmt::from_uint64(abs_start));
    meta = pmt::dict_add(meta, pmt::mp("frame_start"), pmt::from_uint64(abs_start));
    meta = pmt::dict_add(meta, pmt::mp("coarse_peak"), pmt::from_uint64(coarse_abs));
    meta = pmt::dict_add(meta, pmt::mp("peak_metric"), pmt::from_double(coarse_metric));
    meta = pmt::dict_add(meta, pmt::mp("preamble_metric"), pmt::from_double(preamble_metric));
    meta = pmt::dict_add(meta, pmt::mp("coarse_metric"), pmt::from_double(coarse_metric));
    meta = pmt::dict_add(meta, pmt::mp("coarse_zc_root"), pmt::from_long(d_cfg.coarse_zc_root));
    meta = pmt::dict_add(meta, pmt::mp("channel_id"), pmt::from_long(d_cfg.channel_id));
    meta = pmt::dict_add(meta, pmt::mp("cfo"), pmt::from_double(cfo_hz));
    meta = pmt::dict_add(meta, pmt::mp("samp_rate"), pmt::from_double(d_cfg.samp_rate));
    meta = pmt::dict_add(meta, pmt::mp("fft_len"), pmt::from_long(d_cfg.fft_len));
    meta = pmt::dict_add(meta, pmt::mp("cp_len"), pmt::from_long(d_cfg.cp_len));
    meta = pmt::dict_add(meta, pmt::mp("active_bins"), pmt::from_long(d_cfg.active_bins));
    meta = pmt::dict_add(meta, pmt::mp("prs_symbols"), pmt::from_long(d_cfg.prs_symbols));
    meta = pmt::dict_add(meta, pmt::mp("prs_start_rel"), pmt::from_long(prs_start_offset(d_cfg)));
    meta = pmt::dict_add(meta, pmt::mp("prs_len"), pmt::from_long(prs_len(d_cfg)));
    if (d_have_rx_time) {
        const double rx_time =
            static_cast<double>(d_rx_time_secs) + d_rx_time_frac +
            static_cast<double>(abs_start - d_rx_time_tag_offset) / d_cfg.samp_rate;
        meta = pmt::dict_add(meta,
                             pmt::mp("rx_time"),
                             pmt::make_tuple(pmt::from_uint64(static_cast<uint64_t>(std::floor(rx_time))),
                                             pmt::from_double(rx_time - std::floor(rx_time))));
        meta = pmt::dict_add(meta, pmt::mp("rx_time_tag_offset"), pmt::from_uint64(d_rx_time_tag_offset));
    }

    message_port_pub(pmt::mp("frame_out"),
                     pmt::cons(meta, pmt::init_c32vector(frame.size(), frame)));
    d_last_frame_start = static_cast<int64_t>(abs_start);
}

int prs_frame_detector_impl::general_work(int noutput_items,
                                          gr_vector_int& ninput_items,
                                          gr_vector_const_void_star& input_items,
                                          gr_vector_void_star& output_items)
{
    (void)noutput_items;
    (void)output_items;
    const auto in = static_cast<const gr_complex*>(input_items[0]);
    const int ninput = ninput_items[0];
    const uint64_t abs_start = nitems_read(0);
    update_rx_time_tags(abs_start, abs_start + ninput);

    d_buffer.insert(d_buffer.end(), in, in + ninput);
    d_total_seen += ninput;

    size_t frame_start = 0;
    size_t coarse = 0;
    float preamble_metric = 0.0f;
    float coarse_metric = 0.0f;
    while (find_frame(frame_start, coarse, preamble_metric, coarse_metric)) {
        publish_frame(frame_start, coarse, preamble_metric, coarse_metric);
        const size_t drop = frame_start + static_cast<size_t>(frame_len(d_cfg));
        d_buffer.erase(d_buffer.begin(), d_buffer.begin() + drop);
        d_buffer_abs_start += drop;
        d_next_scan_index = d_next_scan_index > drop ? d_next_scan_index - drop : 0;
    }

    const size_t keep = static_cast<size_t>(frame_len(d_cfg) + d_cfg.coarse_sync_len);
    if (d_buffer.size() > keep) {
        const size_t drop = d_buffer.size() - keep;
        d_buffer.erase(d_buffer.begin(), d_buffer.begin() + drop);
        d_buffer_abs_start += drop;
        d_next_scan_index = d_next_scan_index > drop ? d_next_scan_index - drop : 0;
    }

    consume_each(ninput);
    return 0;
}

} // namespace ofdm_prs_ranging
} // namespace gr
