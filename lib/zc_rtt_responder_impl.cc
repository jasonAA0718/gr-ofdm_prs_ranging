/* -*- c++ -*- */
#include "zc_rtt_responder_impl.h"
#include "zc_packet_utils.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace gr {
namespace ofdm_prs_ranging {

zc_rtt_responder::sptr zc_rtt_responder::make(const std::vector<gr_complex>& zc_seq,
                                              double samp_rate,
                                              double delay_secs,
                                              int zc_length,
                                              float peak_metric_threshold,
                                              float fixed_threshold)
{
    return gnuradio::make_block_sptr<zc_rtt_responder_impl>(
        zc_seq, samp_rate, delay_secs, zc_length, peak_metric_threshold, fixed_threshold);
}

zc_rtt_responder_impl::zc_rtt_responder_impl(const std::vector<gr_complex>& zc_seq,
                                             double samp_rate,
                                             double delay_secs,
                                             int zc_length,
                                             float peak_metric_threshold,
                                             float fixed_threshold)
    : gr::block("zc_rtt_responder",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_zc_seq(zc_seq),
      d_samp_rate(samp_rate),
      d_delay_secs(delay_secs),
      d_zc_length(zc_length),
      d_peak_metric_threshold(peak_metric_threshold),
      d_fixed_threshold(fixed_threshold),
      d_next_allowed_tx_offset(static_cast<int64_t>(0.01 * samp_rate)),
      d_cooldown_samples(static_cast<int64_t>(0.001 * samp_rate))
{
    const int nbits = zc_response_packet_bits;
    const int seq_len = d_zc_length + 100 + d_sps * nbits + 100;
    set_min_output_buffer(0, seq_len + 1000);
}

int64_t zc_rtt_responder_impl::pmt_time_to_ticks(pmt::pmt_t time) const
{
    const auto sec = static_cast<int64_t>(pmt::to_uint64(pmt::tuple_ref(time, 0)));
    const double frac = pmt::to_double(pmt::tuple_ref(time, 1));
    const int64_t sr = static_cast<int64_t>(std::llround(d_samp_rate));
    return sec * sr + static_cast<int64_t>(std::llround(frac * sr));
}

pmt::pmt_t zc_rtt_responder_impl::ticks_to_pmt_time(int64_t ticks) const
{
    const int64_t sr = static_cast<int64_t>(std::llround(d_samp_rate));
    const uint64_t sec = static_cast<uint64_t>(ticks / sr);
    const double frac = static_cast<double>(ticks % sr) / static_cast<double>(sr);
    return pmt::make_tuple(pmt::from_uint64(sec), pmt::from_double(frac));
}

int zc_rtt_responder_impl::find_peak(const gr_complex* in, int nitems) const
{
    if (nitems <= 0) {
        return -1;
    }
    double power = 0.0;
    for (int i = 0; i < nitems; ++i) {
        power += std::norm(in[i]);
    }
    const double mean_power = power / static_cast<double>(nitems);
    const double denom = std::sqrt(std::max(1e-18, mean_power) * d_zc_length);
    int best = -1;
    float best_mag = 0.0f;
    for (int i = 0; i < nitems; ++i) {
        const float mag = std::abs(in[i]);
        const float metric = static_cast<float>(mag / denom);
        if (metric >= d_peak_metric_threshold && mag >= d_fixed_threshold && mag > best_mag) {
            best = i;
            best_mag = mag;
        }
    }
    return best;
}

std::vector<gr_complex>
zc_rtt_responder_impl::generate_bpsk(const std::vector<uint8_t>& bits) const
{
    std::vector<gr_complex> samples;
    samples.reserve(bits.size() * d_sps);
    for (const int bit : bits) {
        const gr_complex sym = bit ? gr_complex(-0.7f, 0.0f) : gr_complex(0.7f, 0.0f);
        for (int i = 0; i < d_sps; ++i) {
            samples.push_back(sym);
        }
    }
    return samples;
}

int zc_rtt_responder_impl::output_burst_chunk(gr_complex* out, int noutput_items)
{
    const int remaining = static_cast<int>(d_burst.size()) - d_burst_pos;
    const int to_copy = std::min(remaining, noutput_items);
    if (to_copy <= 0) {
        d_burst_active = false;
        d_burst_pos = 0;
        return 0;
    }

    std::copy(d_burst.begin() + d_burst_pos, d_burst.begin() + d_burst_pos + to_copy, out);
    const uint64_t abs_out_start = nitems_written(0);

    if (d_burst_pos == 0) {
        ++d_transmit_num;
        std::cout << "TX " << d_transmit_num << std::endl;
        add_item_tag(0, abs_out_start, pmt::mp("tx_time"), d_pending_tx_time, pmt::mp("resp"));
        add_item_tag(0, abs_out_start, pmt::mp("tx_sob"), pmt::PMT_T, pmt::mp("resp"));
    }

    d_burst_pos += to_copy;
    if (d_burst_pos >= static_cast<int>(d_burst.size())) {
        add_item_tag(0,
                     abs_out_start + to_copy - 1,
                     pmt::mp("tx_eob"),
                     pmt::PMT_T,
                     pmt::mp("resp"));
        d_burst_active = false;
        d_burst_pos = 0;
        d_burst.clear();
        d_pending_tx_time = pmt::PMT_NIL;
    }
    return to_copy;
}

int zc_rtt_responder_impl::general_work(int noutput_items,
                                       gr_vector_int& ninput_items,
                                       gr_vector_const_void_star& input_items,
                                       gr_vector_void_star& output_items)
{
    const auto in = static_cast<const gr_complex*>(input_items[0]);
    auto out = static_cast<gr_complex*>(output_items[0]);
    const int ninput = ninput_items[0];

    if (d_burst_active) {
        const int produced = output_burst_chunk(out, noutput_items);
        consume(0, ninput);
        produce(0, produced);
        return WORK_CALLED_PRODUCE;
    }

    std::vector<tag_t> tags;
    get_tags_in_window(tags, 0, 0, ninput);
    for (const auto& tag : tags) {
        if (pmt::symbol_to_string(tag.key) == "rx_time" && pmt::is_tuple(tag.value)) {
            d_last_rx_time = tag.value;
            d_last_tag_offset = tag.offset;
            d_have_rx_time = true;
        }
    }

    const int peak_local = find_peak(in, ninput);
    if (peak_local < 0 || !d_have_rx_time) {
        consume(0, ninput);
        return WORK_CALLED_PRODUCE;
    }

    const int64_t current_peak_offset = static_cast<int64_t>(nitems_read(0)) + peak_local;
    if (current_peak_offset < d_next_allowed_tx_offset) {
        consume(0, ninput);
        return WORK_CALLED_PRODUCE;
    }

    const int64_t rx_base_tick = pmt_time_to_ticks(d_last_rx_time);
    const int64_t diff_samples = current_peak_offset - static_cast<int64_t>(d_last_tag_offset);
    const int64_t t2_tick = rx_base_tick + diff_samples - d_zc_length;
    const int64_t delay_samples = static_cast<int64_t>(std::llround(d_delay_secs * d_samp_rate));
    const int64_t t3_tick = t2_tick + delay_samples;
    d_pending_tx_time = ticks_to_pmt_time(t3_tick);

    const int ts_mask = (1 << zc_t2_minus_t3_bits) - 1;
    const int measurement_mask = (1 << zc_measurement_number_bits) - 1;
    const int t2_minus_t3_payload = static_cast<int>((t2_tick - t3_tick) & ts_mask);
    d_measurement_number = (d_measurement_number + 1) & measurement_mask;

    const auto packet_bits =
        zc_build_response_packet(d_responder_id, t2_minus_t3_payload, d_measurement_number);
    const auto bpsk = generate_bpsk(packet_bits);

    d_burst.clear();
    d_burst.reserve(d_zc_seq.size() + 100 + bpsk.size() + 100);
    d_burst.insert(d_burst.end(), d_zc_seq.begin(), d_zc_seq.end());
    d_burst.insert(d_burst.end(), 100, gr_complex(0, 0));
    d_burst.insert(d_burst.end(), bpsk.begin(), bpsk.end());
    d_burst.insert(d_burst.end(), 100, gr_complex(0, 0));
    d_burst_pos = 0;
    d_burst_active = true;
    d_next_allowed_tx_offset = current_peak_offset + d_cooldown_samples;

    const int produced = output_burst_chunk(out, noutput_items);
    consume(0, ninput);
    produce(0, produced);
    return WORK_CALLED_PRODUCE;
}

} // namespace ofdm_prs_ranging
} // namespace gr
