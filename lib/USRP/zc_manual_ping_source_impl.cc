/* -*- c++ -*- */
#include "zc_manual_ping_source_impl.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <algorithm>
#include <cmath>

namespace gr {
namespace ofdm_prs_ranging {

zc_manual_ping_source::sptr
zc_manual_ping_source::make(double samp_rate,
                            const std::vector<gr_complex>& zc_seq,
                            int packet_len,
                            double tx_delay_secs,
                            double min_period_secs)
{
    return gnuradio::make_block_sptr<zc_manual_ping_source_impl>(
        samp_rate, zc_seq, packet_len, tx_delay_secs, min_period_secs);
}

zc_manual_ping_source_impl::zc_manual_ping_source_impl(
    double samp_rate,
    const std::vector<gr_complex>& zc_seq,
    int packet_len,
    double tx_delay_secs,
    double min_period_secs)
    : gr::block("zc_manual_ping_source",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_samp_rate(samp_rate),
      d_tx_delay_secs(tx_delay_secs),
      d_min_period_secs(min_period_secs),
      d_burst_len(std::max(packet_len, static_cast<int>(zc_seq.size())))
{
    d_burst.assign(d_burst_len, gr_complex(0, 0));
    std::copy(zc_seq.begin(), zc_seq.end(), d_burst.begin());
    set_min_noutput_items(d_burst_len);

    message_port_register_in(pmt::mp("trig_in"));
    set_msg_handler(pmt::mp("trig_in"),
                    [this](pmt::pmt_t msg) { this->handle_trigger(msg); });
    message_port_register_out(pmt::mp("tx_time_out"));
}

void zc_manual_ping_source_impl::handle_trigger(pmt::pmt_t)
{
    d_pending_pings += 100;
}

int zc_manual_ping_source_impl::general_work(int noutput_items,
                                            gr_vector_int& ninput_items,
                                            gr_vector_const_void_star& input_items,
                                            gr_vector_void_star& output_items)
{
    const int ninput = ninput_items[0];
    auto out = static_cast<gr_complex*>(output_items[0]);

    std::vector<tag_t> tags;
    get_tags_in_window(tags, 0, 0, ninput);
    for (const auto& tag : tags) {
        if (pmt::symbol_to_string(tag.key) == "rx_time" && pmt::is_tuple(tag.value)) {
            const auto sec = pmt::to_uint64(pmt::tuple_ref(tag.value, 0));
            const auto frac = pmt::to_double(pmt::tuple_ref(tag.value, 1));
            d_current_usrp_time = static_cast<double>(sec) + frac;
            d_last_rx_tag_offset = tag.offset;
            d_have_rx_time = true;
        }
    }

    if (!d_have_rx_time || d_pending_pings <= 0 || noutput_items < d_burst_len) {
        consume(0, ninput);
        return WORK_CALLED_PRODUCE;
    }

    const uint64_t current_offset = nitems_read(0);
    const double now_time =
        d_current_usrp_time +
        (static_cast<double>(current_offset - d_last_rx_tag_offset) / d_samp_rate);
    const double tx_time = now_time + d_tx_delay_secs;

    if ((now_time - d_last_tx_time) <= d_min_period_secs) {
        consume(0, ninput);
        return WORK_CALLED_PRODUCE;
    }

    const uint64_t sec = static_cast<uint64_t>(std::floor(tx_time));
    const double frac = tx_time - static_cast<double>(sec);
    const pmt::pmt_t time_pmt = pmt::make_tuple(pmt::from_uint64(sec), pmt::from_double(frac));
    const uint64_t out_offset = nitems_written(0);

    add_item_tag(0, out_offset, pmt::mp("tx_time"), time_pmt, pmt::mp("ping"));
    add_item_tag(0, out_offset, pmt::mp("tx_sob"), pmt::PMT_T, pmt::mp("ping"));
    add_item_tag(
        0, out_offset + d_burst_len - 1, pmt::mp("tx_eob"), pmt::PMT_T, pmt::mp("ping"));

    std::copy(d_burst.begin(), d_burst.end(), out);
    message_port_pub(pmt::mp("tx_time_out"), time_pmt);

    d_last_tx_time = tx_time;
    --d_pending_pings;
    consume(0, ninput);
    produce(0, d_burst_len);
    return WORK_CALLED_PRODUCE;
}

} // namespace ofdm_prs_ranging
} // namespace gr
