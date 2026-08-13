/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_RTT_RESPONDER_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_RTT_RESPONDER_IMPL_H

#include <gnuradio/ofdm_prs_ranging/zc_rtt_responder.h>

namespace gr {
namespace ofdm_prs_ranging {

class zc_rtt_responder_impl : public zc_rtt_responder
{
public:
    zc_rtt_responder_impl(const std::vector<gr_complex>& zc_seq,
                          double samp_rate,
                          double delay_secs,
                          int zc_length,
                          float peak_metric_threshold,
                          float fixed_threshold);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;

private:
    int output_burst_chunk(gr_complex* out, int noutput_items);
    int find_peak(const gr_complex* in, int nitems) const;
    std::vector<gr_complex> generate_bpsk(const std::vector<uint8_t>& bits) const;
    pmt::pmt_t ticks_to_pmt_time(int64_t ticks) const;
    int64_t pmt_time_to_ticks(pmt::pmt_t time) const;

    std::vector<gr_complex> d_zc_seq;
    double d_samp_rate;
    double d_delay_secs;
    int d_zc_length;
    float d_peak_metric_threshold;
    float d_fixed_threshold;
    int d_responder_id = 0x02;
    int d_measurement_number = 0;
    int d_transmit_num = 0;
    int d_sps = 60;
    bool d_have_rx_time = false;
    pmt::pmt_t d_last_rx_time;
    uint64_t d_last_tag_offset = 0;
    int64_t d_next_allowed_tx_offset;
    int64_t d_cooldown_samples;
    bool d_burst_active = false;
    int d_burst_pos = 0;
    std::vector<gr_complex> d_burst;
    pmt::pmt_t d_pending_tx_time;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
