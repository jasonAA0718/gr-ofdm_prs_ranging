/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_MANUAL_PING_SOURCE_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_MANUAL_PING_SOURCE_IMPL_H

#include <gnuradio/ofdm_prs_ranging/zc_manual_ping_source.h>

namespace gr {
namespace ofdm_prs_ranging {

class zc_manual_ping_source_impl : public zc_manual_ping_source
{
public:
    zc_manual_ping_source_impl(double samp_rate,
                               const std::vector<gr_complex>& zc_seq,
                               int packet_len,
                               double tx_delay_secs,
                               double min_period_secs);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;

private:
    void handle_trigger(pmt::pmt_t msg);

    double d_samp_rate;
    double d_tx_delay_secs;
    double d_min_period_secs;
    std::vector<gr_complex> d_burst;
    int d_burst_len;
    int d_pending_pings = 0;
    double d_last_tx_time = 0.0;
    bool d_have_rx_time = false;
    double d_current_usrp_time = 0.0;
    uint64_t d_last_rx_tag_offset = 0;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
