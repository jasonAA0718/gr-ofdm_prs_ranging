/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_MANUAL_PING_SOURCE_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_MANUAL_PING_SOURCE_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/types.h>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API zc_manual_ping_source : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<zc_manual_ping_source>;
    static sptr make(double samp_rate = 6e6,
                     const std::vector<gr_complex>& zc_seq = { gr_complex(1, 0),
                                                               gr_complex(1, 0) },
                     int packet_len = 1024,
                     double tx_delay_secs = 0.5,
                     double min_period_secs = 0.1);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
