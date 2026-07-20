/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_RTT_RESPONDER_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_RTT_RESPONDER_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/types.h>
#include <vector>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API zc_rtt_responder : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<zc_rtt_responder>;
    static sptr make(const std::vector<gr_complex>& zc_seq = { gr_complex(1, 0),
                                                               gr_complex(1, 0) },
                     double samp_rate = 6e6,
                     double delay_secs = 0.005,
                     int zc_length = 839,
                     float peak_metric_threshold = 0.35f,
                     float fixed_threshold = 0.0f);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
