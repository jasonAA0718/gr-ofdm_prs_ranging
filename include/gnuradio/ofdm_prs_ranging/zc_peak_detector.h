/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_PEAK_DETECTOR_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_PEAK_DETECTOR_H

#include <gnuradio/ofdm_prs_ranging/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API zc_peak_detector : virtual public gr::sync_block
{
public:
    using sptr = std::shared_ptr<zc_peak_detector>;
    static sptr make(int zc_length = 839,
                     float peak_metric_threshold = 0.35f,
                     float fixed_threshold = 0.0f);
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
