/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_RTT_CALCULATOR_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_RTT_CALCULATOR_H

#include <gnuradio/block.h>
#include <gnuradio/ofdm_prs_ranging/api.h>
#include <string>

namespace gr {
namespace ofdm_prs_ranging {

class OFDM_PRS_RANGING_API zc_rtt_calculator : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<zc_rtt_calculator>;
    static sptr make(double samp_rate = 6e6,
                     int zc_length = 839,
                     double delay_secs = 0.005,
                     float peak_metric_threshold = 0.35f,
                     float fixed_threshold = 0.0f,
                     double distance_setting_m = 0.0,
                     const std::string& log_path = "rtt_measurements.csv");
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
