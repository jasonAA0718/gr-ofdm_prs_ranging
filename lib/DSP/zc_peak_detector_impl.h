/* -*- c++ -*- */
#ifndef INCLUDED_OFDM_PRS_RANGING_ZC_PEAK_DETECTOR_IMPL_H
#define INCLUDED_OFDM_PRS_RANGING_ZC_PEAK_DETECTOR_IMPL_H

#include <gnuradio/ofdm_prs_ranging/zc_peak_detector.h>

namespace gr {
namespace ofdm_prs_ranging {

class zc_peak_detector_impl : public zc_peak_detector
{
public:
    zc_peak_detector_impl(int zc_length, float peak_metric_threshold, float fixed_threshold);
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

private:
    int d_zc_length;
    float d_peak_metric_threshold;
    float d_fixed_threshold;
};

} // namespace ofdm_prs_ranging
} // namespace gr

#endif
