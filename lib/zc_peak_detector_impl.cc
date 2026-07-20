/* -*- c++ -*- */
#include "zc_peak_detector_impl.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>

namespace gr {
namespace ofdm_prs_ranging {

zc_peak_detector::sptr zc_peak_detector::make(int zc_length,
                                              float peak_metric_threshold,
                                              float fixed_threshold)
{
    return gnuradio::make_block_sptr<zc_peak_detector_impl>(
        zc_length, peak_metric_threshold, fixed_threshold);
}

zc_peak_detector_impl::zc_peak_detector_impl(int zc_length,
                                             float peak_metric_threshold,
                                             float fixed_threshold)
    : gr::sync_block("zc_peak_detector",
                     gr::io_signature::make(1, 1, sizeof(float)),
                     gr::io_signature::make(1, 1, sizeof(int8_t))),
      d_zc_length(zc_length),
      d_peak_metric_threshold(peak_metric_threshold),
      d_fixed_threshold(fixed_threshold)
{
}

int zc_peak_detector_impl::work(int noutput_items,
                                gr_vector_const_void_star& input_items,
                                gr_vector_void_star& output_items)
{
    const auto in = static_cast<const float*>(input_items[0]);
    auto out = static_cast<int8_t*>(output_items[0]);
    std::fill(out, out + noutput_items, 0);
    double power = 0.0;
    for (int i = 0; i < noutput_items; ++i) {
        power += static_cast<double>(in[i]) * static_cast<double>(in[i]);
    }
    const double mean_power = noutput_items > 0 ? power / noutput_items : 0.0;
    const double denom = std::sqrt(std::max(1e-18, mean_power) * d_zc_length);
    for (int i = 0; i < noutput_items; ++i) {
        const float mag = std::fabs(in[i]);
        const float metric = static_cast<float>(mag / denom);
        if (metric >= d_peak_metric_threshold && mag >= d_fixed_threshold) {
            out[i] = in[i] >= 0.0f ? 1 : -1;
        }
    }
    return noutput_items;
}

} // namespace ofdm_prs_ranging
} // namespace gr
