/* -*- c++ -*- */
#include <gnuradio/ofdm_prs_ranging/zc_peak_detector.h>
#include <boost/test/unit_test.hpp>

namespace gr {
namespace ofdm_prs_ranging {

BOOST_AUTO_TEST_CASE(test_zc_peak_detector_constructs)
{
    auto detector = zc_peak_detector::make(839, 0.35f, 0.0f);
    BOOST_REQUIRE(detector);
    BOOST_CHECK_EQUAL(detector->name(), "zc_peak_detector");
}

} // namespace ofdm_prs_ranging
} // namespace gr
