#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_fft_zc_frame_detector.h>

void bind_prs_fft_zc_frame_detector(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_fft_zc_frame_detector;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "prs_fft_zc_frame_detector")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 10e6,
             py::arg("threshold") = 0.35f,
             py::arg("min_frame_gap") = 10000,
             py::arg("expect_response") = false,
             py::arg("channel_id") = 0,
             py::arg("time_gating") = false,
             py::arg("reply_delay_s") = 0.05,
             py::arg("window_before_s") = 0.0002,
             py::arg("window_after_s") = 0.004,
             py::arg("zc_threshold") = 0.35f,
             py::arg("correlation_fft_len") = 2048,
             py::arg("zc_search_before") = 1024,
             py::arg("zc_search_after") = 3584,
             py::arg("cfo_compensation") = true);
}
