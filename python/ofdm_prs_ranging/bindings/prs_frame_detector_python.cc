#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_frame_detector.h>

void bind_prs_frame_detector(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_frame_detector;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(m, "prs_frame_detector")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 10e6,
             py::arg("fft_len") = 1024,
             py::arg("cp_len") = 128,
             py::arg("active_bins") = 1024,
             py::arg("prs_symbols") = 16,
             py::arg("preamble_len") = 128,
             py::arg("preamble_repeats") = 16,
             py::arg("coarse_sync_len") = 839,
             py::arg("zero_guard_len") = 1000,
             py::arg("tail_guard_len") = 1000,
             py::arg("threshold") = 0.35f,
             py::arg("min_frame_gap") = 10000,
             py::arg("coarse_zc_root") = 25,
             py::arg("channel_id") = 0,
             py::arg("time_gating") = false,
             py::arg("reply_delay_s") = 0.05,
             py::arg("window_before_s") = 0.0002,
             py::arg("window_after_s") = 0.002);
}
