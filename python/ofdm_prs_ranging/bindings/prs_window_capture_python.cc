#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_window_capture.h>

void bind_prs_window_capture(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_window_capture;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "prs_window_capture")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 30e6,
             py::arg("fft_len") = 512,
             py::arg("cp_len") = 64,
             py::arg("active_bins") = 512,
             py::arg("prs_symbols") = 127,
             py::arg("preamble_len") = 128,
             py::arg("preamble_repeats") = 17,
             py::arg("coarse_sync_len") = 819,
             py::arg("threshold") = 0.09f,
             py::arg("min_frame_gap") = 10000,
             py::arg("candidate_margin_before") = 4096,
             py::arg("candidate_margin_after") = 4096,
             py::arg("queue_depth") = 3,
             py::arg("acquisition_enabled") = true,
             py::arg("enable_profiling") = false);
}
