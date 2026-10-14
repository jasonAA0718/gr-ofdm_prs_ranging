#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_zc_frame_sync.h>

void bind_prs_zc_frame_sync(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_zc_frame_sync;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "prs_zc_frame_sync")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 30e6,
             py::arg("fft_len") = 512,
             py::arg("cp_len") = 64,
             py::arg("active_bins") = 512,
             py::arg("prs_symbols") = 127,
             py::arg("preamble_len") = 128,
             py::arg("preamble_repeats") = 17,
             py::arg("coarse_sync_len") = 819,
             py::arg("preamble_threshold") = 0.09f,
             py::arg("zc_threshold") = 0.35f,
             py::arg("coarse_zc_root") = 25,
             py::arg("channel_id") = 0,
             py::arg("enable_profiling") = false);
}
