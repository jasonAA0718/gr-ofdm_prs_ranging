#include <gnuradio/ofdm_prs_ranging/pluto_prs_burst_source.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_pluto_prs_burst_source(py::module& m)
{
    using block = gr::ofdm_prs_ranging::pluto_prs_burst_source;
    py::class_<block, gr::sync_block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "pluto_prs_burst_source")
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
             py::arg("tx_amp") = 0.6f,
             py::arg("seed") = 13990001,
             py::arg("default_packet_type") = 1,
             py::arg("guard_samples") = 0,
             py::arg("coarse_zc_root") = 25)
        .def("frame_len", &block::frame_len)
        .def("prs_start", &block::prs_start)
        .def("prs_len", &block::prs_len)
        .def("frame_samples", &block::frame_samples);
}
