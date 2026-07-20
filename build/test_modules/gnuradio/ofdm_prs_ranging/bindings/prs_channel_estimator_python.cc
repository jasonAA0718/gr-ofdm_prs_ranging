#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_channel_estimator.h>

void bind_prs_channel_estimator(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_channel_estimator;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(m, "prs_channel_estimator")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 10e6,
             py::arg("fft_len") = 1024,
             py::arg("active_bins") = 600,
             py::arg("prs_symbols") = 16,
             py::arg("seed") = 13990001);
}
