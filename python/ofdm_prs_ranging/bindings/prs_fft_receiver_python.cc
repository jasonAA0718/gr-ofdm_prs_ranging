#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_fft_receiver.h>

void bind_prs_fft_receiver(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_fft_receiver;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(m, "prs_fft_receiver")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 10e6,
             py::arg("fft_len") = 1024,
             py::arg("cp_len") = 128,
             py::arg("active_bins") = 600,
             py::arg("prs_symbols") = 16);
}
