#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/zc_rtt_responder.h>

namespace {
std::vector<gr_complex> zc_array_to_vector(py::array_t<gr_complex, py::array::c_style | py::array::forcecast> array)
{
    const auto info = array.request();
    const auto data = static_cast<const gr_complex*>(info.ptr);
    return std::vector<gr_complex>(data, data + info.size);
}
} // namespace

void bind_zc_rtt_responder(py::module& m)
{
    using block = gr::ofdm_prs_ranging::zc_rtt_responder;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "zc_rtt_responder")
        .def(py::init(&block::make),
             py::arg("zc_seq") = std::vector<gr_complex>{ gr_complex(1, 0),
                                                          gr_complex(1, 0) },
             py::arg("samp_rate") = 6e6,
             py::arg("delay_secs") = 0.005,
             py::arg("zc_length") = 839,
             py::arg("peak_metric_threshold") = 0.35f,
             py::arg("fixed_threshold") = 0.0f)
        .def(py::init([](py::array_t<gr_complex, py::array::c_style | py::array::forcecast> zc_seq,
                         double samp_rate,
                         double delay_secs,
                         int zc_length,
                         float peak_metric_threshold,
                         float fixed_threshold) {
                 return block::make(zc_array_to_vector(zc_seq),
                                    samp_rate,
                                    delay_secs,
                                    zc_length,
                                    peak_metric_threshold,
                                    fixed_threshold);
             }),
             py::arg("zc_seq"),
             py::arg("samp_rate") = 6e6,
             py::arg("delay_secs") = 0.005,
             py::arg("zc_length") = 839,
             py::arg("peak_metric_threshold") = 0.35f,
             py::arg("fixed_threshold") = 0.0f);
}
