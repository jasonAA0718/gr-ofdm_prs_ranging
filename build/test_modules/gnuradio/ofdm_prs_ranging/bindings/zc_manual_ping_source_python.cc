#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/zc_manual_ping_source.h>

namespace {
std::vector<gr_complex> zc_array_to_vector(py::array_t<gr_complex, py::array::c_style | py::array::forcecast> array)
{
    const auto info = array.request();
    const auto data = static_cast<const gr_complex*>(info.ptr);
    return std::vector<gr_complex>(data, data + info.size);
}
} // namespace

void bind_zc_manual_ping_source(py::module& m)
{
    using block = gr::ofdm_prs_ranging::zc_manual_ping_source;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "zc_manual_ping_source")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 6e6,
             py::arg("zc_seq") = std::vector<gr_complex>{ gr_complex(1, 0),
                                                          gr_complex(1, 0) },
             py::arg("packet_len") = 1024,
             py::arg("tx_delay_secs") = 0.5,
             py::arg("min_period_secs") = 0.1)
        .def(py::init([](double samp_rate,
                         py::array_t<gr_complex, py::array::c_style | py::array::forcecast> zc_seq,
                         int packet_len,
                         double tx_delay_secs,
                         double min_period_secs) {
                 return block::make(samp_rate,
                                    zc_array_to_vector(zc_seq),
                                    packet_len,
                                    tx_delay_secs,
                                    min_period_secs);
             }),
             py::arg("samp_rate") = 6e6,
             py::arg("zc_seq"),
             py::arg("packet_len") = 1024,
             py::arg("tx_delay_secs") = 0.5,
             py::arg("min_period_secs") = 0.1);
}
