#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/zc_peak_detector.h>

void bind_zc_peak_detector(py::module& m)
{
    using block = gr::ofdm_prs_ranging::zc_peak_detector;
    py::class_<block, gr::sync_block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "zc_peak_detector")
        .def(py::init(&block::make),
             py::arg("zc_length") = 839,
             py::arg("peak_metric_threshold") = 0.35f,
             py::arg("fixed_threshold") = 0.0f);
}
