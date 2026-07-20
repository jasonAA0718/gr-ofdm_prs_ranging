#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/zc_rtt_calculator.h>

void bind_zc_rtt_calculator(py::module& m)
{
    using block = gr::ofdm_prs_ranging::zc_rtt_calculator;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "zc_rtt_calculator")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 6e6,
             py::arg("zc_length") = 839,
             py::arg("delay_secs") = 0.005,
             py::arg("peak_metric_threshold") = 0.35f,
             py::arg("fixed_threshold") = 0.0f,
             py::arg("distance_setting_m") = 0.0,
             py::arg("log_path") = "rtt_measurements.csv");
}
