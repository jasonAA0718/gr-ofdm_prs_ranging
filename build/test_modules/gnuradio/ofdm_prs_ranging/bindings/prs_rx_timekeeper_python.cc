#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_rx_timekeeper.h>

void bind_prs_rx_timekeeper(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_rx_timekeeper;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "prs_rx_timekeeper")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 10e6,
             py::arg("tx_lead_time") = 0.5);
}
