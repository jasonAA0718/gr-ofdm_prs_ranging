#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_acquisition_logger.h>

void bind_prs_acquisition_logger(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_acquisition_logger;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "prs_acquisition_logger")
        .def(py::init(&block::make),
             py::arg("path") = "prs_acquisition.csv",
             py::arg("node") = "rx",
             py::arg("append") = true);
}
