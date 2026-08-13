#include <gnuradio/ofdm_prs_ranging/pluto_prs_responder.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_pluto_prs_responder(py::module& m)
{
    using block = gr::ofdm_prs_ranging::pluto_prs_responder;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "pluto_prs_responder")
        .def(py::init(&block::make), py::arg("guard_samples") = 0);
}
