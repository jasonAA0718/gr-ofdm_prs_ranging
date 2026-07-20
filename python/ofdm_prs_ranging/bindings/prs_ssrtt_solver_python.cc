#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_ssrtt_solver.h>

void bind_prs_ssrtt_solver(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_ssrtt_solver;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(m, "prs_ssrtt_solver")
        .def(py::init(&block::make), py::arg("samp_rate") = 10e6);
}
