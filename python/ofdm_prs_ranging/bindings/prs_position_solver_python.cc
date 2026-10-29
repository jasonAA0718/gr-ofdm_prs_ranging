
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <gnuradio/ofdm_prs_ranging/prs_position_solver.h>

namespace py = pybind11;

void bind_prs_position_solver(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_position_solver;

    py::class_<
        block,
        gr::block,
        gr::basic_block,
        std::shared_ptr<block>>(
            m, "prs_position_solver")

        .def(
            py::init(&block::make),
            py::arg("anchor_ids"),
            py::arg("anchor_positions_m"),
            py::arg("vehicle_height_m") = 0.0
        );
}
