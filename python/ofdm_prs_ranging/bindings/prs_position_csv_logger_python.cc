// SPDX-License-Identifier: GPL-3.0-or-later
#include <gnuradio/ofdm_prs_ranging/prs_position_csv_logger.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

void bind_prs_position_csv_logger(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_position_csv_logger;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "prs_position_csv_logger")
        .def(py::init(&block::make),
             py::arg("path") = "prs_positions.csv",
             py::arg("append") = true);
}
