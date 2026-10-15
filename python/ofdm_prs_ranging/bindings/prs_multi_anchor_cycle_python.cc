/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <gnuradio/ofdm_prs_ranging/prs_multi_anchor_cycle.h>
namespace py = pybind11;
void bind_prs_multi_anchor_cycle(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_multi_anchor_cycle;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(m, "prs_multi_anchor_cycle")
        .def(py::init(&block::make), py::arg("samp_rate") = 30e6,
             py::arg("anchor_ids") = std::vector<int>{0, 1},
             py::arg("time_delays") = std::vector<double>{0.031, 0.081},
             py::arg("preamble_len") = 128, py::arg("preamble_repeats") = 17,
             py::arg("coarse_sync_len") = 819, py::arg("window_before_s") = 0.002,
             py::arg("window_after_s") = 0.005, py::arg("processing_grace_s") = 0.1)
        .def("active_cycle_count", &block::active_cycle_count);
}
