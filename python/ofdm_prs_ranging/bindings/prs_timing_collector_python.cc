#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_timing_collector.h>

void bind_prs_timing_collector(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_timing_collector;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(
        m, "prs_timing_collector")
        .def(py::init(&block::make),
             py::arg("raw_csv_path") = "prs_timing_raw.csv",
             py::arg("summary_csv_path") = "prs_timing_summary.csv",
             py::arg("role") = "unknown");
}
