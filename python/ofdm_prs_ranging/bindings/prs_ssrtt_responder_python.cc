#include <pybind11/pybind11.h>
namespace py = pybind11;
#include <gnuradio/ofdm_prs_ranging/prs_ssrtt_responder.h>

void bind_prs_ssrtt_responder(py::module& m)
{
    using block = gr::ofdm_prs_ranging::prs_ssrtt_responder;
    py::class_<block, gr::block, gr::basic_block, std::shared_ptr<block>>(m, "prs_ssrtt_responder")
        .def(py::init(&block::make),
             py::arg("samp_rate") = 10e6,
             py::arg("reply_delay_samples") = 50000);
}
