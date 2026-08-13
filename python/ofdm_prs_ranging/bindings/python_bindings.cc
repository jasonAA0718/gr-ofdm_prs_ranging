/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <pybind11/pybind11.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

namespace py = pybind11;

// Headers for binding functions
/**************************************/
// The following comment block is used for
// gr_modtool to insert function prototypes
// Please do not delete
/**************************************/
// BINDING_FUNCTION_PROTOTYPES(
    void bind_pluto_prs_burst_source(py::module& m);
    void bind_pluto_prs_responder(py::module& m);
    void bind_prs_acquisition_logger(py::module& m);
    void bind_prs_frame_detector(py::module& m);
    void bind_prs_fft_receiver(py::module& m);
    void bind_prs_channel_estimator(py::module& m);
    void bind_prs_phase_slope_estimator(py::module& m);
    void bind_prs_rx_timekeeper(py::module& m);
    void bind_prs_ssrtt_responder(py::module& m);
    void bind_prs_ssrtt_solver(py::module& m);
    void bind_prs_csv_logger(py::module& m);
    void bind_prs_timed_burst_source(py::module& m);
    void bind_zc_manual_ping_source(py::module& m);
    void bind_zc_peak_detector(py::module& m);
    void bind_zc_rtt_calculator(py::module& m);
    void bind_zc_rtt_responder(py::module& m);
// ) END BINDING_FUNCTION_PROTOTYPES


// We need this hack because import_array() returns NULL
// for newer Python versions.
// This function is also necessary because it ensures access to the C API
// and removes a warning.
void* init_numpy()
{
    import_array();
    return NULL;
}

PYBIND11_MODULE(ofdm_prs_ranging_python, m)
{
    // Initialize the numpy C API
    // (otherwise we will see segmentation faults)
    init_numpy();

    // Allow access to base block methods
    py::module::import("gnuradio.gr");

    /**************************************/
    // The following comment block is used for
    // gr_modtool to insert binding function calls
    // Please do not delete
    /**************************************/
    // BINDING_FUNCTION_CALLS(
    bind_pluto_prs_burst_source(m);
    bind_pluto_prs_responder(m);
    bind_prs_acquisition_logger(m);
    bind_prs_frame_detector(m);
    bind_prs_fft_receiver(m);
    bind_prs_channel_estimator(m);
    bind_prs_phase_slope_estimator(m);
    bind_prs_rx_timekeeper(m);
    bind_prs_ssrtt_responder(m);
    bind_prs_ssrtt_solver(m);
    bind_prs_csv_logger(m);
    bind_prs_timed_burst_source(m);
    bind_zc_manual_ping_source(m);
    bind_zc_peak_detector(m);
    bind_zc_rtt_calculator(m);
    bind_zc_rtt_responder(m);
    // ) END BINDING_FUNCTION_CALLS
}
