# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT ofdm_prs_ranging_sources)
    message(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT ofdm_prs_ranging_sources)

########################################################################
# Check for pygccxml
########################################################################
gr_python_check_module_raw("pygccxml" "import pygccxml" PYGCCXML_FOUND)

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND ofdm_prs_ranging_python_files
    prs_acquisition_logger_python.cc
    prs_frame_detector_python.cc
    prs_fft_receiver_python.cc
    prs_channel_estimator_python.cc
    prs_phase_slope_estimator_python.cc
    prs_rx_timekeeper_python.cc
    prs_ssrtt_responder_python.cc
    prs_ssrtt_solver_python.cc
    prs_csv_logger_python.cc
    prs_timed_burst_source_python.cc
    zc_manual_ping_source_python.cc
    zc_peak_detector_python.cc
    zc_rtt_calculator_python.cc
    zc_rtt_responder_python.cc
    python_bindings.cc)

gr_pybind_make_oot(ofdm_prs_ranging ../../.. gr::ofdm_prs_ranging "${ofdm_prs_ranging_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(
    TARGET ofdm_prs_ranging_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ofdm_prs_ranging_python>
            ${PROJECT_BINARY_DIR}/test_modules/gnuradio/ofdm_prs_ranging/)

install(
    TARGETS ofdm_prs_ranging_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/ofdm_prs_ranging
    COMPONENT pythonapi)
