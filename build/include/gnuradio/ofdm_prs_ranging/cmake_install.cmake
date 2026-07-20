# Install script for directory: /home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gnuradio/ofdm_prs_ranging" TYPE FILE FILES
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/api.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_acquisition_logger.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_timed_burst_source.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_frame_detector.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_fft_receiver.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_channel_estimator.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_phase_slope_estimator.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_ssrtt_responder.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_ssrtt_solver.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/prs_csv_logger.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/zc_manual_ping_source.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/zc_peak_detector.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/zc_rtt_calculator.h"
    "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/include/gnuradio/ofdm_prs_ranging/zc_rtt_responder.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/build/include/gnuradio/ofdm_prs_ranging/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
