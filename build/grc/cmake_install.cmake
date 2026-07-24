# Install script for directory: /home/cnsl/Desktop/gr-ofdm_prs_ranging/grc

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gnuradio/grc/blocks" TYPE FILE FILES
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_acquisition_logger.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_rx_timekeeper.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_text_ui.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_timed_burst_source.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_frame_detector.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_fft_receiver.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_channel_estimator.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_phase_slope_estimator.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_ssrtt_responder.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_ssrtt_solver.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_prs_csv_logger.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_zc_manual_ping_source.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_zc_peak_detector.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_zc_rtt_calculator.block.yml"
    "/home/cnsl/Desktop/gr-ofdm_prs_ranging/grc/ofdm_prs_ranging_zc_rtt_responder.block.yml"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/cnsl/Desktop/gr-ofdm_prs_ranging/build/grc/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
