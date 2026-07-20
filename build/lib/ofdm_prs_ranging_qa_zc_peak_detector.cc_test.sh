#!/usr/bin/sh
export VOLK_GENERIC=1
export GR_DONT_LOAD_PREFS=1
export srcdir=/home/cnsl/Desktop/gr-ofdm_prs_ranging/lib
export GR_CONF_CONTROLPORT_ON=False
export PATH="/home/cnsl/Desktop/gr-ofdm_prs_ranging/build/lib":"$PATH"
export LD_LIBRARY_PATH="":$LD_LIBRARY_PATH
export PYTHONPATH=/home/cnsl/Desktop/gr-ofdm_prs_ranging/build/test_modules:$PYTHONPATH
ofdm_prs_ranging_qa_zc_peak_detector.cc 
