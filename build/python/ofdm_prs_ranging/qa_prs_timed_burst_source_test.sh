#!/usr/bin/sh
export VOLK_GENERIC=1
export GR_DONT_LOAD_PREFS=1
export srcdir=/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/python/ofdm_prs_ranging
export GR_CONF_CONTROLPORT_ON=False
export PATH="/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/build/python/ofdm_prs_ranging":"$PATH"
export LD_LIBRARY_PATH="":$LD_LIBRARY_PATH
export PYTHONPATH=/home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/build/test_modules:$PYTHONPATH
/usr/bin/python3 /home/cnsl/gnuradio-zc-twr/gr-ofdm_prs_ranging/python/ofdm_prs_ranging/qa_prs_timed_burst_source.py 
