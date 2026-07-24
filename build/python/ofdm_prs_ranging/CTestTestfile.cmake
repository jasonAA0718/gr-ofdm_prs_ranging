# CMake generated Testfile for 
# Source directory: /home/cnsl/Desktop/gr-ofdm_prs_ranging/python/ofdm_prs_ranging
# Build directory: /home/cnsl/Desktop/gr-ofdm_prs_ranging/build/python/ofdm_prs_ranging
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(qa_prs_timed_burst_source "/usr/bin/sh" "qa_prs_timed_burst_source_test.sh")
set_tests_properties(qa_prs_timed_burst_source PROPERTIES  _BACKTRACE_TRIPLES "/usr/lib/x86_64-linux-gnu/cmake/gnuradio/GrTest.cmake;119;add_test;/home/cnsl/Desktop/gr-ofdm_prs_ranging/python/ofdm_prs_ranging/CMakeLists.txt;41;GR_ADD_TEST;/home/cnsl/Desktop/gr-ofdm_prs_ranging/python/ofdm_prs_ranging/CMakeLists.txt;0;")
add_test(qa_prs_receiver "/usr/bin/sh" "qa_prs_receiver_test.sh")
set_tests_properties(qa_prs_receiver PROPERTIES  ENVIRONMENT "HOME=/tmp;XDG_CACHE_HOME=/tmp" _BACKTRACE_TRIPLES "/usr/lib/x86_64-linux-gnu/cmake/gnuradio/GrTest.cmake;119;add_test;/home/cnsl/Desktop/gr-ofdm_prs_ranging/python/ofdm_prs_ranging/CMakeLists.txt;42;GR_ADD_TEST;/home/cnsl/Desktop/gr-ofdm_prs_ranging/python/ofdm_prs_ranging/CMakeLists.txt;0;")
add_test(qa_prs_text_ui "/usr/bin/sh" "qa_prs_text_ui_test.sh")
set_tests_properties(qa_prs_text_ui PROPERTIES  _BACKTRACE_TRIPLES "/usr/lib/x86_64-linux-gnu/cmake/gnuradio/GrTest.cmake;119;add_test;/home/cnsl/Desktop/gr-ofdm_prs_ranging/python/ofdm_prs_ranging/CMakeLists.txt;43;GR_ADD_TEST;/home/cnsl/Desktop/gr-ofdm_prs_ranging/python/ofdm_prs_ranging/CMakeLists.txt;0;")
subdirs("bindings")
