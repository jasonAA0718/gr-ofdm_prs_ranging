cd /home/cnsl/Desktop/gr-ofdm_prs_ranging
  cmake --build build -j$(nproc)
  sudo cmake --build build --target install
  sudo ldconfig

