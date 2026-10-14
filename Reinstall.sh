cd /home/cnsl/Desktop/gr-ofdm_prs_ranging
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "$(nproc)"
sudo cmake --install build
sudo ldconfig

