#!/usr/bin/env bash
set -e

echo "=== Updating package list ==="
sudo apt update

echo "=== Installing build dependencies ==="
sudo apt install -y \
    clang-format \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-dev \
    python3-pip \
    pybind11-dev \
    libboost-all-dev \
    gnuradio-dev \
    libvolk-dev \
    linux-tools-common

echo "=== Installing kernel-specific CPU tools ==="
sudo apt install -y "linux-tools-$(uname -r)" || \
sudo apt install -y linux-tools-generic

echo "=== Setting CPU governor to performance ==="
sudo cpupower frequency-set -g performance || true

echo "=== Increasing socket buffer limits ==="
sudo sysctl -w net.core.rmem_max=50000000
sudo sysctl -w net.core.wmem_max=50000000
sudo sysctl -w net.core.rmem_default=50000000
sudo sysctl -w net.core.wmem_default=50000000

echo "=== Creating persistent sysctl configuration ==="
sudo tee /etc/sysctl.d/99-usrp.conf > /dev/null <<EOF
net.core.rmem_max=50000000
net.core.wmem_max=50000000
net.core.rmem_default=50000000
net.core.wmem_default=50000000
EOF

sudo sysctl --system

echo "=== Setup complete ==="
