#!/bin/bash

# Install Pytorch for ARM using nvidia's providied package for the jetson
wget https://nvidia.box.com/shared/static/1v2cc4ro6zvsbu0p8h6qcuaqco1qcsif.whl -O torch-1.4.0-cp27-cp27mu-linux_aarch64.whl
apt-get update && apt-get install -y --no-install-recommends libopenblas-base libopenmpi-dev 
pip install --no-cache-dir future torch-1.4.0-cp27-cp27mu-linux_aarch64.whl
rm torch-1.4.0-cp27-cp27mu-linux_aarch64.whl

# Install torchvision
git clone --branch v0.5.0 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.5.0 
python setup.py install
cd ../
rm -rf torchvision
