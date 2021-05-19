#!/bin/bash

if [[ "${TARGETPLATFORM}" == "linux/arm64" ]] && [[ "${CUDA}" == "true" ]]; then
    pip3 install --no-cache-dir torch-1.8.0a0+5c1479f-cp38-cp38-linux_aarch64.whl
    pip3 install --no-cache-dir torchvision-0.9.0a0+8fb5838-cp38-cp38-linux_aarch64.whl
else
    pip3 install --no-cache-dir torch==1.8.1+cpu torchvision==0.9.1+cpu -f https://download.pytorch.org/whl/torch_stable.html
fi
