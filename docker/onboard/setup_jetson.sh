#!/bin/bash
# shellcheck disable=SC1004

if [[ "${TARGETPLATFORM}" == "linux/arm64" ]] && [[ "${CUDA}" == "true" ]]; then

    apt-get update && apt-get install -y --no-install-recommends \
        freeglut3-dev \
        libcanberra-gtk3-module \
        libgles2 \
        libglu1-mesa-dev \
        libglvnd-dev \
        libopenblas-dev libopenmpi-dev \
        libgtk-3-0 && \
        rm -rf /var/lib/apt/lists/* && apt-get clean

    echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/nvidia-tegra.conf
    echo "/usr/lib/aarch64-linux-gnu/tegra-egl" >> /etc/ld.so.conf.d/nvidia-tegra.conf

    rm /usr/share/glvnd/egl_vendor.d/50_mesa.json
    mkdir -p /usr/share/glvnd/egl_vendor.d/ && echo '\
    {\
        "file_format_version" : "1.0.0",\
        "ICD" : {\
            "library_path" : "libEGL_nvidia.so.0"\
        }\
    }' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

    mkdir -p /usr/share/egl/egl_external_platform.d/ && echo '\
    {\
        "file_format_version" : "1.0.0",\
        "ICD" : {\
            "library_path" : "libnvidia-egl-wayland.so.1"\
        }\
    }' > /usr/share/egl/egl_external_platform.d/nvidia_wayland.json

    echo "/usr/local/cuda-10.2/targets/aarch64-linux/lib" >> /etc/ld.so.conf.d/nvidia.conf

fi
