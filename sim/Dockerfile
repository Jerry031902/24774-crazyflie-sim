# Build from the repository root: docker build -t cf-sim:dev sim/
# Mount the repository at /workspace when creating the development container.
FROM nvcr.io/nvidia/isaac-sim:5.1.0

# Native build tools for the Python extension, not the STM32 firmware toolchain.
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential swig git libncurses-dev \
    && rm -rf /var/lib/apt/lists/*

# Keep the official image's uid/gid and compile against its bundled Python 3.11.
# numpy, PyYAML and setuptools are already provided by the base image.
USER isaac-sim
RUN /isaac-sim/python.sh -m pip install --user --no-cache-dir \
    pytest==8.4.2 wheel==0.45.1

# Make defaults to the correct interpreter; generated bindings stay on the mount.
ENV PYTHON=/isaac-sim/python.sh
ENV PYTHONPATH=/workspace/build
WORKDIR /workspace
ENTRYPOINT ["/bin/bash"]
