# Development container for sfurs_firmware_v2 (NUCLEO-H755ZI-Q).
# Provides the full bench toolchain so a new dev can build + test with no
# host setup: ARM GCC, CMake/Ninja, host GCC, Python 3 + OpenOCD and USB
# (ST-LINK). STM32CubeProgrammer is proprietary and NOT bundled; mount a host
# install for flashing (see docker/README.md).
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        git \
        python3 \
        python3-pip \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libusb-1.0-0 \
        usbutils \
        doxygen \
        openocd \
        graphviz \
        ca-certificates \
        sudo \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --break-system-packages pyelftools \
    && arm-none-eabi-gcc --version | head -n 1 \
    && cmake --version | head -n 1

# Match the host user so mounted-repo files stay owned by the developer.
# ubuntu:24.04 already owns UID/GID 1000 ("ubuntu"); rename it in that case.
ARG UID=1000
ARG GID=1000
RUN if ! getent group "${GID}" >/dev/null; then groupadd -g "${GID}" dev; fi && \
    if ! getent passwd "${UID}" >/dev/null; then \
      useradd -m -u "${UID}" -g "${GID}" -s /bin/bash dev; \
    else \
      EXISTING_USER=$(getent passwd "${UID}" | cut -d: -f1) && \
      EXISTING_GROUP=$(getent group "${GID}" | cut -d: -f1) && \
      [ "${EXISTING_GROUP}" != dev ] && groupmod -n dev "${EXISTING_GROUP}" || true; \
      [ "${EXISTING_USER}" != dev ] && usermod -l dev -d /home/dev -m "${EXISTING_USER}" || true; \
    fi && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev && \
    chmod 0440 /etc/sudoers.d/dev && \
    id dev
USER dev
WORKDIR /work/sfurs_firmware_v2
CMD ["bash"]
