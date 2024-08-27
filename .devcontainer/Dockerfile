FROM node:20-slim AS builder

WORKDIR /tmp

RUN apt-get update -qq          \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends    \
        ca-certificates         \
        wget                    \
    && npm install -g vsce

# Base Image
FROM hdlc/ghdl:yosys AS base

RUN apt-get update -qq              \
    && apt-get upgrade -y           \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends    \
        ca-certificates             \
        git                         \
        libglib2.0-0                \
        libmpc-dev                  \
        libpython3-all-dev          \
        libtcmalloc-minimal4        \
        locales                     \
        make                        \
        npm                         \
        python3                     \
        python3-pip                 \
        sudo                        \
        tar                         \
        wget                        \
        xauth                       \
        xvfb                        \
        xxd                         \
    && apt-get install -y --no-install-recommends    \
        autoconf                    \
        automake                    \
        autotools-dev               \
        curl                        \
        libmpfr-dev                 \
        libgmp-dev                  \
        gawk                        \
        build-essential             \
        bison                       \
        flex                        \
        texinfo                     \
        gperf                       \
        libtool                     \
        patchutils                  \
        bc                          \
        zlib1g-dev                  \
        libexpat-dev                \
        ninja-build                 \
        cmake                       \
        libslirp-dev                \
        libusb-1.0-0-dev            \
        libftdi1-dev                \
        pkg-config                  \
    && apt-get autoclean            \
    && apt-get clean                \
    && apt-get autoremove -y        \
    && rm -r /var/lib/apt/lists/*

# Environment setup
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && locale-gen
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

ENV QUARTUS_PATH=/opt/intelFPGA
ENV QUARTUS_ROOTDIR=${QUARTUS_PATH}/quartus
ENV SOPC_KIT_NIOS2=${QUARTUS_PATH}/nios2eds
ENV RISCV_PATH=/opt/riscv
ENV PATH=${QUARTUS_ROOTDIR}/bin/:${QUARTUS_ROOTDIR}/linux64/gnu/:${QUARTUS_ROOTDIR}/sopc_builder/bin/:$PATH
ENV PATH=${SOPC_KIT_NIOS2}/:${SOPC_KIT_NIOS2}/bin/:${SOPC_KIT_NIOS2}/bin/gnu/H-x86_64-pc-linux-gnu/bin/:${SOPC_KIT_NIOS2}/sdk2/bin/:$PATH
ENV PATH=${RISCV_PATH}/bin/:$PATH

VOLUME /build

# Intel CDN URL
ARG INTEL_CDN="https://downloads.intel.com/akdlm/software/acdsinst"

# Add Quartus installation files
# ADD ${INTEL_CDN}/23.1std/991/ib_installers/cyclone-23.1std.0.991.qdz                .
# ADD ${INTEL_CDN}/23.1std/991/ib_installers/cyclone10lp-23.1std.0.991.qdz            .
ADD ${INTEL_CDN}/23.1std/991/ib_installers/cyclonev-23.1std.0.991.qdz               .
# ADD ${INTEL_CDN}/23.1std/991/ib_installers/max10-23.1std.0.991.qdz                  .
ADD ${INTEL_CDN}/23.1std/991/ib_installers/QuartusLiteSetup-23.1std.0.991-linux.run .

# Fix file permissions and install Quartus Prime Lite
RUN chmod a+x QuartusLiteSetup-23.1std.0.991-linux.run                                                          \
    && ./QuartusLiteSetup-23.1std.0.991-linux.run --mode unattended --accept_eula 1 --installdir /opt/intelFPGA \
    && rm -rf /opt/intelFPGA/uninstall/                                                                         \
    && rm -rf /tmp/*

# RISC-V Toolchain Installation
WORKDIR /tmp

# OpenOCD Installation
RUN git clone --branch v0.11.0 https://github.com/ntfreak/openocd.git /tmp/openocd \
    && cd /tmp/openocd \
    && ./bootstrap \
    && ./configure --prefix=/usr/local \
    && make -j$(nproc) V=1 \
    && make install \
    && rm -rf /tmp/openocd

RUN git clone https://github.com/riscv/riscv-gnu-toolchain \
    && cd riscv-gnu-toolchain \
    && ./configure --prefix=/opt/riscv --with-arch=rv32imc_zicntr_zfinx_zifencei --with-abi=ilp32 \
    && make \
    && rm -rf /tmp/riscv-gnu-toolchain


# Set up bash prompt
RUN echo 'export PS1="\\n\\e[92m\\e[1m\\u@\\h\\e[0m \\e[94m\\w\\n \\e[92m\\e[1m$\\e[0m\\e[0m\\e[39m\\e[49m "' >> ~/.bashrc

# Ensure RISC-V toolchain and OpenOCD paths are added to PATH
ENV PATH=/opt/riscv/bin:$PATH
