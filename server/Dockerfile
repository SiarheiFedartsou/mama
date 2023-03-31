FROM ubuntu:22.04 AS builder

COPY . /src

RUN apt-get update \
    && apt-get install -y \
    clang \
    libssl-dev \
    pkg-config \
    zlib1g-dev \
    protobuf-compiler \
    libprotoc-dev \
    libprotobuf-dev \
    curl \
    && rm -rf /var/lib/apt/lists/* 

# install CMake
RUN curl -sLO https://github.com/Kitware/CMake/releases/download/v3.26.1/cmake-3.26.1-Linux-$(arch).sh && \
    chmod +x cmake-3.26.1-Linux-*.sh && \
    ./cmake-3.26.1-Linux-*.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.26.1-Linux-*.sh

ENV CC=clang CXX=clang++


RUN mkdir -p /build && \
    mkdir -p /mama && \
    cd /build && \
    cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release /src/server && \
    make -j4  && \
    make -j4 install

FROM ubuntu:22.04

EXPOSE 50051


RUN apt-get update && apt-get install --no-install-recommends -y \
    libprotobuf-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/bin/tilegen /usr/local/bin/mama_server /usr/local/bin/
