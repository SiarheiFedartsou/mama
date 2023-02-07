FROM ubuntu:22.04 AS builder

COPY . /src

RUN apt-get update && apt-get install -y \
    clang \
    libssl-dev \
    pkg-config \
    zlib1g-dev \
    protobuf-compiler \
    libprotoc-dev \
    libprotobuf-dev \
    cmake \
    && rm -rf /var/lib/apt/lists/*

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
