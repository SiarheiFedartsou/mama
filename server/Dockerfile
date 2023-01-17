FROM ubuntu:22.04

COPY . /src

RUN apt-get update && apt-get install -y \
    clang \
    libssl-dev \
    pkg-config \
    libexpat1-dev \
    libbz2-dev \
    zlib1g-dev \
    protobuf-compiler \
    libprotoc-dev \
    libprotobuf-dev \
    cmake \
    && rm -rf /var/lib/apt/lists/*





EXPOSE 50051

ENV CC=clang CXX=clang++

RUN mkdir -p /build && \
    cd /build && \
    cmake /src/server && \
    make -j4
RUN cd /build &&  make -j4 tilegen
RUN \
    mkdir -p /app && \
    ln -s /build/mama_server /app/mama_server && \
    ln -s /build/mama/tilegen /app/tilegen
WORKDIR /app
