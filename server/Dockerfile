FROM ubuntu:22.04

COPY . /src
WORKDIR /src

RUN apt-get update && apt-get install -y \
    build-essential \
    libssl-dev \
    pkg-config \
    libexpat1-dev \
    libbz2-dev \
    zlib1g-dev \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# install gRPC
RUN git clone --depth 1 --recurse-submodules --branch v1.51.1 https://github.com/grpc/grpc && \
    cd grpc && \
    mkdir -p cmake/build && \
    cd cmake/build && \
    cmake -DgRPC_INSTALL=ON \
          -DgRPC_BUILD_TESTS=OFF \
          ../.. && \
    make -j4 && \
    make install && \
    ldconfig




EXPOSE 50051

RUN mkdir -p server/build && \
    cd server/build && \
    cmake .. && \
    make -j4


ENTRYPOINT [ "/src/server/build/mama_server" ]