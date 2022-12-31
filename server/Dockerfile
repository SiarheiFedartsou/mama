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
    protobuf-compiler \
    cmake \
    && rm -rf /var/lib/apt/lists/*





EXPOSE 50051

RUN mkdir -p server/build && \
    cd server/build && \
    cmake .. && \
    make -j4


ENTRYPOINT [ "/src/server/build/mama_server" ]