FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    build-essential \
    libssl-dev \
    pkg-config \
    cmake \
    && rm -rf /var/lib/apt/lists/*


COPY . /src
WORKDIR /src


EXPOSE 50051

RUN mkdir -p server/build && \
    cd server/build && \
    cmake .. && \
    make -j4


ENTRYPOINT [ "/src/server/build/greeter_async_server" ]