#!/bin/bash
set -e

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# install ghz
curl -L https://github.com/bojand/ghz/releases/download/v0.114.0/ghz-linux-x86_64.tar.gz | tar xvz
mv ghz /usr/local/bin

CONTAINER_ID=$(docker run -d -p 50051:50051 -v $SCRIPTPATH/tiles:/tiles -it mama-server /app/mama_server /tiles)

# TODO: add normal healthcheck
sleep 3

# TODO: we should also benchmark case when we have previous state
ghz --total 100000 --insecure --proto $SCRIPTPATH/protos/mama.proto --call mama.server.api.MamaService.Match --data '{"entries": [{"location": {"longitude": 21.0245, "latitude": 52.2441}}]}' localhost:50051

docker kill $CONTAINER_ID