#!/bin/bash
set -e

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

install ghz
curl -sSL https://github.com/bojand/ghz/releases/download/v0.114.0/ghz-linux-x86_64.tar.gz | tar xvz
mv ghz /usr/local/bin

CONTAINER_ID=$(docker run -d -p 50051:50051 -v $SCRIPTPATH/tiles:/tiles -it mama-server /app/mama_server /tiles)

trap "docker kill $CONTAINER_ID" EXIT

# TODO: add normal healthcheck
sleep 10

# without state
DATA='{"entries": [{"location": {"longitude": 21.0245, "latitude": 52.2441}}]}'
echo "Results without state:"
ghz --total 100000 --insecure --proto $SCRIPTPATH/protos/mama.proto --call mama.server.api.MamaService.Match --data "$DATA" localhost:50051


# with state
STATE=$(node $SCRIPTPATH/benchmark_get_state.js 21.1245 52.2441)
DATA="{\"entries\": [{\"state\": \"${STATE}\", \"location\": {\"longitude\": 21.0245, \"latitude\": 52.2441}}]}"

echo "Results with state:"
ghz --total 100000 --insecure --proto $SCRIPTPATH/protos/mama.proto --call mama.server.api.MamaService.Match --data "$DATA" localhost:50051
