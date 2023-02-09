#!/bin/bash
set -e

# install ghz
curl -L https://github.com/bojand/ghz/releases/download/v0.114.0/ghz-linux-x86_64.tar.gz | tar xvz
mv ghz /usr/local/bin

echo "PWD $PWD"
ls $(pwd)/tiles

# TODO: kill container afterwards
docker run -d -p 50051:50051 -v $(pwd)/tiles:/tiles -t /app/mama-server /tiles

# TODO: add normal healthcheck
sleep 10

# TODO: we should also benchmark case when we have previous state
ghz --total 1000000 --insecure --proto ./server/mama.proto --call mama.server.api.MamaService.Match --data '{"entries": [{"location": {"longitude": 21.0245, "latitude": 52.2441}}]}' localhost:50051

