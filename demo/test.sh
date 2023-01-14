#!/bin/bash
set -e

# check that mama_server is possible to run (to avoid certain issues we had in the past)
# TODO: this is not enough to be called normal regression test
if $(docker run -t mama-demo /app/mama_server) | grep -vq 'Usage:'; then
  echo "mama_server doesn't return usage info, something is wrong"
fi

# run demo and check that it returns 200
docker compose up --detach
curl --retry-delay 3 --retry 10 --retry-all-errors "http://127.0.0.1:8000/"
docker compose down
