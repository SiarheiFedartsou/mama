#!/bin/bash
set -e

# generate tiles
rm -rf tiles
mkdir tiles 
pushd tiles
wget -q http://download.geofabrik.de/europe/poland-latest.osm.pbf
# extract Warsaw area using osmium (`brew install osmium-tool` or `apt-get install osmium-tool`)
osmium extract --bbox=20.6,51.8,21.5,52.6 poland-latest.osm.pbf -o warsaw.osm.pbf
popd

# build mama server
docker build -t mama-server -f ../server/Dockerfile .. 

# generate tiles
docker run -v $(pwd)/tiles:/tiles -t mama-server tilegen /tiles/warsaw.osm.pbf /tiles --max-precompute-path-length 250

# build demo
docker compose build
