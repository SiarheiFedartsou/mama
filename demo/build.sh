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

docker build -t mama-demo -f ../server/Dockerfile .. 
docker run -v $(pwd)/tiles:/tiles -t mama-demo /app/tilegen /tiles/warsaw.osm.pbf /tiles
echo "RUNNING mama_server"
#docker run -v $(pwd)/tiles:/tiles -t mama-demo /app/mama_server
echo "STOPPED mama_server"

# # run server
docker compose build
