# MaMa 
[![CI](https://github.com/SiarheiFedartsou/mama/actions/workflows/ci.yml/badge.svg)](https://github.com/SiarheiFedartsou/mama/actions/workflows/ci.yml)


MaMa (short for Map Matcher) is a library that lets you perform real-time map matching of locations coming from multiple clients as part of distributed system (but in its core it is just a library which can easily be used locally). Main target of MaMa is blazingly fast performance.

⚠️ Warning: This project is currently in its early stage of development and may contain bugs, incomplete features, and other issues. Please use it at your own risk and feel free to report any problems you encounter.
🌟 If you find this project helpful or interesting, please consider giving it a star! It helps to boost the project's visibility and encourages us to keep improving it. Thank you! 🙏

##  Quick start & Demo 

Here is example how one can configure and run `mama` map matching service for Berlin:
```
mkdir tiles
cd tiles
# download OSM data
wget http://download.geofabrik.de/europe/germany/berlin-latest.osm.pbf
# generate mama tiles
docker run -t -v $(pwd):/tiles ghcr.io/siarheifedartsou/mama:main tilegen /tiles/berlin-latest.osm.pbf /tiles
# run mama-server
docker run -t -p 50051:50051 -v $(pwd):/tiles ghcr.io/siarheifedartsou/mama:main mama_server /tiles 
```

This service exposes API via [gRPC](https://grpc.io/), so you have to use gRPC client to access it(see [mama.proto](https://github.com/SiarheiFedartsou/mama/blob/main/server/mama.proto) for service definition). For test purposes it can be done with [`grpcurl`](https://github.com/fullstorydev/grpcurl):
```
docker run --add-host host.docker.internal:host-gateway fullstorydev/grpcurl \
  -plaintext \
  -d '{"entries": [{"location": {"latitude": 52.517037, "longitude": 13.388860, "timestamp": "2023-01-01T00:00:00Z"}}]}' \
  host.docker.internal:50051 \
  mama.server.api.MamaService.Match
```
It should output something like:
```
{
  "entries": [
    {
      "location": {
        "timestamp": "2023-01-01T00:00:00Z",
        "latitude": 52.51703335973838,
        "longitude": 13.38879800553748,
        "bearing": 354.47464844888776
      },
      "state": "ChYKDwoICMfC+ggQsk4VYb4bPxXuZgi/ChYKDwoICMfC+ggQ6U8V5TLCPhXuZgi/ChYKDwoICMfC+ggQsk4V/xUHPxVdNkK/ChYKDwoICMfC+ggQ6U8VqYPrPhVdNkK/ChYKDwoICMfC+ggQojAVfiZuPxXDgaDAChYKDwoICMfC+ggQ60MVwmd9PxWFtLDAChEKCgoICMfC+ggQ508VhbSwwAoRCgoKCAjHwvoIEOhPFYW0sMAKFgoPCggIx8L6CBCiMBUm40c/FSLD48AKFgoPCggIx8L6CBCiMBWosi0/FSi1L8EKFgoPCggIx8L6CBDoTxXF60Q+FV6MQMEKFgoPCggIx8L6CBCxThVt/oM9FUrOSMEKFgoPCggIx8L6CBCQQRWYL38/FSnjVcEKFgoPCggIx8L6CBCuThWX9H4/FSnjVcEKEQoKCggIx8L6CBCwThUp41XBChYKDwoICMfC+ggQsU4V+4wXPhX5yV7BChYKDwoICMfC+ggQ60MVrQc9PxVh8pDBChYKDwoICMfC+ggQ508VKcCAPhVh8pDBEhoKBgiAmsOdBhE26bZELkJKQBnzcW2oGMcqQBolCgYIgJrDnQYROIItJi5CSkAZJxE9iBDHKkAiCQkf0fgomCd2QA=="
    }
  ]
}
```
Note `state` field, it encodes map matching state for particular vehicle. You can pass this state field in subsequent requests to take advantage of the knowledge of past vehicle locations:
```
docker run --add-host host.docker.internal:host-gateway fullstorydev/grpcurl \
  -plaintext \
  -d '{"entries": [{"state": "ChYKDwoICMfC+ggQsk4VYb4bPxXuZgi/ChYKDwoICMfC+ggQ6U8V5TLCPhXuZgi/ChYKDwoICMfC+ggQsk4V/xUHPxVdNkK/ChYKDwoICMfC+ggQ6U8VqYPrPhVdNkK/ChYKDwoICMfC+ggQojAVfiZuPxXDgaDAChYKDwoICMfC+ggQ60MVwmd9PxWFtLDAChEKCgoICMfC+ggQ508VhbSwwAoRCgoKCAjHwvoIEOhPFYW0sMAKFgoPCggIx8L6CBCiMBUm40c/FSLD48AKFgoPCggIx8L6CBCiMBWosi0/FSi1L8EKFgoPCggIx8L6CBDoTxXF60Q+FV6MQMEKFgoPCggIx8L6CBCxThVt/oM9FUrOSMEKFgoPCggIx8L6CBCQQRWYL38/FSnjVcEKFgoPCggIx8L6CBCuThWX9H4/FSnjVcEKEQoKCggIx8L6CBCwThUp41XBChYKDwoICMfC+ggQsU4V+4wXPhX5yV7BChYKDwoICMfC+ggQ60MVrQc9PxVh8pDBChYKDwoICMfC+ggQ508VKcCAPhVh8pDBEhoKBgiAmsOdBhE26bZELkJKQBnzcW2oGMcqQBolCgYIgJrDnQYROIItJi5CSkAZJxE9iBDHKkAiCQkf0fgomCd2QA==", "location": {"latitude": 52.517301, "longitude": 13.38884, "timestamp": "2023-01-01T00:00:03Z"}}]}' \
  host.docker.internal:50051 \
  mama.server.api.MamaService.Match
```

See our [demo](./demo) for example of real life usage.

## How it works

#### `tilegen`

In order to work `mama` preprocess OSM data to more convenient representation. `tilegen` is a tool used for that.

`tilegen` takes `.osm.pbf` as input and slices it to tiles. Tile scheme corresponds to level 11 of [S2Geometry](https://s2geometry.io/), i.e. has approximately 20 km^2 in area:

<img width="700" alt="Screenshot 2023-01-24 at 17 55 55" src="https://user-images.githubusercontent.com/266271/214357432-6d7ec7af-9c13-418f-8d11-0ed338056dc9.png">

Each tile stores metadata about road graph edges accessible by cars in particular area and [S2 index](https://s2geometry.io/devguide/s2shapeindex.html) which allows to effectively find projections on road graph. 

`tilegen` also optionally precomputes shortest paths used in map matching algorithm, which significantly boosts runtime performance. 

#### `libmama`
`libmama` is C++ library implementing core features of `mama`. [`mama_server`](https://github.com/SiarheiFedartsou/mama/blob/cb15123ca81d6712f2cccff06cc842a1b6e2b9ce/server/mama_server.cc#L57) can be a good example of how it should be used. 

#### `mama_server`
`mama_server` is gRPC service which wraps `libmama` and provides map matching capabilities to external clients. This service doesn't store any state and it is responsibility of clients to store map matching states(see [demo](./demo) for example how Redis can be used for that purpose). 


## Credits
- Map matching algorithm itself is mainly inspired by "classic" Microsoft paper [`Hidden Markov Map Matching Through Noise and Sparseness`](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/map-matching-ACM-GIS-camera-ready.pdf).
- Shortest paths precomputation is inspired by [`Fast map matching, an algorithm integrating hidden
Markov model with precomputation`](https://people.kth.se/~cyang/bib/fmm.pdf) paper.

## License
MaMa uses [MIT license](https://github.com/SiarheiFedartsou/mama/blob/main/LICENSE).
