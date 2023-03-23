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

This service exposes API via [gRPC](https://grpc.io/), so you have to use gRPC client to access it(see [mama.proto](https://github.com/SiarheiFedartsou/mama/blob/main/server/mama.proto) for service definition). For test purposes it can be done with [`grpc_cli`](https://github.com/grpc/grpc/blob/master/doc/command_line_tool.md):

```
grpc_cli call localhost:50051 Match "entries: [{location: {latitude: 52.517037, longitude: 13.388860}}]"
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
