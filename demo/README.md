# Mama Demo

This demo polls [GTFS-RT feed of Warsaw's public transport](https://mkuran.pl/gtfs/) and visualizes map matched positions of vehicles on a map.

## Build and run

**Prerequisites**: [`osmium`](https://osmcode.org/osmium-tool/), `wget` and [Docker](https://www.docker.com/)


1. [`./build.sh`](./build.sh) - it prepares data and build required Docker images
2. `docker compose up`
3. Open web browser and navigate to `http://127.0.0.1:8000/`
4. You should see something like this(vehicles positions are updated every 10 seconds):
<img width="1500" alt="Screenshot 2023-01-24 at 17 01 34" src="https://user-images.githubusercontent.com/266271/214345648-73f70fc9-fbd3-4e98-8f12-55eae71f3f24.png">


## Design

For basic understanding how this demo works look at this diagram:

```mermaid
sequenceDiagram 
  Browser->>Node: GET /
  Node-->>Browser: index.html
  loop Every second
    Node->>GTFS-RT: Request 
    GTFS-RT-->>Node: Vehicle positions
    Node->>Redis: Get map-matching states
    Redis-->>Node: Map-matching states or null
    Node->>Mama: Vehicle positions + Map-matching states
    Mama-->>Node: Map matched locations + Map-matching states
    Node->>Redis: Store map-matching states
    Node->>Browser: Map matched locations
  end
```
