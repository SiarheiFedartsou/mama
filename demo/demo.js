const http = require('http');
const https = require('node:https');
const fs = require('fs');
const protobuf = require('protobufjs');
const crypto = require("crypto");
const redis = require("redis");



// TODO: proto is copy-pasted
const PROTO_PATH = __dirname + '/protos/mama.proto';
const grpc = require('@grpc/grpc-js');
const protoLoader = require('@grpc/proto-loader');
const packageDefinition = protoLoader.loadSync(
    PROTO_PATH,
    {keepCase: true,
     longs: String,
     enums: String,
     defaults: true,
     oneofs: true
    });
const mama_proto = grpc.loadPackageDefinition(packageDefinition).mama.server.api;

const MAMA_GRPC_URL = `${process.env['MAMA_HOST'] || 'localhost'}:${process.env['MAMA_PORT'] || '50051'}`;
const PORT = 5000;
const POLLING_INTERVAL = 1000;
const GTFS_URL = 'https://mkuran.pl/gtfs/warsaw/vehicles.pb';
const VEHICLE_STATE_EXPIRE_S = 180;
const subscribers = [];

let positions = [];

const mamaClient = new mama_proto.MamaService(MAMA_GRPC_URL, grpc.credentials.createInsecure()); 
const redisClient = redis.createClient({
    url: `redis://${process.env['REDIS_HOST'] || 'localhost'}:${process.env['REDIS_PORT'] || '6379'}`
});



let FeedMessage = null;

function generateId() {
    return crypto.randomBytes(16).toString("hex");
}

function log(msg) {
    process.stderr.write(msg + '\n');
}

function notify(subscriber) {
    log(`notify`);
    const toNotify = subscriber ? [subscriber] : subscribers;
    toNotify.forEach((subscriber) => {
        subscriber.res.write('id: ' + subscribe.id + '\n');
        subscriber.res.write('data: ' + JSON.stringify(positions) + '\n\n');
    });
}

function subscribe(res) {
    res.writeHead(200, {
        'Content-Type' : 'text/event-stream',
        'Cache-Control' : 'no-cache',
        'Connection' : 'keep-alive'
    });

    subscribers.push({id: generateId(), res});

    notify(subscribers[subscribers.length - 1]);
}

async function makeGrpcRequest(feedMessage, callback) {
    const buildRedisKey = (id) => `vehicle:${id}:state`;

    const request = {
        entries: []
    };
    log('Getting state from Redis...');
    // TODO: likely we can request all states in one request to Redis
    for (let i = 0; i < feedMessage.entity.length; ++i) {
        const entity = feedMessage.entity[i];
        const location = {
            timestamp: {
                seconds: entity.vehicle.timestamp,
                nanos: 0
            },
            latitude: entity.vehicle.position.latitude,
            longitude: entity.vehicle.position.longitude,
            speed: null,
            bearing: null
        };
        const redisKey = buildRedisKey(entity.vehicle.vehicle.id);
        const state = await redisClient.get(redis.commandOptions({ returnBuffers: true }), redisKey);
        request.entries.push({location, state});
    }

    log('Calling service...');
    mamaClient.match(request, async function(err, response) {
        if (err || response.entries.length != feedMessage.entity.length) {
            log(`Map matching error: ${err}`);
            callback(err);
            return;
        }

        const positions = [];

        for (let i = 0; i < feedMessage.entity.length; ++i) {
            const state = response.entries[i].state;            
            const redisKey = buildRedisKey(feedMessage.entity[i].vehicle.vehicle.id);
            await redisClient.setEx(redisKey, VEHICLE_STATE_EXPIRE_S, state);

            positions.push({
                id: feedMessage.entity[i].vehicle.vehicle.id,
                label: feedMessage.entity[i].vehicle.vehicle.label,
                position: response.entries[i].location
            });
        }
        callback(null, positions);
    });
}

async function polling() {
    https.get(GTFS_URL, (res) => {
        log('polling');

        const buffers = [];
        res.on('data', (d) => {
            buffers.push(d);
        });

        res.on('end', async () => {
            try {
                const buffer = Buffer.concat(buffers);
                const message = FeedMessage.decode(buffer);
               // message.entity = message.entity.filter(x => x.id == 'V/518/1');
              //  console.log(JSON.stringify(message.entity[0]));
                log(`Got ${message.entity.length} vehicles. Matching...`);
                // message.entity = [message.entity[0]];
                await makeGrpcRequest(message, (err, matchedPositions) => {
                    if (err) {
                        return;
                    }
                    positions = matchedPositions;
                    log(`Got ${positions.length} positions`);
                    notify();
                });
            } catch(err) {
                log(`Error: ${err}`);
            } finally {
                setTimeout(polling, POLLING_INTERVAL);
            }
        });
    }).on('error', (e) => {
        log(`Error on GTFS polling: ${e}. Retrying...`);
        setTimeout(polling, POLLING_INTERVAL);
    });
}

function startPolling() {
    protobuf.load("gtfs.proto", function(err, root) {
        if (err) {
            throw error;
        }
        FeedMessage = root.lookupType("transit_realtime.FeedMessage");
        polling();
    });
}


async function main() {
    await redisClient.connect();

    fs.readFile('./index.html', function (err, html) {
        if (err) {
            throw err; 
        }       
        http.createServer(function(request, response) {  
            if (request.headers.accept && request.headers.accept == 'text/event-stream') {
                if (request.url == '/talk') {
                    subscribe(response);
                }   
            } else {
                response.writeHeader(200, {"Content-Type": "text/html"});  
                response.write(html);  
                response.end();  
            }
    
    
        }).listen(PORT);
    
        
        log(`Server running at http://localhost:${PORT}/`)
        
        startPolling();
    });
}

main();