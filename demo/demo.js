const http = require('http');
const https = require('node:https');
const fs = require('fs');
const protobuf = require('protobufjs');
const crypto = require("crypto");

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
const mama_proto = grpc.loadPackageDefinition(packageDefinition).mama_server;

const MAMA_GRPC_URL = `${process.env['MAMA_HOST'] || 'localhost'}:${process.env['MAMA_PORT'] || '50051'}`;
const PORT = 5000;
const POLLING_INTERVAL = 1000;
const GTFS_URL = 'https://mkuran.pl/gtfs/warsaw/vehicles.pb';

const subscribers = [];

let positions = [];


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

function polling() {
    https.get(GTFS_URL, (res) => {
        log('polling');

        const buffers = [];
        res.on('data', (d) => {
            buffers.push(d);
        });

        res.on('end', () => {
            try {
                const buffer = Buffer.concat(buffers);
                const message = FeedMessage.decode(buffer);
    
    
                positions = message.entity.map((entity) => {
                    return {
                        id: entity.vehicle.vehicle.id,
                        label: entity.vehicle.vehicle.label,
                        position: entity.vehicle.position
                    }
                });
                log(`Got ${positions.length} positions`);
                notify();
            } finally {
                setTimeout(polling, POLLING_INTERVAL);
            }
        });
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


function main() {
    setTimeout(() => {
        const client = new mama_proto.MamaService(MAMA_GRPC_URL, grpc.credentials.createInsecure());
        const request = {
            location: {
                timestamp: {
                    seconds: 0,
                    nanos: 0
                },
                latitude: 0,
                longitude: 0,
                speed: null,
                bearing: null
            },
            state: null
        };
        client.match(request, function(err, response) {
            console.log('Greeting:', response);
        });
    }, 1000);


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