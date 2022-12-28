const http = require('http');
const https = require('node:https');
const fs = require('fs');
const protobuf = require('protobufjs');
const crypto = require("crypto");


const POLLING_INTERVAL = 1000;
const GTFS_URL = 'https://mkuran.pl/gtfs/warsaw/vehicles.pb';

const subscribers = [];

let positions = [];


let FeedMessage = null;

function generateId() {
    return crypto.randomBytes(16).toString("hex");
}

function notify(subscriber) {
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


fs.readFile('./index.html', function (err, html) {
    if (err) {
        throw err; 
    }       
    const server = http.createServer(function(request, response) {  
        if (request.headers.accept && request.headers.accept == 'text/event-stream') {
            if (request.url == '/talk') {
                subscribe(response);
            }   
        } else {
            response.writeHeader(200, {"Content-Type": "text/html"});  
            response.write(html);  
            response.end();  
        }


    }).listen(8000);
    
    startPolling();
});