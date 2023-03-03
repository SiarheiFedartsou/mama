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
const mamaClient = new mama_proto.MamaService(MAMA_GRPC_URL, grpc.credentials.createInsecure()); 


const request = {
    entries: [
        {
            location: {
                timestamp: {
                    seconds: -1,
                },
                longitude: process.argv[2],
                latitude: process.argv[3]
            }
        }
    ]
};

mamaClient.match(request, async function(err, response) {
    if (err) {
        console.log(err);
        process.exit(1);
    }
    console.log((response.entries[0].state).toString('base64'));
});
