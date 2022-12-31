#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <grpc/support/log.h>
#include <grpcpp/grpcpp.h>

#include "mama.grpc.pb.h"
// TODO: do something with includes here
#include "mama.hpp"

using grpc::Server;
using grpc::ServerAsyncResponseWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;
using mama_server::MamaService;
using mama_server::MapMatchingRequest;
using mama_server::MapMatchingResponse;

class ServerImpl final {
 public:
  explicit ServerImpl(const std::string& tiles_folder) : graph_(std::make_shared<mama::Graph>(tiles_folder)) {}

  ~ServerImpl() {
    server_->Shutdown();
    // Always shutdown the completion queue after the server.
    cq_->Shutdown();
  }

  // There is no shutdown handling in this code.
  void Run() {
    std::string server_address("0.0.0.0:50051");

    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service_" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *asynchronous* service.
    builder.RegisterService(&service_);
    // Get hold of the completion queue used for the asynchronous communication
    // with the gRPC runtime.
    cq_ = builder.AddCompletionQueue();
    // Finally assemble the server.
    server_ = builder.BuildAndStart();
    std::cout << "Server listening on " << server_address << std::endl;

    // Proceed to the server's main loop.
    HandleRpcs();
  }

 private:
  // Class encompasing the state and logic needed to serve a request.
  class CallData {
   public:
    // Take in the "service" instance (in this case representing an asynchronous
    // server) and the completion queue "cq" used for asynchronous communication
    // with the gRPC runtime.
    CallData(std::shared_ptr<mama::Graph> graph, MamaService::AsyncService* service, ServerCompletionQueue* cq)
        : graph_(std::move(graph)), service_(service), cq_(cq), responder_(&ctx_), status_(CREATE) {
      // Invoke the serving logic right away.
      Proceed();
    }

    void Proceed() {
      if (status_ == CREATE) {
        // Make this instance progress to the PROCESS state.
        status_ = PROCESS;

        // As part of the initial CREATE state, we *request* that the system
        // start processing SayHello requests. In this request, "this" acts are
        // the tag uniquely identifying the request (so that different CallData
        // instances can serve different requests concurrently), in this case
        // the memory address of this CallData instance.
        service_->RequestMatch(&ctx_, &request_, &responder_, cq_, cq_,
                                  this);
      } else if (status_ == PROCESS) {
        // Spawn a new CallData instance to serve new clients while we process
        // the one for this CallData. The instance will deallocate itself as
        // part of its FINISH state.
        new CallData(graph_, service_, cq_);

        mama::MapMatcher map_matcher{graph_};
        // The actual processing.
        for (const auto& entry: request_.entries()) {
          std::string entry_state = entry.state();

          mama::Location input_location;
          input_location.coordinate = {entry.location().longitude(), entry.location().latitude()};
          input_location.timestamp = entry.location().timestamp().seconds() + entry.location().timestamp().nanos() / 1e9;
          if (entry.location().has_bearing()) {
            input_location.bearing = entry.location().bearing().value();
          }
          if (entry.location().has_speed()) {
            input_location.speed = entry.location().speed().value();
          }


          auto map_matched_location = map_matcher.Update(input_location, entry_state);

          auto reply_entry = reply_.add_entries();
          *reply_entry->mutable_location() = entry.location();
          reply_entry->mutable_location()->set_longitude(map_matched_location.coordinate.lng());
          reply_entry->mutable_location()->set_latitude(map_matched_location.coordinate.lat());
          //reply_entry->mutable_location()->set_bearing(map_matched_location.bearing_deg);
          
          reply_entry->set_state(entry_state);
          
          // auto reply_entry = reply_.add_entries();
          
          // mama::Coordinate request_coordinate{entry.location().longitude(), entry.location().latitude()};
          // auto projections = graph_->Project(request_coordinate, 100);

          // *reply_entry->mutable_location() = entry.location();
          // if (projections.empty()) {
          //   reply_entry->mutable_location()->mutable_speed()->set_value(42.0);
          // } else {
          //   std::sort(projections.begin(), projections.end(), [](const auto& a, const auto& b) {
          //     return a.distance_m < b.distance_m;
          //   });

          //   reply_entry->mutable_location()->set_longitude(projections[0].coordinate.lon);
          //   reply_entry->mutable_location()->set_latitude(projections[0].coordinate.lat);
          // }
        }

        // And we are done! Let the gRPC runtime know we've finished, using the
        // memory address of this instance as the uniquely identifying tag for
        // the event.
        status_ = FINISH;
        responder_.Finish(reply_, Status::OK, this);
      } else {
        GPR_ASSERT(status_ == FINISH);
        // Once in the FINISH state, deallocate ourselves (CallData).
        delete this;
      }
    }

   private:
    std::shared_ptr<mama::Graph> graph_;
    // The means of communication with the gRPC runtime for an asynchronous
    // server.
    MamaService::AsyncService* service_;
    // The producer-consumer queue where for asynchronous server notifications.
    ServerCompletionQueue* cq_;
    // Context for the rpc, allowing to tweak aspects of it such as the use
    // of compression, authentication, as well as to send metadata back to the
    // client.
    ServerContext ctx_;

    // What we get from the client.
    MapMatchingRequest request_;
    // What we send back to the client.
    MapMatchingResponse reply_;

    // The means to get back to the client.
    ServerAsyncResponseWriter<MapMatchingResponse> responder_;

    // Let's implement a tiny state machine with the following states.
    enum CallStatus { CREATE, PROCESS, FINISH };
    CallStatus status_;  // The current serving state.
  };

  // This can be run in multiple threads if needed.
  void HandleRpcs() {
    // Spawn a new CallData instance to serve new clients.
    new CallData(graph_, &service_, cq_.get());
    void* tag;  // uniquely identifies a request.
    bool ok;
    while (true) {
      // Block waiting to read the next event from the completion queue. The
      // event is uniquely identified by its tag, which in this case is the
      // memory address of a CallData instance.
      // The return value of Next should always be checked. This return value
      // tells us whether there is any kind of event or cq_ is shutting down.
      GPR_ASSERT(cq_->Next(&tag, &ok));
      GPR_ASSERT(ok);
      static_cast<CallData*>(tag)->Proceed();
    }
  }

  std::unique_ptr<ServerCompletionQueue> cq_;
  MamaService::AsyncService service_;
  std::shared_ptr<mama::Graph> graph_;
  std::unique_ptr<Server> server_;
};

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " TILESFOLDER\n";
    return 1;
  }
  ServerImpl server(argv[1]);
  server.Run();
  return 0;
}
