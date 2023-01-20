#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <grpc/support/log.h>
#include <grpcpp/grpcpp.h>
#include <base/log.hpp>
#include "mama.grpc.pb.h"
#include "mama.hpp"

namespace mama {
namespace server {
class ServerImpl final {
public:
  explicit ServerImpl(const std::string &tiles_folder)
      : graph_(std::make_shared<Graph>(tiles_folder)) {}

  ~ServerImpl() {
    server_->Shutdown();
    cq_->Shutdown();
  }

  // There is no shutdown handling in this code.
  void Run() {
    std::string server_address("0.0.0.0:50051");

    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service_);
    cq_ = builder.AddCompletionQueue();
    server_ = builder.BuildAndStart();
    MAMA_INFO("Server listening on {}", server_address);
    HandleRpcs();
  }

private:
  class CallData {
  public:
    CallData(std::shared_ptr<Graph> graph,
             api::MamaService::AsyncService *service,
             grpc::ServerCompletionQueue *cq)
        : graph_(std::move(graph)), service_(service), cq_(cq),
          responder_(&ctx_), status_(CREATE) {
      Proceed();
    }

    void Proceed() {
      if (status_ == CREATE) {
        status_ = PROCESS;

        service_->RequestMatch(&ctx_, &request_, &responder_, cq_, cq_, this);
      } else if (status_ == PROCESS) {
        new CallData(graph_, service_, cq_);

        MAMA_INFO("Received request with {} entries for ID = {}", request_.entries_size(), reinterpret_cast<void*>(this));
        MapMatchingController map_matcher{graph_};
        for (const auto &entry : request_.entries()) {
          std::string entry_state = entry.state();

          auto map_matched_location = map_matcher.Update(
              ConvertProtoToLocation(entry.location()), entry_state);

          auto reply_entry = reply_.add_entries();
          *reply_entry->mutable_location() =
              ConvertLocationToProto<api::Location>(map_matched_location);

          reply_entry->set_state(entry_state);
        }

        MAMA_INFO("Sending reply with {} entries for ID = {}", reply_.entries_size(), reinterpret_cast<void*>(this));

        status_ = FINISH;
        responder_.Finish(reply_, grpc::Status::OK, this);
      } else {
        GPR_ASSERT(status_ == FINISH);
        delete this;
      }
    }

  private:
    std::shared_ptr<Graph> graph_;
    api::MamaService::AsyncService *service_;
    grpc::ServerCompletionQueue *cq_;
    grpc::ServerContext ctx_;

    api::MapMatchingRequest request_;
    api::MapMatchingResponse reply_;

    grpc::ServerAsyncResponseWriter<api::MapMatchingResponse> responder_;

    enum CallStatus { CREATE, PROCESS, FINISH };
    CallStatus status_;
  };

  void HandleRpcs() {
    new CallData(graph_, &service_, cq_.get());
    void *tag;
    bool ok;
    while (true) {
      GPR_ASSERT(cq_->Next(&tag, &ok));
      GPR_ASSERT(ok);
      static_cast<CallData *>(tag)->Proceed();
    }
  }

  std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  api::MamaService::AsyncService service_;
  std::shared_ptr<Graph> graph_;
  std::unique_ptr<grpc::Server> server_;
};

} // namespace server
} // namespace mama

int main(int argc, char **argv) {
  mama::base::InitializeLogging();

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " TILESFOLDER\n";
    return 1;
  }

  mama::server::ServerImpl server(argv[1]);
  server.Run();
  return 0;
}
