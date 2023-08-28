#include <iostream>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "ecloud.grpc.pb.h"
#include "ecloud.pb.h"

ABSL_FLAG(uint16_t, port, 50051, "Server port for the service");

using grpc::CallbackServerContext;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerUnaryReactor;
using grpc::Status;

using ecloud::Ecloud;
using ecloud::EcloudResponse;
using ecloud::VehicleUpdate;
using ecloud::Empty;
using ecloud::Ping;
using ecloud::State;
using ecloud::Command;
using ecloud::VehicleState;
using ecloud::SimulationState;
using ecloud::WaypointBuffer;
using ecloud::Waypoint;
using ecloud::Transform;
using ecloud::Location;
using ecloud::Rotation;
using ecloud::LocDebugHelper;
using ecloud::AgentDebugHelper;
using ecloud::PlanerDebugHelper;
using ecloud::ClientDebugHelper;

std::atomic<int8_t> numCars;
std::atomic<int8_t> completedCars;
std::atomic<int8_t> repliedCars;
std::atomic<int16_t> tickId;
std::atomic<int8_t> simState;

static std::vector<int8_t> repliedVehicles;
static std::vector<int8_t> completedVehicles;

static std::string latestMessage; // serialized protobuf
static std::string configYaml;

enum log_level {
  DEBUG,
  INFO,
  WARNING,
  ERROR,
};

class Logger {
public:
    void debug(const std::string& message) {
        if ( level == DEBUG )
            std::cout << "DEBUG: " << message << std::endl;
    }

    void info(const std::string& message) {
        if ( level == INFO )
            std::cout << "INFO: "<< message << std::endl;
        }

    void warn(const std::string& message) {
        if ( level <= WARNING )
            std::cout << "WARNING: " << message << std::endl;
        }

    void error(const std::string& message) {
        if ( level <= ERROR )
            std::cout << "ERROR: " << message << std::endl;
    }

    int level = DEBUG;
};

// Logic and data behind the server's behavior.
class EcloudServiceImpl final : public Ecloud::CallbackService {
    ServerUnaryReactor* Client_SendUpdate(CallbackServerContext* context,
                               const VehicleUpdate* request,
                               SimulationState* reply) override {
        //std::string prefix("Hello ");
        //reply->set_message(prefix + request->name());

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Client_RegisterVehicle(CallbackServerContext* context,
                               const VehicleUpdate* request,
                               SimulationState* reply) override {
        //std::string prefix("Hello ");
        //reply->set_message(prefix + request->name());

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_DoTick(CallbackServerContext* context,
                               const SimulationState* request,
                               EcloudResponse* reply) override {
        //std::string prefix("Hello ");
        //reply->set_message(prefix + request->name());
        simState = State::ACTIVE;

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_StartScenario(CallbackServerContext* context,
                               const SimulationState* request,
                               EcloudResponse* reply) override {
        //std::string prefix("Hello ");
        //reply->set_message(prefix + request->name());
        simState = State::NEW;

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_EndScenario(CallbackServerContext* context,
                               const Empty* request,
                               Empty* reply) override {
        //std::string prefix("Hello ");
        //reply->set_message(prefix + request->name());
        simState = State::ENDED;

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }
};

void RunServer(uint16_t port) {
    std::string server_address = absl::StrFormat("0.0.0.0:%d", port);
    EcloudServiceImpl service;

    simState = State::UNDEFINED;
    numCars = 0;
    completedCars = 0;
    repliedCars = 0;
    tickId = 0;

    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Sample way of setting keepalive arguments on the server. Here, we are
    // configuring the server to send keepalive pings at a period of 10 minutes
    // with a timeout of 20 seconds. Additionally, pings will be sent even if
    // there are no calls in flight on an active HTTP2 connection. When receiving
    // pings, the server will permit pings at an interval of 10 seconds.
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS,
                                10 * 60 * 1000 /*10 min*/);
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS,
                                20 * 1000 /*20 sec*/);
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
    builder.AddChannelArgument(
        GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS,
        10 * 1000 /*10 sec*/);
    // Finally assemble the server.
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    server->Wait();
}

int main(int argc, char** argv) {
    absl::ParseCommandLine(argc, argv);
    RunServer(absl::GetFlag(FLAGS_port));
    return 0;
}
