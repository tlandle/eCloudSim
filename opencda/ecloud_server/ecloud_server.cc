#include <iostream>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <cassert>
#include <stdexcept>
#include <errno.h>
#include <csignal>
#include <unistd.h>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "ecloud.grpc.pb.h"
#include "ecloud.pb.h"

//#include <glog/logging.h>

#define SLEEP_TIME_S 0.000000001

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

static void _sig_handler(int signo) 
{
    if (signo == SIGTERM || signo == SIGINT) 
    {
        // server_interrupt_func();
        // shutdown_func();
        // google::ShutdownGoogleLogging();
        exit(signo);
    }
}

// Logic and data behind the server's behavior.
class EcloudServiceImpl final : public Ecloud::CallbackService {
public:
    explicit EcloudServiceImpl() {
        numCompletedVehicles_ = 0;
        numRepliedVehicles_ = 0;
        numRegisteredVehicles_ = 0;
        tickId_ = 0;
        simState_ = State::UNDEFINED;
        command_ = Command::TICK;
        
        numCars_ = 0;
        configYaml_ = "";

        pendingReplies_.clear();
    }

    ServerUnaryReactor* Client_Ping(CallbackServerContext* context,
                               const Empty* empty,
                               Ping* ping) override {
        ping->set_tick_id(tickId_);
        ping->set_command(command_);

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_Ping(CallbackServerContext* context,
                               const Empty* empty,
                               Ping* ping) override {
        ping->set_tick_id(( numRepliedVehicles_ + numCompletedVehicles_ ) == numCars_ ? 1 : 0);

        if (ping->tick_id() == 1)
            std::cout << "LOG(DEBUG) " << "Server_Ping - TICK_COMPLETE - tick id: " << tickId_ << std::endl;

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_GetVehicleUpdates(CallbackServerContext* context,
                               const Empty* empty,
                               EcloudResponse* reply) override {
        for ( int i = 0; i < pendingReplies_.size(); i++ )
        {
            VehicleUpdate *update = reply->add_vehicle_update();
            update->ParseFromString(pendingReplies_[i]);
        }

        numRepliedVehicles_ = 0;
        pendingReplies_.clear();

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Client_SendUpdate(CallbackServerContext* context,
                               const VehicleUpdate* request,
                               SimulationState* reply) override {

        if ( request->vehicle_state() == VehicleState::TICK_DONE || request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
        {   
            std::string msg;
            request->SerializeToString(&msg);
            mu_.Lock();
            pendingReplies_.push_back(msg);
            mu_.Unlock();
        }

        // std::cout << "LOG(DEBUG) " << "Client_SendUpdate - received reply from vehicle " << request->vehicle_index() << " for tick id:" << request->tick_id() << std::endl;

        if ( request->vehicle_state() == VehicleState::TICK_DONE )
        {
            numCompletedVehicles_++;
            std::cout << "LOG(DEBUG) " << "Client_SendUpdate - TICK_DONE - tick id: " << tickId_ << " vehicle id: " << request->vehicle_index() << std::endl;
        }
        else if ( request->vehicle_state() == VehicleState::TICK_OK )
        {
            numRepliedVehicles_++;
        }
        else if ( request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
        {
            numCompletedVehicles_++;
            std::cout << "LOG(DEBUG) " << "Client_SendUpdate - DEBUG_INFO_UPDATE - tick id: " << tickId_ << " vehicle id: " << request->vehicle_index() << std::endl;
        }

        // std::cout << "LOG(DEBUG) " << "Client_SendUpdate - replying tick id: " << tickId_ << std::endl;
        reply->set_tick_id(tickId_);

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Client_RegisterVehicle(CallbackServerContext* context,
                               const VehicleUpdate* request,
                               SimulationState* reply) override {
        // TODO: remove once done debugging
        //assert( configYaml_ != " " );

        if ( request->vehicle_state() == VehicleState::REGISTERING )
        {
            //std::cout << "LOG(DEBUG) " << "got a registration update" << std::endl;

            reply->set_state(State::NEW);
            reply->set_tick_id(0);
            reply->set_vehicle_index(numRegisteredVehicles_);
            reply->set_test_scenario(configYaml_);
            reply->set_application(application_);
            reply->set_version(version_);
            
            std::cout << "LOG(DEBUG) " << "RegisterVehicle - REGISTERING - vehicle id: " << reply->vehicle_index() << std::endl;
            
            numRegisteredVehicles_++;
        }
        else if ( request->vehicle_state() == VehicleState::CARLA_UPDATE )
        {
            // std::cout << "LOG(DEBUG) " << "got a carla update" << std::endl;
            
            reply->set_state(State::START); // # do we need a new state? like "registering"?
            reply->set_tick_id(0);
            reply->set_vehicle_index(request->vehicle_index());
            
            std::cout << "LOG(DEBUG) " << "RegisterVehicle - CARLA_UPDATE - vehicle_index: " << request->vehicle_index() << " | actor_id: " << request->actor_id() << " | vid: " << request->vid() << std::endl;
            
            mu_.Lock();
            std::string msg;
            request->SerializeToString(&msg);
            pendingReplies_.push_back(msg);
            mu_.Unlock();
            
            // std::cout << "LOG(DEBUG) " << "RegisterVehicle - CARLA_UPDATE - message id: " << reply->message_id() << std::endl;

            numRepliedVehicles_++;
        }
        else
        {
            assert(false);
        }   

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_DoTick(CallbackServerContext* context,
                               const SimulationState* request,
                               EcloudResponse* reply) override {
        simState_ = State::ACTIVE;

        numRepliedVehicles_ = 0;
        tickId_ = request->tick_id();
        command_ = request->command();

        // std::cout << "LOG(DEBUG) Server_DoTick: " << tickId_ << std::endl;

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_StartScenario(CallbackServerContext* context,
                               const SimulationState* request,
                               EcloudResponse* reply) override {
        simState_ = State::NEW;

        configYaml_ = request->test_scenario();
        application_ = request->application();
        version_ = request->version();
        numCars_ = request->vehicle_index(); // bit of a hack to use vindex as count

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_EndScenario(CallbackServerContext* context,
                               const Empty* request,
                               Empty* reply) override {
        simState_ = State::ENDED;

        // need to collect debug info and then send back

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

private:
    std::atomic<int8_t> numRegisteredVehicles_;
    std::atomic<int8_t> numCompletedVehicles_;
    std::atomic<int8_t> numRepliedVehicles_;
    std::atomic<int16_t> tickId_;

    int8_t numCars_;
    std::string configYaml_;
    std::string application_;
    std::string version_;

    State simState_;
    Command command_;

    absl::Mutex mu_;
    std::vector<std::string> pendingReplies_ ABSL_GUARDED_BY(mu_); // serialized protobuf
};

void RunServer(uint16_t port) {
    std::string server_address = absl::StrFormat("0.0.0.0:%d", port);
    EcloudServiceImpl service;

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
    std::cout << "LOG(INFO) " << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    server->Wait();
}

int main(int argc, char** argv) {

    if (signal(SIGINT, _sig_handler) == SIG_ERR) {
            fprintf(stderr, "Can't catch SIGINT...exiting.\n");
            exit(EXIT_FAILURE);
    }

    if (signal(SIGTERM, _sig_handler) == SIG_ERR) {
            fprintf(stderr, "Can't catch SIGTERM...exiting.\n");
            exit(EXIT_FAILURE);
    }

    //int debug_level = static_cast<int>(google::INFO);

    absl::ParseCommandLine(argc, argv);

    //FLAGS_alsologtostderr = debug_level == google::INFO ? true : false;
    //FLAGS_minloglevel = debug_level;
    //google::InitGoogleLogging(argv[0]);

    RunServer(absl::GetFlag(FLAGS_port));
    return 0;
}