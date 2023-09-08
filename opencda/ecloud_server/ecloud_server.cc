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
#include <chrono>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "absl/log/log.h"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "ecloud.grpc.pb.h"
#include "ecloud.pb.h"

//#include <glog/logging.h>

ABSL_FLAG(uint16_t, sim_port, 50052, "Server port for the service");
ABSL_FLAG(uint16_t, vehicle_one_port, 50051, "Server port for the service");
ABSL_FLAG(uint16_t, vehicle_two_port, 50053, "Server port for the service");
ABSL_FLAG(uint16_t, log_level, 0, "Server port for the service");

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
using ecloud::WaypointRequest;
using ecloud::EdgeWaypoints;

#define _DEBUG
//#define NDEBUG
#define WORLD_TICK_DEFAULT_MS 50

static void _sig_handler(int signo) 
{
    if (signo == SIGTERM || signo == SIGINT) 
    {
        exit(signo);
    }
}

volatile std::atomic<int16_t> numRegisteredVehicles_;
volatile std::atomic<int16_t> numCompletedVehicles_;
volatile std::atomic<int16_t> numRepliedVehicles_;
volatile std::atomic<int32_t> tickId_;
volatile std::atomic<int32_t> lastWorldTickTimeMS_;

bool init_;
int8_t logLevel_;
int16_t numCars_;
std::string configYaml_;
std::string application_;
std::string version_;

State simState_;
Command command_;

std::vector<std::pair<int16_t, std::string>> serializedEdgeWaypoints_; // vehicleIdx, serializedWPBuffer

absl::Mutex mu_;
std::vector<std::string> pendingReplies_ ABSL_GUARDED_BY(mu_); // serialized protobuf

// Logic and data behind the server's behavior.
class EcloudServiceImpl final : public Ecloud::CallbackService {
public:
    explicit EcloudServiceImpl() {
        if ( !init_ )
        {
            numCompletedVehicles_.store(0);
            numRepliedVehicles_.store(0);
            numRegisteredVehicles_.store(0);
            tickId_.store(0);
            lastWorldTickTimeMS_.store(WORLD_TICK_DEFAULT_MS);

            simState_ = State::UNDEFINED;
            command_ = Command::TICK;
            
            numCars_ = 0;
            configYaml_ = "";

            pendingReplies_.clear();
            init_ = true;
        }
    }

    ServerUnaryReactor* Client_Ping(CallbackServerContext* context,
                               const Ping* ping,
                               Ping* pong) override {                                           
        const int32_t tick_ = tickId_.load();
        const Command comm_ = command_;
        const bool new_ = tickId_ != ping->tick_id() ? true : false;

        pong->set_tick_id(tick_);
        pong->set_command(comm_);

        if ( logLevel_ > 0 && ( pong->tick_id() - 1 ) % 5 == 0 && new_ )
        {
            const auto now = std::chrono::system_clock::now();
            LOG(INFO) << "sent new tick " << pong->tick_id() << " at " << std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_Ping(CallbackServerContext* context,
                               const Empty* empty,
                               Ping* ping) override {
        const int16_t replies_ = numRepliedVehicles_.load();
        const int16_t completions_ = numCompletedVehicles_.load();
        const bool complete_ = ( replies_ + completions_ ) == numCars_;

        if ( complete_ )
            LOG(INFO) << "Server_Ping COMPLETE";

        if ( simState_ == State::NEW )
        {
            assert( replies_ == pendingReplies_.size() );
            ping->set_tick_id( replies_ );
        }
        else
        {
            ping->set_tick_id( complete_ ? 1 : 0 );
        }

        if ( logLevel_ > 0 && complete_ )
        {
            const auto now = std::chrono::system_clock::now();
            LOG(INFO) << "tick " << tickId_ << " complete at " << std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();    
        }

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

        if ( logLevel_ > 0 && ( tickId_ - 1 ) % 5 == 0 )
        {
            const auto now = std::chrono::system_clock::now();
            LOG(INFO) << "received OK from vehicle " << request->vehicle_index() << " taking " << request->step_time_ms() << "ms for tick " << tickId_ << " at " << std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();
        }    

        DLOG(INFO) << "Client_SendUpdate - received reply from vehicle " << request->vehicle_index() << " for tick id:" << request->tick_id();

        if ( request->vehicle_state() == VehicleState::TICK_DONE )
        {
            numCompletedVehicles_++;
            DLOG(INFO) << "Client_SendUpdate - TICK_DONE - tick id: " << tickId_ << " vehicle id: " << request->vehicle_index();
        }
        else if ( request->vehicle_state() == VehicleState::TICK_OK )
        {
            numRepliedVehicles_++;
        }
        else if ( request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
        {
            numCompletedVehicles_++;
            DLOG(INFO) << "Client_SendUpdate - DEBUG_INFO_UPDATE - tick id: " << tickId_ << " vehicle id: " << request->vehicle_index();
        }

        DLOG(INFO) << "Client_SendUpdate - replying tick id: " << tickId_;
        reply->set_tick_id(tickId_.load());
        reply->set_last_world_tick_time_ms(lastWorldTickTimeMS_.load());

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // server can push WP *before* ticking world and client can fetch them before it ticks
    ServerUnaryReactor* Client_GetWaypoints(CallbackServerContext* context,
                               const WaypointRequest* request,
                               WaypointBuffer* buffer) override {
        
        for ( int i = 0; i < serializedEdgeWaypoints_.size(); i++ )
        {
            const std::pair<int16_t, std::string > wpPair = serializedEdgeWaypoints_[i];
            if ( wpPair.first == request->vehicle_index() )
            {
                buffer->set_vehicle_index(request->vehicle_index());
                WaypointBuffer *wpBuf;
                wpBuf->ParseFromString(wpPair.second);
                for ( Waypoint wp : wpBuf->waypoint_buffer())
                {
                    Waypoint *p = buffer->add_waypoint_buffer();
                    p->CopyFrom(wp);
                }
                break;
            }
        }
        
        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Client_RegisterVehicle(CallbackServerContext* context,
                               const VehicleUpdate* request,
                               SimulationState* reply) override {
        
        assert( configYaml_ != "" );

        if ( request->vehicle_state() == VehicleState::REGISTERING )
        {
            DLOG(INFO) << "got a registration update";

            reply->set_state(State::NEW);
            reply->set_tick_id(0);
            reply->set_vehicle_index(numRegisteredVehicles_);
            reply->set_test_scenario(configYaml_);
            reply->set_application(application_);
            reply->set_version(version_);
            
            DLOG(INFO) << "RegisterVehicle - REGISTERING - vehicle id: " << reply->vehicle_index();
            
            numRegisteredVehicles_++;
        }
        else if ( request->vehicle_state() == VehicleState::CARLA_UPDATE )
        {            
            reply->set_state(State::START); // # do we need a new state? like "registering"?
            reply->set_tick_id(0);
            reply->set_vehicle_index(request->vehicle_index());
            
            DLOG(INFO) << "RegisterVehicle - CARLA_UPDATE - vehicle_index: " << request->vehicle_index() << " | actor_id: " << request->actor_id() << " | vid: " << request->vid();
            
            mu_.Lock();
            std::string msg;
            request->SerializeToString(&msg);
            pendingReplies_.push_back(msg);
            mu_.Unlock();
            
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
        assert(tickId_ == request->tick_id() - 1);
        tickId_++;
        lastWorldTickTimeMS_.store(request->last_world_tick_time_ms());
        command_ = request->command();
        
        if (logLevel_ > 0)
        {
            const auto now = std::chrono::system_clock::now();
            LOG(INFO) << "received new tick " << request->tick_id() << " at " << std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_PushEdgeWaypoints(CallbackServerContext* context,
                               const EdgeWaypoints* edgeWaypoints,
                               Empty* empty) override {
        serializedEdgeWaypoints_.clear();

        for ( WaypointBuffer wpBuf : edgeWaypoints->all_waypoint_buffers() )
        {   std::string serializedWPs;
            wpBuf.SerializeToString(&serializedWPs);
            const std::pair< int16_t, std::string > wpPair = std::make_pair( wpBuf.vehicle_index(), serializedWPs );
            serializedEdgeWaypoints_.push_back(wpPair);
        }

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
    LOG(INFO) << "Server listening on " << server_address;

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

    absl::ParseCommandLine(argc, argv);

    std::thread vehicle_one_server = std::thread(&RunServer,absl::GetFlag(FLAGS_vehicle_one_port));
    std::thread vehicle_two_server = std::thread(&RunServer,absl::GetFlag(FLAGS_vehicle_two_port));
    std::thread sim_server = std::thread(&RunServer,absl::GetFlag(FLAGS_sim_port));

    vehicle_one_server.join();
    vehicle_two_server.join();
    sim_server.join();

    return 0;
}
