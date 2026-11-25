#include "ros_gateserver/grpc_status_server.hpp"
#include "gateway_status.grpc.pb.h"
#include <boost/asio/ip/address.hpp>
#include <chrono>
#include <condition_variable>
#include <grpcpp/grpcpp.h>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace ros_gateserver
{
class GrpcStatusServer::Service final : public protocol::GatewayStatusService::Service
{
public:
    void start(const std::string& host, int port)
    {
        boost::system::error_code error;
        const auto address = boost::asio::ip::make_address(host, error);
        if (error)
        {
            throw std::invalid_argument("grpc_host must be an IPv4 or IPv6 address: " + host);
        }

        std::string endpoint = address.to_string();
        if (address.is_v6())
        {
            endpoint = "[" + endpoint + "]";
        }
        endpoint += ":" + std::to_string(port);

        grpc::ServerBuilder builder;
        // Fail startup if another process already owns the configured endpoint.
        builder.AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);
        builder.AddListeningPort(endpoint, grpc::InsecureServerCredentials());
        builder.RegisterService(this);
        server_ = builder.BuildAndStart();
        if (!server_)
        {
            throw std::runtime_error("Failed to start gRPC status server at " + endpoint);
        }
    }

    void shutdown()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        changed_.notify_all();

        // The deadline cancels writes that are waiting on a slow or disconnected client.
        server_->Shutdown(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
        server_->Wait();
    }

    void update(protocol::GatewayStatus status)
    {
        auto next = std::make_shared<const protocol::GatewayStatus>(std::move(status));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(next);
        }
        changed_.notify_all();
    }

    grpc::Status WatchStatus(grpc::ServerContext* context,
        const protocol::WatchStatusRequest*,
        grpc::ServerWriter<protocol::GatewayStatus>* writer) override
    {
        std::shared_ptr<const protocol::GatewayStatus> sent;
        while (!context->IsCancelled())
        {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                // A timeout lets idle streams observe client cancellation without a new status.
                if (!changed_.wait_for(lock,
                        std::chrono::milliseconds(100),
                        [&]
                        {
                            return stopping_ || latest_ != sent;
                        }))
                {
                    continue;
                }
                if (stopping_)
                {
                    return grpc::Status::OK;
                }
                sent = latest_;
            }

            // Each stream owns its in-flight snapshot. Socket writes never hold the shared lock.
            if (!writer->Write(*sent))
            {
                break;
            }
        }
        return grpc::Status(grpc::StatusCode::CANCELLED, "Status stream closed");
    }

private:
    std::mutex mutex_;
    std::condition_variable changed_;
    std::shared_ptr<const protocol::GatewayStatus> latest_;
    bool stopping_{false};
    std::unique_ptr<grpc::Server> server_;
};

GrpcStatusServer::GrpcStatusServer(const std::string& host, int port)
    : service_(std::make_unique<Service>())
{
    service_->start(host, port);
}

GrpcStatusServer::~GrpcStatusServer()
{
    service_->shutdown();
}

void GrpcStatusServer::update(protocol::GatewayStatus status)
{
    service_->update(std::move(status));
}
} // namespace ros_gateserver
