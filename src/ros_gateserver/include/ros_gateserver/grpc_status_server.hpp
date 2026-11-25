#pragma once

#include <memory>
#include <string>

namespace ros_gateserver
{
namespace protocol
{
class GatewayStatus;
}

class GrpcStatusServer
{
public:
    GrpcStatusServer(const std::string& host, int port);
    ~GrpcStatusServer();

    // The ROS executor finishes all updates before the server is destroyed.
    void update(protocol::GatewayStatus status);

private:
    class Service;
    std::unique_ptr<Service> service_;
};
} // namespace ros_gateserver
