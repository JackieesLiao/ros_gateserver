#pragma once

#include "gateway_status.pb.h"
#include <nlohmann/json.hpp>
#include <string>

namespace ros_gateserver
{
protocol::GatewayStatus makeProtobufStatus(const nlohmann::json& status);
std::string encodeProtobufStatus(const protocol::GatewayStatus& message);
} // namespace ros_gateserver
