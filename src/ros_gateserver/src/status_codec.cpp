#include "ros_gateserver/status_codec.hpp"
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace ros_gateserver
{
namespace
{
using Json = nlohmann::json;

std::optional<double> finiteNumber(const Json& value)
{
    if (value.is_null())
    {
        return std::nullopt;
    }
    const double number = value.get<double>();
    if (!std::isfinite(number))
    {
        // 与 JSON dump() 对非有限数的 null 表示保持一致。
        return std::nullopt;
    }
    return number;
}

template <typename Message>
void fillMetadata(const Json& data, Message* message)
{
    message->set_frame_id(data.at("frame_id").get<std::string>());
    const auto& stamp = data.at("stamp");
    message->mutable_stamp()->set_sec(stamp.at("sec").get<int32_t>());
    message->mutable_stamp()->set_nanosec(stamp.at("nanosec").get<uint32_t>());
}

template <typename Message>
void fillCoordinates(const Json& data, Message* message)
{
    if (const auto x = finiteNumber(data.at("x")))
    {
        message->set_x(*x);
    }
    if (const auto y = finiteNumber(data.at("y")))
    {
        message->set_y(*y);
    }
    if (const auto z = finiteNumber(data.at("z")))
    {
        message->set_z(*z);
    }
}

void fillPoint(const Json& data, protocol::Point* message)
{
    message->set_frame_id(data.at("frame_id").get<std::string>());
    fillCoordinates(data, message);
}

void fillSensor(const Json& sensor, protocol::SensorStatus* message)
{
    message->set_received(sensor.at("received").get<uint64_t>());
    message->set_fresh(sensor.at("fresh").get<bool>());
    if (const auto age = finiteNumber(sensor.at("age_sec")))
    {
        message->set_age_sec(*age);
    }

    const auto& data = sensor.at("diagnostics");
    auto* diagnostics = message->mutable_diagnostics();
    diagnostics->set_state(data.at("state").get<std::string>());
    diagnostics->set_topic(data.at("topic").get<std::string>());
    diagnostics->set_reliability(data.at("reliability").get<std::string>());
    diagnostics->set_publishers(data.at("publishers").get<uint64_t>());
    diagnostics->set_compatible_publishers(data.at("compatible_publishers").get<uint64_t>());
    diagnostics->set_incompatible_publishers(data.at("incompatible_publishers").get<uint64_t>());
    diagnostics->set_uncertain_publishers(data.at("uncertain_publishers").get<uint64_t>());
    diagnostics->set_type_mismatches(data.at("type_mismatches").get<uint64_t>());
    diagnostics->set_incompatible_events(data.at("incompatible_events").get<uint64_t>());
    diagnostics->set_last_policy(data.at("last_policy").get<std::string>());
    diagnostics->set_event_supported(data.at("event_supported").get<bool>());
    diagnostics->set_graph_error(data.at("graph_error").get<std::string>());
    if (const auto rate = finiteNumber(data.at("receive_hz")))
    {
        diagnostics->set_receive_hz(*rate);
    }
    for (const auto& reason : data.at("reasons"))
    {
        diagnostics->add_reasons(reason.get<std::string>());
    }
}

void fillImageQuality(const Json& data, protocol::ImageQuality* message)
{
    message->set_status(data.at("status").get<std::string>());
    message->set_reason(data.at("reason").get<std::string>());
    message->set_analysis_width(data.at("analysis_width").get<uint32_t>());
    message->set_analysis_height(data.at("analysis_height").get<uint32_t>());
    if (const auto mean = finiteNumber(data.at("mean_brightness")))
    {
        message->set_mean_brightness(*mean);
    }
    if (const auto contrast = finiteNumber(data.at("contrast_stddev")))
    {
        message->set_contrast_stddev(*contrast);
    }
    if (const auto sharpness = finiteNumber(data.at("laplacian_variance")))
    {
        message->set_laplacian_variance(*sharpness);
    }
    for (const auto& flag : data.at("flags"))
    {
        message->add_flags(flag.get<std::string>());
    }
}

void fillImage(const Json& data, protocol::ImageData* message)
{
    fillMetadata(data, message);
    message->set_width(data.at("width").get<uint32_t>());
    message->set_height(data.at("height").get<uint32_t>());
    message->set_encoding(data.at("encoding").get<std::string>());
    message->set_byte_count(data.at("bytes").get<uint64_t>());
    if (data.contains("quality"))
    {
        fillImageQuality(data.at("quality"), message->mutable_quality());
    }
}

void fillScan(const Json& data, protocol::ScanData* message)
{
    fillMetadata(data, message);
    message->set_samples(data.at("samples").get<uint64_t>());
    message->set_valid_samples(data.at("valid_samples").get<uint64_t>());
    message->set_transform_status(data.at("transform_status").get<std::string>());
    if (const auto nearest = finiteNumber(data.at("nearest_m")))
    {
        message->set_nearest_m(*nearest);
    }
    if (!data.at("nearest_point_sensor").is_null())
    {
        fillPoint(data.at("nearest_point_sensor"), message->mutable_nearest_point_sensor());
    }
    if (!data.at("nearest_point").is_null())
    {
        fillPoint(data.at("nearest_point"), message->mutable_nearest_point());
    }
}

void fillOdometry(const Json& data, protocol::OdometryData* message)
{
    fillMetadata(data, message);
    message->set_child_frame_id(data.at("child_frame_id").get<std::string>());
    fillCoordinates(data.at("position"), message->mutable_position());
    fillCoordinates(data.at("orientation"), message->mutable_orientation());
    if (const auto w = finiteNumber(data.at("orientation").at("w")))
    {
        message->mutable_orientation()->set_w(*w);
    }
    if (const auto linear = finiteNumber(data.at("linear_x")))
    {
        message->set_linear_x(*linear);
    }
    if (const auto angular = finiteNumber(data.at("angular_z")))
    {
        message->set_angular_z(*angular);
    }
}
} // namespace

protocol::GatewayStatus makeProtobufStatus(const Json& status)
{
    protocol::GatewayStatus message;
    message.set_schema_version(status.at("schema_version").get<uint32_t>());
    message.set_sequence(status.at("sequence").get<uint64_t>());
    message.set_enabled(status.at("enabled").get<bool>());
    message.set_stamp_ns(status.at("stamp_ns").get<int64_t>());
    const auto& clock = status.at("clock");
    message.mutable_clock()->set_source(clock.at("source").get<std::string>());
    message.mutable_clock()->set_initialized(clock.at("initialized").get<bool>());
    message.mutable_clock()->set_epoch(clock.at("epoch").get<uint64_t>());

    const auto& image = status.at("sensors").at("image");
    fillSensor(image, message.mutable_image());
    if (!image.at("data").is_null())
    {
        fillImage(image.at("data"), message.mutable_image()->mutable_image_data());
    }

    const auto& scan = status.at("sensors").at("scan");
    fillSensor(scan, message.mutable_scan());
    if (!scan.at("data").is_null())
    {
        fillScan(scan.at("data"), message.mutable_scan()->mutable_scan_data());
    }

    const auto& odom = status.at("sensors").at("odom");
    fillSensor(odom, message.mutable_odom());
    if (!odom.at("data").is_null())
    {
        fillOdometry(odom.at("data"), message.mutable_odom()->mutable_odom_data());
    }

    return message;
}

std::string encodeProtobufStatus(const protocol::GatewayStatus& message)
{
    std::string payload;
    if (!message.SerializeToString(&payload))
    {
        throw std::runtime_error("Failed to serialize gateway protobuf status");
    }
    return payload;
}
} // namespace ros_gateserver
