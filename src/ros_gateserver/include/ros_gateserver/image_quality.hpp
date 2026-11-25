#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rclcpp
{
class Node;
}

namespace ros_gateserver
{
struct ImageQualityConfig
{
    bool enabled{false};
    int max_side{320};
    int64_t max_pixels{8294400};
    double dark_threshold{30.0};
    double bright_threshold{225.0};
    double contrast_threshold{10.0};
    double sharpness_threshold{50.0};
};

ImageQualityConfig declareImageQualityConfig(rclcpp::Node& node);

// 配置构造后只读；分析不修改 ROS 像素，不保留消息或 cv::Mat 到下一次回调。
class ImageQualityAnalyzer
{
public:
    explicit ImageQualityAnalyzer(const ImageQualityConfig& config);

    nlohmann::json analyze(const sensor_msgs::msg::Image::ConstSharedPtr& message) const;

private:
    ImageQualityConfig config_;
};
} // namespace ros_gateserver
