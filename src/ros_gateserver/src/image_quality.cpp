#include "ros_gateserver/image_quality.hpp"
#include <algorithm>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stdexcept>
#include <string>

namespace ros_gateserver
{
namespace
{
using Json = nlohmann::json;

Json unavailable(const std::string& status, const std::string& reason)
{
    return {
        {"status", status},
        {"reason", reason},
        {"analysis_width", 0},
        {"analysis_height", 0},
        {"mean_brightness", nullptr},
        {"contrast_stddev", nullptr},
        {"laplacian_variance", nullptr},
        {"flags", Json::array()},
    };
}

std::optional<int> grayscaleConversion(const std::string& encoding)
{
    if (encoding == "mono8")
    {
        return -1;
    }
    if (encoding == "bgr8")
    {
        return cv::COLOR_BGR2GRAY;
    }
    if (encoding == "rgb8")
    {
        return cv::COLOR_RGB2GRAY;
    }
    if (encoding == "bgra8")
    {
        return cv::COLOR_BGRA2GRAY;
    }
    if (encoding == "rgba8")
    {
        return cv::COLOR_RGBA2GRAY;
    }
    return std::nullopt;
}

cv::Mat boundedGrayscale(const cv::Mat& source, int conversion, int max_side)
{
    cv::Mat resized = source;
    const int longest = std::max(source.cols, source.rows);
    if (longest > max_side)
    {
        const double scale = static_cast<double>(max_side) / longest;
        const int width = std::max(1, static_cast<int>(std::floor(source.cols * scale)));
        const int height = std::max(1, static_cast<int>(std::floor(source.rows * scale)));
        // 先限制尺寸，再做灰度转换和梯度运算，避免为完整彩色帧分配分析副本。
        cv::resize(source, resized, cv::Size(width, height), 0.0, 0.0, cv::INTER_AREA);
    }

    cv::Mat gray;
    if (conversion == -1)
    {
        gray = resized;
    }
    else
    {
        cv::cvtColor(resized, gray, conversion);
    }
    return gray;
}

Json measure(const cv::Mat& gray, const ImageQualityConfig& config)
{
    cv::Scalar mean;
    cv::Scalar deviation;
    cv::meanStdDev(gray, mean, deviation);

    cv::Mat laplacian;
    // 使用带符号浮点梯度，保留黑白边缘两个方向；方差仅作为细节强度的启发式指标。
    cv::Laplacian(gray, laplacian, CV_64F, 1, 1.0, 0.0, cv::BORDER_REFLECT_101);
    cv::Scalar gradient_mean;
    cv::Scalar gradient_deviation;
    cv::meanStdDev(laplacian, gradient_mean, gradient_deviation);
    const double sharpness = gradient_deviation[0] * gradient_deviation[0];

    auto flags = Json::array();
    if (mean[0] < config.dark_threshold)
    {
        flags.push_back("too_dark");
    }
    if (mean[0] > config.bright_threshold)
    {
        flags.push_back("too_bright");
    }
    if (deviation[0] < config.contrast_threshold)
    {
        flags.push_back("low_contrast");
    }
    if (sharpness < config.sharpness_threshold)
    {
        flags.push_back("low_sharpness");
    }

    return {
        {"status", "ok"},
        {"reason", ""},
        {"analysis_width", gray.cols},
        {"analysis_height", gray.rows},
        {"mean_brightness", mean[0]},
        {"contrast_stddev", deviation[0]},
        {"laplacian_variance", sharpness},
        {"flags", flags},
    };
}
} // namespace

ImageQualityConfig declareImageQualityConfig(rclcpp::Node& node)
{
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    ImageQualityConfig config;
    config.enabled = node.declare_parameter<bool>("image_quality_enabled", false, startup_only);
    config.max_side = node.declare_parameter<int>("image_quality_max_side", 320, startup_only);
    config.max_pixels =
        node.declare_parameter<int64_t>("image_quality_max_pixels", 8294400, startup_only);
    config.dark_threshold =
        node.declare_parameter<double>("image_dark_threshold", 30.0, startup_only);
    config.bright_threshold =
        node.declare_parameter<double>("image_bright_threshold", 225.0, startup_only);
    config.contrast_threshold =
        node.declare_parameter<double>("image_contrast_threshold", 10.0, startup_only);
    config.sharpness_threshold =
        node.declare_parameter<double>("image_sharpness_threshold", 50.0, startup_only);
    return config;
}

ImageQualityAnalyzer::ImageQualityAnalyzer(const ImageQualityConfig& config)
    : config_(config)
{
    const bool valid_thresholds =
        std::isfinite(config.dark_threshold) && std::isfinite(config.bright_threshold) &&
        std::isfinite(config.contrast_threshold) && std::isfinite(config.sharpness_threshold) &&
        config.dark_threshold >= 0.0 && config.dark_threshold < config.bright_threshold &&
        config.bright_threshold <= 255.0 && config.contrast_threshold >= 0.0 &&
        config.contrast_threshold <= 255.0 && config.sharpness_threshold >= 0.0;
    if (config.max_side < 32 || config.max_side > 1024 || config.max_pixels < 1 ||
        config.max_pixels > 67108864 || !valid_thresholds)
    {
        throw std::invalid_argument("Invalid image quality size limits or thresholds");
    }
}

Json ImageQualityAnalyzer::analyze(const sensor_msgs::msg::Image::ConstSharedPtr& message) const
{
    if (!config_.enabled)
    {
        return unavailable("disabled", "Enable image_quality_enabled at startup");
    }
    if (message->width == 0 || message->height == 0)
    {
        return unavailable("invalid_image", "Image width and height must be positive");
    }
    const uint64_t pixels = static_cast<uint64_t>(message->width) * message->height;
    if (pixels > static_cast<uint64_t>(config_.max_pixels))
    {
        return unavailable("size_limit", "Image exceeds image_quality_max_pixels");
    }
    const auto conversion = grayscaleConversion(message->encoding);
    if (!conversion)
    {
        return unavailable("unsupported_encoding", "Expected mono8, bgr8, rgb8, bgra8 or rgba8");
    }

    try
    {
        const uint64_t channels =
            static_cast<uint64_t>(sensor_msgs::image_encodings::numChannels(message->encoding));
        const uint64_t row_bytes = static_cast<uint64_t>(message->width) * channels;
        const uint64_t expected_bytes = static_cast<uint64_t>(message->step) * message->height;
        // 使用 64 位算术检查步长和缓冲区长度；允许行尾 padding，但不能越界读取。
        if (message->step < row_bytes || expected_bytes != message->data.size())
        {
            return unavailable("invalid_image", "Image step or buffer length is inconsistent");
        }

        // 原编码视图尽可能共享 ROS 像素；owner 保证整个分析期间底层消息仍然存活。
        const auto owner = cv_bridge::toCvShare(message, message->encoding);
        const auto gray = boundedGrayscale(owner->image, *conversion, config_.max_side);
        return measure(gray, config_);
    }
    catch (const cv_bridge::Exception& error)
    {
        return unavailable("invalid_image", error.what());
    }
    catch (const cv::Exception& error)
    {
        return unavailable("processing_error", error.what());
    }
}
} // namespace ros_gateserver
