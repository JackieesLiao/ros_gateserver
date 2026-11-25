#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <iterator>
#include <nlohmann/json.hpp>
#include <rclcpp/qos.hpp>
#include <string>
#include <vector>

namespace ros_gateserver
{
// 与传感器缓存共用 state_mutex_；本类型不自行加锁，也不访问 ROS 图。
struct InputDiagnostics
{
    using Clock = std::chrono::steady_clock;

    std::string topic;
    std::string message_type;
    std::string reliability;
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};
    bool event_supported{true};
    uint64_t incompatible_events{};
    std::string last_policy;
    uint64_t publishers{};
    uint64_t compatible{};
    uint64_t incompatible{};
    uint64_t uncertain{};
    uint64_t wrong_type{};
    std::vector<std::string> reasons;
    std::string graph_error;
    std::deque<Clock::time_point> arrivals;

    void recordArrival(Clock::time_point now)
    {
        arrivals.push_back(now);
        // 限制内存；高频输入只保留最近 256 次回调到达用于估计应用接收频率。
        while (arrivals.size() > 256)
        {
            arrivals.pop_front();
        }
    }

    nlohmann::json report(Clock::time_point now, bool has_data, bool fresh) const
    {
        std::string state = "receiving";
        if (!graph_error.empty())
        {
            state = "graph_error";
        }
        else if (publishers == 0)
        {
            state = "no_publisher";
        }
        else if (wrong_type == publishers)
        {
            state = "type_mismatch";
        }
        else if (compatible == 0 && uncertain == 0 && incompatible > 0)
        {
            state = "qos_incompatible";
        }
        else if (compatible == 0 && uncertain > 0)
        {
            state = "qos_uncertain";
        }
        else if (!has_data)
        {
            state = "waiting_for_data";
        }
        else if (!fresh)
        {
            state = "stale";
        }

        auto first = arrivals.begin();
        while (first != arrivals.end() && now - *first > std::chrono::seconds(2))
        {
            ++first;
        }
        const auto count = std::distance(first, arrivals.end());
        nlohmann::json rate = nullptr;
        if (count == 0)
        {
            rate = 0.0;
        }
        else if (count > 1)
        {
            const double seconds = std::chrono::duration<double>(now - *first).count();
            if (seconds > 0.0)
            {
                rate = static_cast<double>(count - 1) / seconds;
            }
        }

        return {
            {"state", state},
            {"topic", topic},
            {"reliability", reliability},
            {"publishers", publishers},
            {"compatible_publishers", compatible},
            {"incompatible_publishers", incompatible},
            {"uncertain_publishers", uncertain},
            {"type_mismatches", wrong_type},
            {"incompatible_events", incompatible_events},
            {"last_policy", last_policy},
            {"event_supported", event_supported},
            {"receive_hz", rate},
            {"reasons", reasons},
            {"graph_error", graph_error},
        };
    }
};
} // namespace ros_gateserver
