#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros_gateserver_interfaces/action/check_sensors.hpp>
#include <set>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <utility>

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using CheckSensors = ros_gateserver_interfaces::action::CheckSensors;
using GoalHandle = rclcpp_action::ServerGoalHandle<CheckSensors>;

class SensorCheckServer : public rclcpp::Node
{
public:
    SensorCheckServer()
        : Node("sensor_check_server")
    {
        startup_timeout_ = declare_parameter<double>("startup_timeout_sec", 5.0);
        status_timeout_ = declare_parameter<double>("status_timeout_sec", 1.0);
        if (!std::isfinite(startup_timeout_) || startup_timeout_ <= 0.0 ||
            !std::isfinite(status_timeout_) || status_timeout_ <= 0.0)
        {
            throw std::invalid_argument("Task timeouts must be finite and positive");
        }

        // Action、状态订阅和定时器都使用默认互斥组，任务上下文只会被串行访问。
        status_ = create_subscription<std_msgs::msg::String>(
            declare_parameter<std::string>("status_topic", "/gateway/status"),
            10,
            [this](std_msgs::msg::String::ConstSharedPtr message)
            {
                onStatus(message->data);
            });

        server_ = rclcpp_action::create_server<CheckSensors>(
            this,
            "/gateway/check_sensors",
            [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const CheckSensors::Goal> goal)
            {
                return handleGoal(*goal);
            },
            [this](const std::shared_ptr<GoalHandle> goal)
            {
                if (task_ && task_->handle == goal)
                {
                    return rclcpp_action::CancelResponse::ACCEPT;
                }
                return rclcpp_action::CancelResponse::REJECT;
            },
            [this](const std::shared_ptr<GoalHandle> goal)
            {
                handleAccepted(goal);
            });

        timer_ = create_wall_timer(std::chrono::milliseconds(100),
            [this]
            {
                tick();
            });
    }

private:
    static constexpr std::array<const char*, 3> sensor_names_{"image", "scan", "odom"};

    struct SensorView
    {
        uint64_t received{};
        bool fresh{};
    };

    struct Snapshot
    {
        std::array<SensorView, 3> sensors;
        bool enabled{};
        std::string scan_transform;
        Clock::time_point received_at;
        std::optional<uint64_t> clock_epoch;
    };

    struct Task
    {
        std::shared_ptr<GoalHandle> handle;
        Clock::time_point accepted_at;
        Clock::time_point last_feedback;
        std::optional<Clock::time_point> observing_since;
        std::array<uint64_t, 3> baseline{};
        std::array<uint64_t, 3> last_counts{};
        std::set<std::string> failures;
        bool counter_regressed{false};
        std::optional<uint64_t> clock_epoch;
        bool clock_changed{false};
    };

    enum class Outcome
    {
        Completed,
        Aborted,
        Canceled,
    };

    static double elapsed(Clock::time_point start, Clock::time_point end)
    {
        return std::chrono::duration<double>(end - start).count();
    }

    rclcpp_action::GoalResponse handleGoal(const CheckSensors::Goal& goal)
    {
        if (!std::isfinite(goal.duration_sec) || goal.duration_sec < 1.0 ||
            goal.duration_sec > 60.0 || goal_reserved_)
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 接受时立即占用名额，覆盖接受目标到 handleAccepted() 之间的窗口。
        goal_reserved_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle>& handle)
    {
        task_ = std::make_unique<Task>();
        task_->handle = handle;
        task_->accepted_at = Clock::now();
        task_->last_feedback = task_->accepted_at - std::chrono::seconds(1);
        publishFeedback();
    }

    Snapshot parseStatus(const std::string& text)
    {
        const auto data = Json::parse(text);
        if (data.at("schema_version") != 1)
        {
            throw std::invalid_argument("Unsupported gateway status schema");
        }

        Snapshot snapshot;
        snapshot.enabled = data.at("enabled").get<bool>();
        if (data.contains("clock"))
        {
            const auto& epoch = data.at("clock").at("epoch");
            if (!epoch.is_number_unsigned())
            {
                throw std::invalid_argument("Clock epoch must be an unsigned integer");
            }
            snapshot.clock_epoch = epoch.get<uint64_t>();
        }
        for (size_t index = 0; index < sensor_names_.size(); ++index)
        {
            const auto& sensor = data.at("sensors").at(sensor_names_[index]);
            const auto& count = sensor.at("received");
            if (!count.is_number_unsigned())
            {
                throw std::invalid_argument("Sensor count must be an unsigned integer");
            }
            snapshot.sensors[index].received = count.get<uint64_t>();
            snapshot.sensors[index].fresh = sensor.at("fresh").get<bool>();
        }

        const auto& scan = data.at("sensors").at("scan").at("data");
        if (!scan.is_null())
        {
            snapshot.scan_transform = scan.at("transform_status").get<std::string>();
        }
        snapshot.received_at = Clock::now();
        return snapshot;
    }

    void onStatus(const std::string& text)
    {
        try
        {
            auto snapshot = parseStatus(text);
            if (task_ && task_->observing_since)
            {
                if (task_->clock_epoch != snapshot.clock_epoch)
                {
                    task_->clock_changed = true;
                }
                for (size_t index = 0; index < sensor_names_.size(); ++index)
                {
                    const auto count = snapshot.sensors[index].received;
                    if (count < task_->last_counts[index])
                    {
                        task_->counter_regressed = true;
                    }
                    task_->last_counts[index] = count;
                }
            }
            latest_ = std::move(snapshot);
            if (task_ && task_->observing_since)
            {
                checkSensors();
            }
        }
        catch (const std::exception& error)
        {
            // 无效消息不刷新心跳，避免持续损坏的数据掩盖网关失联。
            RCLCPP_WARN_THROTTLE(get_logger(),
                *get_clock(),
                5000,
                "Ignoring invalid gateway status: %s",
                error.what());
        }
    }

    void startObservation(Clock::time_point now)
    {
        task_->observing_since = now;
        task_->clock_epoch = latest_->clock_epoch;
        for (size_t index = 0; index < sensor_names_.size(); ++index)
        {
            task_->baseline[index] = latest_->sensors[index].received;
            task_->last_counts[index] = task_->baseline[index];
        }
        checkSensors();
    }

    void checkSensors()
    {
        for (size_t index = 0; index < sensor_names_.size(); ++index)
        {
            const auto& sensor = latest_->sensors[index];
            const std::string name = sensor_names_[index];
            if (sensor.received == 0)
            {
                task_->failures.insert(name + "_missing");
            }
            else if (!sensor.fresh)
            {
                task_->failures.insert(name + "_stale");
            }
        }

        if (task_->handle->get_goal()->require_scan_tf)
        {
            if (!latest_->enabled)
            {
                // 转发关闭时详情被隐藏，明确标记不可观测，不判定雷达本身故障。
                task_->failures.insert("scan_tf_not_observable");
            }
            else if (latest_->scan_transform != "ok")
            {
                task_->failures.insert("scan_tf_unavailable");
            }
        }
    }

    void publishFeedback()
    {
        const auto now = Clock::now();
        if (elapsed(task_->last_feedback, now) < 0.5)
        {
            return;
        }

        auto feedback = std::make_shared<CheckSensors::Feedback>();
        feedback->phase = "WAITING_FOR_GATEWAY";
        if (task_->observing_since)
        {
            feedback->phase = "OBSERVING";
            const double progress =
                elapsed(*task_->observing_since, now) / task_->handle->get_goal()->duration_sec;
            feedback->progress = static_cast<float>(std::clamp(progress, 0.0, 1.0));
            for (size_t index = 0; index < sensor_names_.size(); ++index)
            {
                if (!latest_->sensors[index].fresh || latest_->sensors[index].received == 0)
                {
                    feedback->unhealthy_sensors.emplace_back(sensor_names_[index]);
                }
            }
        }
        feedback->failed_checks.assign(task_->failures.begin(), task_->failures.end());
        task_->handle->publish_feedback(feedback);
        task_->last_feedback = now;
    }

    void finish(Outcome outcome, const std::string& message)
    {
        // 所有终态都从这里收尾；移出活动任务后，不会再次发送结果或反馈。
        auto task = std::move(task_);
        goal_reserved_ = false;
        if (outcome != Outcome::Completed)
        {
            task->failures.insert(message);
        }

        auto result = std::make_shared<CheckSensors::Result>();
        result->passed = outcome == Outcome::Completed && task->failures.empty();
        result->message = message;
        if (task->observing_since)
        {
            result->observed_sec = elapsed(*task->observing_since, Clock::now());
        }
        result->failed_checks.assign(task->failures.begin(), task->failures.end());

        if (outcome == Outcome::Canceled)
        {
            task->handle->canceled(result);
        }
        else if (outcome == Outcome::Aborted)
        {
            task->handle->abort(result);
        }
        else
        {
            // 检查执行完成与设备检查通过分离，客户端必须读取 passed。
            task->handle->succeed(result);
        }
    }

    void tick()
    {
        if (!task_)
        {
            return;
        }
        // 取消回调只接受请求；等 GoalHandle 进入 canceling 后再执行收尾。
        if (task_->handle->is_canceling())
        {
            finish(Outcome::Canceled, "canceled");
            return;
        }

        const auto now = Clock::now();
        const bool has_status = latest_ && elapsed(latest_->received_at, now) <= status_timeout_;
        if (!task_->observing_since)
        {
            if (has_status && latest_->received_at >= task_->accepted_at)
            {
                startObservation(now);
            }
            else if (elapsed(task_->accepted_at, now) >= startup_timeout_)
            {
                finish(Outcome::Aborted, "gateway_startup_timeout");
                return;
            }
            publishFeedback();
            return;
        }

        if (task_->clock_changed)
        {
            // 观察窗口跨越了不同时间线，不能把回跳前后的数据合成一份有效报告。
            finish(Outcome::Aborted, "gateway_clock_changed");
            return;
        }
        if (task_->counter_regressed)
        {
            finish(Outcome::Aborted, "sensor_counter_regressed");
            return;
        }
        if (!has_status)
        {
            finish(Outcome::Aborted, "gateway_status_timeout");
            return;
        }

        checkSensors();
        if (elapsed(*task_->observing_since, now) >= task_->handle->get_goal()->duration_sec)
        {
            for (size_t index = 0; index < sensor_names_.size(); ++index)
            {
                if (task_->last_counts[index] == task_->baseline[index])
                {
                    task_->failures.insert(std::string(sensor_names_[index]) + "_not_updated");
                }
            }
            finish(Outcome::Completed, "observation_completed");
            return;
        }
        publishFeedback();
    }

    double startup_timeout_{};
    double status_timeout_{};
    bool goal_reserved_{false};
    std::optional<Snapshot> latest_;
    std::unique_ptr<Task> task_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
    rclcpp_action::Server<CheckSensors>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<SensorCheckServer>());
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sensor_check_server"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
