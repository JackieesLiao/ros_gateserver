#include "ros_gateserver/grpc_status_server.hpp"
#include "ros_gateserver/image_quality.hpp"
#include "ros_gateserver/input_diagnostics.hpp"
#include "ros_gateserver/status_codec.hpp"
#include "ros_gateserver/websocket_server.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

class Gateway : public rclcpp::Node
{
public:
    Gateway()
        : Node("gateway")
    {
        const auto host = declare_parameter<std::string>("ws_host", "127.0.0.1");
        const auto port = declare_parameter<int>("ws_port", 8765);
        const auto period = declare_parameter<int>("publish_period_ms", 200);
        const auto ws_format = declare_parameter<std::string>("ws_format", "json");
        stale_seconds_ = declare_parameter<double>("stale_after_sec", 2.0);

        if (ws_format != "json" && ws_format != "protobuf")
        {
            throw std::invalid_argument("ws_format must be json or protobuf");
        }
        protobuf_output_ = ws_format == "protobuf";
        image_quality_ = std::make_unique<ros_gateserver::ImageQualityAnalyzer>(
            ros_gateserver::declareImageQualityConfig(*this));

        if (port < 1 || port > 65535 || period < 10 || !std::isfinite(stale_seconds_) ||
            stale_seconds_ <= 0)
        {
            throw std::invalid_argument("Invalid port, publish period, or stale timeout");
        }

        target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
        rcl_interfaces::msg::ParameterDescriptor startup_only;
        startup_only.read_only = true;
        const auto grpc_enabled = declare_parameter<bool>("grpc_enabled", false, startup_only);
        const auto grpc_host =
            declare_parameter<std::string>("grpc_host", "127.0.0.1", startup_only);
        const auto grpc_port = declare_parameter<int64_t>("grpc_port", 50051, startup_only);
        tf_wait_timeout_ = declare_parameter<double>("tf_wait_timeout_sec", 0.0, startup_only);
        if (!std::isfinite(tf_wait_timeout_) || tf_wait_timeout_ < 0.0 || tf_wait_timeout_ > 5.0)
        {
            throw std::invalid_argument("tf_wait_timeout_sec must be finite and within 0..5");
        }
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // TF 订阅保留在默认互斥组，与传感器组、控制组分别调度。
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
        trackClockChanges();

        sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 初始化三个传感器的缓存。
        for (const auto* name : {"image", "scan", "odom"})
        {
            sensors_[name] = Sensor{};
        }

        status_ = create_publisher<std_msgs::msg::String>("/gateway/status", 10);
        createSensorSubscriptions();
        if (tf_wait_timeout_ > 0.0)
        {
            // 与雷达回调共用互斥组；等待期间不占用执行线程，也不访问网络线程。
            tf_retry_timer_ = create_wall_timer(
                std::chrono::milliseconds(20),
                [this]
                {
                    retryPendingScans();
                },
                sensor_group_);
        }

        // 注册开关服务回调。
        enable_ = create_service<std_srvs::srv::SetBool>(
            "/gateway/set_enabled",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response)
            {
                handleSetEnabled(*request, *response);
            },
            rclcpp::ServicesQoS(),
            control_group_);

        if (grpc_enabled)
        {
            if (grpc_port < 1 || grpc_port > 65535)
            {
                throw std::invalid_argument("grpc_port must be within 1..65535");
            }
            grpc_server_ = std::make_unique<ros_gateserver::GrpcStatusServer>(grpc_host,
                static_cast<int>(grpc_port));
            RCLCPP_INFO(get_logger(),
                "gRPC status listening on %s port %d",
                grpc_host.c_str(),
                static_cast<int>(grpc_port));
        }

        // 首次发布填充网络快照；gRPC 客户端会等到第一份状态可用。
        publish();

        // 创建定时器，定期发布状态。
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period),
            [this]
            {
                logTransformOnce();
                publish();
            },
            control_group_);

        // 启动 WebSocket 服务及其线程。
        server_ = std::make_unique<ros_gateserver::WebSocketServer>(
            host,
            port,
            [this]
            {
                return readSnapshot();
            },
            std::chrono::milliseconds(period),
            protobuf_output_);

        RCLCPP_INFO(get_logger(),
            "WebSocket status: ws://%s:%d (%s)",
            host.c_str(),
            static_cast<int>(port),
            ws_format.c_str());
    }

private:
    struct Sensor
    {
        Json data = nullptr;
        uint64_t count = 0;
        Clock::time_point received{};
        ros_gateserver::InputDiagnostics diagnostics;
        uint64_t epoch{};
    };

    struct PendingScan
    {
        geometry_msgs::msg::PointStamped point;
        Json data;
        Clock::time_point queued_at;
        uint64_t epoch{};
    };

    void trackClockChanges()
    {
        rcl_jump_threshold_t threshold{};
        threshold.on_clock_change = true;
        threshold.min_backward.nanoseconds = -1;
        clock_jump_handler_ = get_clock()->create_jump_callback(
            nullptr,
            [this](const rcl_time_jump_t&)
            {
                // 时钟回调可能持有 Clock 内部锁：这里只更新原子代号，不调用 ROS 或加缓存锁。
                // tf2::Buffer 自行清理 TF；代号让旧摘要和处理中的旧扫描失效。
                clock_epoch_.fetch_add(1);
            },
            threshold);
    }

    template <typename Message, typename Callback>
    typename rclcpp::Subscription<Message>::SharedPtr
    createInput(const std::string& name, const std::string& default_topic, Callback callback)
    {
        auto& diagnostics = sensors_.at(name).diagnostics;
        diagnostics.topic = declare_parameter<std::string>(name + "_topic", default_topic);
        diagnostics.message_type = rosidl_generator_traits::name<Message>();
        rcl_interfaces::msg::ParameterDescriptor startup_only;
        startup_only.read_only = true;
        diagnostics.reliability =
            declare_parameter<std::string>(name + "_reliability", "best_effort", startup_only);
        if (diagnostics.reliability == "reliable")
        {
            diagnostics.qos.reliable();
        }
        else if (diagnostics.reliability != "best_effort")
        {
            throw std::invalid_argument(name + "_reliability must be best_effort or reliable");
        }

        rclcpp::SubscriptionOptions options;
        options.callback_group = sensor_group_;
        options.event_callbacks.incompatible_qos_callback =
            [this, name](rclcpp::QOSRequestedIncompatibleQoSInfo& event)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto& input = sensors_.at(name).diagnostics;
            input.incompatible_events = static_cast<uint64_t>(event.total_count);
            input.last_policy = rclcpp::qos_policy_name_from_kind(event.last_policy_kind);
        };

        auto on_message = [this, name, callback](typename Message::ConstSharedPtr message)
        {
            {
                // 在进入处理或 TF 排队前计时，避免成批提交摘要扭曲接收频率。
                std::lock_guard<std::mutex> lock(state_mutex_);
                sensors_.at(name).diagnostics.recordArrival(Clock::now());
            }
            callback(message);
        };

        try
        {
            auto subscription = create_subscription<Message>(diagnostics.topic,
                diagnostics.qos,
                on_message,
                options);
            // 使用 RMW 最终解析的配置，避免 system_default 造成伪不确定结果。
            diagnostics.qos = subscription->get_actual_qos();
            diagnostics.topic = subscription->get_topic_name();
            return subscription;
        }
        catch (const rclcpp::UnsupportedEventTypeException&)
        {
            // 部分 RMW 不支持该事件；保留订阅和图端点检查，并显式暴露能力降级。
            diagnostics.event_supported = false;
            options.event_callbacks.incompatible_qos_callback = nullptr;
            RCLCPP_WARN(get_logger(), "%s: incompatible QoS event is unavailable", name.c_str());
            auto subscription = create_subscription<Message>(diagnostics.topic,
                diagnostics.qos,
                on_message,
                options);
            diagnostics.qos = subscription->get_actual_qos();
            diagnostics.topic = subscription->get_topic_name();
            return subscription;
        }
    }

    void createSensorSubscriptions()
    {
        // 回调仅转交消息；处理函数在同一次回调内完成处理和缓存更新。
        image_ = createInput<sensor_msgs::msg::Image>("image",
            "/camera/image_raw",
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg)
            {
                processImage(msg, clock_epoch_.load());
            });

        scan_ = createInput<sensor_msgs::msg::LaserScan>("scan",
            "/scan",
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
            {
                processScan(*msg, clock_epoch_.load());
            });

        odom_ = createInput<nav_msgs::msg::Odometry>("odom",
            "/odom",
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                processOdometry(*msg, clock_epoch_.load());
            });
    }

    void inspectPublishers()
    {
        const auto now = Clock::now();
        if (now - last_graph_check_ < std::chrono::seconds(1))
        {
            return;
        }
        last_graph_check_ = now;

        for (const auto* name : {"image", "scan", "odom"})
        {
            ros_gateserver::InputDiagnostics input;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                input = sensors_.at(name).diagnostics;
            }
            input.publishers = 0;
            input.compatible = 0;
            input.incompatible = 0;
            input.uncertain = 0;
            input.wrong_type = 0;
            input.reasons.clear();
            input.graph_error.clear();
            try
            {
                // 图查询和 QoS 比较在锁外执行，不阻塞传感器缓存提交。
                for (const auto& endpoint : get_publishers_info_by_topic(input.topic))
                {
                    ++input.publishers;
                    if (endpoint.topic_type() != input.message_type)
                    {
                        ++input.wrong_type;
                        continue;
                    }
                    const auto check =
                        rclcpp::qos_check_compatible(endpoint.qos_profile(), input.qos);
                    if (check.compatibility == rclcpp::QoSCompatibility::Ok)
                    {
                        ++input.compatible;
                    }
                    else
                    {
                        if (check.compatibility == rclcpp::QoSCompatibility::Error)
                        {
                            ++input.incompatible;
                        }
                        else
                        {
                            ++input.uncertain;
                        }
                        if (input.reasons.size() < 8)
                        {
                            input.reasons.push_back(check.reason);
                        }
                    }
                }
            }
            catch (const std::exception& error)
            {
                input.graph_error = error.what();
            }

            std::lock_guard<std::mutex> lock(state_mutex_);
            auto& current = sensors_.at(name).diagnostics;
            // 只提交图快照，不能覆盖查询期间新增的到达记录和 QoS 事件。
            current.publishers = input.publishers;
            current.compatible = input.compatible;
            current.incompatible = input.incompatible;
            current.uncertain = input.uncertain;
            current.wrong_type = input.wrong_type;
            current.reasons = std::move(input.reasons);
            current.graph_error = std::move(input.graph_error);
        }
    }

    void processImage(const sensor_msgs::msg::Image::ConstSharedPtr& message, uint64_t epoch)
    {
        const auto& msg = *message;
        Json data = {
            {"width", msg.width},
            {"height", msg.height},
            {"encoding", msg.encoding},
            {"bytes", msg.data.size()},
            {"quality", image_quality_->analyze(message)},
        };

        update("image", msg, std::move(data), epoch);
    }

    void processScan(const sensor_msgs::msg::LaserScan& msg, uint64_t epoch)
    {
        double nearest = INFINITY;
        size_t nearest_index = 0;
        size_t valid = 0;

        for (size_t index = 0; index < msg.ranges.size(); ++index)
        {
            const auto range = msg.ranges[index];
            const bool valid_range =
                std::isfinite(range) && range >= msg.range_min && range <= msg.range_max;

            if (!valid_range)
            {
                continue;
            }

            ++valid;

            // 保留最近距离对应的下标，后面用它还原扫描角度。
            if (range < nearest)
            {
                nearest = range;
                nearest_index = index;
            }
        }

        Json data = {
            {"samples", msg.ranges.size()},
            {"valid_samples", valid},
            {"nearest_m", std::isfinite(nearest) ? Json(nearest) : Json(nullptr)},
            {"nearest_point_sensor", nullptr},
            {"nearest_point", nullptr},
            {"transform_status", "no_valid_point"},
        };

        geometry_msgs::msg::PointStamped source_point;
        source_point.header = msg.header;
        if (std::isfinite(nearest))
        {
            const double angle = static_cast<double>(msg.angle_min) +
                                 static_cast<double>(nearest_index) * msg.angle_increment;

            if (std::isfinite(angle))
            {
                // 保留原始坐标系和扫描时间，供 tf2 选择对应的变换。
                source_point.point.x = nearest * std::cos(angle);
                source_point.point.y = nearest * std::sin(angle);
                source_point.point.z = 0.0;

                data["nearest_point_sensor"] = {
                    {"frame_id", source_point.header.frame_id},
                    {"x", source_point.point.x},
                    {"y", source_point.point.y},
                    {"z", source_point.point.z},
                };

                appendTransformedPoint(source_point, data);
            }
            else
            {
                data["transform_status"] = "invalid_scan_geometry";
            }
        }

        if (tf_wait_timeout_ == 0.0)
        {
            update("scan", msg, std::move(data), epoch);
            return;
        }

        if (pending_scans_.size() == 32)
        {
            auto& oldest = pending_scans_.front();
            oldest.data["transform_status"] = "queue_overflow";
            update("scan", oldest.point, std::move(oldest.data), oldest.epoch);
            pending_scans_.pop_front();
        }
        pending_scans_.push_back({source_point, std::move(data), Clock::now(), epoch});
        retryPendingScans();
    }

    void retryPendingScans()
    {
        while (!pending_scans_.empty())
        {
            auto& pending = pending_scans_.front();
            if (pending.epoch != clock_epoch_.load())
            {
                pending_scans_.pop_front();
                continue;
            }
            if (pending.data.at("transform_status") == "unavailable")
            {
                const double waited =
                    std::chrono::duration<double>(Clock::now() - pending.queued_at).count();
                if (waited >= tf_wait_timeout_)
                {
                    pending.data["transform_status"] = "timeout";
                }
                else
                {
                    appendTransformedPoint(pending.point, pending.data, false);
                    if (pending.data.at("transform_status") == "unavailable")
                    {
                        return;
                    }
                }
            }
            // FIFO 提交，防止旧扫描的迟到 TF 覆盖较新摘要；只保存摘要和一个点。
            update("scan", pending.point, std::move(pending.data), pending.epoch);
            pending_scans_.pop_front();
        }
    }

    void processOdometry(const nav_msgs::msg::Odometry& msg, uint64_t epoch)
    {
        const auto& position = msg.pose.pose.position;
        const auto& orientation = msg.pose.pose.orientation;
        Json data = {
            {"child_frame_id", msg.child_frame_id},
            {"position",
                {
                    {"x", position.x},
                    {"y", position.y},
                    {"z", position.z},
                }},
            {"orientation",
                {
                    {"x", orientation.x},
                    {"y", orientation.y},
                    {"z", orientation.z},
                    {"w", orientation.w},
                }},
            {"linear_x", msg.twist.twist.linear.x},
            {"angular_z", msg.twist.twist.angular.z},
        };

        update("odom", msg, std::move(data), epoch);
    }

    void appendTransformedPoint(const geometry_msgs::msg::PointStamped& source_point,
        Json& data,
        bool warn = true)
    {
        try
        {
            // 零等待查询，避免阻塞当前 ROS 执行线程。
            const auto target_point =
                tf_buffer_->transform(source_point, target_frame_, tf2::durationFromSec(0.0));

            data["nearest_point"] = {
                {"frame_id", target_point.header.frame_id},
                {"x", target_point.point.x},
                {"y", target_point.point.y},
                {"z", target_point.point.z},
            };
            data["transform_status"] = "ok";
        }
        catch (const tf2::TransformException& error)
        {
            // 原始雷达统计仍然可用，单独标记坐标转换失败。
            data["transform_status"] = "unavailable";

            if (warn)
            {
                RCLCPP_WARN_THROTTLE(get_logger(),
                    *get_clock(),
                    5000,
                    "Cannot transform point from %s to %s: %s",
                    source_point.header.frame_id.c_str(),
                    target_frame_.c_str(),
                    error.what());
            }
        }
    }

    void logTransformOnce()
    {
        if (tf_ready_logged_)
        {
            return;
        }

        try
        {
            // 参数顺序：目标坐标系、源坐标系、查询时间、等待时长。
            // 零等待立即返回，让执行器有机会继续处理 TF 订阅回调。
            const auto transform = tf_buffer_->lookupTransform(target_frame_,
                "laser_link",
                tf2::TimePointZero,
                tf2::durationFromSec(0.0));
            const auto& translation = transform.transform.translation;

            RCLCPP_INFO(get_logger(),
                "TF %s <- laser_link: x=%.3f, y=%.3f, z=%.3f",
                target_frame_.c_str(),
                translation.x,
                translation.y,
                translation.z);

            tf_ready_logged_ = true;
        }
        catch (const tf2::TransformException& error)
        {
            RCLCPP_WARN_THROTTLE(get_logger(),
                *get_clock(),
                5000,
                "Waiting for TF: %s",
                error.what());
        }
    }

    template <typename Message>
    void update(const std::string& name, const Message& msg, Json data, uint64_t epoch)
    {
        if (get_clock()->ros_time_is_active() && now().nanoseconds() == 0)
        {
            return;
        }
        data["frame_id"] = msg.header.frame_id;
        data["stamp"] = {{"sec", msg.header.stamp.sec}, {"nanosec", msg.header.stamp.nanosec}};

        // 只在写入摘要时持锁；雷达遍历和 TF 查询均已在锁外完成。
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (epoch != clock_epoch_.load())
        {
            return;
        }
        auto& sensor = sensors_.at(name);
        sensor.epoch = epoch;
        sensor.data = std::move(data);
        ++sensor.count;
        sensor.received = Clock::now();
    }

    Json buildStatus()
    {
        uint64_t epoch{};
        int64_t stamp{};
        bool ros_time{};
        do
        {
            epoch = clock_epoch_.load();
            stamp = now().nanoseconds();
            ros_time = get_clock()->ros_time_is_active();
        } while (epoch != clock_epoch_.load());
        std::map<std::string, Sensor> sensors;
        bool enabled{};
        {
            // 缓存和开关一起复制，确保一份状态使用同一次读取的转发配置。
            std::lock_guard<std::mutex> lock(state_mutex_);
            sensors = sensors_;
            enabled = enabled_;
        }

        // 使用局部副本组装 JSON，不阻塞其他线程更新传感器摘要。
        Json result = {
            {"schema_version", 1},
            {"sequence", ++sequence_},
            {"enabled", enabled},
            {"stamp_ns", stamp},
            {"clock",
                {
                    {"source", ros_time ? "ros" : "system"},
                    {"initialized", !ros_time || stamp != 0},
                    {"epoch", epoch},
                }},
            {"sensors", Json::object()},
        };

        for (const auto& [name, sensor] : sensors)
        {
            const bool has_data = sensor.count > 0 && sensor.epoch == epoch;
            const double age =
                std::chrono::duration<double>(Clock::now() - sensor.received).count();
            const bool fresh = has_data && age <= stale_seconds_;
            Json sensor_status = {
                {"received", sensor.count},
                {"fresh", fresh},
                {"age_sec", has_data ? Json(age) : Json(nullptr)},
                {"data", enabled && has_data ? sensor.data : Json(nullptr)},
                {"diagnostics", sensor.diagnostics.report(Clock::now(), has_data, fresh)},
            };
            result["sensors"][name] = std::move(sensor_status);
        }

        return result;
    }

    void publish()
    {
        // 运行期间仅由控制组调用，服务与定时器的发布顺序由互斥组保证。
        inspectPublishers();
        const auto status = buildStatus();
        std_msgs::msg::String message;
        message.data = status.dump();

        // ROS 始终保留 JSON，自检节点不依赖 WebSocket 的传输格式。
        std::string payload;
        if (protobuf_output_ || grpc_server_)
        {
            auto protobuf = ros_gateserver::makeProtobufStatus(status);
            if (protobuf_output_)
            {
                payload = ros_gateserver::encodeProtobufStatus(protobuf);
            }
            if (grpc_server_)
            {
                grpc_server_->update(std::move(protobuf));
            }
        }
        if (!protobuf_output_)
        {
            payload = message.data;
        }

        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot_ = std::move(payload);
        }

        status_->publish(message);
    }

    void handleSetEnabled(const std_srvs::srv::SetBool::Request& request,
        std_srvs::srv::SetBool::Response& response)
    {
        const bool enabled = request.data;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            enabled_ = enabled;
        }

        // publish() 会再次读取状态，调用前必须释放 state_mutex_，避免重入死锁。
        publish();

        response.success = true;
        response.message =
            enabled ? "Telemetry forwarding enabled" : "Telemetry forwarding disabled";
        RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
    }

    std::string readSnapshot()
    {
        // 网络线程只在锁内复制字符串，不访问传感器缓存，也不持锁等待发送。
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return snapshot_;
    }

    std::string target_frame_;
    std::unique_ptr<ros_gateserver::ImageQualityAnalyzer> image_quality_;
    std::atomic<uint64_t> clock_epoch_{0};
    rclcpp::JumpHandler::SharedPtr clock_jump_handler_;
    double tf_wait_timeout_{};
    std::deque<PendingScan> pending_scans_;
    // Listener 依赖 Buffer；成员逆序析构，确保 Listener 先释放。
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    bool tf_ready_logged_{false};

    // 组对象必须覆盖订阅、服务和定时器的生命周期；组内串行，组间允许并行。
    rclcpp::CallbackGroup::SharedPtr sensor_group_;
    rclcpp::CallbackGroup::SharedPtr control_group_;

    // 传感器组写缓存，控制组读取缓存和修改开关，运行期访问均持有此锁。
    std::mutex state_mutex_;
    std::map<std::string, Sensor> sensors_;
    bool enabled_{true};
    // 此配置只在构造时写入，执行器运行后保持不变。
    double stale_seconds_{};
    bool protobuf_output_{false};
    Clock::time_point last_graph_check_{};
    uint64_t sequence_{};

    // 只有编码后的状态载荷跨 ROS 与网络线程共享，读写均持有此锁。
    std::mutex snapshot_mutex_;
    std::string snapshot_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_retry_timer_;
    // 服务器析构时停止并等待网络线程，早于快照和互斥锁的析构。
    std::unique_ptr<ros_gateserver::WebSocketServer> server_;
    std::unique_ptr<ros_gateserver::GrpcStatusServer> grpc_server_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto gateway = std::make_shared<Gateway>();
        // 三个执行线程允许不同组并行；回调组不绑定到某个固定线程。
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
        executor.add_node(gateway);
        executor.spin();
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("gateway"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
