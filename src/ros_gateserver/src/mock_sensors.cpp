#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

class MockSensors : public rclcpp::Node
{
public:
    MockSensors()
        : Node("mock_sensors")
    {
        const auto qos = rclcpp::SensorDataQoS();
        image_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos);
        scan_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", qos);
        odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", qos);

        // 仅演示时显式开启；真实机器人应由自己的里程计或状态估计节点发布此关系。
        if (declare_parameter<bool>("publish_odom_tf", false))
        {
            odom_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        timer_ = create_wall_timer(std::chrono::milliseconds(100),
            [this]
            {
                publish();
            });
    }

private:
    void publish()
    {
        const auto stamp = now();
        const double phase = static_cast<double>(tick_) * 0.02;

        sensor_msgs::msg::Image image;
        image.header.stamp = stamp;
        image.header.frame_id = "camera_link";
        image.height = 48;
        image.width = 64;
        image.encoding = "mono8";
        image.step = image.width;
        image.data.resize(image.height * image.step);

        for (size_t i = 0; i < image.data.size(); ++i)
        {
            image.data[i] = (i + tick_) % 256;
        }

        image_->publish(image);

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = stamp;
        scan.header.frame_id = "laser_link";
        scan.angle_min = -3.14159265f;
        scan.angle_increment = 2.0f * 3.14159265f / 36.0f;
        scan.angle_max = scan.angle_min + 35 * scan.angle_increment;
        scan.scan_time = 0.1f;
        scan.range_min = 0.1f;
        scan.range_max = 10.0f;

        for (int i = 0; i < 36; ++i)
        {
            scan.ranges.push_back(2.0f + 0.5f * std::sin(phase + i * 0.2));
        }

        scan_->publish(scan);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = std::cos(phase);
        odom.pose.pose.position.y = std::sin(phase);
        const double yaw = phase + 3.14159265 / 2.0;
        odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
        odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
        odom.twist.twist.linear.x = 0.2;
        odom.twist.twist.angular.z = 0.2;
        odom_->publish(odom);
        if (odom_tf_)
        {
            geometry_msgs::msg::TransformStamped transform;
            transform.header = odom.header;
            transform.child_frame_id = odom.child_frame_id;
            transform.transform.translation.x = odom.pose.pose.position.x;
            transform.transform.translation.y = odom.pose.pose.position.y;
            transform.transform.translation.z = odom.pose.pose.position.z;
            transform.transform.rotation = odom.pose.pose.orientation;
            odom_tf_->sendTransform(transform);
        }
        ++tick_;
    }

    uint64_t tick_{0};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockSensors>());
    rclcpp::shutdown();
    return 0;
}
