#!/usr/bin/env python3
"""Exercise concurrent sensor input, forwarding changes, and status snapshots."""

import json
import math
import os
from pathlib import Path
import queue
import socket
import tempfile
import threading
import time

os.environ.setdefault('ROS_DOMAIN_ID', '131')

import rclpy
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from smoke_test import all_sensors_fresh
from smoke_test import stop
from tf_test import TestProcesses


def validate_status(state, previous_counts):
    assert state['schema_version'] == 1
    for name, sensor in state['sensors'].items():
        count = sensor['received']
        assert count >= previous_counts.get(name, 0), (name, sensor, previous_counts)
        previous_counts[name] = count
        if count > 0:
            assert math.isfinite(sensor['age_sec']) and sensor['age_sec'] >= 0

        data = sensor['data']
        if not state['enabled'] or count == 0:
            assert data is None, state
            continue

        assert data is not None, state
        if name == 'image':
            assert data['width'] == 4 and data['height'] == 2 and data['bytes'] == 8
        elif name == 'scan':
            assert data['samples'] == data['valid_samples'] == 3
            assert data['nearest_m'] == 1.0
            if data['transform_status'] == 'ok':
                point = data['nearest_point']
                assert point['frame_id'] == 'base_link'
                assert math.isclose(point['x'], 0.2, abs_tol=1e-6), point
                assert math.isclose(point['y'], 1.0, abs_tol=1e-6), point
        elif name == 'odom':
            assert data['child_frame_id'] == 'base_link'
            assert data['linear_x'] == 0.25


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('gateway_concurrency_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    spin_errors = []
    ros_states = queue.SimpleQueue()

    image_publisher = node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
    scan_publisher = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    odom_publisher = node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    service = node.create_client(SetBool, '/gateway/set_enabled')

    def record_status(message):
        ros_states.put(json.loads(message.data))

    subscription = node.create_subscription(String, '/gateway/status', record_status, 100)

    def publish_sensors():
        stamp = node.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'camera_link'
        image.height = 2
        image.width = 4
        image.encoding = 'mono8'
        image.step = 4
        image.data = bytes(range(8))
        image_publisher.publish(image)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser_link'
        scan.angle_min = 0.0
        scan.angle_increment = math.pi / 2
        scan.angle_max = math.pi
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [2.0, 1.0, 3.0]
        scan_publisher.publish(scan)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.25
        odom_publisher.publish(odom)

    timer = node.create_timer(0.01, publish_sensors)

    def spin():
        try:
            executor.spin()
        except Exception as error:
            spin_errors.append(error)

    thread = threading.Thread(target=spin)
    with tempfile.TemporaryDirectory(prefix='gateway_concurrency_test_') as temporary:
        directory = Path(temporary)
        with tempfile.TemporaryFile(mode='w+') as logs:
            try:
                thread.start()
                with TestProcesses(domain, logs, directory) as case:
                    case.start_tf()
                    with socket.socket() as probe:
                        probe.bind(('127.0.0.1', 0))
                        port = probe.getsockname()[1]
                    gateway = case.start([
                        'ros2', 'run', 'ros_gateserver', 'gateway',
                        '--ros-args', '-p', 'publish_period_ms:=10',
                        '-p', f'ws_port:={port}',
                    ])
                    client = case.connect(gateway, port)
                    assert service.wait_for_service(timeout_sec=15), 'Service unavailable'

                    def ready(state):
                        scan_data = state['sensors']['scan']['data'] or {}
                        return (
                            all_sensors_fresh(state)
                            and scan_data.get('transform_status') == 'ok'
                        )

                    websocket_counts = {}
                    ros_counts = {}
                    observed_ros = 0

                    def validate_both(state):
                        nonlocal observed_ros
                        validate_status(state, websocket_counts)
                        while not ros_states.empty():
                            validate_status(ros_states.get_nowait(), ros_counts)
                            observed_ros += 1
                        assert not spin_errors, spin_errors

                    validate_both(client.until(ready, timeout=15))

                    def set_enabled(enabled):
                        request = SetBool.Request()
                        request.data = enabled
                        started = time.monotonic()
                        future = service.call_async(request)
                        while not future.done():
                            assert not spin_errors, spin_errors
                            assert time.monotonic() - started < 5, 'Service response timed out'
                            time.sleep(0.001)
                        assert future.result().success, future.result()

                        deadline = time.monotonic() + 5
                        while time.monotonic() < deadline:
                            state = client.receive()
                            validate_both(state)
                            if state['enabled'] == enabled:
                                return state
                        raise AssertionError(f'Forwarding state did not become {enabled}')

                    # 100 Hz 输入和 100 Hz 定时发布期间，连续切换控制并检查两路快照一致性。
                    for index in range(60):
                        set_enabled(index % 2 == 1)

                    disabled = set_enabled(False)
                    counts_at_disable = {
                        name: sensor['received']
                        for name, sensor in disabled['sensors'].items()
                    }

                    def disabled_input_advanced(state):
                        validate_both(state)
                        return not state['enabled'] and all(
                            sensor['received'] >= counts_at_disable[name] + 5
                            for name, sensor in state['sensors'].items()
                        )

                    # 关闭转发后仍要更新计数；重新开启时恢复最近数据，不能死锁或丢失开关。
                    client.until(disabled_input_advanced)
                    restored = set_enabled(True)
                    assert all_sensors_fresh(restored), restored
                    assert observed_ros >= 20, observed_ros
                    assert min(websocket_counts.values()) >= 20, websocket_counts

                    stop(gateway)
                    assert gateway.returncode == 0, gateway.returncode
                    print(
                        'PASS: 100 Hz sensors/status, 60 forwarding changes, '
                        'ROS/WebSocket consistency, monotonic counts, TF, clean shutdown',
                        flush=True,
                    )
            except Exception:
                logs.seek(0)
                print(logs.read())
                raise
            finally:
                executor.shutdown(timeout_sec=5)
                if thread.ident is not None:
                    thread.join(timeout=5)
                node.destroy_timer(timer)
                node.destroy_subscription(subscription)
                node.destroy_node()
                context.shutdown()
                assert not thread.is_alive(), 'Test executor did not stop'
                assert not spin_errors, spin_errors


if __name__ == '__main__':
    main()
