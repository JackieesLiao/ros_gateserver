#!/usr/bin/env python3
"""Verify timestamped dynamic TF, clock pause/jump and isolated simulated-time replay."""

import math
import os
from pathlib import Path
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '136')

import rosbag2_py
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from ros_gateserver_interfaces.action import CheckSensors
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

from protobuf_test import StatusReader
from protobuf_test import free_port
from rosbag_test import TOPIC_TYPES
from rosbag_test import check_replayed_status
from smoke_test import all_sensors_stale
from smoke_test import stop
from tf_test import TestProcesses


def stamp_ns(stamp):
    return stamp.sec * 10 ** 9 + stamp.nanosec


def scan_data(state):
    return state['sensors']['scan']['data'] or {}


def transform_ready(state):
    return state['sensors']['scan']['fresh'] and scan_data(state).get('transform_status') == 'ok'


def assert_point(state, x, y):
    point = scan_data(state)['nearest_point']
    assert point['frame_id'] == 'odom'
    assert math.isclose(point['x'], x, abs_tol=1e-5), point
    assert math.isclose(point['y'], y, abs_tol=1e-5), point


class TimeFixture:
    def __init__(self, reader):
        self.reader = reader
        self.seconds = 100
        self.clock = reader.node.create_publisher(Clock, '/clock', 1)
        self.scans = reader.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.dynamic = TransformBroadcaster(reader.node)
        self.static = StaticTransformBroadcaster(reader.node)
        self.clock_timer = None

    def publish_clock(self):
        message = Clock()
        message.clock.sec = self.seconds
        self.clock.publish(message)

    def start_clock(self):
        self.clock_timer = self.reader.node.create_timer(0.02, self.publish_clock)

    def mount(self):
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_link'
        transform.transform.translation.x = 0.2
        transform.transform.rotation.w = 1.0
        self.static.sendTransform(transform)

    def motion(self, seconds, x, yaw):
        transform = TransformStamped()
        transform.header.stamp.sec = seconds
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self.dynamic.sendTransform(transform)

    def scan(self, seconds, distance):
        message = LaserScan()
        message.header.stamp.sec = seconds
        message.header.frame_id = 'laser_link'
        message.range_min = 0.1
        message.range_max = 10.0
        message.ranges = [distance]
        self.scans.publish(message)

    def pump(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.reader.executor.spin_once(timeout_sec=0.01)


def check_clock_and_late_tf(domain, directory, logs):
    with StatusReader(domain) as reader:
        with TestProcesses(domain, logs, directory / 'clock_logs') as case:
            port = free_port()
            gateway = case.start([
                'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py',
                'mock:=false', 'use_sim_time:=true', 'self_check:=true',
                'target_frame:=odom', 'tf_wait_timeout_sec:=0.4',
                'ws_format:=protobuf', f'ws_port:={port}',
            ])
            reader.connect(case, gateway, port)
            initial = reader.read(lambda state: not state['clock']['initialized'])
            assert initial['stamp_ns'] == 0 and initial['clock']['source'] == 'ros'
            fixture = TimeFixture(reader)
            fixture.start_clock()
            fixture.mount()
            reader.read(lambda state: state['stamp_ns'] == 100 * 10 ** 9)
            deadline = time.monotonic() + 5
            while fixture.scans.get_subscription_count() == 0:
                assert time.monotonic() < deadline
                fixture.pump(0.05)

            fixture.scan(100, 1.0)
            fixture.pump(0.1)
            fixture.motion(100, 2.0, math.pi / 2)
            state = reader.read(transform_ready)
            assert_point(state, 2.0, 1.2)

            fixture.seconds = 101
            fixture.motion(101, 10.0, 0.0)
            fixture.pump(0.1)
            fixture.scan(100, 2.0)
            state = reader.read(
                lambda state: transform_ready(state) and scan_data(state)['nearest_m'] == 2.0
            )
            assert_point(state, 2.0, 2.2)
            fixture.scan(99, 3.0)
            state = reader.read(
                lambda state: scan_data(state).get('transform_status') == 'timeout'
            )
            assert scan_data(state)['nearest_point'] is None
            paused = reader.read(all_sensors_stale, timeout=8)
            assert paused['stamp_ns'] == 101 * 10 ** 9
            assert paused['sequence'] > state['sequence']
            assert scan_data(paused)['stamp']['sec'] == 99
            epoch = paused['clock']['epoch']
            received = paused['sensors']['scan']['received']

            fixture.seconds = 50
            reset = reader.read(lambda state: state['clock']['epoch'] > epoch)
            assert reset['stamp_ns'] == 50 * 10 ** 9
            assert reset['sensors']['scan']['data'] is None
            assert reset['sensors']['scan']['received'] == received
            fixture.mount()
            fixture.scan(50, 1.0)
            reader.read(lambda state: scan_data(state).get('transform_status') == 'timeout')
            fixture.motion(50, -3.0, 0.0)
            fixture.scan(50, 2.0)
            state = reader.read(transform_ready)
            assert_point(state, -0.8, 0.0)

            assert reader.action.wait_for_server(timeout_sec=5)
            goal = CheckSensors.Goal()
            goal.duration_sec = 5.0
            feedback = []

            def record_feedback(message):
                feedback.append(message.feedback.phase)

            handle = reader.wait(
                reader.action.send_goal_async(goal, feedback_callback=record_feedback)
            )
            assert handle.accepted
            deadline = time.monotonic() + 5
            while 'OBSERVING' not in feedback:
                assert time.monotonic() < deadline
                fixture.pump(0.05)
            fixture.seconds = 25
            result = reader.wait(handle.get_result_async())
            assert result.status == GoalStatus.STATUS_ABORTED, result
            assert 'gateway_clock_changed' in result.result.failed_checks
            reader.node.destroy_timer(fixture.clock_timer)
    print(
        'PASS: clock initialization, late/historical TF, timeout, paused clock with wall watchdog, '
        'backward reset, Action abort and protobuf equality', flush=True,
    )


def read_dynamic_bag(bag_path):
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    messages = {topic: {} for topic in TOPIC_TYPES}
    types = {topic: get_message(name) for topic, name in TOPIC_TYPES.items()}
    transforms = {}
    try:
        while reader.has_next():
            topic, serialized, _ = reader.read_next()
            if topic in types:
                message = deserialize_message(serialized, types[topic])
                stamp = message.header.stamp
                messages[topic][(stamp.sec, stamp.nanosec)] = message
            elif topic == '/tf':
                for transform in deserialize_message(serialized, TFMessage).transforms:
                    if (
                        transform.header.frame_id == 'odom'
                        and transform.child_frame_id == 'base_link'
                    ):
                        transforms[stamp_ns(transform.header.stamp)] = transform
    finally:
        reader.close()
    assert transforms and all(messages.values())
    return messages, transforms


def assert_recorded_transform(state, transforms):
    data = scan_data(state)
    time_ns = data['stamp']['sec'] * 10 ** 9 + data['stamp']['nanosec']
    transform = transforms[time_ns]
    rotation = transform.transform.rotation
    yaw = 2 * math.atan2(rotation.z, rotation.w)
    source = data['nearest_point_sensor']
    x = source['x'] + 0.2
    y = source['y']
    expected_x = transform.transform.translation.x + math.cos(yaw) * x - math.sin(yaw) * y
    expected_y = transform.transform.translation.y + math.sin(yaw) * x + math.cos(yaw) * y
    assert_point(state, expected_x, expected_y)


def check_dynamic_replay(domain, directory, logs):
    bag_path = directory / 'dynamic_demo'
    with TestProcesses(domain, logs, directory / 'record_logs') as case:
        live = case.start([
            'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py',
            'demo_tf:=true', 'demo_motion:=true', f'ws_port:={free_port()}',
        ])
        case.run([
            'ros2', 'run', 'ros_gateserver', 'record_bag',
            '--with-static-tf', '--with-dynamic-tf',
            '--duration', '4', '--output', str(bag_path),
        ])
        # 检查指定动态关系，不能仅因为 /tf 有其他关系就报告成功。
        case.run([
            'ros2', 'run', 'ros_gateserver', 'record_bag', '--topics', '/scan',
            '--with-dynamic-tf', '--dynamic-child-frame', 'wrong_frame',
            '--duration', '1', '--output', str(directory / 'wrong_dynamic'),
        ], expected_returncode=1)
        stop(live)
        case.run([
            'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py',
            'mock:=false', 'demo_motion:=true',
        ], expected_returncode=1)
        case.run([
            'ros2', 'launch', 'ros_gateserver', 'replay.launch.py',
            f'bag_path:={bag_path}', 'loop:=true', 'use_sim_time:=false',
        ], expected_returncode=1)
        case.run([
            'ros2', 'launch', 'ros_gateserver', 'replay.launch.py',
            f'bag_path:={bag_path}', 'clock_rate:=0',
        ], expected_returncode=1)
    messages, transforms = read_dynamic_bag(bag_path)

    with StatusReader(domain + 1) as reader:
        with TestProcesses(domain + 1, logs, directory / 'replay_logs') as case:
            port = free_port()
            replay = case.start([
                'ros2', 'launch', 'ros_gateserver', 'replay.launch.py',
                f'bag_path:={bag_path}', f'ws_port:={port}', 'ws_format:=protobuf',
                'target_frame:=odom', 'use_sim_time:=true', 'loop:=true',
                'tf_wait_timeout_sec:=0.5', 'rate:=1.5',
            ])
            reader.connect(case, replay, port)

            def ready(state):
                data = scan_data(state)
                if not transform_ready(state) or not all(
                    sensor['data'] is not None for sensor in state['sensors'].values()
                ):
                    return False
                time_ns = data['stamp']['sec'] * 10 ** 9 + data['stamp']['nanosec']
                return time_ns in transforms

            first = reader.read(ready, timeout=15)
            assert first['clock']['source'] == 'ros' and first['clock']['initialized']
            check_replayed_status(first, messages)
            assert_recorded_transform(first, transforms)
            stamp = first['stamp_ns']
            assert abs(stamp - scan_data(first)['stamp']['sec'] * 10 ** 9) < 2 * 10 ** 9
            assert abs(time.time_ns() - stamp) > 2 * 10 ** 9
            epoch = first['clock']['epoch']
            after_loop = reader.read(
                lambda state: state['clock']['epoch'] > epoch and ready(state), timeout=15
            )
            assert after_loop['sequence'] > first['sequence']
            check_replayed_status(after_loop, messages)
            assert_recorded_transform(after_loop, transforms)
    print(
        'PASS: dynamic TF recording validation, isolated sim-time replay and loop recovery',
        flush=True,
    )


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    with tempfile.TemporaryDirectory(prefix='gateway_time_tf_') as temporary:
        with tempfile.TemporaryFile(mode='w+') as logs:
            try:
                directory = Path(temporary)
                check_clock_and_late_tf(domain, directory, logs)
                check_dynamic_replay(domain, directory, logs)
            except Exception:
                logs.seek(0)
                print(logs.read())
                raise


if __name__ == '__main__':
    main()
