#!/usr/bin/env python3
"""Compare binary status with ROS JSON and exercise Action and isolated bag replay."""

import json
import math
import os
from pathlib import Path
import socket
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '133')

import rclpy
from action_msgs.msg import GoalStatus
from google.protobuf.message import DecodeError
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from ros_gateserver_interfaces.action import CheckSensors
from ros_gateserver_protocol.client import ProtobufWebSocketClient
from ros_gateserver_protocol.codec import decode_status
from ros_gateserver_protocol.gateway_status_pb2 import GatewayStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from rosbag_test import check_replayed_status
from smoke_test import all_sensors_fresh
from smoke_test import all_sensors_stale
from smoke_test import stop
from tf_test import TestProcesses
from tf_test import read_tf_bag


def free_port():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


def ready(state):
    data = state['sensors']['scan']['data'] or {}
    return all_sensors_fresh(state) and data.get('transform_status') == 'ok'


def check_translation(state):
    data = state['sensors']['scan']['data']
    source = data['nearest_point_sensor']
    target = data['nearest_point']
    assert source['frame_id'] == 'laser_link' and target['frame_id'] == 'base_link'
    assert math.isclose(target['x'], source['x'] + 0.2, abs_tol=1e-6)
    assert math.isclose(target['y'], source['y'], abs_tol=1e-6)


class StatusReader:
    def __init__(self, domain):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = rclpy.create_node('protobuf_status_test', context=self.context)
        # 回放使用独立 Context，执行器必须绑定同一个 Context，不能使用默认全局执行器。
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.states = {}
        self.client = None
        self.last_payload = None
        self.subscription = self.node.create_subscription(
            String, '/gateway/status', self.record, 100
        )
        self.service = self.node.create_client(SetBool, '/gateway/set_enabled')
        self.action = ActionClient(self.node, CheckSensors, '/gateway/check_sensors')

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        self.executor.shutdown()
        self.action.destroy()
        self.node.destroy_node()
        self.context.shutdown()

    def record(self, message):
        state = json.loads(message.data)
        self.states[state.get('sequence', state['stamp_ns'])] = state
        if len(self.states) > 500:
            del self.states[next(iter(self.states))]

    def connect(self, case, process, port):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            assert process.poll() is None, 'Gateway launch failed'
            try:
                self.client = ProtobufWebSocketClient(port=port, timeout=2)
                case.clients.append(self.client)
                return
            except OSError:
                time.sleep(0.1)
        raise AssertionError('Binary WebSocket did not start')

    def read(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        state = None
        while time.monotonic() < deadline:
            self.last_payload = self.client.receive_payload()
            state = decode_status(self.last_payload)
            stamp = state.get('sequence', state['stamp_ns'])
            match_deadline = time.monotonic() + 0.3
            while stamp not in self.states and time.monotonic() < match_deadline:
                self.executor.spin_once(timeout_sec=0.02)
            # ROS 时间可暂停或回跳；优先按发布序号匹配，旧协议回退到时间戳。
            if stamp not in self.states:
                continue
            assert state == self.states[stamp], (state, self.states[stamp])
            if predicate(state):
                return state
        raise AssertionError(f'Matching ROS/protobuf status condition timed out; last={state}')

    def wait(self, future, timeout=6):
        deadline = time.monotonic() + timeout
        while not future.done():
            assert time.monotonic() < deadline, 'ROS request timed out'
            self.executor.spin_once(timeout_sec=0.02)
        assert future.exception() is None, future.exception()
        return future.result()

    def set_enabled(self, enabled):
        assert self.service.wait_for_service(timeout_sec=10)
        request = SetBool.Request()
        request.data = enabled
        assert self.wait(self.service.call_async(request)).success

    def check_action(self):
        assert self.action.wait_for_server(timeout_sec=10)
        goal = CheckSensors.Goal()
        goal.duration_sec = 1.0
        goal.require_scan_tf = True
        handle = self.wait(self.action.send_goal_async(goal))
        assert handle.accepted
        result = self.wait(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.passed, result


def rejects_payload(payload):
    try:
        decode_status(payload)
    except (ValueError, DecodeError):
        return
    raise AssertionError('Malformed or incompatible protobuf payload was accepted')


def check_decoder(payload):
    original = decode_status(payload)
    assert decode_status(payload + b'\xa0\x06\x01') == original
    legacy = GatewayStatus()
    legacy.ParseFromString(payload)
    legacy.ClearField('clock')
    legacy.ClearField('sequence')
    for name in ('image', 'scan', 'odom'):
        getattr(legacy, name).ClearField('diagnostics')
    decoded = decode_status(legacy.SerializeToString())
    assert 'clock' not in decoded and 'sequence' not in decoded
    assert all('diagnostics' not in sensor for sensor in decoded['sensors'].values())
    rejects_payload(b'')
    rejects_payload(b'\x80')
    message = GatewayStatus()
    message.ParseFromString(payload)
    message.schema_version = 99
    rejects_payload(message.SerializeToString())
    message.schema_version = 1
    message.scan.scan_data.nearest_m = math.nan
    rejects_payload(message.SerializeToString())
    message.ParseFromString(payload)
    message.image.scan_data.samples = 1
    rejects_payload(message.SerializeToString())


def publish_until(reader, publisher, message, predicate):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        publisher.publish(message)
        try:
            return reader.read(predicate, timeout=0.4)
        except AssertionError as error:
            if 'condition timed out' not in str(error):
                raise
    raise AssertionError('Explicit sensor sample was not processed')


def check_live(domain, directory, logs):
    bag_path = directory / 'proto_demo'
    with StatusReader(domain) as reader:
        with TestProcesses(domain, logs, directory / 'live_logs') as case:
            port = free_port()
            gateway = case.start([
                'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py',
                'mock:=false', 'demo_tf:=true', 'ws_format:=protobuf', 'self_check:=true',
                f'ws_port:={port}',
            ])
            reader.connect(case, gateway, port)

            def empty(state):
                return all(sensor['received'] == 0 for sensor in state['sensors'].values())

            initial = reader.read(empty)
            assert all(
                sensor['data'] is None and sensor['age_sec'] is None
                for sensor in initial['sensors'].values()
            )
            mock = case.start(['ros2', 'run', 'ros_gateserver', 'mock_sensors'])
            live = reader.read(ready)
            check_translation(live)
            assert isinstance(live['stamp_ns'], int) and live['stamp_ns'] > 2 ** 53
            assert b'\x00' in reader.last_payload, 'Binary zero bytes were not exercised'
            reader.check_action()

            case.run([
                'ros2', 'run', 'ros_gateserver', 'record_bag',
                '--with-static-tf', '--output', str(bag_path), '--duration', '3',
            ])
            reader.set_enabled(False)

            def disabled(state):
                return not state['enabled'] and all(
                    sensor['received'] > live['sensors'][name]['received']
                    for name, sensor in state['sensors'].items()
                )

            hidden = reader.read(disabled)
            assert all(sensor['data'] is None for sensor in hidden['sensors'].values())
            cli_output = case.run([
                'ros2', 'run', 'ros_gateserver', 'protobuf_client',
                '--port', str(port), '--count', '1',
            ])
            assert json.loads(cli_output)['enabled'] is False
            reader.set_enabled(True)
            reader.read(ready)
            stop(mock)
            assert mock.returncode == 0

            scan_publisher = reader.node.create_publisher(
                LaserScan, '/scan', qos_profile_sensor_data
            )
            scan = LaserScan()
            scan.header.frame_id = 'laser_link'
            scan.angle_increment = math.pi / 2
            scan.range_min = 0.1
            scan.range_max = 10.0
            scan.ranges = [math.nan, math.inf, -1.0, 11.0]

            def invalid_scan(state):
                data = state['sensors']['scan']['data'] or {}
                return data.get('samples') == 4 and data.get('valid_samples') == 0

            invalid = publish_until(reader, scan_publisher, scan, invalid_scan)
            data = invalid['sensors']['scan']['data']
            assert data['nearest_m'] is None and data['nearest_point'] is None

            odom_publisher = reader.node.create_publisher(
                Odometry, '/odom', qos_profile_sensor_data
            )
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = math.nan

            def nonfinite_odometry(state):
                data = state['sensors']['odom']['data'] or {}
                return data.get('linear_x', 0.0) is None

            invalid = publish_until(reader, odom_publisher, odom, nonfinite_odometry)
            assert invalid['sensors']['odom']['data']['position']['x'] == 0.0
            check_decoder(reader.last_payload)
            case.run([
                'ros2', 'run', 'ros_gateserver', 'gateway',
                '--ros-args', '-p', 'ws_format:=invalid',
            ], expected_returncode=1)
    print(
        'PASS: binary/ROS equality, Action, TF, client CLI, forwarding, null/zero/NaN',
        flush=True,
    )
    return bag_path


def check_replay(domain, directory, logs, bag_path):
    messages, transforms = read_tf_bag(bag_path)
    assert transforms
    with StatusReader(domain) as reader:
        with TestProcesses(domain, logs, directory / 'replay_logs') as case:
            port = free_port()
            gateway = case.start([
                'ros2', 'launch', 'ros_gateserver', 'replay.launch.py',
                f'bag_path:={bag_path}', 'ws_format:=protobuf', f'ws_port:={port}',
            ])
            reader.connect(case, gateway, port)
            state = reader.read(ready, timeout=15)
            check_replayed_status(state, messages)
            check_translation(state)
            reader.read(all_sensors_stale, timeout=15)
    print('PASS: isolated protobuf replay matches raw bag and ROS JSON; EOF stale', flush=True)


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    if not 0 <= domain < 232:
        raise ValueError('Use ROS_DOMAIN_ID 0..231; replay uses the next domain.')
    with tempfile.TemporaryDirectory(prefix='gateway_protobuf_test_') as temporary:
        directory = Path(temporary)
        with tempfile.TemporaryFile(mode='w+') as logs:
            try:
                bag_path = check_live(domain, directory, logs)
                check_replay(domain + 1, directory, logs, bag_path)
            except Exception:
                logs.seek(0)
                print(logs.read())
                raise


if __name__ == '__main__':
    main()
