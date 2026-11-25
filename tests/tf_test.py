#!/usr/bin/env python3
"""Verify fixed radar TF, late recording, isolated replay, and missing-frame behavior."""

import json
import math
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '129')

import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

from rosbag_test import TOPIC_TYPES
from rosbag_test import check_replayed_status
from smoke_test import WebSocketClient
from smoke_test import all_sensors_fresh
from smoke_test import all_sensors_stale
from smoke_test import stop


SCAN_MESSAGE = (
    '{header: {frame_id: laser_link}, angle_min: 0.0, '
    'angle_max: 3.1415927, angle_increment: 1.5707963, '
    'range_min: 0.1, range_max: 10.0, ranges: [2.0, 1.0, 3.0]}'
)


class TestProcesses:
    """Own only this case's processes and sockets; never stop existing demo nodes."""

    def __init__(self, domain, logs, log_directory):
        self.environment = os.environ.copy()
        self.environment['ROS_DOMAIN_ID'] = str(domain)
        self.environment['ROS_LOG_DIR'] = str(log_directory)
        self.processes = []
        self.clients = []
        self.logs = logs

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        for client in self.clients:
            client.close()
        failures = []
        for process in reversed(self.processes):
            stop(process)
            if process.args[:2] == ['ros2', 'launch'] and process.returncode != 0:
                failures.append((process.args, process.returncode))
        if exception_type is None:
            assert not failures, f'Launch failed to shut down normally: {failures}'

    def start(self, command):
        process = subprocess.Popen(
            command,
            env=self.environment,
            stdout=self.logs,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.processes.append(process)
        return process

    def run(self, command, expected_returncode=0):
        process = subprocess.Popen(
            command,
            env=self.environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            output, _ = process.communicate(timeout=25)
        except subprocess.TimeoutExpired:
            # 先让录制包装脚本处理 SIGINT，收尾它自行创建的录制进程。
            stop(process)
            output, _ = process.communicate(timeout=3)
            self.logs.write(output)
            self.logs.flush()
            raise

        self.logs.write(output)
        self.logs.flush()
        assert process.returncode == expected_returncode, (
            f'Unexpected exit {process.returncode}: {command}\n{output}'
        )
        return output

    def gateway(self, *arguments):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        process = self.start([
            'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py',
            f'ws_port:={port}', *arguments,
        ])
        return self.connect(process, port)

    def replay(self, bag_path):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        process = self.start([
            'ros2', 'launch', 'ros_gateserver', 'replay.launch.py',
            f'bag_path:={bag_path}', f'ws_port:={port}',
        ])
        return self.connect(process, port)

    def connect(self, process, port):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            assert process.poll() is None, 'Gateway launch exited during startup'
            try:
                client = WebSocketClient(port)
                self.clients.append(client)
                return client
            except OSError:
                time.sleep(0.1)
        raise AssertionError('Gateway WebSocket did not start')

    def start_scan(self):
        return self.start([
            'ros2', 'topic', 'pub', '--rate', '5',
            '--qos-reliability', 'best_effort',
            '/scan', 'sensor_msgs/msg/LaserScan', SCAN_MESSAGE,
        ])

    def start_tf(self, child_frame='laser_link'):
        return self.start([
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0.2', '--frame-id', 'base_link', '--child-frame-id', child_frame,
        ])

    def ros_status(self):
        output = self.run([
            'ros2', 'topic', 'echo', '/gateway/status', 'std_msgs/msg/String',
            '--once', '--field', 'data',
        ])
        # --field data 输出 JSON 正文，结尾可能包含 ROS CLI 的 YAML 分隔符。
        state, _ = json.JSONDecoder().raw_decode(output.lstrip())
        return state


def transformed(state):
    scan = state['sensors']['scan']
    data = scan['data'] or {}
    return scan['fresh'] and data.get('transform_status') == 'ok'


def unavailable(state):
    scan = state['sensors']['scan']
    data = scan['data'] or {}
    return scan['fresh'] and data.get('transform_status') == 'unavailable'


def replay_ready(state):
    return all_sensors_fresh(state) and transformed(state)


def legacy_ready(state):
    return all_sensors_fresh(state) and unavailable(state)


def check_point(point, frame, expected):
    assert point['frame_id'] == frame, point
    for axis, value in zip(('x', 'y', 'z'), expected):
        assert math.isclose(point[axis], value, abs_tol=1e-6), point


def check_fixed_scan(state, target_frame, expected):
    assert transformed(state), state
    data = state['sensors']['scan']['data']
    assert data['samples'] == data['valid_samples'] == 3
    assert data['nearest_m'] == 1.0
    check_point(data['nearest_point_sensor'], 'laser_link', (0.0, 1.0, 0.0))
    check_point(data['nearest_point'], target_frame, expected)


def check_live(domain, logs, directory, yaw, target_frame, expected):
    with TestProcesses(domain, logs, directory) as case:
        case.start_scan()
        client = case.gateway(
            'mock:=false', 'demo_tf:=true',
            f'laser_yaw:={yaw}', f'target_frame:={target_frame}',
        )
        check_fixed_scan(client.until(transformed), target_frame, expected)
        check_fixed_scan(case.ros_status(), target_frame, expected)
    print(f'PASS: live yaw={yaw}, target={target_frame}, ROS and WebSocket', flush=True)


def check_late_tf(domain, logs, directory):
    with TestProcesses(domain, logs, directory) as case:
        case.start_scan()
        client = case.gateway('mock:=false')
        state = client.until(unavailable)
        assert state['sensors']['scan']['data']['nearest_point'] is None
        check_point(
            state['sensors']['scan']['data']['nearest_point_sensor'],
            'laser_link',
            (0.0, 1.0, 0.0),
        )
        # 先收到扫描、后收到 TF：后续扫描必须恢复，不能依赖固定 sleep。
        case.start_tf()
        check_fixed_scan(client.until(transformed), 'base_link', (0.2, 1.0, 0.0))
    print('PASS: missing TF preserves source data; late TF recovers', flush=True)


def read_tf_bag(bag_path):
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    messages = {topic: {} for topic in TOPIC_TYPES}
    transforms = []
    types = {topic: get_message(name) for topic, name in TOPIC_TYPES.items()}
    try:
        while reader.has_next():
            topic, serialized, _ = reader.read_next()
            if topic == '/tf_static':
                transforms.extend(deserialize_message(serialized, TFMessage).transforms)
            else:
                message = deserialize_message(serialized, types[topic])
                stamp = message.header.stamp
                messages[topic][(stamp.sec, stamp.nanosec)] = message
    finally:
        reader.close()
    assert all(messages.values()), 'Sensor messages missing from bag'
    return messages, transforms


def check_replayed_point(state, messages):
    check_replayed_status(state, messages)
    data = state['sensors']['scan']['data']
    stamp = data['stamp']
    scan = messages['/scan'][(stamp['sec'], stamp['nanosec'])]
    nearest_index = min(range(len(scan.ranges)), key=scan.ranges.__getitem__)
    angle = scan.angle_min + nearest_index * scan.angle_increment
    x = scan.ranges[nearest_index] * math.cos(angle)
    y = scan.ranges[nearest_index] * math.sin(angle)
    check_point(data['nearest_point_sensor'], 'laser_link', (x, y, 0.0))
    check_point(data['nearest_point'], 'base_link', (0.2 - y, x, 0.0))


def check_record_replay(domain, logs, directory):
    bag_path = directory / 'with_tf'
    legacy_path = directory / 'legacy'
    with TestProcesses(domain, logs, directory / 'live_logs') as case:
        client = case.gateway('demo_tf:=true', f'laser_yaw:={math.pi / 2}')
        client.until(transformed)
        # TF 已经发布后才启动录制器，验证 transient_local 的晚加入接收行为。
        case.run([
            'ros2', 'run', 'ros_gateserver', 'record_bag',
            '--with-static-tf', '--duration', '4', '--output', str(bag_path),
        ])
        case.run([
            'ros2', 'run', 'ros_gateserver', 'record_bag',
            '--duration', '4', '--output', str(legacy_path),
        ])

    messages, transforms = read_tf_bag(bag_path)
    radar_tf = [
        value for value in transforms
        if value.header.frame_id == 'base_link' and value.child_frame_id == 'laser_link'
    ]
    assert radar_tf, 'Expected radar installation transform was not recorded'
    assert math.isclose(radar_tf[0].transform.translation.x, 0.2, abs_tol=1e-6)
    assert math.isclose(radar_tf[0].transform.rotation.z, math.sqrt(0.5), abs_tol=1e-6)
    assert math.isclose(radar_tf[0].transform.rotation.w, math.sqrt(0.5), abs_tol=1e-6)

    # 全部实时进程已退出，回放另用一个 Domain，排除任何现场 TF 的帮助。
    with TestProcesses(domain + 1, logs, directory / 'replay_logs') as case:
        client = case.replay(bag_path)
        state = client.until(replay_ready, timeout=15)
        check_replayed_point(state, messages)
        check_replayed_point(case.ros_status(), messages)
        client.until(all_sensors_stale, timeout=15)
        assert all(process.poll() is None for process in case.processes)

    with TestProcesses(domain + 1, logs, directory / 'legacy_logs') as case:
        client = case.replay(legacy_path)
        state = client.until(legacy_ready, timeout=15)
        data = state['sensors']['scan']['data']
        assert data['nearest_point_sensor'] is not None
        assert data['nearest_point'] is None
    print('PASS: late TF recording, isolated replay values, EOF stale, legacy bag', flush=True)


def check_wrong_frames(domain, logs, directory):
    bag_path = directory / 'wrong_tf'
    with TestProcesses(domain, logs, directory / 'wrong_live_logs') as case:
        case.start_tf(child_frame='unrelated_sensor')
        client = case.gateway()
        client.until(all_sensors_fresh)
        result = case.run([
            'ros2', 'run', 'ros_gateserver', 'record_bag',
            '--with-static-tf', '--duration', '3', '--output', str(bag_path),
        ], expected_returncode=1)
        assert 'No recorded static transform: base_link -> laser_link' in result, result

    with TestProcesses(domain + 1, logs, directory / 'wrong_replay_logs') as case:
        client = case.replay(bag_path)
        state = client.until(unavailable, timeout=15)
        assert state['sensors']['scan']['data']['nearest_point'] is None

    with TestProcesses(domain, logs, directory / 'missing_logs') as case:
        client = case.gateway()
        client.until(all_sensors_fresh)
        result = case.run([
            'ros2', 'run', 'ros_gateserver', 'record_bag',
            '--with-static-tf', '--duration', '2',
            '--output', str(directory / 'missing_tf'),
        ], expected_returncode=1)
        assert 'No messages recorded on: /tf_static' in result, result
    print(
        'PASS: wrong TF rejected, wrong-frame replay unavailable, missing TF rejected',
        flush=True,
    )


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    if not 0 <= domain < 232:
        raise ValueError('Choose a ROS_DOMAIN_ID from 0 to 231; replay uses the next ID.')
    with tempfile.TemporaryDirectory(prefix='gateway_tf_test_') as temporary:
        directory = Path(temporary)
        with tempfile.TemporaryFile(mode='w+') as logs:
            try:
                check_live(domain, logs, directory, 0.0, 'base_link', (0.2, 1.0, 0.0))
                check_live(domain, logs, directory, math.pi / 2, 'base_link', (-0.8, 0.0, 0.0))
                check_live(domain, logs, directory, 0.0, 'laser_link', (0.0, 1.0, 0.0))
                check_late_tf(domain, logs, directory)
                check_record_replay(domain, logs, directory)
                check_wrong_frames(domain, logs, directory)
                with TestProcesses(domain, logs, directory) as case:
                    case.run([
                        'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py',
                        'demo_tf:=true', 'laser_yaw:=nan',
                    ], expected_returncode=1)
                print(
                    'PASS: invalid demo TF configuration rejected before node startup',
                    flush=True,
                )
            except Exception:
                logs.seek(0)
                print(logs.read())
                raise


if __name__ == '__main__':
    main()
