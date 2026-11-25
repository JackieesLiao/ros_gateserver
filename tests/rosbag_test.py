#!/usr/bin/env python3
"""Record real ROS messages, replay them, and compare the gateway output with the bag."""

import math
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '128')

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from smoke_test import WebSocketClient
from smoke_test import all_sensors_fresh
from smoke_test import all_sensors_stale
from smoke_test import stop


TOPIC_TYPES = {
    '/camera/image_raw': 'sensor_msgs/msg/Image',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/odom': 'nav_msgs/msg/Odometry',
}


def read_recorded_messages(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    messages = {topic: {} for topic in TOPIC_TYPES}
    try:
        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        assert topic_types == TOPIC_TYPES, f'Unexpected recorded topics: {topic_types}'
        message_types = {topic: get_message(name) for topic, name in TOPIC_TYPES.items()}

        while reader.has_next():
            topic, serialized, received_time = reader.read_next()
            assert received_time > 0
            message = deserialize_message(serialized, message_types[topic])
            stamp = message.header.stamp
            messages[topic][(stamp.sec, stamp.nanosec)] = message
    finally:
        reader.close()

    assert all(messages.values()), 'At least one sensor topic has no recorded messages'
    return messages


def recorded_message(messages, topic, data):
    stamp = data['stamp']
    key = (stamp['sec'], stamp['nanosec'])
    assert key in messages[topic], f'Replayed timestamp was not recorded: {topic}, {key}'
    return messages[topic][key]


def check_replayed_status(state, messages):
    # 用各路原始时间戳查回 MCAP 消息；三路最新值不要求来自同一时刻。
    image_data = state['sensors']['image']['data']
    image = recorded_message(messages, '/camera/image_raw', image_data)
    assert image_data['width'] == image.width == 64
    assert image_data['height'] == image.height == 48
    assert image_data['encoding'] == image.encoding == 'mono8'
    assert image_data['bytes'] == len(image.data) == 3072
    assert image_data['frame_id'] == image.header.frame_id

    scan_data = state['sensors']['scan']['data']
    scan = recorded_message(messages, '/scan', scan_data)
    valid_ranges = [
        value
        for value in scan.ranges
        if math.isfinite(value) and scan.range_min <= value <= scan.range_max
    ]
    assert scan_data['samples'] == len(scan.ranges) == 36
    assert scan_data['valid_samples'] == len(valid_ranges)
    assert math.isclose(scan_data['nearest_m'], min(valid_ranges), abs_tol=1e-6)
    assert scan_data['frame_id'] == scan.header.frame_id

    odom_data = state['sensors']['odom']['data']
    odom = recorded_message(messages, '/odom', odom_data)
    for axis in ('x', 'y', 'z'):
        assert odom_data['position'][axis] == getattr(odom.pose.pose.position, axis)
    for axis in ('x', 'y', 'z', 'w'):
        assert odom_data['orientation'][axis] == getattr(odom.pose.pose.orientation, axis)
    assert odom_data['linear_x'] == odom.twist.twist.linear.x
    assert odom_data['angular_z'] == odom.twist.twist.angular.z
    assert odom_data['frame_id'] == odom.header.frame_id
    assert odom_data['child_frame_id'] == odom.child_frame_id


def connect_gateway(process, port):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        assert process.poll() is None, 'Replay launch exited during startup'
        try:
            return WebSocketClient(port)
        except OSError:
            time.sleep(0.1)
    raise AssertionError('Replay gateway did not accept WebSocket connections')


def run_command(command, logs, expected_returncode=0):
    result = subprocess.run(
        command,
        stdout=logs,
        stderr=subprocess.STDOUT,
        timeout=25,
    )
    assert result.returncode == expected_returncode, (
        f'Command returned {result.returncode}, expected {expected_returncode}: {command}'
    )


def main():
    mock = None
    recording = None
    replay = None
    client = None

    with tempfile.TemporaryDirectory(prefix='gateway_bag_test_') as directory:
        root = Path(directory)
        bag_path = root / 'demo'
        record_command = ['ros2', 'run', 'ros_gateserver', 'record_bag']

        with tempfile.TemporaryFile(mode='w+') as logs:
            try:
                run_command(record_command + ['--duration', '0'], logs, expected_returncode=2)
                assert not bag_path.exists()

                mock = subprocess.Popen(
                    ['ros2', 'run', 'ros_gateserver', 'mock_sensors'],
                    stdout=logs,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                run_command(
                    record_command + ['--output', str(bag_path), '--duration', '4'],
                    logs,
                )
                assert (bag_path / 'metadata.yaml').is_file()
                assert list(bag_path.glob('*.mcap'))
                messages = read_recorded_messages(bag_path)

                # 原始 Image 像素也必须落盘；网关 JSON 本身不包含这些像素。
                image = next(iter(messages['/camera/image_raw'].values()))
                assert len(image.data) == 3072
                assert image.data[1] == (image.data[0] + 1) % 256

                metadata = (bag_path / 'metadata.yaml').read_bytes()
                run_command(
                    record_command + ['--output', str(bag_path)],
                    logs,
                    expected_returncode=1,
                )
                assert (bag_path / 'metadata.yaml').read_bytes() == metadata

                run_command(
                    [
                        'ros2',
                        'launch',
                        'ros_gateserver',
                        'replay.launch.py',
                        f'bag_path:={bag_path}',
                        'rate:=nan',
                    ],
                    logs,
                    expected_returncode=1,
                )

                # 验证 Ctrl+C 也能让包装脚本收尾其独立进程组中的录制器。
                interrupted_path = root / 'interrupted'
                recording = subprocess.Popen(
                    record_command + ['--output', str(interrupted_path), '--duration', '60'],
                    stdout=logs,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                deadline = time.monotonic() + 15
                while not list(interrupted_path.glob('*.mcap')):
                    assert recording.poll() is None, 'Recorder exited before creating MCAP'
                    assert time.monotonic() < deadline, 'Recorder startup timed out'
                    time.sleep(0.1)
                time.sleep(1)
                stop(recording)
                assert recording.returncode == 0, 'Interrupted recording did not finalize cleanly'
                recording = None
                read_recorded_messages(interrupted_path)

                stop(mock)
                assert mock.returncode == 0
                mock = None

                run_command(
                    record_command + [
                        '--output', str(root / 'empty'),
                        '--duration', '0.3',
                        '--topics', '/demo/no_publisher',
                    ],
                    logs,
                    expected_returncode=1,
                )

                # 回放阶段完全停止模拟发布者，使输出只能来自刚才保存的消息。
                with socket.socket() as probe:
                    probe.bind(('127.0.0.1', 0))
                    port = probe.getsockname()[1]
                replay = subprocess.Popen(
                    [
                        'ros2',
                        'launch',
                        'ros_gateserver',
                        'replay.launch.py',
                        f'bag_path:={bag_path}',
                        f'ws_port:={port}',
                    ],
                    stdout=logs,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                client = connect_gateway(replay, port)
                state = client.until(all_sensors_fresh, timeout=15)
                check_replayed_status(state, messages)
                client.until(all_sensors_stale, timeout=15)
                assert replay.poll() is None, 'Gateway should remain running after playback'

                stop(replay)
                assert replay.returncode == 0, 'Replay launch did not exit cleanly'
                replay = None

                # 元数据存在但 MCAP 丢失：播放器应报错，Launch 应关闭已启动的网关。
                corrupt_path = root / 'corrupt'
                corrupt_path.mkdir()
                (corrupt_path / 'metadata.yaml').write_bytes(metadata)
                run_command(
                    [
                        'ros2',
                        'launch',
                        'ros_gateserver',
                        'replay.launch.py',
                        f'bag_path:={corrupt_path}',
                        f'ws_port:={port}',
                        'start_delay:=0',
                    ],
                    logs,
                    expected_returncode=1,
                )
                print(
                    'PASS: timed MCAP recording, raw pixels, three topic types, overwrite refusal, '
                    'Ctrl+C finalization, invalid input, empty recording, replayed timestamps and '
                    'values, stale detection, shutdown, playback failure cleanup'
                )
            except Exception:
                logs.seek(0)
                print(logs.read())
                raise
            finally:
                if client is not None:
                    client.close()
                stop(recording)
                stop(mock)
                stop(replay)


if __name__ == '__main__':
    main()
