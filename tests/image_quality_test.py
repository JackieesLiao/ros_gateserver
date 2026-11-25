#!/usr/bin/env python3
"""Verify image quality metrics, malformed inputs, protobuf and isolated bag replay."""

import math
import os
from pathlib import Path
import tempfile

os.environ.setdefault('ROS_DOMAIN_ID', '138')

from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from ros_gateserver_protocol.codec import decode_status
from ros_gateserver_protocol.gateway_status_pb2 import GatewayStatus
from sensor_msgs.msg import Image

from protobuf_test import StatusReader
from protobuf_test import free_port
from protobuf_test import publish_until
from rosbag_test import check_replayed_status
from rosbag_test import read_recorded_messages
from smoke_test import all_sensors_fresh
from smoke_test import stop
from tf_test import TestProcesses


def image_data(state):
    return state['sensors']['image']['data'] or {}


def image_quality(state):
    return image_data(state).get('quality', {})


def make_image(width, height, encoding, pixels, step=None):
    channels = {'mono8': 1, 'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono16': 2}
    message = Image()
    message.width = width
    message.height = height
    message.encoding = encoding
    message.step = width * channels.get(encoding, 1) if step is None else step
    message.data = bytes(pixels)
    return message


def send_image(reader, publisher, name, message):
    message.header.frame_id = name
    message.header.stamp = reader.node.get_clock().now().to_msg()
    state = publish_until(
        reader, publisher, message,
        lambda state: image_data(state).get('frame_id') == name,
    )
    data = image_data(state)
    assert data['width'] == message.width and data['height'] == message.height
    assert data['encoding'] == message.encoding and data['bytes'] == len(message.data)
    assert state['sensors']['image']['fresh']
    return state


def assert_uniform(state, brightness):
    quality = image_quality(state)
    assert quality['status'] == 'ok', quality
    assert math.isclose(quality['mean_brightness'], brightness, abs_tol=0.01), quality
    assert math.isclose(quality['contrast_stddev'], 0.0, abs_tol=1e-5), quality
    assert math.isclose(quality['laplacian_variance'], 0.0, abs_tol=1e-5), quality
    assert 'low_contrast' in quality['flags'] and 'low_sharpness' in quality['flags']


def assert_unavailable(state, status):
    quality = image_quality(state)
    assert quality['status'] == status, quality
    assert quality['reason'] and quality['flags'] == []
    assert quality['analysis_width'] == quality['analysis_height'] == 0
    for field in ('mean_brightness', 'contrast_stddev', 'laplacian_variance'):
        assert quality[field] is None, quality


def check_wire_compatibility(payload):
    original = decode_status(payload)
    message = GatewayStatus()
    message.ParseFromString(payload)
    message.image.image_data.ClearField('quality')
    legacy = decode_status(message.SerializeToString())
    expected = dict(original['sensors']['image']['data'])
    del expected['quality']
    assert legacy['sensors']['image']['data'] == expected
    message.image.image_data.quality.mean_brightness = math.nan
    try:
        decode_status(message.SerializeToString())
    except ValueError:
        return
    raise AssertionError('Non-finite image quality metric was accepted')


def check_live(domain, directory, logs):
    with StatusReader(domain) as reader:
        with TestProcesses(domain, logs, directory / 'controlled_logs') as case:
            port = free_port()
            gateway = case.start([
                'ros2', 'run', 'ros_gateserver', 'gateway', '--ros-args',
                '-p', f'ws_port:={port}', '-p', 'ws_format:=protobuf',
                '-p', 'image_quality_enabled:=true', '-p', 'image_quality_max_side:=64',
            ])
            reader.connect(case, gateway, port)
            publisher = reader.node.create_publisher(
                Image, '/camera/image_raw', qos_profile_sensor_data
            )
            black = make_image(16, 16, 'mono8', [0] * 256)
            state = send_image(reader, publisher, 'black', black)
            assert_uniform(state, 0)
            assert 'too_dark' in image_quality(state)['flags']
            assert 'too_bright' not in image_quality(state)['flags']
            white = make_image(16, 16, 'mono8', [255] * 256)
            state = send_image(reader, publisher, 'white', white)
            assert_uniform(state, 255)
            assert 'too_bright' in image_quality(state)['flags']

            checker = [(row + column) % 2 * 255 for row in range(16) for column in range(16)]
            state = send_image(reader, publisher, 'checker', make_image(16, 16, 'mono8', checker))
            quality = image_quality(state)
            assert quality['status'] == 'ok' and quality['flags'] == [], quality
            assert math.isclose(quality['mean_brightness'], 127.5)
            assert math.isclose(quality['contrast_stddev'], 127.5)
            assert math.isclose(quality['laplacian_variance'], 1040400.0)

            colors = {
                'rgb8': [255, 0, 0], 'bgr8': [0, 0, 255],
                'rgba8': [255, 0, 0, 7], 'bgra8': [0, 0, 255, 127],
            }
            for encoding, pixel in colors.items():
                state = send_image(
                    reader, publisher, encoding, make_image(8, 8, encoding, pixel * 64)
                )
                assert_uniform(state, 76)
            padded = make_image(4, 3, 'mono8', ([100] * 4 + [255] * 4) * 3, step=8)
            assert_uniform(send_image(reader, publisher, 'padded', padded), 100)
            assert_uniform(
                send_image(reader, publisher, 'single_pixel', make_image(1, 1, 'mono8', [77])), 77
            )
            large = make_image(640, 480, 'mono8', [100] * (640 * 480))
            state = send_image(reader, publisher, 'resized', large)
            assert_uniform(state, 100)
            assert image_quality(state)['analysis_width'] == 64
            assert image_quality(state)['analysis_height'] == 48
            print(
                'PASS: analytical metrics, 5 encodings, padding, tiny image and bounded resize',
                flush=True,
            )

            bad_images = (
                ('empty', make_image(0, 0, 'mono8', []), 'invalid_image'),
                ('step', make_image(8, 8, 'mono8', [0] * 56, step=7), 'invalid_image'),
                ('short', make_image(8, 8, 'mono8', [0] * 63), 'invalid_image'),
                ('long', make_image(8, 8, 'mono8', [0] * 65), 'invalid_image'),
                ('encoding', make_image(8, 8, 'mono16', [0] * 128), 'unsupported_encoding'),
                ('size', make_image(9000000, 1, 'mono8', []), 'size_limit'),
            )
            for name, message, status in bad_images:
                assert_unavailable(send_image(reader, publisher, name, message), status)
                assert gateway.poll() is None
            state = send_image(reader, publisher, 'recovered', black)
            assert_uniform(state, 0)
            check_wire_compatibility(reader.last_payload)

            parameters = AsyncParameterClient(reader.node, 'gateway')
            assert parameters.wait_for_services(timeout_sec=5)
            response = reader.wait(parameters.set_parameters([
                Parameter('image_quality_enabled', value=False),
                Parameter('image_quality_max_side', value=128),
            ]))
            assert all(not result.successful for result in response.results)
            reader.set_enabled(False)
            received = state['sensors']['image']['received']
            publisher.publish(white)
            reader.read(
                lambda state: not state['enabled']
                and state['sensors']['image']['received'] > received
            )
            reader.set_enabled(True)
            state = reader.read(lambda state: image_data(state).get('frame_id') == 'white')
            assert_uniform(state, 255)
            reader.node.destroy_publisher(publisher)
            print(
                'PASS: malformed inputs, recovery, read-only settings, forwarding '
                'and wire compatibility', flush=True,
            )

            for setting in (
                'image_quality_max_side:=31', 'image_quality_max_pixels:=0',
                'image_dark_threshold:=240.0', 'image_sharpness_threshold:=.nan',
            ):
                case.run([
                    'ros2', 'run', 'ros_gateserver', 'gateway', '--ros-args', '-p', setting,
                ], expected_returncode=1)


def check_calibration(domain, directory, logs):
    with StatusReader(domain) as reader:
        with TestProcesses(domain, logs, directory / 'calibration_logs') as case:
            port = free_port()
            gateway = case.start([
                'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py', 'mock:=false',
                f'ws_port:={port}', 'ws_format:=protobuf', 'image_quality_enabled:=true',
                'image_dark_threshold:=50.0', 'image_bright_threshold:=200.0',
                'image_contrast_threshold:=0.0', 'image_sharpness_threshold:=0.0',
            ])
            reader.connect(case, gateway, port)
            publisher = reader.node.create_publisher(
                Image, '/camera/image_raw', qos_profile_sensor_data
            )
            for value, flags in ((49, ['too_dark']), (50, []), (200, []), (201, ['too_bright'])):
                state = send_image(
                    reader, publisher, f'calibration_{value}',
                    make_image(8, 8, 'mono8', [value] * 64),
                )
                assert image_quality(state)['flags'] == flags, state
            reader.node.destroy_publisher(publisher)
    print(
        'PASS: launch configuration, calibrated thresholds and strict boundary comparisons',
        flush=True,
    )


def check_record_replay(domain, directory, logs):
    bag_path = directory / 'images'
    with StatusReader(domain) as reader:
        with TestProcesses(domain, logs, directory / 'disabled_logs') as case:
            port = free_port()
            live = case.start([
                'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py',
                f'ws_port:={port}', 'ws_format:=protobuf',
            ])
            reader.connect(case, live, port)
            state = reader.read(all_sensors_fresh)
            assert_unavailable(state, 'disabled')
            case.run([
                'ros2', 'run', 'ros_gateserver', 'record_bag',
                '--duration', '3', '--output', str(bag_path),
            ])
            stop(live)
    messages = read_recorded_messages(bag_path)
    with StatusReader(domain + 1) as reader:
        with TestProcesses(domain + 1, logs, directory / 'replay_logs') as case:
            port = free_port()
            replay = case.start([
                'ros2', 'launch', 'ros_gateserver', 'replay.launch.py', f'bag_path:={bag_path}',
                f'ws_port:={port}', 'ws_format:=protobuf', 'image_quality_enabled:=true',
                'image_quality_max_side:=64', 'use_sim_time:=true',
            ])
            reader.connect(case, replay, port)
            state = reader.read(all_sensors_fresh, timeout=15)
            check_replayed_status(state, messages)
            data = image_data(state)
            stamp = data['stamp']
            image = messages['/camera/image_raw'][(stamp['sec'], stamp['nanosec'])]
            pixels = image.data
            mean = sum(pixels) / len(pixels)
            variance = sum((value - mean) ** 2 for value in pixels) / len(pixels)
            quality = image_quality(state)
            assert quality['status'] == 'ok', quality
            assert (quality['analysis_width'], quality['analysis_height']) == (64, 48)
            assert math.isclose(quality['mean_brightness'], mean, abs_tol=1e-6)
            assert math.isclose(quality['contrast_stddev'], math.sqrt(variance), abs_tol=1e-6)
            assert image.data[1] == (image.data[0] + 1) % 256
    print(
        'PASS: default disabled, raw MCAP pixels, isolated sim-time replay '
        'and ROS/protobuf equality', flush=True,
    )


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    with tempfile.TemporaryDirectory(prefix='gateway_image_quality_') as temporary:
        with tempfile.TemporaryFile(mode='w+') as logs:
            try:
                directory = Path(temporary)
                check_live(domain, directory, logs)
                check_calibration(domain + 2, directory, logs)
                check_record_replay(domain, directory, logs)
            except Exception:
                logs.seek(0)
                print(logs.read())
                raise


if __name__ == '__main__':
    main()
