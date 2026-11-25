#!/usr/bin/env python3
"""End-to-end MVP check; run after sourcing ROS and install/setup.bash."""
import base64
import hashlib
import json
import os
import signal
import socket
import struct
import subprocess
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '127')

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool


class WebSocketClient:
    """Small standard-library client used only by this integration check."""

    def __init__(self, port):
        self.socket = socket.create_connection(('127.0.0.1', port), timeout=2)
        key = base64.b64encode(os.urandom(16)).decode()
        request = (
            f'GET / HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n'
            'Upgrade: websocket\r\nConnection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n'
        )
        self.socket.sendall(request.encode())

        header = bytearray()
        while not header.endswith(b'\r\n\r\n'):
            header.extend(self.exact(1))
            if len(header) > 16384:
                raise AssertionError('Oversized handshake')

        accept_key = (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()
        accept_hash = hashlib.sha1(accept_key).digest()
        expected = base64.b64encode(accept_hash)
        assert header.startswith(b'HTTP/1.1 101 '), header
        assert expected in header, 'Invalid WebSocket handshake'

    def exact(self, count):
        data = bytearray()
        while len(data) < count:
            chunk = self.socket.recv(count - len(data))
            if not chunk:
                raise ConnectionError('WebSocket closed')
            data.extend(chunk)

        return bytes(data)

    def receive(self):
        while True:
            first, second = self.exact(2)
            assert first & 0x80, 'Expected complete frame'
            assert not second & 0x80, 'Server frames must be unmasked'

            length = second & 0x7f
            if length == 126:
                length = struct.unpack('!H', self.exact(2))[0]
            elif length == 127:
                length = struct.unpack('!Q', self.exact(8))[0]
            assert length < 1024 * 1024, 'Oversized status'
            payload = self.exact(length)

            opcode = first & 0xf
            if opcode == 9:
                mask = os.urandom(4)
                masked = bytes(value ^ mask[i % 4] for i, value in enumerate(payload))
                self.socket.sendall(bytes([0x8a, 0x80 | length]) + mask + masked)
                continue

            assert opcode == 1, f'Unexpected frame opcode {opcode}'
            return json.loads(payload)

    def until(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.receive()
            if predicate(last):
                return last

        raise AssertionError(f'Status condition timed out; last={last}')

    def close(self):
        self.socket.close()


def stop(process):
    if process is None or process.poll() is not None:
        return

    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


def has_current_schema(state):
    return state['schema_version'] == 1


def all_sensors_fresh(state):
    return all(sensor['fresh'] for sensor in state['sensors'].values())


def all_sensors_stale(state):
    return all(not sensor['fresh'] for sensor in state['sensors'].values())


def main():
    # Choose an available test port, avoiding the normal demo port.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]

    launch = None
    mock = None
    clients = []
    rclpy.init()
    node = rclpy.create_node('gateway_smoke_test')
    ros_status = []

    def record_ros_status(message):
        ros_status.append(json.loads(message.data))

    subscription = node.create_subscription(
        String,
        '/gateway/status',
        record_ros_status,
        10,
    )
    service = node.create_client(SetBool, '/gateway/set_enabled')
    publisher = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

    with tempfile.TemporaryFile(mode='w+') as logs:
        try:
            launch = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'ros_gateserver',
                    'mvp.launch.py',
                    'mock:=false',
                    f'ws_port:={port}',
                ],
                stdout=logs,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            assert service.wait_for_service(timeout_sec=15), 'Gateway did not start'

            first = WebSocketClient(port)
            clients.append(first)
            initial = first.until(has_current_schema)
            assert initial['enabled']
            assert all(
                sensor['received'] == 0 and not sensor['fresh'] and sensor['data'] is None
                for sensor in initial['sensors'].values()
            )

            second = WebSocketClient(port)
            clients.append(second)

            mock = subprocess.Popen(
                ['ros2', 'run', 'ros_gateserver', 'mock_sensors'],
                stdout=logs,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            live = first.until(all_sensors_fresh)
            assert live['sensors']['image']['data']['width'] == 64
            assert live['sensors']['scan']['data']['valid_samples'] == 36
            assert live['sensors']['odom']['data']['frame_id'] == 'odom'
            second.until(all_sensors_fresh)

            for enabled in (False, True):
                request = SetBool.Request()
                request.data = enabled
                future = service.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().success, (
                    f'Enable service failed: enabled={enabled}, done={future.done()}, '
                    f'result={future.result() if future.done() else None}'
                )

                def forwarding_matches_request(state):
                    return state['enabled'] == enabled

                state = first.until(forwarding_matches_request)
                assert all((s['data'] is not None) == enabled for s in state['sensors'].values())

            assert ros_status and ros_status[-1]['schema_version'] == 1, 'ROS status missing'

            stop(mock)
            mock = None
            first.until(all_sensors_stale)

            # Invalid laser samples must produce JSON null rather than NaN/Infinity.
            scan = LaserScan()
            scan.header.frame_id = 'invalid_scan_test'
            scan.range_min = 0.1
            scan.range_max = 10.0
            scan.ranges = [float('nan'), float('inf'), -1.0, 11.0]
            deadline = time.monotonic() + 5

            while time.monotonic() < deadline:
                publisher.publish(scan)
                state = first.receive()
                if state['sensors']['scan']['data']['frame_id'] == 'invalid_scan_test':
                    data = state['sensors']['scan']['data']
                    assert data['valid_samples'] == 0 and data['nearest_m'] is None
                    break
            else:
                raise AssertionError('Invalid scan was not received')

            second.close()
            clients.remove(second)
            third = WebSocketClient(port)
            clients.append(third)
            assert third.receive()['schema_version'] == 1, 'Reconnect failed'

            stop(launch)
            assert launch.returncode == 0, f'Launch shutdown failed: {launch.returncode}'
            launch = None

            print(
                'PASS: Launch, 3 sensor topics, 2 WebSocket clients, ROS status, '
                'disable/enable, stale detection, invalid scan, reconnect, clean shutdown'
            )
        except Exception:
            logs.seek(0)
            print(logs.read())
            raise
        finally:
            for client in clients:
                client.close()
            stop(mock)
            stop(launch)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
