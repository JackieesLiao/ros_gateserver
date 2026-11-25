#!/usr/bin/env python3
"""Exercise real DDS QoS mismatches and compare ROS/protobuf diagnostics."""

import os
from pathlib import Path
import tempfile

os.environ.setdefault('ROS_DOMAIN_ID', '135')

from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from protobuf_test import StatusReader
from protobuf_test import free_port
from tf_test import TestProcesses


def scan_diagnostics(state):
    return state['sensors']['scan']['diagnostics']


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    with tempfile.TemporaryDirectory(prefix='gateway_qos_test_') as temporary:
        with tempfile.TemporaryFile(mode='w+') as logs:
            try:
                with StatusReader(domain) as reader:
                    with TestProcesses(domain, logs, Path(temporary)) as case:
                        port = free_port()
                        gateway = case.start([
                            'ros2', 'run', 'ros_gateserver', 'gateway', '--ros-args',
                            '-p', f'ws_port:={port}', '-p', 'ws_format:=protobuf',
                            '-p', 'scan_topic:=/diagnostic_scan',
                            '-p', 'scan_reliability:=reliable',
                            '-p', 'stale_after_sec:=0.4', '-p', 'publish_period_ms:=50',
                        ])
                        reader.connect(case, gateway, port)
                        parameters = AsyncParameterClient(reader.node, 'gateway')
                        assert parameters.wait_for_services(timeout_sec=5)
                        response = reader.wait(parameters.set_parameters([
                            Parameter('scan_reliability', value='best_effort'),
                        ]))
                        assert not response.results[0].successful
                        state = reader.read(
                            lambda state: scan_diagnostics(state)['state'] == 'no_publisher'
                        )
                        assert state['sensors']['scan']['received'] == 0
                        wrong_type = reader.node.create_publisher(String, '/diagnostic_scan', 5)
                        reader.read(
                            lambda state: scan_diagnostics(state)['state'] == 'type_mismatch'
                        )
                        reader.node.destroy_publisher(wrong_type)

                        best_effort = reader.node.create_publisher(
                            LaserScan, '/diagnostic_scan', qos_profile_sensor_data
                        )
                        state = reader.read(
                            lambda state: scan_diagnostics(state)['state'] == 'qos_incompatible'
                        )
                        assert state['sensors']['scan']['received'] == 0
                        assert scan_diagnostics(state)['reasons']
                        if scan_diagnostics(state)['event_supported']:
                            reader.read(
                                lambda state: scan_diagnostics(state)['incompatible_events'] > 0
                            )
                        print(
                            'PASS: absent publisher, wrong type and actual DDS mismatch',
                            flush=True,
                        )

                        reliable = reader.node.create_publisher(
                            LaserScan, '/diagnostic_scan',
                            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
                        )
                        scan = LaserScan()
                        scan.header.frame_id = 'laser_link'
                        scan.range_min = 0.1
                        scan.range_max = 10.0
                        scan.ranges = [1.0]

                        def publish_scan():
                            scan.header.stamp = reader.node.get_clock().now().to_msg()
                            reliable.publish(scan)

                        timer = reader.node.create_timer(0.05, publish_scan)

                        def healthy(state):
                            diagnostics = scan_diagnostics(state)
                            return (
                                diagnostics['state'] == 'receiving'
                                and diagnostics['compatible_publishers'] == 1
                                and diagnostics['incompatible_publishers'] == 1
                                and state['sensors']['scan']['received'] > 5
                                and (diagnostics['receive_hz'] or 0) > 1
                            )

                        state = reader.read(healthy)
                        received = state['sensors']['scan']['received']
                        events = scan_diagnostics(state)['incompatible_events']
                        reader.set_enabled(False)
                        state = reader.read(lambda state: not state['enabled'])
                        assert state['sensors']['scan']['data'] is None
                        assert scan_diagnostics(state)['compatible_publishers'] == 1
                        reader.node.destroy_timer(timer)
                        reader.read(
                            lambda state: scan_diagnostics(state)['state'] == 'stale'
                            and scan_diagnostics(state)['receive_hz'] == 0.0
                        )

                        reader.node.destroy_publisher(best_effort)
                        timer = reader.node.create_timer(0.05, publish_scan)
                        state = reader.read(
                            lambda state: scan_diagnostics(state)['state'] == 'receiving'
                            and scan_diagnostics(state)['incompatible_publishers'] == 0
                            and state['sensors']['scan']['received'] > received
                        )
                        assert scan_diagnostics(state)['incompatible_events'] >= events
                        reader.node.destroy_timer(timer)
                        reader.node.destroy_publisher(reliable)
                        reader.read(
                            lambda state: scan_diagnostics(state)['state'] == 'no_publisher'
                        )
                        print(
                            'PASS: mixed endpoints, rate/stale/recovery, retained diagnostics, '
                            'ROS/protobuf equality', flush=True,
                        )
                        case.run([
                            'ros2', 'run', 'ros_gateserver', 'gateway', '--ros-args',
                            '-p', 'scan_reliability:=invalid',
                        ], expected_returncode=1)
            except Exception:
                logs.seek(0)
                print(logs.read())
                raise


if __name__ == '__main__':
    main()
