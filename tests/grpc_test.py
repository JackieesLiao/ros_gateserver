#!/usr/bin/env python3
"""Verify gRPC status streaming against the ROS and WebSocket outputs."""

import json
import os
from pathlib import Path
import socket
import tempfile

os.environ.setdefault('ROS_DOMAIN_ID', '142')

import grpc

from ros_gateserver_protocol.codec import decode_status_message
from ros_gateserver_protocol.gateway_status_pb2 import WatchStatusRequest
from ros_gateserver_protocol.gateway_status_pb2_grpc import GatewayStatusServiceStub

from protobuf_test import StatusReader
from protobuf_test import free_port
from smoke_test import all_sensors_fresh
from smoke_test import all_sensors_stale
from smoke_test import stop
from tf_test import TestProcesses


def watch(channel, timeout=30):
    return GatewayStatusServiceStub(channel).WatchStatus(
        WatchStatusRequest(), timeout=timeout, wait_for_ready=True,
    )


def match_websocket_and_ros(stream, reader, predicate):
    expected = reader.read(predicate)
    for message in stream:
        status = decode_status_message(message)
        sequence = status['sequence']
        if sequence > expected['sequence']:
            expected = reader.read(lambda state: state['sequence'] >= sequence and predicate(state))
        if sequence == expected['sequence']:
            assert status == expected, (status, expected)
            return status
    raise AssertionError('gRPC stream ended before a matching status arrived')


def check_live(domain, directory, logs, bag_path):
    ws_port = free_port()
    grpc_port = free_port()
    target = f'127.0.0.1:{grpc_port}'

    with TestProcesses(domain, logs, directory) as case, StatusReader(domain) as reader:
        gateway = case.start([
            'ros2', 'launch', 'ros_gateserver', 'mvp.launch.py',
            'demo_tf:=true', 'ws_format:=protobuf', f'ws_port:={ws_port}',
            'grpc_enabled:=true', f'grpc_port:={grpc_port}',
        ])
        reader.connect(case, gateway, ws_port)

        with grpc.insecure_channel(target) as channel, grpc.insecure_channel(target) as second:
            stream = watch(channel)
            fresh = match_websocket_and_ros(stream, reader, all_sensors_fresh)
            second_stream = watch(second)
            newer = match_websocket_and_ros(second_stream, reader, all_sensors_fresh)
            assert newer['sequence'] >= fresh['sequence']
            assert second_stream.cancel()
            assert second_stream.code() == grpc.StatusCode.CANCELLED

            reader.set_enabled(False)
            disabled = match_websocket_and_ros(stream, reader, lambda state: not state['enabled'])
            assert disabled['sequence'] > fresh['sequence']
            assert all(sensor['data'] is None for sensor in disabled['sensors'].values())
            reader.set_enabled(True)
            resumed = match_websocket_and_ros(stream, reader, lambda state: state['enabled'])
            assert resumed['sequence'] > disabled['sequence']

            output = case.run([
                'ros2', 'run', 'ros_gateserver', 'grpc_client', '--target', target, '--count', '3',
            ])
            sequences = [json.loads(line)['sequence'] for line in output.splitlines()]
            assert len(sequences) == 3 and sequences[0] < sequences[1] < sequences[2]

            case.run([
                'ros2', 'run', 'ros_gateserver', 'record_bag', '--output', str(bag_path),
                '--duration', '2.0', '--with-static-tf',
            ])

            stop(gateway)
            assert gateway.returncode == 0
            for _ in stream:
                pass
            assert stream.code() == grpc.StatusCode.OK

    print('PASS: ROS/WebSocket/gRPC equality, concurrent streams, cancellation, CLI, shutdown')


def check_replay(domain, directory, logs, bag_path):
    ws_port = free_port()
    grpc_port = free_port()
    with TestProcesses(domain, logs, directory) as case:
        replay = case.start([
            'ros2', 'launch', 'ros_gateserver', 'replay.launch.py',
            f'bag_path:={bag_path}', f'ws_port:={ws_port}',
            'grpc_enabled:=true', f'grpc_port:={grpc_port}',
        ])
        websocket = case.connect(replay, ws_port)
        with grpc.insecure_channel(f'127.0.0.1:{grpc_port}') as channel:
            stream = watch(channel, timeout=15)
            def transformed(state):
                return (
                    all_sensors_fresh(state)
                    and state['sensors']['scan']['data']['transform_status'] == 'ok'
                )

            for message in stream:
                fresh = decode_status_message(message)
                if transformed(fresh):
                    break
            else:
                raise AssertionError('Replay did not publish fresh gRPC status with TF')

            websocket.until(transformed)

            for message in stream:
                if all_sensors_stale(decode_status_message(message)):
                    break
            else:
                raise AssertionError('Replay did not publish stale gRPC status after playback')
    print('PASS: gRPC and default WebSocket JSON during isolated replay; stale after playback')


def check_occupied_port(domain, directory, logs):
    with socket.socket() as occupied, TestProcesses(domain, logs, directory) as case:
        occupied.bind(('127.0.0.1', 0))
        occupied.listen()
        port = occupied.getsockname()[1]
        output = case.run([
            'ros2', 'run', 'ros_gateserver', 'gateway', '--ros-args',
            '-p', 'grpc_enabled:=true', '-p', f'grpc_port:={port}',
        ], expected_returncode=1)
        assert 'Failed to start gRPC status server' in output, output
    print('PASS: occupied gRPC port fails startup')


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        bag_path = directory / 'grpc_replay'
        with tempfile.TemporaryFile(mode='w+t') as logs:
            try:
                check_live(domain, directory, logs, bag_path)
                check_replay(domain + 1, directory, logs, bag_path)
                check_occupied_port(domain, directory, logs)
            except Exception:
                logs.seek(0)
                print(logs.read())
                raise


if __name__ == '__main__':
    main()
