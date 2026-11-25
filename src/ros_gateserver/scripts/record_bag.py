#!/usr/bin/env python3
"""Manually record a short sensor demo, then finalize the bag before returning."""

import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


DEFAULT_TOPICS = ['/camera/image_raw', '/scan', '/odom']


def positive_duration(value):
    duration = float(value)
    if not math.isfinite(duration) or duration <= 0:
        raise argparse.ArgumentTypeError('Duration must be finite and greater than zero.')
    return duration


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output',
        type=Path,
        help='New bag directory; existing paths are rejected.',
    )
    parser.add_argument(
        '--duration',
        type=positive_duration,
        default=10.0,
        help='Seconds, default: 10.',
    )
    parser.add_argument(
        '--topics',
        nargs='+',
        default=DEFAULT_TOPICS,
        help='Sensor topics to record.',
    )
    parser.add_argument(
        '--with-static-tf',
        action='store_true',
        help='Also record /tf_static and require the specified direct frame relationship.',
    )
    parser.add_argument('--tf-parent-frame', default='base_link')
    parser.add_argument('--tf-child-frame', default='laser_link')
    parser.add_argument(
        '--with-dynamic-tf', action='store_true',
        help='Also record /tf and require the specified direct dynamic relationship.',
    )
    parser.add_argument('--dynamic-parent-frame', default='odom')
    parser.add_argument('--dynamic-child-frame', default='base_link')
    arguments = parser.parse_args()
    if any(not topic.startswith('/') for topic in arguments.topics):
        parser.error('Use absolute topic names, for example /scan.')
    if arguments.with_static_tf:
        if not arguments.tf_parent_frame or not arguments.tf_child_frame:
            parser.error('TF frame names must not be empty.')
        if arguments.tf_parent_frame == arguments.tf_child_frame:
            parser.error('TF parent and child frames must differ.')
        arguments.topics = list(dict.fromkeys([*arguments.topics, '/tf_static']))
    if arguments.with_dynamic_tf:
        if not arguments.dynamic_parent_frame or not arguments.dynamic_child_frame:
            parser.error('Dynamic TF frame names must not be empty.')
        if arguments.dynamic_parent_frame == arguments.dynamic_child_frame:
            parser.error('Dynamic TF parent and child frames must differ.')
        arguments.topics = list(dict.fromkeys([*arguments.topics, '/tf']))
    return arguments


def wait_until_recording(process, output, stop_requested):
    deadline = time.monotonic() + 15.0
    while not stop_requested.is_set():
        if process.poll() is not None:
            raise RuntimeError(f'Recorder exited during startup: {process.returncode}')

        # MCAP 文件创建后再开始演示计时，减少 ros2 命令启动耗时的影响。
        # 此时 DDS 仍可能正在发现发布者，实际消息覆盖的时长会略短。
        if any(output.glob('*.mcap')):
            return True

        if time.monotonic() >= deadline:
            raise RuntimeError('Recorder did not create an MCAP file within 15 seconds.')
        stop_requested.wait(0.1)

    return False


def record_for_duration(process, duration, stop_requested):
    deadline = time.monotonic() + duration
    while process.poll() is None and not stop_requested.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        stop_requested.wait(min(remaining, 0.1))


def stop_recorder(process):
    if process.poll() is not None:
        return

    # 子进程拥有独立进程组；只向本次录制发送信号，不影响传感器或网关。
    # SIGINT 让 rosbag2 执行收尾，关闭 MCAP 并写出 metadata.yaml。
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        print('Recorder shutdown timed out; requesting termination.', file=sys.stderr)

    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)

    raise RuntimeError('Recorder required forced termination; the bag may be incomplete.')


def check_transform(output, topic, parent_frame, child_frame):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(output), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        while reader.has_next():
            _, serialized, _ = reader.read_next()
            message = deserialize_message(serialized, TFMessage)
            for transform in message.transforms:
                if (
                    transform.header.frame_id == parent_frame
                    and transform.child_frame_id == child_frame
                ):
                    return
    finally:
        reader.close()

    # Topic 有消息不代表包含雷达安装关系，必须检查实际写入的 TF 内容。
    kind = 'static' if topic == '/tf_static' else 'dynamic'
    raise RuntimeError(f'No recorded {kind} transform: {parent_frame} -> {child_frame}.')


def report_recording(output, topics, static_frames=None, dynamic_frames=None):
    from rosbag2_py import Info

    if not (output / 'metadata.yaml').is_file():
        raise RuntimeError(f'Recorder did not finalize metadata: {output}')

    metadata = Info().read_metadata(str(output), 'mcap')
    counts = {
        topic.topic_metadata.name: topic.message_count
        for topic in metadata.topics_with_message_count
    }
    missing = [topic for topic in topics if counts.get(topic, 0) == 0]
    if missing:
        raise RuntimeError(f'No messages recorded on: {", ".join(missing)}. Check the publishers.')

    if static_frames is not None:
        check_transform(output, '/tf_static', *static_frames)
    if dynamic_frames is not None:
        check_transform(output, '/tf', *dynamic_frames)

    print(f'Saved {metadata.message_count} messages to {output}', flush=True)
    for topic in topics:
        print(f'  {topic}: {counts[topic]} messages', flush=True)


def main():
    arguments = parse_arguments()
    output = arguments.output
    if output is None:
        name = datetime.now().strftime('demo_%Y%m%d_%H%M%S_%f')
        output = Path('bags') / name
    output = output.expanduser().resolve()

    if output.exists():
        print(f'Output path already exists: {output}', file=sys.stderr)
        return 1

    stop_requested = threading.Event()

    def request_stop(signum, frame):
        # 信号处理仅请求退出；等待进程和检查文件由正常控制流完成。
        stop_requested.set()

    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_stop)

    process = None
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        command = [
            'ros2',
            'bag',
            'record',
            '--storage',
            'mcap',
            '--output',
            str(output),
            '--disable-keyboard-controls',
            '--topics',
            *arguments.topics,
        ]
        if '/tf_static' in arguments.topics:
            from ament_index_python.packages import get_package_share_directory

            package_share = Path(get_package_share_directory('ros_gateserver'))
            command.extend(
                [
                    '--qos-profile-overrides-path',
                    str(package_share / 'config' / 'rosbag_qos.yaml'),
                ]
            )
        print(f'Recording topics: {", ".join(arguments.topics)}', flush=True)
        process = subprocess.Popen(command, start_new_session=True)
        try:
            if wait_until_recording(process, output, stop_requested):
                print(
                    f'Recording for {arguments.duration:g} seconds; Ctrl+C stops early.',
                    flush=True,
                )
                record_for_duration(process, arguments.duration, stop_requested)
        finally:
            stop_recorder(process)

        if process.returncode != 0:
            raise RuntimeError(f'Recorder failed with exit code {process.returncode}.')
        static_frames = None
        if arguments.with_static_tf:
            static_frames = (arguments.tf_parent_frame, arguments.tf_child_frame)
        dynamic_frames = None
        if arguments.with_dynamic_tf:
            dynamic_frames = (arguments.dynamic_parent_frame, arguments.dynamic_child_frame)
        report_recording(output, arguments.topics, static_frames, dynamic_frames)
        return 0
    except (OSError, RuntimeError) as error:
        print(f'Recording failed: {error}', file=sys.stderr)
        return 1
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    sys.exit(main())
