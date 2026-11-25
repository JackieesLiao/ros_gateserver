#!/usr/bin/env python3
"""Check task reports, cancellation, heartbeat loss, and counter regression."""

import json
import math
import os
from pathlib import Path
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '132')

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from ros_gateserver_interfaces.action import CheckSensors
from std_msgs.msg import String

from smoke_test import stop
from tf_test import TestProcesses


class ActionFixture:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, CheckSensors, '/gateway/check_sensors')
        self.publisher = node.create_publisher(String, '/gateway/status', 10)
        self.publishing = False
        self.malformed = False
        self.count = 0
        self.freeze = False
        self.enabled = True
        self.stale = set()
        self.missing = set()
        self.transform = 'ok'
        self.feedback = []
        self.timer = node.create_timer(0.05, self.publish)

    def publish(self):
        if not self.publishing:
            return
        if not self.freeze:
            self.count += 1
        message = String()
        if self.malformed:
            message.data = '{invalid json'
        else:
            sensors = {}
            for name in ('image', 'scan', 'odom'):
                data = None
                if name == 'scan' and self.enabled:
                    data = {'transform_status': self.transform}
                sensors[name] = {
                    'received': 0 if name in self.missing else self.count,
                    'fresh': name not in self.stale and name not in self.missing,
                    'data': data,
                }
            message.data = json.dumps({
                'schema_version': 1,
                'enabled': self.enabled,
                'sensors': sensors,
            })
        self.publisher.publish(message)

    def wait(self, future, timeout=5):
        deadline = time.monotonic() + timeout
        while not future.done():
            assert time.monotonic() < deadline, 'Action future timed out'
            rclpy.spin_once(self.node, timeout_sec=0.02)
        assert future.exception() is None, future.exception()
        return future.result()

    def send(self, duration=1.0, require_tf=False, accepted=True):
        goal = CheckSensors.Goal()
        goal.duration_sec = duration
        goal.require_scan_tf = require_tf
        self.feedback = []

        def record_feedback(message):
            self.feedback.append(message.feedback)

        handle = self.wait(self.client.send_goal_async(goal, feedback_callback=record_feedback))
        assert handle.accepted == accepted, (duration, handle.accepted)
        return handle

    def wait_phase(self, phase):
        deadline = time.monotonic() + 4
        while not any(item.phase == phase for item in self.feedback):
            assert time.monotonic() < deadline, (phase, self.feedback)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def result(self, handle, status, passed, failure=None):
        result = self.wait(handle.get_result_async())
        assert result.status == status, (result.status, result.result)
        assert result.result.passed == passed, result.result
        if failure is not None:
            assert failure in result.result.failed_checks, result.result
        assert all(0.0 <= item.progress <= 1.0 for item in self.feedback)
        return result.result

    def cancel(self, handle):
        response = self.wait(handle.cancel_goal_async())
        assert response.goals_canceling, response
        return self.result(handle, GoalStatus.STATUS_CANCELED, False, 'canceled')

    def close(self):
        self.node.destroy_timer(self.timer)
        self.node.destroy_publisher(self.publisher)
        self.client.destroy()


def check_goals(fixture):
    for invalid in (0.0, -1.0, 61.0, math.nan, math.inf):
        fixture.send(invalid, accepted=False)

    handle = fixture.send()
    result = fixture.result(handle, GoalStatus.STATUS_ABORTED, False, 'gateway_startup_timeout')
    assert result.observed_sec == 0.0

    handle = fixture.send(5.0)
    result = fixture.cancel(handle)
    assert result.observed_sec == 0.0
    print('PASS: invalid targets, startup timeout, cancel while waiting', flush=True)

    fixture.publishing = True
    handle = fixture.send(1.2, require_tf=True)
    result = fixture.result(handle, GoalStatus.STATUS_SUCCEEDED, True)
    assert result.observed_sec >= 1.2
    assert any(item.phase == 'OBSERVING' for item in fixture.feedback)
    assert not result.failed_checks
    print('PASS: observation duration, feedback, successful report', flush=True)


def check_reports(fixture):
    fixture.transform = 'unavailable'
    fixture.result(
        fixture.send(require_tf=True), GoalStatus.STATUS_SUCCEEDED, False, 'scan_tf_unavailable'
    )
    fixture.transform = 'ok'
    fixture.enabled = False
    result = fixture.result(
        fixture.send(require_tf=True), GoalStatus.STATUS_SUCCEEDED, False, 'scan_tf_not_observable'
    )
    assert 'scan_stale' not in result.failed_checks
    fixture.result(fixture.send(), GoalStatus.STATUS_SUCCEEDED, True)
    fixture.enabled = True

    fixture.stale = {'image'}
    handle = fixture.send(1.5)
    fixture.wait_phase('OBSERVING')
    assert any('image' in item.unhealthy_sensors for item in fixture.feedback)
    fixture.stale.clear()
    # 中途恢复不抹去已经观察到的异常，结果报告必须保留失败项。
    fixture.result(handle, GoalStatus.STATUS_SUCCEEDED, False, 'image_stale')
    fixture.missing = {'image'}
    fixture.result(fixture.send(), GoalStatus.STATUS_SUCCEEDED, False, 'image_missing')
    fixture.missing.clear()
    fixture.freeze = True
    fixture.result(fixture.send(), GoalStatus.STATUS_SUCCEEDED, False, 'scan_not_updated')
    fixture.freeze = False
    print('PASS: TF failure, forwarding visibility, stale/missing/frozen sensors', flush=True)


def check_running_faults(fixture):
    handle = fixture.send(5.0)
    fixture.wait_phase('OBSERVING')
    fixture.send(accepted=False)
    canceled = fixture.cancel(handle)
    assert canceled.observed_sec > 0.0
    fixture.result(fixture.send(), GoalStatus.STATUS_SUCCEEDED, True)

    handle = fixture.send(5.0)
    fixture.wait_phase('OBSERVING')
    fixture.publishing = False
    fixture.result(handle, GoalStatus.STATUS_ABORTED, False, 'gateway_status_timeout')
    fixture.publishing = True

    handle = fixture.send(5.0)
    fixture.wait_phase('OBSERVING')
    fixture.count = 0
    fixture.result(handle, GoalStatus.STATUS_ABORTED, False, 'sensor_counter_regressed')

    handle = fixture.send(5.0)
    fixture.wait_phase('OBSERVING')
    fixture.malformed = True
    fixture.result(handle, GoalStatus.STATUS_ABORTED, False, 'gateway_status_timeout')
    fixture.malformed = False
    print('PASS: busy rejection, running cancellation, lost/invalid status, reset', flush=True)


def check_terminal_race(fixture):
    handle = fixture.send()
    fixture.wait_phase('OBSERVING')
    deadline = time.monotonic() + 0.45
    while time.monotonic() < deadline:
        rclpy.spin_once(fixture.node, timeout_sec=0.01)
    fixture.wait(handle.cancel_goal_async())
    result = fixture.wait(handle.get_result_async())
    assert result.status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED), result
    fixture.result(fixture.send(), GoalStatus.STATUS_SUCCEEDED, True)
    print('PASS: cancellation near completion reaches one terminal state', flush=True)


def main():
    rclpy.init()
    node = rclpy.create_node('sensor_action_test')
    fixture = ActionFixture(node)
    try:
        with tempfile.TemporaryDirectory(prefix='gateway_action_test_') as directory:
            with tempfile.TemporaryFile(mode='w+') as logs:
                try:
                    domain = int(os.environ['ROS_DOMAIN_ID'])
                    with TestProcesses(domain, logs, Path(directory)) as case:
                        server = case.start([
                            'ros2', 'run', 'ros_gateserver', 'sensor_check_server',
                            '--ros-args', '-p', 'startup_timeout_sec:=0.6',
                            '-p', 'status_timeout_sec:=0.4',
                        ])
                        assert fixture.client.wait_for_server(timeout_sec=15)
                        check_goals(fixture)
                        check_reports(fixture)
                        check_running_faults(fixture)
                        check_terminal_race(fixture)

                        fixture.send(5.0)
                        fixture.wait_phase('OBSERVING')
                        stop(server)
                        assert server.returncode == 0, server.returncode
                        print('PASS: server exits normally with an active task', flush=True)
                except Exception:
                    logs.seek(0)
                    print(logs.read())
                    raise
    finally:
        fixture.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
