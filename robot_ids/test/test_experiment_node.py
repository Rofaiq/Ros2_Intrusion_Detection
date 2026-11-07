"""The experiment runner must stop every process an attacker trial starts.

"ros2 run" starts the attacker as a child process. Signalling only
"ros2 run" leaves that child running, and a leftover attacker spoils every
later trial, so the runner signals the whole process group.
"""

import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter  # noqa: E402

from robot_ids.experiment_node import ExperimentNode  # noqa: E402


def alive(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().split(')')[-1].split()[0] != 'Z'  # zombies are dead
    except FileNotFoundError:
        return False


def test_stop_attacker_also_stops_grandchildren(tmp_path):
    rclpy.init()
    try:
        node = ExperimentNode(parameter_overrides=[
            Parameter('output_dir', value=str(tmp_path))])
        pid_file = tmp_path / 'child.pid'
        # Stands in for "ros2 run": a shell that starts a child and waits.
        node.proc = subprocess.Popen(
            ['sh', '-c', f'sleep 300 & echo $! > {pid_file}; wait'],
            start_new_session=True)
        deadline = time.monotonic() + 5.0
        while not (pid_file.exists() and pid_file.read_text().strip()):
            assert time.monotonic() < deadline, 'child did not start'
            time.sleep(0.05)
        child = int(pid_file.read_text())
        assert alive(child)
        node.stop_attacker()
        deadline = time.monotonic() + 5.0
        while alive(child) and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not alive(child), 'the attacker started by "ros2 run" survived'
        assert node.proc is None
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
