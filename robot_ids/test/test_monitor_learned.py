"""The monitor loads a trained model and publishes feature windows."""

import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('sklearn')

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from robot_ids import train  # noqa: E402
from robot_ids.features import FEATURES  # noqa: E402
from robot_ids.monitor_node import MonitorNode  # noqa: E402


def test_monitor_runs_learned_detectors(tmp_path):
    import sys
    sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent))
    from test_learned import synthetic_rows
    models, _ = train.train_and_report(synthetic_rows())
    path = tmp_path / 'model.json'
    path.write_text(json.dumps(models))
    rclpy.init()
    try:
        monitor = MonitorNode(parameter_overrides=[
            Parameter('learned_model_path', value=str(path)),
            Parameter('detector_set', value='learned'),
            Parameter('startup_grace_s', value=0.5),
            Parameter('feature_period_s', value=0.1)])
        assert monitor.enabled == {'mlp', 'autoencoder'}
        assert len(monitor.learned) == 2
        helper = rclpy.create_node('test_helper')
        windows = []
        helper.create_subscription(String, '/ids/features',
                                   lambda m: windows.append(json.loads(m.data)), 10)
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)
        executor.add_node(helper)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert windows, 'no feature windows published'
        assert list(windows[-1]['features']) == list(FEATURES)
        monitor.destroy_node()
        helper.destroy_node()
    finally:
        rclpy.try_shutdown()
