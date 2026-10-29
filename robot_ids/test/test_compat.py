from robot_ids.compat import default_cmd_vel_stamped


def test_humble_uses_twist(monkeypatch):
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    assert default_cmd_vel_stamped() is False


def test_jazzy_uses_twist_stamped(monkeypatch):
    monkeypatch.setenv('ROS_DISTRO', 'jazzy')
    assert default_cmd_vel_stamped() is True
