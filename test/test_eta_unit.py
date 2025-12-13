import pytest
import rclpy

# Import module + class (important: we need module for main() test)
import eta_component.eta_node as eta_module
from eta_component.eta_node import EtaPublisher

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32


# =========================================================
# FIXTURE: create ETA node per test (real node, but no ROS graph)
# =========================================================
@pytest.fixture
def eta_node():
    rclpy.init()
    node = EtaPublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


# =========================================================
# TEST 1 — compute_path_distance: empty / short path
# =========================================================
def test_compute_distance_empty_path(eta_node):
    eta_node.path = []
    assert eta_node.compute_path_distance() == 0.0

    eta_node.path = [(0.0, 0.0)]
    assert eta_node.compute_path_distance() == 0.0


# =========================================================
# TEST 2 — compute_path_distance: valid path
# =========================================================
def test_compute_distance_valid_path(eta_node):
    eta_node.path = [(0.0, 0.0), (3.0, 4.0)]  # distance = 5
    assert eta_node.compute_path_distance() == pytest.approx(5.0)


# =========================================================
# TEST 3 — odom_callback sets speed (sqrt(vx^2 + vy^2))
# =========================================================
def test_odom_callback_sets_speed(eta_node):
    msg = Odometry()
    msg.twist.twist.linear.x = 3.0
    msg.twist.twist.linear.y = 4.0  # speed = 5
    eta_node.odom_callback(msg)
    assert eta_node.current_speed == pytest.approx(5.0)


# =========================================================
# TEST 4 — ack_callback sets speed (abs)
# =========================================================
def test_ackermann_callback_sets_speed(eta_node):
    msg = AckermannDrive()
    msg.speed = -2.5
    eta_node.ack_callback(msg)
    assert eta_node.current_speed == 2.5


# =========================================================
# TEST 5 — path_callback stores path coordinates
# =========================================================
def test_path_callback_stores_path(eta_node):
    path = Path()

    p1 = PoseStamped()
    p1.pose.position.x = 1.0
    p1.pose.position.y = 2.0

    p2 = PoseStamped()
    p2.pose.position.x = 3.0
    p2.pose.position.y = 4.0

    path.poses = [p1, p2]
    eta_node.path_callback(path)

    assert eta_node.path == [(1.0, 2.0), (3.0, 4.0)]


# =========================================================
# TEST 6 — update_eta: no path → publish_eta(0)
# =========================================================
def test_update_eta_no_path(eta_node, monkeypatch):
    eta_node.path = []
    eta_node.current_speed = 1.0

    published = {}
    monkeypatch.setattr(eta_node, "publish_eta", lambda v: published.update({"eta": v}))

    eta_node.update_eta()
    assert published["eta"] == 0


# =========================================================
# TEST 7 — update_eta: robot stopped → publish_eta(999)
# =========================================================
def test_update_eta_robot_stopped(eta_node, monkeypatch):
    eta_node.path = [(0.0, 0.0), (10.0, 0.0)]
    eta_node.current_speed = 0.0

    published = {}
    monkeypatch.setattr(eta_node, "publish_eta", lambda v: published.update({"eta": v}))

    eta_node.update_eta()
    assert published["eta"] == 999


# =========================================================
# TEST 8 — update_eta: normal moving robot (10m / 2m/s = 5s)
# =========================================================
def test_update_eta_normal_case(eta_node, monkeypatch):
    eta_node.path = [(0.0, 0.0), (10.0, 0.0)]
    eta_node.current_speed = 2.0

    published = {}
    monkeypatch.setattr(eta_node, "publish_eta", lambda v: published.update({"eta": v}))

    eta_node.update_eta()
    assert published["eta"] == 5


# =========================================================
# TEST 9 — publish_eta(): covers Int32 creation + publish + logger
# This directly targets the red block in your screenshot.
# =========================================================
def test_publish_eta_publishes_int32_and_logs(eta_node, monkeypatch):
    captured = {"msg": None, "log": None}

    # fake publisher
    class FakePub:
        def publish(self, msg):
            captured["msg"] = msg

    # fake logger
    class FakeLogger:
        def info(self, s):
            captured["log"] = s

    monkeypatch.setattr(eta_node, "eta_pub", FakePub())
    monkeypatch.setattr(eta_node, "get_logger", lambda: FakeLogger())

    eta_node.publish_eta(12)

    assert isinstance(captured["msg"], Int32)
    assert captured["msg"].data == 12
    assert captured["log"] is not None
    assert "ETA Published" in captured["log"]


# =========================================================
# TEST 10 — main(): covers rclpy.init/spin/shutdown + destroy_node
# No blocking: we stub spin to return immediately.
# This covers the red main() block in your screenshot.
# =========================================================
def test_main_calls_rclpy_lifecycle(monkeypatch):
    calls = []

    # fake node returned by EtaPublisher()
    class DummyNode:
        def destroy_node(self):
            calls.append("destroy_node")

    # Patch EtaPublisher constructor used inside main()
    monkeypatch.setattr(eta_module, "EtaPublisher", lambda: DummyNode())

    # Patch rclpy lifecycle
    monkeypatch.setattr(eta_module.rclpy, "init", lambda args=None: calls.append("init"))
    monkeypatch.setattr(eta_module.rclpy, "spin", lambda node: calls.append("spin"))
    monkeypatch.setattr(eta_module.rclpy, "shutdown", lambda: calls.append("shutdown"))

    eta_module.main(args=None)

    assert calls == ["init", "spin", "destroy_node", "shutdown"]
