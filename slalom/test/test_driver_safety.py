"""Safety tests for the driver: every way of stopping must leave a ZERO command.

Why: the real car keeps executing the LAST /cmd_vel it received
(docs/car_measurements.md). So whenever the driver stops, the last thing it
publishes must be linear.x = 0.

These tests can never move the car:
  - they use their own ROS domain (TEST_DOMAIN) with localhost-only discovery,
  - the driver's topics are remapped to /test_safety/...,
  - the fast tests replace the publisher with a fake one that only records messages.

Run (inside the ROS workspace):
    colcon test --packages-select slalom && colcon test-result --verbose
"""
import os
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from slalom_interfaces.msg import Cone, ConeArray

from slalom.driver import Driver, SlalomBrain

TEST_DOMAIN = 99
CMD_TOPIC = '/test_safety/cmd_vel'
CONES_TOPIC = '/test_safety/cones'
REMAP_ARGS = ['--ros-args',
              '-p', f'cmd_topic:={CMD_TOPIC}',
              '-p', f'cones_topic:={CONES_TOPIC}']
CIRCLE, L1 = 1, 9   # SDL button numbers, see docs/car_measurements.md

os.environ['ROS_DOMAIN_ID'] = str(TEST_DOMAIN)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'


# ---------- helpers ----------
class FakePublisher:
    """Stands in for the real publisher: records messages instead of sending them."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def cones(*items):
    """cones((Cone.YELLOW, 2.0, 0.0), ...) -> ConeArray"""
    msg = ConeArray()
    for color, x, y in items:
        c = Cone()
        c.color, c.x, c.y, c.confidence = color, float(x), float(y), 1.0
        msg.cones.append(c)
    return msg


def joy(button):
    msg = Joy()
    msg.buttons = [0] * 13
    msg.buttons[button] = 1
    return msg


def is_zero(cmd):
    return cmd.linear.x == 0.0 and cmd.angular.z == 0.0


@pytest.fixture
def driver():
    """A Driver node whose output goes into a FakePublisher. Nothing is spun or sent."""
    rclpy.init(args=['test_driver_safety', *REMAP_ARGS], domain_id=TEST_DOMAIN)
    node = Driver()
    node.pub = FakePublisher()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def start_driving(driver):
    driver.brain.start()
    driver.on_cones(cones((Cone.YELLOW, 2.0, 0.0)))


# ---------- driver node: stop paths ----------
def test_driving_publishes_throttle(driver):
    start_driving(driver)
    assert driver.pub.msgs[-1].linear.x == pytest.approx(0.3)


@pytest.mark.parametrize('button', [CIRCLE, L1], ids=['circle', 'L1'])
def test_stop_button_publishes_zero_immediately(driver, button):
    start_driving(driver)
    driver.on_joy(joy(button))          # no cone message after this!
    assert driver.brain.state == 'IDLE'
    assert is_zero(driver.pub.msgs[-1]), \
        'stop button must publish a zero right away, not wait for the next cone message'


def test_watchdog_stops_when_cone_data_is_old(driver):
    start_driving(driver)
    driver.last_msg_time = driver.get_clock().now() - Duration(seconds=1.0)
    driver.check_watchdog()
    assert is_zero(driver.pub.msgs[-1])


def test_watchdog_stays_silent_when_idle(driver):
    # IDLE = the human may be driving with the PS controller -> we must not publish
    driver.last_msg_time = driver.get_clock().now() - Duration(seconds=1.0)
    driver.check_watchdog()
    assert driver.pub.msgs == []


# ---------- SlalomBrain: pure logic, no ROS needed ----------
def test_pink_cone_close_means_done():
    brain = SlalomBrain()
    brain.start()
    assert brain.update([(Cone.PINK, 0.5, 0.0)]) is None
    assert brain.state == 'DONE'


def test_side_switches_after_passing_a_cone():
    brain = SlalomBrain()                               # first cone: pass on the left
    brain.start()
    assert brain.update([(Cone.YELLOW, 0.8, 0.0)]) > 0  # close cone (< pass_max_x): steer left
    # the close cone leaves the view, only the next one (3 m) is seen: the brain
    # first waits hold_frames updates, in case it was only a detection flicker
    for _ in range(brain.hold_frames):
        assert brain.update([(Cone.YELLOW, 3.0, 0.0)]) > 0  # still left
    assert brain.update([(Cone.YELLOW, 3.0, 0.0)]) < 0      # confirmed: steer right
    assert brain.passed == 1


# ---------- end to end: the real driver process gets Ctrl+C ----------
def test_ctrl_c_leaves_zero_command():
    rclpy.init(domain_id=TEST_DOMAIN)
    node = Node('ctrl_c_tester')
    received = []
    node.create_subscription(Twist, CMD_TOPIC, received.append, 50)
    cones_pub = node.create_publisher(ConeArray, CONES_TOPIC, 10)
    # watchdog_s is huge so that a zero can only come from the shutdown code
    proc = subprocess.Popen([sys.executable, '-m', 'slalom.driver', *REMAP_ARGS,
                             '-p', 'autostart:=true', '-p', 'watchdog_s:=60.0'])
    try:
        deadline = time.time() + 10.0
        while time.time() < deadline and not (received and received[-1].linear.x > 0.0):
            cones_pub.publish(cones((Cone.YELLOW, 2.0, 0.0)))
            rclpy.spin_once(node, timeout_sec=0.05)
        assert received and received[-1].linear.x > 0.0, 'driver never started driving'

        proc.send_signal(signal.SIGINT)                  # same as pressing Ctrl+C
        proc.wait(timeout=5.0)
        end = time.time() + 1.0                          # collect messages still in flight
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if proc.poll() is None:
            proc.kill()
        node.destroy_node()
        rclpy.shutdown()

    last = received[-1]
    assert is_zero(last), f'after Ctrl+C the car would keep driving with linear.x={last.linear.x}'
    assert proc.returncode == 0
