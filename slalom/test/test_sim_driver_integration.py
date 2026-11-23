"""Integration test (software-in-the-loop): the real simulator + the real driver.

    this test process                                  separate process
    +-----------------------------------+              +--------------+
    | Sim node (sim.py, no plot)        | --cones-->   | driver.py    |
    | + observer of the driver commands | <--cmd_vel-- | (autostart)  |
    +-----------------------------------+              +--------------+

The simulator runs inside the test, so we can read the car's true position
(sim.trace) without changing sim.py. The driver runs as its own process, like on
the car, so everything goes through real ROS 2 topics.

The driver gets the same parameters as on the real car: they are read from
car_scripts/run_driver.sh, so the test follows every re-tuning automatically.

It can never move the car: own ROS domain (99), localhost-only discovery and
topics remapped to /it/... (never /cmd_vel).

One drive through a 3-cone course (1.5 m spacing as in the proposal) at the real
speed (about 20 s), then six checks.
Run: colcon test --packages-select slalom && colcon test-result --verbose
"""
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from slalom_interfaces.msg import Cone

from slalom.sim import Sim

TEST_DOMAIN = 99
CMD_TOPIC = '/it/cmd_vel'
CONES_TOPIC = '/it/cones'
TOPIC_ARGS = ['-p', f'cmd_topic:={CMD_TOPIC}', '-p', f'cones_topic:={CONES_TOPIC}']
N_CONES = 3
CONE_SPACING = 1.5  # m, as in the project proposal
TIME_LIMIT = 60.0   # s, the drive takes about 20 s
HIT_DIST = 0.15     # m, the simulator reports a hit below this distance

RUN_DRIVER = Path(__file__).resolve().parents[2] / 'car_scripts' / 'run_driver.sh'
# parameters of run_driver.sh that do not apply to the simulator:
#   cones_topic / cmd_topic -> remapped to /it/... above
#   steering_trim           -> compensates the real car pulling to one side; the sim does not pull
CAR_ONLY = {'cones_topic', 'cmd_topic', 'steering_trim'}

os.environ['ROS_DOMAIN_ID'] = str(TEST_DOMAIN)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'


def car_params():
    """The driver parameters of the real car, as ROS args: ['-p', 'offset:=0.35', ...]."""
    found = re.findall(r'-p\s+(\w+):=(\S+)', RUN_DRIVER.read_text())
    return [arg for name, value in found if name not in CAR_ONLY
            for arg in ('-p', f'{name}:={value}')]


@pytest.fixture(scope='module')
def run():
    """Drive once through the course and record everything (shared by all checks)."""
    rclpy.init(args=['test_sim_driver_integration', '--ros-args', '-p', 'plot:=false',
                     '-p', f'n_cones:={N_CONES}', '-p', f'spacing:={CONE_SPACING}', *TOPIC_ARGS],
               domain_id=TEST_DOMAIN)
    sim = Sim()
    observer = Node('it_observer')
    cmds = []
    observer.create_subscription(Twist, CMD_TOPIC, cmds.append, 100)
    executor = SingleThreadedExecutor()
    executor.add_node(sim)
    executor.add_node(observer)
    params = car_params()
    driver = subprocess.Popen(
        [sys.executable, '-m', 'slalom.driver', '--ros-args', *TOPIC_ARGS, *params,
         '-p', 'autostart:=true'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    moved = stopped = False
    stop_time = None
    start = time.time()
    try:
        while time.time() - start < TIME_LIMIT:
            executor.spin_once(timeout_sec=0.02)
            if sim.speed > 0.0:
                moved, stop_time = True, None
            elif moved and stop_time is None:
                stop_time = time.time()
            if stop_time is not None and time.time() - stop_time > 1.0:   # still for 1 s
                stopped = True
                break
    finally:
        driver.send_signal(signal.SIGINT)
        try:
            log, _ = driver.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            driver.kill()
            log, _ = driver.communicate()
        result = SimpleNamespace(
            trace=list(sim.trace), cmds=list(cmds), moved=moved, stopped=stopped,
            drive_time=time.time() - start,
            yellow=[(x, y) for c, x, y in sim.cones if c == Cone.YELLOW],
            pink=next((x, y) for c, x, y in sim.cones if c == Cone.PINK),
            all_cones=[(x, y) for c, x, y in sim.cones])
        sim.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
    # shown by pytest only if a check fails
    print('car parameters used:', ' '.join(params[1::2]))
    print(f'drive time {result.drive_time:.1f} s, driver log:')
    print('\n'.join(line for line in log.splitlines() if 'State' in line or 'WARN' in line))
    return result


def side_of(trace, cone_x, cone_y):
    """'left' or 'right': where the car was when it reached the cone's x."""
    for x, y in trace:
        if x >= cone_x:
            return 'left' if y > cone_y else 'right'
    return 'never reached'


# ---------- the six checks ----------
def test_car_drives_off(run):
    # cones reach the driver and its commands reach the simulator
    moved = run.moved   # (local name keeps pytest's failure message short)
    assert moved, 'the car never moved: are the topics connected?'


def test_car_stops_by_itself(run):
    stopped = run.stopped
    assert stopped, f'the car did not stop within {TIME_LIMIT:.0f} s'


def test_cones_passed_left_right_alternating(run):
    expected = ['left' if i % 2 == 0 else 'right' for i in range(N_CONES)]
    assert [side_of(run.trace, x, y) for x, y in run.yellow] == expected


def test_no_cone_is_hit(run):
    closest = min(math.hypot(x - cx, y - cy) for x, y in run.trace for cx, cy in run.all_cones)
    assert closest > HIT_DIST, f'the car came {closest:.2f} m close to a cone'


def test_stops_just_before_the_pink_cone(run):
    x, y = run.trace[-1]
    px, py = run.pink
    distance = math.hypot(px - x, py - y)
    assert x < px, 'the car drove past the pink cone'
    assert 0.4 < distance < 1.0, f'stopped {distance:.2f} m from the pink cone'


def test_last_command_is_zero(run):
    last = run.cmds[-1]
    assert last.linear.x == 0.0 and last.angular.z == 0.0
