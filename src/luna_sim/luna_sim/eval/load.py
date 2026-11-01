"""Read a recorded run (ROS 2 bag, MCAP or sqlite3) into numpy time series.

Uses `rosbags` (pure Python), so scoring needs no ROS install. Message definitions are read
from the bag itself, so custom luna_interfaces types work without registering anything as
long as the recorder stored them (MCAP always does).

Time: everything is converted to **sim time** in seconds.
  - Messages with a header use `header.stamp` (the measurement time, per the contract).
  - Headerless messages (std_msgs/Float32 masses, plain Twist) only have the bag's receive
    time, which `ros2 bag record` takes from the *wall* clock unless run with
    --use-sim-time. We map receive time -> sim time using /clock if it was recorded,
    otherwise using the (receive time, header.stamp) pairs of the ground-truth pose.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

TOPICS = {
    'gt_pose': '/sim/ground_truth/pose',            # geometry_msgs/PoseStamped, map
    'gt_twist': '/sim/ground_truth/twist',          # geometry_msgs/TwistStamped, base_link
    'est_odom': '/odometry/filtered',               # nav_msgs/Odometry, map -> base_link
    'cmd_vel': '/cmd_vel',                          # geometry_msgs/TwistStamped (or Twist)
    'berm_mass': '/sim/ground_truth/berm_mass_kg',  # std_msgs/Float32
    'misplaced_mass': '/sim/ground_truth/misplaced_mass_kg',  # std_msgs/Float32
    'clock': '/clock',                              # rosgraph_msgs/Clock
}


def _stamp(s):
    return s.sec + s.nanosec * 1e-9


def _yaw(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class Series:
    t: np.ndarray = field(default_factory=lambda: np.array([]))
    cols: dict = field(default_factory=dict)

    def __len__(self):
        return len(self.t)

    def __getitem__(self, k):
        return self.cols[k]


@dataclass
class Run:
    path: Path
    gt_pose: Series          # x, y, yaw
    gt_twist: Series         # vx, vy, wz
    est: Series              # x, y, yaw
    cmd: Series              # lin, ang
    dumped_mass: Series      # kg (berm + misplaced)
    berm_mass: Series        # kg
    topics_present: list
    warnings: list


def _sorted(t, cols):
    t = np.asarray(t, dtype=float)
    order = np.argsort(t, kind='stable')
    return Series(t[order], {k: np.asarray(v, dtype=float)[order] for k, v in cols.items()})


def load_run(bag_path, topics=None) -> Run:
    topics = {**TOPICS, **(topics or {})}
    by_topic = {v: k for k, v in topics.items()}
    raw = {k: [] for k in topics}
    warnings = []

    with AnyReader([Path(bag_path)]) as reader:
        present = sorted(c.topic for c in reader.connections)
        conns = [c for c in reader.connections if c.topic in by_topic]
        for conn, log_ns, data in reader.messages(connections=conns):
            key = by_topic[conn.topic]
            raw[key].append((log_ns * 1e-9, reader.deserialize(data, conn.msgtype)))

    # ---- receive-time -> sim-time map
    if raw['clock']:
        lt = [lt for lt, _ in raw['clock']]
        st = [_stamp(m.clock) for _, m in raw['clock']]
    elif raw['gt_pose']:
        lt = [lt for lt, _ in raw['gt_pose']]
        st = [_stamp(m.header.stamp) for _, m in raw['gt_pose']]
        warnings.append('no /clock in bag; mapping receive time to sim time via ground-truth pose')
    else:
        lt = st = []
    if len(lt) >= 2:
        o = np.argsort(lt)
        lt_a, st_a = np.asarray(lt)[o], np.asarray(st)[o]
        to_sim = lambda x: np.interp(x, lt_a, st_a)  # noqa: E731
    else:
        to_sim = lambda x: np.asarray(x, dtype=float)  # noqa: E731
        warnings.append('no sim-time reference; headerless topics use bag receive time')

    def header_or_log(lt_, m):
        h = getattr(m, 'header', None)
        return _stamp(h.stamp) if h is not None else float(to_sim(lt_))

    # ---- ground truth pose
    gp = raw['gt_pose']
    gt_pose = _sorted([_stamp(m.header.stamp) for _, m in gp], {
        'x': [m.pose.position.x for _, m in gp],
        'y': [m.pose.position.y for _, m in gp],
        'yaw': [_yaw(m.pose.orientation) for _, m in gp],
    })
    if not gp:
        warnings.append(f"missing {topics['gt_pose']}: nothing can be scored")

    gtw = raw['gt_twist']
    gt_twist = _sorted([_stamp(m.header.stamp) for _, m in gtw], {
        'vx': [m.twist.linear.x for _, m in gtw],
        'vy': [m.twist.linear.y for _, m in gtw],
        'wz': [m.twist.angular.z for _, m in gtw],
    })

    eo = raw['est_odom']
    est = _sorted([_stamp(m.header.stamp) for _, m in eo], {
        'x': [m.pose.pose.position.x for _, m in eo],
        'y': [m.pose.pose.position.y for _, m in eo],
        'yaw': [_yaw(m.pose.pose.orientation) for _, m in eo],
    })
    if eo and eo[0][1].header.frame_id not in ('map', ''):
        warnings.append(f"{topics['est_odom']} frame_id is {eo[0][1].header.frame_id!r}, "
                        "expected 'map'; localization error may be meaningless")

    cv = raw['cmd_vel']
    def _tw(m):
        return m.twist if hasattr(m, 'twist') else m  # TwistStamped or Twist
    cmd = _sorted([header_or_log(lt_, m) for lt_, m in cv], {
        'lin': [_tw(m).linear.x for _, m in cv],
        'ang': [_tw(m).angular.z for _, m in cv],
    })

    # ---- masses: build total dumped mass as a step signal of berm + misplaced
    events = ([(float(to_sim(lt_)), 'berm', m.data) for lt_, m in raw['berm_mass']]
              + [(float(to_sim(lt_)), 'mis', m.data) for lt_, m in raw['misplaced_mass']])
    events.sort(key=lambda e: e[0])
    cur = {'berm': 0.0, 'mis': 0.0}
    mt, mtot, mberm = [], [], []
    for t_, which, v in events:
        cur[which] = float(v)
        mt.append(t_)
        mtot.append(cur['berm'] + cur['mis'])
        mberm.append(cur['berm'])
    dumped = Series(np.asarray(mt), {'kg': np.asarray(mtot)})
    berm = Series(np.asarray(mt), {'kg': np.asarray(mberm)})

    return Run(Path(bag_path), gt_pose, gt_twist, est, cmd, dumped, berm, present, warnings)
