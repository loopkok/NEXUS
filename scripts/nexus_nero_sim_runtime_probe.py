#!/usr/bin/env python3
"""Hardware-free servo/ROS/viewer performance probe for the Nero MuJoCo driver."""
import argparse
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from nero_mujoco_sim.mujoco_sim_node import NeroMujocoSimNode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', required=True)
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--duration', type=float, default=12.0)
    args = parser.parse_args()
    rclpy.init(args=['--ros-args', '-p', f'profile_file:={args.profile}',
                    '-p', f'enable_viewer:={str(args.viewer).lower()}'])
    sim = NeroMujocoSimNode()  # constructor rejects all physical-driver profiles
    feeder = Node('nero_runtime_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(feeder)
    home = {}
    for name, spec in sim.specs.items():
        home[name] = list(sim.profile.adapter_config('nero_mujoco').get(
            'home_pose', {}).get(spec.side, [0.] * spec.dim)) if spec.kind == 'arm' else [0.1] * spec.dim
        sim._enabled[name] = True
        sim._targets[name] = home[name].copy()
        sim._last_commands[name] = time.monotonic()
        for jid, value in zip(sim._joint_ids[name], home[name]):
            sim.data.qpos[sim.model.jnt_qposadr[jid]] = value
    sim._mujoco.mj_forward(sim.model, sim.data)
    origin = time.monotonic()
    samples = {name: [] for name in sim.specs}
    received = {name: [] for name in sim.specs}
    publishers = {}
    for name in sim.specs:
        publishers[name] = feeder.create_publisher(
            JointState, sim.profile.topic(name, 'joint_commands'), qos_profile_sensor_data)
        def observe(msg, component=name):
            samples[component].append((time.monotonic() - origin, msg.position[0]))
        feeder.create_subscription(JointState, sim.profile.topic(name, 'joint_states'),
                                   observe, qos_profile_sensor_data)
    original_command = sim._on_command
    def command(name, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        received[name].append((time.monotonic() - origin, (time.time() - stamp) * 1000.))
        original_command(name, msg)
    sim._on_command = command
    def publish():
        elapsed = time.monotonic() - origin
        for name, pub in publishers.items():
            spec = sim.specs[name]
            msg = JointState()
            msg.header.stamp = feeder.get_clock().now().to_msg()
            msg.name = list(spec.joints)
            msg.position = home[name].copy()
            msg.position[0] += .02 * math.sin(2. * math.pi * .8 * elapsed)
            pub.publish(msg)
    feeder.create_timer(.01, publish)
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
    worker = threading.Thread(target=spin, daemon=True)
    worker.start()
    sync_times = []
    step_times = []
    original_step = sim._step_physics
    def step():
        original_step()
        step_times.append(time.monotonic() - origin)
    sim._step_physics = step
    launch = sim._viewer_module.launch_passive
    class ViewerProxy:
        def __init__(self, viewer):
            self.viewer = viewer
            self.threads = set(threading.enumerate()) - before_viewer_threads
        def __getattr__(self, name):
            return getattr(self.viewer, name)
        def sync(self):
            started = time.monotonic()
            result = self.viewer.sync()
            sync_times.append((time.monotonic() - origin, (time.monotonic() - started) * 1000.))
            return result
        def close(self):
            self.viewer.close()
            for thread in self.threads:
                thread.join(timeout=5.)
    before_viewer_threads = set(threading.enumerate())
    sim._viewer_module.launch_passive = lambda *a, **kw: ViewerProxy(launch(*a, **kw))
    timer = threading.Timer(args.duration, getattr(sim, 'request_stop', rclpy.shutdown))
    timer.start()
    sim_origin = sim.data.time
    try:
        sim.run()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        timer.cancel()
        executor.shutdown(timeout_sec=2.)
        worker.join(timeout=2.)
    elapsed = step_times[-1] - step_times[0]
    # Exclude native window creation/destruction and the first second of DDS
    # discovery from measured stream rates and servo phase lag.
    start, end = step_times[0] + 1., step_times[-1]
    stream_elapsed = end - start
    sync_durations = [dt for at, dt in sync_times if start <= at <= end]
    result = {'viewer': args.viewer, 'elapsed_s': elapsed,
              'real_time_factor': (sim.data.time - sim_origin) / elapsed,
              'physics_hz': len(step_times) / elapsed,
              'viewer_sync_hz': len(sync_durations) / stream_elapsed,
              'viewer_sync_p95_ms': float(np.percentile(sync_durations, 95)) if sync_durations else 0.,
              'components': {}}
    for name, observations in samples.items():
        ages = [age for at, age in received[name] if start <= at <= end]
        observations = [(at, value) for at, value in observations if start <= at <= end]
        item = {'command_hz': len(ages) / stream_elapsed, 'state_hz': len(observations) / stream_elapsed,
                'command_age_p95_ms': float(np.percentile(ages, 95)) if ages else None}
        trace = np.asarray(observations)
        if len(trace):
            centered = trace[:, 1] - np.mean(trace[:, 1])
            lags = np.arange(0., .601, .001)
            errors = [np.mean((centered - .02 * np.sin(2 * np.pi * .8 * (trace[:, 0] - lag))) ** 2)
                      for lag in lags]
            item['servo_lag_ms'] = float(lags[np.argmin(errors)] * 1000.)
        result['components'][name] = item
    print('RUNTIME_PROBE_RESULT ' + json.dumps(result), flush=True)
    sim.destroy_node()
    feeder.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
