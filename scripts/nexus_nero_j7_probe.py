#!/usr/bin/env python3
"""Observe Nero CAN feedback; explicitly opt in to ONE planned J7 move.

No enable, homing, retries, JS commands, calibration or return motion. The
default opens a receive connection with ALL transmit calls blocked. --execute
switches to CAN/J mode and submits one <=1 degree J7 target from fresh measured
positions; failures after taking control request the SDK damped emergency stop.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time


JOINT_FRAMES = {0x2A5: (0, 1), 0x2A6: (2, 3), 0x2A7: (4, 5), 0x2A9: (6,)}


class Feedback:
    def __init__(self):
        self.lock = threading.Lock()
        self.frames = {}

    def receive(self, frame):
        if frame.arbitration_id in JOINT_FRAMES and len(frame.data) == 8:
            with self.lock:
                self.frames[frame.arbitration_id] = (time.monotonic(), bytes(frame.data))

    def joints(self):
        now = time.monotonic()
        with self.lock:
            frames = dict(self.frames)
        q, ages = [None] * 7, {}
        for can_id, indices in JOINT_FRAMES.items():
            if can_id not in frames:
                ages[hex(can_id)] = None
                continue
            stamp, data = frames[can_id]
            ages[hex(can_id)] = now - stamp
            for word, joint in enumerate(indices):
                q[joint] = int.from_bytes(data[word*4:word*4+4], 'big', signed=True) * math.pi / 180_000
        return q, ages


def snapshot(arm, feedback):
    q, ages = feedback.joints()
    joint_record = arm.get_joint_angles()
    sdk_q = None if joint_record is None else list(joint_record.msg)
    record = arm.get_arm_status()
    controller = None if record is None else {
        'stamp_s': record.timestamp,
        'age_s': time.time() - record.timestamp,
        **{key: int(getattr(record.msg, key)) for key in
           ('ctrl_mode', 'arm_status', 'mode_feedback', 'motion_status', 'err_code')}}
    drivers = []
    for joint in range(1, 8):
        record = arm.get_driver_states(joint)
        drivers.append(None if record is None else {
            'age_s': time.time() - record.timestamp,
            **{key: bool(getattr(record.msg.foc_status, key)) for key in
               ('driver_enable_status', 'driver_error_status', 'collision_status', 'stall_status')}})
    return {'q_rad': q, 'q_sdk_rad': sdk_q, 'joint_frame_ages_s': ages,
            'controller': controller, 'drivers': drivers}


def validate_feedback(data, enabled=False):
    if any(age is None or not math.isfinite(age) or not 0 <= age < .25
           for age in data['joint_frame_ages_s'].values()):
        raise RuntimeError('All four position frames, including J7 0x2A9, must be fresh (<250ms); enable CAN feedback in the factory UI if absent')
    if len(data['q_rad']) != 7 or any(q is None or not math.isfinite(q) for q in data['q_rad']):
        raise RuntimeError('Invalid seven-joint measured feedback')
    if not enabled:
        return
    controller = data['controller']
    if (controller is None or not math.isfinite(controller['age_s'])
            or not -.1 <= controller['age_s'] < .25):
        raise RuntimeError('Fresh controller status required (<250ms); controller feedback missing or stale')
    if controller['err_code'] != 0 or controller['arm_status'] != 0:
        status = controller['arm_status']
        name = ' (JOINT_BRAKE_NOT_RELEASED)' if status == 6 else ''
        raise RuntimeError(f"Controller not ready: arm_status={status}{name}, err_code={controller['err_code']}; verify joint enable/brake status in the factory UI")
    if len(data['drivers']) != 7 or any(
            driver is None or not math.isfinite(driver['age_s'])
            or not -.1 <= driver['age_s'] < .25 or not driver['driver_enable_status']
            or any(driver[key] for key in ('driver_error_status', 'collision_status', 'stall_status'))
            for driver in data['drivers']):
        raise RuntimeError('All seven joints must already be enabled and healthy in the factory UI; this tool does not enable them')


def make_target(q, component, delta_deg):
    if not math.isfinite(delta_deg) or not .5 <= abs(delta_deg) <= 1:
        raise ValueError('J7 displacement magnitude must be 0.5..1 degree')
    if len(q) != 7 or len(component['lower']) != 7 or len(component['upper']) != 7:
        raise ValueError('Exactly seven measured joints and limit pairs required')
    target = list(q)
    target[6] += math.radians(delta_deg)
    for value, low, high in zip(target, component['lower'], component['upper']):
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError('Measured pose / target outside profile joint limits')
    return target


def active_nero_processes():
    conflicts = []
    for process in Path('/proc').glob('[0-9]*'):
        if int(process.name) == os.getpid():
            continue
        try:
            argv = (process / 'cmdline').read_bytes().split(b'\0')
            executables = [Path(arg.decode(errors='replace')).name for arg in argv[:3]]
            if any(name in ('nexus_nero_driver', 'nero_teleop_node', 'nero_teleop_node.py')
                   for name in executables):
                conflicts.append(int(process.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return conflicts


def trial(arm, read, component, delta_deg, speed, duration, emit):
    """One J move; take no retry/return/JS path, including on frozen J7."""
    before = read()
    validate_feedback(before, enabled=True)
    make_target(before['q_rad'], component, delta_deg)
    if before['controller']['motion_status'] != 0:
        raise RuntimeError('Controller has an unfinished motion; finish/stop it in the factory UI first')
    took_control, passed = False, False
    try:
        took_control = True
        mode_started = time.time()
        arm.set_motion_mode('j')
        arm.set_auto_set_motion_mode_enabled(False)
        arm.set_speed_percent(speed)
        # Wait for actual mode feedback rather than assuming the mode write applied.
        deadline = time.monotonic() + 1
        while True:
            state = read()
            validate_feedback(state, enabled=True)
            if max(abs(a-b) for a, b in zip(state['q_rad'], before['q_rad'])) > math.radians(.1):
                raise RuntimeError('Pose changed while switching control; no J7 target submitted')
            if (state['controller']['stamp_s'] >= mode_started
                    and state['controller']['ctrl_mode'] == 1
                    and state['controller']['mode_feedback'] == 1):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('CAN/J mode not confirmed; no J7 target submitted')
            time.sleep(.02)
        start = list(state['q_rad'])
        target = make_target(start, component, delta_deg)
        emit({'event': 'target', 'start_rad': start, 'target_rad': target, 'delta_deg': delta_deg})
        arm.move_j(target[:])
        deadline, settled_since = time.monotonic() + duration, None
        while time.monotonic() < deadline:
            state = read()
            validate_feedback(state, enabled=True)
            q = state['q_rad']
            emit({'event': 'sample', **state})
            if state['controller']['ctrl_mode'] != 1 or state['controller']['mode_feedback'] != 1:
                raise RuntimeError('CAN/J control mode changed during trial')
            if max(abs(a-b) for a, b in zip(q[:6], start[:6])) > math.radians(.5):
                raise RuntimeError('J1-J6 moved >0.5 degree; trial aborted')
            if abs(q[6] - start[6]) > math.radians(abs(delta_deg) + .5):
                raise RuntimeError('J7 moved beyond test envelope; trial aborted')
            if abs(q[6] - target[6]) < math.radians(.2):
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= .3:
                    passed = True
                    return {'result': 'passed', 'target_rad': target, 'actual_rad': q}
            else:
                settled_since = None
            time.sleep(.02)
        raise RuntimeError('J7 did not reach the single target within 0.2 degree; no retry/JS fallback')
    finally:
        if took_control and not passed:
            # A log write failure must not prevent requesting the stop.
            try:
                arm.electronic_emergency_stop()
            except Exception as exc:
                emit({'event': 'failure_stop_failed', 'error': str(exc)})
                print('Stop transmission failed; use the physical emergency stop.', file=sys.stderr)
            else:
                emit({'event': 'failure_stop_requested'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=Path(__file__).resolve().parents[1] / 'src/nexus_core/profiles/nero_dual_xhand.json')
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    parser.add_argument('--execute', action='store_true', help='Take CAN/J control and issue ONE relative J7 move; operator must stop NEXUS first')
    parser.add_argument('--delta-deg', type=float, default=1)
    parser.add_argument('--speed-percent', type=int, default=5)
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or not .5 <= args.duration <= 10
            or not 1 <= args.speed_percent <= 10
            or not math.isfinite(args.delta_deg) or not .5 <= abs(args.delta_deg) <= 1):
        parser.error('duration .5..10s, speed 1..10%, delta magnitude .5..1 degree required')
    raw = args.profile.read_bytes()
    profile = json.loads(raw)
    component = next(c for c in profile['components'] if c['name'] == args.side + '_arm')
    if (component['driver'] != 'nero_can' or len(component['joints']) != 7
            or component['unit'] != 'rad' or len(component['lower']) != 7
            or len(component['upper']) != 7):
        parser.error('A physical seven-joint nero_can component is required')
    if args.execute and active_nero_processes():
        parser.error('Nero ROS drivers are still running; stop the robot session in NEXUS first (closing the browser is insufficient)')
    channel = profile['adapter_config']['nero_can']['channels'][args.side]
    output = args.output or Path('logs') / ('nero_j7_' + args.side + '_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jsonl')
    output.parent.mkdir(parents=True, exist_ok=True)
    # Importing and constructing the SDK does not connect or send commands.
    from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
    import pyAgxArm
    arm = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT, channel=channel, interface='socketcan'))
    feedback = Feedback()
    arm.get_context().register_parser_packet_fun(feedback.receive)
    result, tx_count, exit_code = None, 0, 1
    with output.open('x', encoding='utf-8') as log:
        def emit(value):
            log.write(json.dumps({'monotonic_s': time.monotonic(), **value}, ensure_ascii=False, allow_nan=False) + '\n')
            log.flush()
        def send(frame, *pos, **kw):
            nonlocal tx_count
            event = {'event': 'tx', 'can_id': hex(frame.arbitration_id), 'data_hex': bytes(frame.data).hex()}
            if not args.execute:
                emit({**event, 'result': 'blocked_read_only'})
                raise RuntimeError('Transmit forbidden in observation mode')
            try:
                result = original_send(frame, *pos, **kw)
                if result is False:
                    raise RuntimeError('Transport returned False')
            except Exception as exc:
                emit({**event, 'result': 'failed', 'error': str(exc)})
                raise
            tx_count += 1
            emit({**event, 'result': 'transport_accepted_not_controller_ack'})
            return result
        emit({'event': 'start', 'mode': 'execute' if args.execute else 'read_only', 'channel': channel,
              'profile_sha256': hashlib.sha256(raw).hexdigest(), 'sdk_file': pyAgxArm.__file__, 'firmware_driver': 'DEFAULT'})
        print('Mode:', 'ONE planned J7 move' if args.execute else 'READ ONLY (all CAN sends blocked)', '| Log:', output)
        try:
            # Install the guard before starting SDK reader threads. Socket
            # setup performs no enable, CAN push, mode or feedback request.
            comm = arm.get_context().init_comm()
            original_send = comm.send
            comm.send = send
            arm.connect()
            read = lambda: snapshot(arm, feedback)
            deadline = time.monotonic() + 3
            while True:
                state = read()
                try:
                    validate_feedback(state)
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        emit({'event': 'feedback_unavailable', **state})
                        raise
                    time.sleep(.02)
            emit({'event': 'initial_feedback', **state})
            if args.execute:
                if active_nero_processes():
                    raise RuntimeError('A Nero ROS driver started during preflight; refusing motion')
                result = trial(arm, read, component, args.delta_deg, args.speed_percent, args.duration, emit)
            else:
                deadline = time.monotonic() + args.duration
                while time.monotonic() < deadline:
                    state = read()
                    validate_feedback(state)
                    emit({'event': 'sample', **state})
                    time.sleep(.05)
                # Good position feedback alone is insufficient to execute a
                # trial: a disabled arm can still publish all seven angles.
                try:
                    validate_feedback(state, enabled=True)
                    make_target(state['q_rad'], component, args.delta_deg)
                    if state['controller']['motion_status'] != 0:
                        raise RuntimeError('Controller has an unfinished motion')
                    if active_nero_processes():
                        raise RuntimeError('Nero ROS drivers are still running; stop the robot session in NEXUS before executing a trial')
                    ready, reason = True, None
                except RuntimeError as exc:
                    ready, reason = False, str(exc)
                result = {'result': 'observed_only', 'ready_for_trial': ready,
                          'blocking_reason': reason, 'last_feedback': state}
            exit_code = 0
        except (Exception, KeyboardInterrupt) as exc:
            result = {'result': 'failed', 'error': str(exc) or type(exc).__name__}
        finally:
            # disconnect closes sockets/threads only; no return motion or disable.
            arm.disconnect()
            emit({'event': 'summary', 'tx_count': tx_count, **result})
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print('Successful send means transport acceptance, not controller acknowledgement.')
    if args.execute:
        print('No automatic return or disable. Check the log for a failure_stop_requested event before performing factory fault recovery.')
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
