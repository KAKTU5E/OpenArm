#!/usr/bin/env python3
import math
import os
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'Cycloidal_Simplified_for_FEA-1_Revolute-28',
    'Cycloidal_Simplified_for_FEA-2_Revolute-30',
    'Secondary-Joint-Cartilage_Revolute-11',
    'Openhand1.0_Hand_simplifiedmain-v1_Revolute-33',
    'Openhand1.0_Hand_simplifiedmain-v1_Revolute-21',
    'L1-v1_Revolute-23',
    'L1-v1-1_Revolute-24',
    'L2-v1_Revolute-25',
    'L2-v1-1_Revolute-26',
]
WRIST_JOINT_NAME = 'dog_wrist_joint'

DEFAULT_LOGICAL_VELOCITIES_DEG = {
    'base': 5.0,
    'shoulder': 2.5,
    'elbow': 2.5,
    'wrist': 10.0,
    'gripper': 0.05,
}

# Angle limit infrastructure.
# Leave entries as None until you are ready to enforce limits.
# Each value should be (min_deg, max_deg) for that logical joint.
LOGICAL_ANGLE_LIMITS_DEG = {
    'base': None,
    'shoulder': None,
    'elbow': None,
    'wrist': None,
    'gripper_open_close': None,
}


class AnglePublisher(Node):
    def __init__(self, angles, logical_velocities_deg):
        super().__init__('angle_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer_period_sec = 0.1

        self.joint_names = JOINT_NAMES + [WRIST_JOINT_NAME]
        self.joint_positions = [0.0] * len(self.joint_names)
        self.target_joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.is_calibrated = False

        self.logical_angles = list(angles)
        self.logical_velocities_deg = list(logical_velocities_deg)

        self._apply_angles(self.logical_angles, self.logical_velocities_deg)
        self.joint_positions = list(self.target_joint_positions)
        self.timer = self.create_timer(self.timer_period_sec, self.publish_once)

    def _apply_angles(self, angles, logical_velocities_deg=None):
        if logical_velocities_deg is not None:
            self.logical_velocities_deg = list(logical_velocities_deg)

        self.logical_angles = list(angles)
        self.target_joint_positions = self._logical_angles_to_joint_positions(self.logical_angles)
        self.joint_velocities = self._logical_velocities_to_joint_velocities(self.logical_velocities_deg)

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'Angles(deg): base={self.logical_angles[0]:.2f}, '
            f'shoulder={self.logical_angles[1]:.2f}, '
            f'elbow={self.logical_angles[2]:.2f}, '
            f'wrist={self.logical_angles[3]:.2f}, '
            f"gripper={'OPEN' if self.logical_angles[4] == 1.0 else 'CLOSED'}"
        )
        self.get_logger().info(
            f'Velocities(deg/s): base={self.logical_velocities_deg[0]:.2f}, '
            f'shoulder={self.logical_velocities_deg[1]:.2f}, '
            f'elbow={self.logical_velocities_deg[2]:.2f}, '
            f'wrist={self.logical_velocities_deg[3]:.2f}, '
            f'gripper={self.logical_velocities_deg[4]:.2f}'
        )
        self.get_logger().info('=' * 60)

    def _logical_angles_to_joint_positions(self, logical_angles):
        base_theta = math.radians(logical_angles[0])
        blue_theta1 = math.radians(logical_angles[1])
        white_theta2 = math.radians(logical_angles[2])
        wrist_spin_theta = math.radians(logical_angles[3])
        open_close = logical_angles[4]

        if open_close == 1.0:
            wrist_theta3 = math.radians(-90)
            finger_a1 = math.radians(38)
            finger_a2 = math.radians(-30)
        else:
            wrist_theta3 = math.radians(25)
            finger_a1 = math.radians(45)
            finger_a2 = math.radians(45)

        return [
            base_theta,
            blue_theta1,
            white_theta2,
            wrist_theta3,
            -wrist_theta3,
            finger_a1,
            -finger_a1,
            finger_a2,
            -finger_a2,
            wrist_spin_theta,
        ]

    def _logical_velocities_to_joint_velocities(self, logical_velocities_deg):
        base_vel = math.radians(logical_velocities_deg[0])
        shoulder_vel = math.radians(logical_velocities_deg[1])
        elbow_vel = math.radians(logical_velocities_deg[2])
        wrist_vel = math.radians(logical_velocities_deg[3])
        gripper_vel = math.radians(logical_velocities_deg[4])

        return [
            base_vel,
            shoulder_vel,
            elbow_vel,
            gripper_vel,
            gripper_vel,
            gripper_vel,
            gripper_vel,
            gripper_vel,
            gripper_vel,
            wrist_vel,
        ]

    def _step_joint_positions_toward_targets(self):
        max_steps = [velocity * self.timer_period_sec for velocity in self.joint_velocities]

        for index, target in enumerate(self.target_joint_positions):
            current = self.joint_positions[index]
            delta = target - current
            max_step = abs(max_steps[index])

            if max_step <= 0.0 or abs(delta) <= max_step:
                self.joint_positions[index] = target
                continue

            self.joint_positions[index] = current + math.copysign(max_step, delta)

    def update_angles(self, angles, logical_velocities_deg=None):
        """Update joint positions and (optionally) velocities live without restarting."""
        _validate_angles_against_limits(angles)
        self._apply_angles(angles, logical_velocities_deg)

    def calibrate(self):
        self.is_calibrated = True
        self.get_logger().info('Motors calibrated. Angle commands are now enabled.')

    def publish_once(self):
        if not self.is_calibrated:
            return

        ##self._step_joint_positions_toward_targets()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = []
        self.publisher_.publish(msg)


def _parse_angle_velocity_values(parts):
    if len(parts) not in (4, 5, 8, 10):
        raise ValueError(f'expected 4, 5, 8, or 10 values, got {len(parts)}')

    values = [float(p) for p in parts]
    if len(values) in (4, 8):
        angles = values[:3] + [0.0] + [values[3]]
        velocity_values = values[4:]
    else:
        angles = values[:5]
        velocity_values = values[5:]

    if angles[4] not in (0.0, 1.0):
        raise ValueError(f'open/close must be 0 or 1, got {angles[4]}')

    _validate_angles_against_limits(angles)

    logical_velocities = (
        velocity_values
        if len(values) in (8, 10)
        else [
            DEFAULT_LOGICAL_VELOCITIES_DEG['base'],
            DEFAULT_LOGICAL_VELOCITIES_DEG['shoulder'],
            DEFAULT_LOGICAL_VELOCITIES_DEG['elbow'],
            DEFAULT_LOGICAL_VELOCITIES_DEG['wrist'],
            DEFAULT_LOGICAL_VELOCITIES_DEG['gripper'],
        ]
    )
    return angles, logical_velocities


def _validate_angles_against_limits(angles):
    logical_angles = {
        'base': angles[0],
        'shoulder': angles[1],
        'elbow': angles[2],
        'wrist': angles[3],
        'gripper_open_close': angles[4],
    }

    for joint_name, angle in logical_angles.items():
        limits = LOGICAL_ANGLE_LIMITS_DEG.get(joint_name)
        if limits is None:
            continue
        min_deg, max_deg = limits
        if angle < min_deg or angle > max_deg:
            raise ValueError(
                f'{joint_name} angle {angle} deg is outside limits [{min_deg}, {max_deg}]'
            )


def parse_cli_inputs():
    cli_parts = sys.argv[1:]
    if not cli_parts:
        return (
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [
                DEFAULT_LOGICAL_VELOCITIES_DEG['base'],
                DEFAULT_LOGICAL_VELOCITIES_DEG['shoulder'],
                DEFAULT_LOGICAL_VELOCITIES_DEG['elbow'],
                DEFAULT_LOGICAL_VELOCITIES_DEG['wrist'],
                DEFAULT_LOGICAL_VELOCITIES_DEG['gripper'],
            ],
        )

    try:
        angles, logical_velocities = _parse_angle_velocity_values(cli_parts)
    except ValueError as exc:
        print(f'Error: {exc}')
        print('Usage: ros2 run <pkg> angle_publisher')
        print('  Usage 1: <base> <shoulder> <elbow> <wrist> <open/close>')
        print('  Usage 2: <base> <shoulder> <elbow> <wrist> <open/close> <v_base> <v_shoulder> <v_elbow> <v_wrist> <v_gripper>')
        print('  No wrist: 45 90 135 1')
        print('  w/ wrist motion: 45 90 135 180 1')
        print('  DIRECTIONS: <CW/CCW>= (-,+) , <U/D>=(-,+) , <U/D>=(+,-) , <OPEN/CLOSE>=(1/0)')
        print('  If velocities are omitted, per-joint defaults are used.')
        print('Angles are in degrees; velocities are in degrees/second.')
        print('When velocities are omitted, defaults come from DEFAULT_LOGICAL_VELOCITIES_DEG.')
        sys.exit(1)

    return angles, logical_velocities


def _print_interactive_directions():
    print('\n' + '=' * 70)
    print('  Interactive Calibration + Angle Input')
    print('  Press Enter once to calibrate and enable publishing.')
    print('  Usage 1: <base> <shoulder> <elbow> <wrist> <open/close>')
    print('  Usage 2: <base> <shoulder> <elbow> <wrist> <open/close> <v_base> <v_shoulder> <v_elbow> <v_wrist> <v_gripper>')
    print('  No wrist: 45 90 135 1')
    print('  w/ wrist motion: 45 90 135 180 1')
    print('  DIRECTIONS: <CW/CCW>= (-,+) , <U/D>=(-,+) , <U/D>=(+,-) , <OPEN/CLOSE>=(1/0)')
    print('  If velocities are omitted, per-joint defaults are used.')
    print('  Press Ctrl+C to quit.')
    print('=' * 70 + '\n')


def interactive_loop(node):
    """
    Continuously prompts the user for manual angle commands.
    Accepted inputs are parsed by _parse_angle_velocity_values().
    """
    _print_interactive_directions()

    while True:
        try:
            raw = input('Enter values: ').strip()
        except (EOFError, KeyboardInterrupt):
            print('\n[interactive_loop] Exiting - Ctrl+C received.')
            break

        if not raw and not node.is_calibrated:
            node.calibrate()
            print('  Motors calibrated. You can now enter angles.\n')
            continue

        if not raw:
            print('  (empty input - skipping)')
            continue

        if not node.is_calibrated:
            print('  Press Enter to calibrate before sending angles.\n')
            continue

        parts = raw.split()
        try:
            new_angles, new_velocities = _parse_angle_velocity_values(parts)
        except ValueError as exc:
            print(f'  Error: {exc}. Try again.')
            continue

        node.update_angles(new_angles, new_velocities)

        vel_text = (
            f'v_base={new_velocities[0]} v_shoulder={new_velocities[1]} '
            f'v_elbow={new_velocities[2]} v_wrist={new_velocities[3]} '
            f'v_gripper={new_velocities[4]} deg/s'
        )

        print(
            f"  Published: base={new_angles[0]} shoulder={new_angles[1]} elbow={new_angles[2]} "
            f"wrist={new_angles[3]} "
            f"{'OPEN' if new_angles[4] == 1.0 else 'CLOSED'} | {vel_text}\n"
        )


def open_interactive_terminal():
    """
    Opens a new x-terminal-emulator window running this script in --interactive mode.
    The new terminal writes inputs back to the parent process via named pipe.
    """
    pipe_path = tempfile.mktemp(prefix='angle_pipe_')
    os.mkfifo(pipe_path)

    script_path = os.path.abspath(__file__)
    cmd = (
        f'source /opt/ros/humble/setup.bash && '
        f'source ~/OpenMutt/openarm_ws/install/setup.bash && '
        f'python3 {script_path} --interactive {pipe_path}'
    )
    subprocess.Popen(['x-terminal-emulator', '-e', f'bash -c "{cmd}; exec bash"'])

    return pipe_path


def wait_for_startup_delay():
    print('\nWaiting briefly for motor startup before opening controls.\n')
    time.sleep(3.0)
    print('Opening interactive controls.\n')


def pipe_reader_loop(node, pipe_path):
    """
    Reads lines sent from the interactive terminal via named pipe and applies updates.
    """
    try:
        with open(pipe_path, 'r') as pipe:
            for line in pipe:
                line = line.strip()
                if not line:
                    continue
                if line == '__CALIBRATE__':
                    if not node.is_calibrated:
                        node.calibrate()
                    continue
                if not node.is_calibrated:
                    continue
                try:
                    new_angles, new_velocities = _parse_angle_velocity_values(line.split())
                except ValueError:
                    continue
                node.update_angles(new_angles, new_velocities)
    finally:
        os.unlink(pipe_path)


def main(args=None):
    # --interactive mode: launched by x-terminal-emulator
    if '--interactive' in sys.argv:
        idx = sys.argv.index('--interactive')
        if idx + 1 >= len(sys.argv):
            print('Error: --interactive requires a pipe path argument')
            return

        pipe_path = sys.argv[idx + 1]
        interactive_is_calibrated = False
        _print_interactive_directions()

        try:
            with open(pipe_path, 'w', buffering=1) as pipe:
                while True:
                    try:
                        raw = input('Enter values: ').strip()
                    except (EOFError, KeyboardInterrupt):
                        print('\n[interactive_loop] Exiting - Ctrl+C received.')
                        break

                    if not raw:
                        if not interactive_is_calibrated:
                            pipe.write('__CALIBRATE__\n')
                            interactive_is_calibrated = True
                            print('  Calibration request sent.\n')
                        else:
                            print('  (empty input - skipping)')
                        continue

                    parts = raw.split()
                    try:
                        new_angles, new_velocities = _parse_angle_velocity_values(parts)
                    except ValueError as exc:
                        print(f'  Error: {exc}. Try again.')
                        continue

                    pipe.write(' '.join(parts) + '\n')

                    print(
                        f"  Sent: base={new_angles[0]} shoulder={new_angles[1]} elbow={new_angles[2]} "
                        f"wrist={new_angles[3]} "
                        f"{'OPEN' if new_angles[4] == 1.0 else 'CLOSED'} | "
                        f"v_base={new_velocities[0]} v_shoulder={new_velocities[1]} "
                        f"v_elbow={new_velocities[2]} v_wrist={new_velocities[3]} "
                        f"v_gripper={new_velocities[4]} deg/s\n"
                    )
        except Exception:
            pass
        return

    angles, logical_velocities = parse_cli_inputs()
    rclpy.init(args=args)
    node = AnglePublisher(angles, logical_velocities)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    wait_for_startup_delay()
    pipe_path = open_interactive_terminal()
    pipe_thread = threading.Thread(target=pipe_reader_loop, args=(node, pipe_path), daemon=True)
    pipe_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
