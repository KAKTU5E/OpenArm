#!/usr/bin/env python3
import math
import struct
from abc import ABC, abstractmethod
from typing import Dict, Iterable, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from angle_publisher_subscriber.angle_publisher import JOINT_NAMES, WRIST_JOINT_NAME

try:
    import can
except ImportError as exc:  # pragma: no cover - depends on host environment
    can = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


DEFAULT_CAN_JOINT_NAMES = JOINT_NAMES + [WRIST_JOINT_NAME]

ODRIVE_CMD_SET_AXIS_STATE = 0x07
ODRIVE_CMD_SET_CONTROLLER_MODE = 0x0B
ODRIVE_CMD_SET_INPUT_POS = 0x0C
ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8
ODRIVE_CONTROL_MODE_POSITION_CONTROL = 3
ODRIVE_INPUT_MODE_PASSTHROUGH = 1

CUBEMARS_SERVO_SET_POS = 4
CUBEMARS_SERVO_SET_POS_SPD = 6


class CanBackend(ABC):
    def __init__(self, bridge: 'JointStateCanBridge') -> None:
        self.bridge = bridge

    def startup_messages(self) -> Iterable[can.Message]:
        return []

    @abstractmethod
    def joint_messages(
        self,
        joint_name: str,
        motor_id: int,
        position_rad: float,
        velocity_rad_s: float,
    ) -> Iterable[can.Message]:
        raise NotImplementedError


class CubeMarsServoBackend(CanBackend):
    def __init__(self, bridge: 'JointStateCanBridge') -> None:
        super().__init__(bridge)
        self.position_mode = bridge.get_parameter('cubemars_position_mode').value
        self.acceleration_erpm_s = int(bridge.get_parameter('cubemars_acceleration_erpm_s').value)
        self.velocity_to_erpm_scale = float(bridge.get_parameter('cubemars_velocity_to_erpm_scale').value)

    def joint_messages(
        self,
        joint_name: str,
        motor_id: int,
        position_rad: float,
        velocity_rad_s: float,
    ) -> Iterable[can.Message]:
        del joint_name
        position_deg = math.degrees(position_rad)
        position_raw = self._clamp_int32(position_deg * 10000.0)

        if self.position_mode == 'position_speed':
            velocity_erpm = self._clamp_uint16(abs(velocity_rad_s) * self.velocity_to_erpm_scale)
            acceleration_erpm_s = self._clamp_uint16(self.acceleration_erpm_s)
            yield can.Message(
                arbitration_id=(CUBEMARS_SERVO_SET_POS_SPD << 8) | motor_id,
                is_extended_id=True,
                data=struct.pack('>ihh', position_raw, velocity_erpm, acceleration_erpm_s),
            )
            return

        yield can.Message(
            arbitration_id=(CUBEMARS_SERVO_SET_POS << 8) | motor_id,
            is_extended_id=True,
            data=struct.pack('>i', position_raw),
        )

    @staticmethod
    def _clamp_uint16(value: float) -> int:
        return max(0, min(65535, int(round(value))))

    @staticmethod
    def _clamp_int32(value: float) -> int:
        return max(-(2 ** 31), min((2 ** 31) - 1, int(round(value))))


class ODriveCanSimpleBackend(CanBackend):
    def __init__(self, bridge: 'JointStateCanBridge') -> None:
        super().__init__(bridge)
        self.position_to_turns = float(bridge.get_parameter('odrive_position_to_turns_scale').value)
        self.velocity_to_turns = float(bridge.get_parameter('odrive_velocity_to_turns_scale').value)
        self.send_startup_commands = bool(bridge.get_parameter('odrive_send_startup_commands').value)

    def startup_messages(self) -> Iterable[can.Message]:
        if not self.send_startup_commands:
            return []

        messages: List[can.Message] = []
        for motor_id in self.bridge.motor_ids:
            messages.append(
                can.Message(
                    arbitration_id=self._odrive_arbitration_id(motor_id, ODRIVE_CMD_SET_CONTROLLER_MODE),
                    is_extended_id=False,
                    data=struct.pack(
                        '<II',
                        ODRIVE_CONTROL_MODE_POSITION_CONTROL,
                        ODRIVE_INPUT_MODE_PASSTHROUGH,
                    ),
                )
            )
            messages.append(
                can.Message(
                    arbitration_id=self._odrive_arbitration_id(motor_id, ODRIVE_CMD_SET_AXIS_STATE),
                    is_extended_id=False,
                    data=struct.pack('<I', ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL),
                )
            )
        return messages

    def joint_messages(
        self,
        joint_name: str,
        motor_id: int,
        position_rad: float,
        velocity_rad_s: float,
    ) -> Iterable[can.Message]:
        del joint_name
        input_pos_turns = position_rad * self.position_to_turns
        velocity_ff = self._clamp_int16(velocity_rad_s * self.velocity_to_turns * 1000.0)
        yield can.Message(
            arbitration_id=self._odrive_arbitration_id(motor_id, ODRIVE_CMD_SET_INPUT_POS),
            is_extended_id=False,
            data=struct.pack('<fhh', input_pos_turns, velocity_ff, 0),
        )

    @staticmethod
    def _odrive_arbitration_id(node_id: int, command_id: int) -> int:
        return (node_id << 5) | command_id

    @staticmethod
    def _clamp_int16(value: float) -> int:
        if math.isnan(value) or math.isinf(value):
            return 0
        return max(-32768, min(32767, int(round(value))))


class JointStateCanBridge(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_can_bridge')

        if can is None:
            raise RuntimeError(
                'python-can is not installed. Run: python3 -m pip install --user python-can'
            ) from IMPORT_ERROR

        self.declare_parameter('source_topic', 'joint_states')
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_interface', 'socketcan')
        self.declare_parameter('protocol', 'cubemars_servo')
        self.declare_parameter('joint_names', DEFAULT_CAN_JOINT_NAMES)
        self.declare_parameter('motor_ids', list(range(1, len(DEFAULT_CAN_JOINT_NAMES) + 1)))
        self.declare_parameter('send_only_named_joints', True)

        self.declare_parameter('cubemars_position_mode', 'position')
        self.declare_parameter('cubemars_acceleration_erpm_s', 40000)
        self.declare_parameter('cubemars_velocity_to_erpm_scale', 1.0)

        self.declare_parameter('odrive_position_to_turns_scale', 1.0 / (2.0 * math.pi))
        self.declare_parameter('odrive_velocity_to_turns_scale', 1.0 / (2.0 * math.pi))
        self.declare_parameter('odrive_send_startup_commands', False)

        self.source_topic = self.get_parameter('source_topic').value
        self.can_channel = self.get_parameter('can_channel').value
        self.can_interface = self.get_parameter('can_interface').value
        self.protocol = self.get_parameter('protocol').value
        self.joint_names: List[str] = list(self.get_parameter('joint_names').value)
        self.motor_ids: List[int] = [int(v) for v in self.get_parameter('motor_ids').value]
        self.send_only_named_joints = bool(self.get_parameter('send_only_named_joints').value)

        if len(self.motor_ids) != len(self.joint_names):
            raise ValueError('motor_ids and joint_names must have the same length')

        self.joint_to_motor_id: Dict[str, int] = dict(zip(self.joint_names, self.motor_ids))
        self.bus = can.interface.Bus(channel=self.can_channel, interface=self.can_interface)
        self.backend = self._create_backend(self.protocol)

        self.subscription = self.create_subscription(
            JointState,
            self.source_topic,
            self.joint_state_callback,
            10,
        )

        self._send_startup_messages()
        self.get_logger().info(
            f'Bridging {self.source_topic} to {self.can_channel} with {self.can_interface} '
            f'using protocol={self.protocol}.'
        )

    def _create_backend(self, protocol: str) -> CanBackend:
        if protocol == 'cubemars_servo':
            return CubeMarsServoBackend(self)
        if protocol == 'odrive_cansimple':
            return ODriveCanSimpleBackend(self)
        raise ValueError(
            f'Unsupported protocol "{protocol}". Use cubemars_servo or odrive_cansimple.'
        )

    def _send_startup_messages(self) -> None:
        for message in self.backend.startup_messages():
            self.bus.send(message)

    def joint_state_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            self.get_logger().warning('Received JointState without names or positions; skipping.')
            return

        joint_names = list(msg.name)
        joint_positions = list(msg.position)
        joint_velocities = list(msg.velocity) if msg.velocity else []

        expected_names = self.joint_names if self.send_only_named_joints else joint_names

        for joint_name in expected_names:
            source_index = self._find_joint_index(joint_names, joint_name)
            if source_index is None:
                if self.send_only_named_joints:
                    self.get_logger().warning(f'Joint "{joint_name}" missing from JointState; skipping.')
                continue

            motor_id = self.joint_to_motor_id.get(joint_name)
            if motor_id is None:
                self.get_logger().warning(f'No motor ID configured for joint "{joint_name}"; skipping.')
                continue

            position_rad = joint_positions[source_index]
            velocity_rad_s = joint_velocities[source_index] if source_index < len(joint_velocities) else 0.0

            for frame in self.backend.joint_messages(
                joint_name=joint_name,
                motor_id=motor_id,
                position_rad=position_rad,
                velocity_rad_s=velocity_rad_s,
            ):
                self.bus.send(frame)

    @staticmethod
    def _find_joint_index(joint_names: List[str], target_joint_name: str):
        try:
            return joint_names.index(target_joint_name)
        except ValueError:
            return None

    def destroy_node(self) -> bool:
        try:
            self.bus.shutdown()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
