#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from angle_publisher_subscriber.angle_publisher import (
    AnglePublisher,
    DEFAULT_LOGICAL_VELOCITIES_DEG,
)


DEFAULT_ROUTINE = [
    {
        'name': 'home',
        'angles_deg': [0.0, 0.0, 0.0, 0.0, 0.0],
        'hold_seconds': 2.0,
    },
    {
        'name': 'reach_left',
        'angles_deg': [30.0, 35.0, 20.0, 0.0, 1.0],
        'hold_seconds': 2.0,
    },
    {
        'name': 'pick',
        'angles_deg': [30.0, 45.0, 35.0, 0.0, 0.0],
        'hold_seconds': 1.5,
    },
    {
        'name': 'lift',
        'angles_deg': [30.0, 15.0, 10.0, 0.0, 0.0],
        'hold_seconds': 1.5,
    },
    {
        'name': 'place_right',
        'angles_deg': [-30.0, 30.0, 20.0, 0.0, 0.0],
        'hold_seconds': 2.0,
    },
    {
        'name': 'release',
        'angles_deg': [-30.0, 30.0, 20.0, 0.0, 1.0],
        'hold_seconds': 1.5,
    },
]


class AngleRoutineNode(Node):
    def __init__(self):
        super().__init__('angle_routine')

        self.declare_parameter('loop', True)
        self.declare_parameter('startup_delay', 2.0)

        self.loop_enabled = bool(self.get_parameter('loop').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.routine = self._build_routine(DEFAULT_ROUTINE)

        first_step = self.routine[0]
        self.publisher_node = AnglePublisher(
            first_step['angles_deg'],
            first_step['velocities_deg'],
        )

        self.active_index = 0
        self.started = False
        self.segment_deadline = None
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            'Routine loaded with '
            f'{len(self.routine)} steps, startup_delay={self.startup_delay:.1f}s, '
            f'loop={"on" if self.loop_enabled else "off"}'
        )

    def _build_routine(self, steps):
        routine = []
        default_velocities = [
            DEFAULT_LOGICAL_VELOCITIES_DEG['base'],
            DEFAULT_LOGICAL_VELOCITIES_DEG['shoulder'],
            DEFAULT_LOGICAL_VELOCITIES_DEG['elbow'],
            DEFAULT_LOGICAL_VELOCITIES_DEG['wrist'],
            DEFAULT_LOGICAL_VELOCITIES_DEG['gripper'],
        ]

        for index, step in enumerate(steps):
            name = step.get('name', f'step_{index}')
            angles_deg = step['angles_deg']
            hold_seconds = float(step.get('hold_seconds', 1.0))
            velocities_deg = list(step.get('velocities_deg', default_velocities))

            if len(angles_deg) != 5:
                raise ValueError(
                    f'routine step "{name}" must provide 5 logical values: '
                    '[base, shoulder, elbow, wrist, gripper]'
                )
            if len(velocities_deg) != 5:
                raise ValueError(
                    f'routine step "{name}" must provide 5 velocity values when present'
                )
            if hold_seconds <= 0.0:
                raise ValueError(f'routine step "{name}" must have hold_seconds > 0')

            routine.append(
                {
                    'name': name,
                    'angles_deg': list(angles_deg),
                    'hold_seconds': hold_seconds,
                    'velocities_deg': velocities_deg,
                }
            )

        if not routine:
            raise ValueError('routine must contain at least one step')

        return routine

    def _tick(self):
        now = self.get_clock().now()

        if not self.started:
            elapsed = (now - self.start_time).nanoseconds / 1e9
            if elapsed < self.startup_delay:
                return

            self.publisher_node.calibrate()
            self._apply_step(self.active_index, now)
            self.started = True
            return

        if self.segment_deadline is None or now < self.segment_deadline:
            return

        next_index = self.active_index + 1
        if next_index >= len(self.routine):
            if not self.loop_enabled:
                self.get_logger().info('Routine complete. Holding final position.')
                self.timer.cancel()
                return
            next_index = 0

        self._apply_step(next_index, now)

    def _apply_step(self, step_index, now):
        step = self.routine[step_index]
        self.active_index = step_index
        self.segment_deadline = now + rclpy.duration.Duration(
            seconds=step['hold_seconds']
        )

        self.publisher_node.update_angles(
            step['angles_deg'],
            step['velocities_deg'],
        )

        self.get_logger().info(
            f'Step {step_index + 1}/{len(self.routine)}: {step["name"]} | '
            f'hold={step["hold_seconds"]:.1f}s | '
            f'angles={step["angles_deg"]}'
        )

    def destroy_node(self):
        self.publisher_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AngleRoutineNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.publisher_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node.publisher_node)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
