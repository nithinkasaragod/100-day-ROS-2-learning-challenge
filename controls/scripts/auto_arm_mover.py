#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class AutoArmMover(Node):
    def __init__(self):
        super().__init__('auto_arm_mover')

        # Create publisher to arm_controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Send command every 3 seconds
        self.timer = self.create_timer(3.0, self.send_trajectory)

        # Keep track of target position
        self.position = 0.0
        self.direction = 1.0

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1']   # ⚠️ Use your joint name

        point = JointTrajectoryPoint()
        self.position += 1.0 * self.direction

        # Reverse direction if too far
        if self.position > 1.5:
            self.direction = -1.0
        elif self.position < -1.5:
            self.direction = 1.0

        # Position goal
        point.positions = [self.position]

        # Velocity goal (optional)
        point.velocities = [0.5]

        # Reach target in 2 seconds
        point.time_from_start.sec = 2

        traj.points.append(point)

        self.publisher_.publish(traj)
        self.get_logger().info(f"Sent trajectory: {point.positions}")


def main(args=None):
    rclpy.init(args=args)
    node = AutoArmMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
