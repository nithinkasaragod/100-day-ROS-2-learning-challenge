import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist

from custom_interfaces2.action import MoveAndRotate  # Ensure this matches your .action package


class MoveAndRotateActionServer(Node):
    def __init__(self):
        super().__init__('move_and_rotate_action_server')

        self._action_server = ActionServer(
            self,
            MoveAndRotate,
            'move_and_rotate',
            self.execute_callback
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        linear_speed = goal_handle.request.linear_velocity
        angular_speed = goal_handle.request.angular_velocity
        duration = goal_handle.request.duration

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        feedback_msg = MoveAndRotate.Feedback()
        feedback_msg.current_time = 0.0

        timer_period = 0.1  # seconds
        elapsed_time = 0.0

        while elapsed_time < duration:
            self.cmd_vel_publisher.publish(twist)

            feedback_msg.current_time = elapsed_time
            goal_handle.publish_feedback(feedback_msg)

            await rclpy.sleep(timer_period)
            elapsed_time += timer_period

        # Stop the turtle
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        goal_handle.succeed()

        result = MoveAndRotate.Result()
        result.total_time = elapsed_time

        self.get_logger().info('Goal completed.')
        return result


def main():
    rclpy.init()
    node = MoveAndRotateActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
