import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interfaces.action import Patrol  # Update this import if needed
import time

class PatrolActionClient(Node):

    def __init__(self):
        super().__init__('patrol_action_client')
        self._action_client = ActionClient(self, Patrol, 'patrol')

    def send_continuous_patrol(self, start_radius=0.5, end_radius=0.5, increment=0.4):
        self._action_client.wait_for_server()

        while rclpy.ok():
            radius = start_radius
            while radius <= end_radius and rclpy.ok():
                self.get_logger().info(f'Sending patrol goal with radius: {radius:.2f}')

                goal_msg = Patrol.Goal()
                goal_msg.radius = radius  # Robot moves in circle centered at origin

                send_goal_future = self._action_client.send_goal_async(
                    goal_msg,
                    feedback_callback=self.feedback_callback
                )

                rclpy.spin_until_future_complete(self, send_goal_future)
                goal_handle = send_goal_future.result()

                if not goal_handle.accepted:
                    self.get_logger().warn(f'Goal with radius {radius:.2f} was rejected.')
                    break

                get_result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, get_result_future)
                result = get_result_future.result().result

                if result.success:
                    self.get_logger().info(f'Patrol with radius {radius:.2f} completed successfully!')
                else:
                    self.get_logger().warn(f'Patrol with radius {radius:.2f} failed.')

                radius += increment
                # No sleep to keep the robot moving continuously

            self.get_logger().info('Restarting patrol from smallest radius.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Time left: {feedback.time_left:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    client = PatrolActionClient()
    try:
        client.send_continuous_patrol(start_radius=0.5, end_radius=0.5, increment=0.4)
    except KeyboardInterrupt:
        client.get_logger().info('Stopped by user.')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

