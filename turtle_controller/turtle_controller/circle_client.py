import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interfaces.action import MoveAndRotate
from geometry_msgs.msg import Pose2D
import math

class CircleClient(Node):
    def __init__(self):
        super().__init__('circle_client')
        self._action_client = ActionClient(self, MoveAndRotate, 'move_and_rotate')
        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Generate circle points
        cx, cy, r = 5.5, 5.5, 2.5
        steps = 36  # 10° per step
        targets = []

        for i in range(steps):
            angle = 2 * math.pi * i / steps
            x = cx + r * math.cos(angle)
            y = cy + r * math.sin(angle)
            targets.append(Pose2D(x=x, y=y, theta=angle))

        goal_msg = MoveAndRotate.Goal()
        goal_msg.targets = targets
        goal_msg.rotate_radius = 0.5
        goal_msg.rotate_cycles = 1

        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)

    def feedback_cb(self, feedback):
        f = feedback.feedback
        self.get_logger().info(f'Moving to point {f.current_index}')

def main(args=None):
    rclpy.init(args=args)
    node = CircleClient()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
