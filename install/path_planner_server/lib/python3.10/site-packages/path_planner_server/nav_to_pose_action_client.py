import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav2_msgs.action import NavigateToPose


class MyActionClient(Node):

    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.subscriber = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def pose_callback(self, msg : PointStamped):
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = msg.header.frame_id
        goal_msg.pose.pose.position = msg.point

        self.send_goal(goal_msg)


    def send_goal(self, goal_msg):
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)



    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        current_pose = feedback_msg.feedback.current_pose
        self.get_logger().info(
            'Received feedback: {0}'.format(current_pose))


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
