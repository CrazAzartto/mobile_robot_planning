#!/usr/bin/env python3
"""
goal_publisher.py
=================
One-shot utility node that publishes a single PoseStamped message to /goal_pose.
Used by full_system.launch.py to set the navigation goal automatically.
Can also be run standalone:
  ros2 run robot_simulation goal_publisher --ros-args -p goal_x:=8.0 -p goal_y:=0.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        self.declare_parameter('goal_x',     8.0)
        self.declare_parameter('goal_y',     0.0)
        self.declare_parameter('goal_yaw',   0.0)
        self.declare_parameter('delay_sec',  2.0)   # wait before publishing
        self.declare_parameter('frame_id', 'odom')

        self._gx  = self.get_parameter('goal_x').value
        self._gy  = self.get_parameter('goal_y').value
        self._yaw = self.get_parameter('goal_yaw').value
        self._fid = self.get_parameter('frame_id').value
        delay     = self.get_parameter('delay_sec').value

        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._timer = self.create_timer(delay, self._publish_goal)
        self.get_logger().info(
            f'GoalPublisher: will publish goal ({self._gx}, {self._gy}) '
            f'in {delay} s on /goal_pose')

    def _publish_goal(self):
        import math
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._fid
        msg.pose.position.x = self._gx
        msg.pose.position.y = self._gy
        msg.pose.position.z = 0.0

        # Yaw → quaternion
        cy = math.cos(self._yaw * 0.5)
        sy = math.sin(self._yaw * 0.5)
        msg.pose.orientation.w = cy
        msg.pose.orientation.z = sy
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0

        self.pub.publish(msg)
        self.get_logger().info(
            f'Goal published: ({self._gx:.2f}, {self._gy:.2f}) in frame "{self._fid}"')

        # Cancel timer — publish only once
        self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
