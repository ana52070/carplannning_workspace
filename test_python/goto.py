#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveExact(Node):
    def __init__(self, target_dist=20.0, speed=0.2):
        super().__init__('move_exact')
        self.target_dist = target_dist
        self.speed = speed

        self.start_x = None
        self.start_y = None
        self.done = False

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.get_logger().info(f'目标: 前进 {target_dist} m，速度 {speed} m/s')

    def odom_cb(self, msg):
        if self.done:
            return

        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y

        # 记录起点
        if self.start_x is None:
            self.start_x = cx
            self.start_y = cy
            self.get_logger().info(f'起点: ({cx:.3f}, {cy:.3f})')
            return

        # 计算已走距离
        dist = math.sqrt((cx - self.start_x)**2 + (cy - self.start_y)**2)
        self.get_logger().info(f'已走: {dist:.2f}m / 目标: {self.target_dist:.2f}m')

        if dist >= self.target_dist:
            # 到达目标，停止
            self.stop()
        else:
            # 继续前进
            self.move_forward()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        self.done = True
        self.get_logger().info('已到达目标距离，停止运动！')

def main(args=None):
    rclpy.init(args=args)
    node = MoveExact(target_dist=20.0, speed=0.2)  # 20米，速度0.2m/s
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
