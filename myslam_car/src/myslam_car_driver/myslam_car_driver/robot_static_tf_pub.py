#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
# ���� tf_transformations ����������Ԫ��
import tf_transformations

class StaticTFPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.make_transforms()

    def make_transforms(self):
        # ==================== IMU �任�����޸ģ� ====================
        # base_link -> imu_link
        t_imu = TransformStamped()
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'imu_link'
        t_imu.transform.translation.x = -0.030
        t_imu.transform.translation.y = -0.030
        t_imu.transform.translation.z = 0.050
        
        # �������� Z ����ת 180 �� (math.pi) ������ IMU �ķ���װ
        q = tf_transformations.quaternion_from_euler(0, 0, math.pi) 
        t_imu.transform.rotation.x = q[0]
        t_imu.transform.rotation.y = q[1]
        t_imu.transform.rotation.z = q[2]
        t_imu.transform.rotation.w = q[3]
        # =========================================================

        # base_link -> laser
        t_laser = TransformStamped()
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = -0.0098
        t_laser.transform.translation.y = 0.0264
        t_laser.transform.translation.z = 0.140
        # 根据雷达的实际安装方向，绕 Z 轴旋转-90度 (-math.pi / 2)
        q_laser = tf_transformations.quaternion_from_euler(0, 0, -math.pi / 2)
        t_laser.transform.rotation.x = q_laser[0]
        t_laser.transform.rotation.y = q_laser[1]
        t_laser.transform.rotation.z = q_laser[2]
        t_laser.transform.rotation.w = q_laser[3]
        
        # base_link -> base_footprint
        t_footprint = TransformStamped()
        t_footprint.header.frame_id = 'base_link'
        t_footprint.child_frame_id = 'base_footprint'
        t_footprint.transform.translation.x = 0.0
        t_footprint.transform.translation.y = 0.0
        t_footprint.transform.translation.z = 0.0
        t_footprint.transform.rotation.x = 0.0
        t_footprint.transform.rotation.y = 0.0
        t_footprint.transform.rotation.z = 0.0
        t_footprint.transform.rotation.w = 1.0

        # ... (�������ӵ� TF ���ֲ���) ...
        # base_link -> front_left_wheel
        t_fl_wheel = TransformStamped()
        t_fl_wheel.header.frame_id = 'base_link'
        t_fl_wheel.child_frame_id = 'front_left_wheel'
        t_fl_wheel.transform.translation.x = -0.125
        t_fl_wheel.transform.translation.y = 0.090
        t_fl_wheel.transform.translation.z = 0.035
        t_fl_wheel.transform.rotation.x = 0.0
        t_fl_wheel.transform.rotation.y = 0.0
        t_fl_wheel.transform.rotation.z = 0.0
        t_fl_wheel.transform.rotation.w = 1.0

        # base_link -> front_right_wheel
        t_fr_wheel = TransformStamped()
        t_fr_wheel.header.frame_id = 'base_link'
        t_fr_wheel.child_frame_id = 'front_right_wheel'
        t_fr_wheel.transform.translation.x = 0.125
        t_fr_wheel.transform.translation.y = 0.090
        t_fr_wheel.transform.translation.z = 0.035
        t_fr_wheel.transform.rotation.x = 0.0
        t_fr_wheel.transform.rotation.y = 0.0
        t_fr_wheel.transform.rotation.z = 0.0
        t_fr_wheel.transform.rotation.w = 1.0

        # base_link -> back_left_wheel
        t_bl_wheel = TransformStamped()
        t_bl_wheel.header.frame_id = 'base_link'
        t_bl_wheel.child_frame_id = 'back_left_wheel'
        t_bl_wheel.transform.translation.x = -0.125
        t_bl_wheel.transform.translation.y = -0.100
        t_bl_wheel.transform.translation.z = 0.035
        t_bl_wheel.transform.rotation.x = 0.0
        t_bl_wheel.transform.rotation.y = 0.0
        t_bl_wheel.transform.rotation.z = 0.0
        t_bl_wheel.transform.rotation.w = 1.0

        # base_link -> back_right_wheel
        t_br_wheel = TransformStamped()
        t_br_wheel.header.frame_id = 'base_link'
        t_br_wheel.child_frame_id = 'back_right_wheel'
        t_br_wheel.transform.translation.x = 0.125
        t_br_wheel.transform.translation.y = -0.100
        t_br_wheel.transform.translation.z = 0.035
        t_br_wheel.transform.rotation.x = 0.0
        t_br_wheel.transform.rotation.y = 0.0
        t_br_wheel.transform.rotation.z = 0.0
        t_br_wheel.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform([
            t_imu, t_laser, 
            t_footprint,
            t_fl_wheel, t_fr_wheel, 
            t_bl_wheel, t_br_wheel
        ])

def main(args=None):
    rclpy.init(args=args)
    # ��װ tf_transformations: sudo apt install ros-foxy-tf-transformations
    # ���� pip install transforms3d
    node = StaticTFPublisher("robot_static_tf_publisher")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()