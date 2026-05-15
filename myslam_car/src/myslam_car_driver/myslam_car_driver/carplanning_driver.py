#!/usr/-bin/env python
# encoding: utf-8
# 指定 Python 解释器路径和文件编码格式为 UTF-8

# ============================================================
# CarPlanning 底盘驱动节点
# 功能：通过串口与 STM32 下位机通信，完成以下任务：
#   1. 订阅 /cmd_vel 话题，将速度指令发送给电机
#   2. 读取编码器数据，计算并发布里程计 /odom
#   3. 读取 IMU 数据，发布 /imu/data_raw 和 /imu/mag
#   4. 发布 odom → base_link 的 TF 变换
#   5. 发布电池电压、固件版本等辅助信息
# ============================================================

# ==================== 导入标准库 ====================
import sys          # 系统相关功能（此处未实际使用，保留为兼容性）
import math         # 数学函数库（此处未直接调用，pi 从 math 单独导入）
import random       # 随机数库（此处未实际使用，保留为兼容性）
import threading    # 多线程库（此处未直接调用，Rosmaster 内部使用）
from math import pi          # 导入圆周率常量 π = 3.14159...
from time import sleep       # 导入延时函数（此处未实际使用）
from Rosmaster_Lib import Rosmaster  # 导入亚博智能底盘通信库，封装了与 STM32 的串口协议

# ==================== 导入 ROS2 库 ====================
import rclpy                          # ROS2 Python 客户端库的核心模块
from rclpy.node import Node           # ROS2 节点基类，所有节点都继承自它
from std_msgs.msg import String, Float32, Int32, Bool  # 标准消息类型：字符串、浮点数、整数、布尔值
from geometry_msgs.msg import Twist, TransformStamped  # Twist: 线速度+角速度; TransformStamped: 带时间戳的坐标变换
from sensor_msgs.msg import Imu, MagneticField, JointState  # Imu: 惯性测量单元数据; MagneticField: 磁力计数据; JointState: 关节状态
from nav_msgs.msg import Odometry     # 里程计消息：包含位置、姿态、速度信息
from rclpy.clock import Clock         # ROS2 时钟类（此处未直接使用，用 self.get_clock() 代替）
import tf2_ros                        # TF2 坐标变换库
from tf2_ros import TransformBroadcaster  # TF 广播器，用于发布坐标系之间的变换关系
import numpy as np                    # NumPy 数值计算库，此处用于三角函数计算
import tf_transformations             # TF 变换工具库，提供欧拉角↔四元数转换等功能

# ==================== 车型字典 ====================
# 定义亚博智能支持的车型编号映射
# R2 对应编号 5，X3/X1 对应编号 1，NONE 表示未识别
car_type_dic = {
    'R2': 5,       # R2 型底盘（麦克纳姆轮）
    'X3': 1,       # X3 型底盘（差速驱动），X1 也使用此编号
    'NONE': -1     # 未知车型
}


class carPlanning_driver(Node):
    """
    yahboomcar 底盘驱动节点类
    继承自 rclpy.node.Node，是一个标准的 ROS2 节点
    负责：接收速度指令 → 驱动电机，读取传感器 → 发布话题
    """

    def __init__(self, name):
        """
        节点初始化函数
        参数:
            name: 节点名称，传给 ROS2 注册，这里是 'driver_node'
        """
        super().__init__(name)  # 调用父类 Node 的构造函数，向 ROS2 注册节点名称
        global car_type_dic    # 声明使用全局变量 car_type_dic（车型字典）

        self.RA2DE = 180 / pi  # 弧度转角度的系数：1 rad = 57.2958°（此处未实际使用，保留为工具常量）

        # ==================== 声明并读取 ROS2 参数 ====================
        # declare_parameter: 向 ROS2 参数服务器注册参数，第二个值为默认值
        # get_parameter: 读取参数的实际值（可通过 launch 文件或命令行覆盖）

        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # 声明串口设备路径参数，默认 /dev/ttyUSB0
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value  # 读取串口路径
        print(f"Serial port: {self.serial_port}")  # 打印当前使用的串口路径，方便调试

        # 初始化 Rosmaster 底盘通信对象
        # Rosmaster 是亚博智能提供的 Python 库，封装了与 STM32 的串口通信协议
        # com 参数指定串口设备路径
        self.car = Rosmaster(com=self.serial_port)
        self.car.set_car_type(1)  # 设置车型为 1（X3/X1 差速驱动底盘）

        self.declare_parameter('car_type', 'X3')  # 声明车型参数，默认 X3
        self.car_type = self.get_parameter('car_type').get_parameter_value().string_value  # 读取车型字符串
        print(self.car_type)  # 打印车型

        self.declare_parameter('imu_link', 'imu_link')  # 声明 IMU 坐标系名称参数，默认 'imu_link'
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value  # 读取 IMU frame_id
        print(self.imu_link)  # 打印 IMU 坐标系名称

        self.declare_parameter('Prefix', "")  # 声明话题前缀参数，默认为空（多机器人场景下用于区分命名空间）
        self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value  # 读取前缀
        print(self.Prefix)  # 打印前缀

        self.declare_parameter('xlinear_limit', 1.0)  # 声明 X 方向（前后）线速度限制，单位 m/s，默认 1.0
        self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value  # 读取限制值
        print(self.xlinear_limit)  # 打印限制值

        self.declare_parameter('ylinear_limit', 1.0)  # 声明 Y 方向（左右）线速度限制，单位 m/s，默认 1.0
        self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value  # 读取限制值
        print(self.ylinear_limit)  # 打印限制值

        self.declare_parameter('angular_limit', 5.0)  # 声明角速度限制，单位 rad/s，默认 5.0
        self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value  # 读取限制值
        print(self.angular_limit)  # 打印限制值

        # ==================== 创建订阅者（Subscriber）====================
        # 订阅者负责接收其他节点发布的消息，触发对应的回调函数

        # 订阅 /cmd_vel 话题，消息类型 Twist（线速度 + 角速度）
        # Nav2 或遥控器会往这个话题发布运动指令
        # 队列深度为 1：只保留最新的一条指令，旧的丢弃（实时控制场景的标准做法）
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)

        # # 订阅 /RGBLight 话题，控制底盘 RGB 灯效，消息类型 Int32（灯效编号）
        # self.sub_RGBLight = self.create_subscription(Int32, "RGBLight", self.RGBLightcallback, 100)

        # 订阅 /Buzzer 话题，控制蜂鸣器开关，消息类型 Bool（True=响，False=关）
        self.sub_BUzzer = self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 100)

        # ==================== 创建发布者（Publisher）====================
        # 发布者负责向 ROS2 话题网络输出数据，供其他节点订阅

        # self.EdiPublisher = self.create_publisher(Float32, "edition", 100)          # 发布固件版本号
        self.volPublisher = self.create_publisher(Float32, "voltage", 100)          # 发布电池电压（V）
        self.staPublisher = self.create_publisher(JointState, "joint_states", 100)  # 发布关节状态（轮子角度等）
        self.velPublisher = self.create_publisher(Twist, "vel_raw", 50)             # 发布底盘原始速度（修正后的 ROS 坐标系速度）
        self.imuPublisher = self.create_publisher(Imu, "/imu/data_raw", 100)        # 发布 IMU 原始数据（加速度 + 角速度）
        self.magPublisher = self.create_publisher(MagneticField, "/imu/mag", 100)   # 发布磁力计数据
        self.odomPublisher = self.create_publisher(Odometry, "/odom", 100)          # 发布里程计数据（位置 + 速度）
        self.tf_broadcaster = TransformBroadcaster(self)  # 创建 TF 广播器，用于发布 odom → base_link 坐标变换

        # ==================== 创建定时器 ====================
        # 每 0.1 秒（10Hz）执行一次 pub_data 函数
        # pub_data 负责读取传感器数据并发布所有话题
        self.timer = self.create_timer(0.1, self.pub_data)

        # ==================== 初始化里程计状态变量 ====================
        self.edition = Float32()      # 固件版本消息对象
        self.edition.data = 1.0       # 默认版本号 1.0
        self.car.create_receive_threading()  # 启动 Rosmaster 库的后台接收线程，持续从串口读取 STM32 数据
        self.x = 0.0                  # 里程计累积 X 坐标（米），初始为原点
        self.y = 0.0                  # 里程计累积 Y 坐标（米），初始为原点
        self.th = 0.0                 # 里程计累积航向角（弧度），初始朝向 0
        self.last_time = self.get_clock().now()  # 记录上一次计算里程计的时间戳，用于计算 dt

    # ==================== 回调函数 ====================

    def cmd_vel_callback(self, msg):
        """
        /cmd_vel 话题的回调函数
        当 Nav2 或遥控器发布速度指令时，此函数被调用
        将 ROS 标准坐标系的速度指令转换为硬件坐标系，发送给 STM32 控制电机

        参数:
            msg: geometry_msgs/Twist 消息
                 msg.linear.x  → 前进/后退速度（正=前进，负=后退）
                 msg.linear.y  → 侧向速度（差速车通常为 0）
                 msg.angular.z → 旋转角速度（正=逆时针，负=顺时针）
        """
        if not isinstance(msg, Twist):  # 类型检查：确保收到的消息是 Twist 类型
            return  # 如果不是，直接返回，不做处理

        # ==================== Odom Fix v4 (Final) ===================
        # 坐标系映射说明：
        # ROS 标准坐标系：X 朝前，Y 朝左，Z 朝上（右手系）
        # 硬件坐标系：与 ROS 存在轴向映射关系（取决于底盘电机接线和轮子安装方向）
        #
        # 经过实测验证：
        #   - 物理小车运动方向和 RViz 显示一致，但都与 /cmd_vel 指令相反
        #   - 说明里程计逻辑正确，但发送给硬件的速度需要取反
        #
        # set_car_motion(vy, vx, vz) 的参数顺序是 (y轴速度, x轴速度, 角速度)
        # 注意：参数顺序不是 (vx, vy, vz)，这是亚博智能库的特殊定义
        # ============================================================

        vx_ros = msg.linear.x       # 提取 ROS 坐标系下的前进速度
        vy_ros = msg.linear.y       # 提取 ROS 坐标系下的侧向速度（差速车通常为 0）
        angular_ros = msg.angular.z  # 提取 ROS 坐标系下的旋转角速度

        # 发送给硬件：两个线速度都取反以修正方向
        # 参数顺序：set_car_motion(硬件Y速度, 硬件X速度, 角速度)
        self.car.set_car_motion(-vy_ros, -vx_ros, angular_ros)

    def RGBLightcallback(self, msg):
        """
        /RGBLight 话题的回调函数
        控制底盘 RGB LED 灯效

        参数:
            msg: std_msgs/Int32，灯效编号
        """
        if not isinstance(msg, Int32):  # 类型检查
            return
        # 连续发送 3 次以确保 STM32 收到（串口通信可能丢包）
        # set_colorful_effect(效果编号, 速度, parm=模式)
        for i in range(3):
            self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzercallback(self, msg):
        """
        /Buzzer 话题的回调函数
        控制蜂鸣器开关

        参数:
            msg: std_msgs/Bool，True=开启蜂鸣器，False=关闭蜂鸣器
        """
        if not isinstance(msg, Bool):  # 类型检查
            return
        if msg.data:  # 如果为 True
            for i in range(3):        # 连续发送 3 次开启指令
                self.car.set_beep(1)  # 1 = 开启蜂鸣器
        else:                         # 如果为 False
            for i in range(3):        # 连续发送 3 次关闭指令
                self.car.set_beep(0)  # 0 = 关闭蜂鸣器

    # ==================== 定时发布函数（核心）====================

    def pub_data(self):
        """
        定时器回调函数，每 0.1 秒执行一次（10Hz）
        功能：
            1. 从 STM32 读取编码器速度、IMU 数据、电池电压
            2. 进行坐标系转换（硬件坐标系 → ROS 标准坐标系）
            3. 计算里程计（位置累积）
            4. 发布所有话题：/odom, /imu/data_raw, /imu/mag, /vel_raw, /voltage, /edition
            5. 广播 TF 变换：odom → base_link
        """
        current_time = self.get_clock().now()  # 获取当前 ROS 时间戳

        # ==================== 创建消息对象 ====================
        imu = Imu()                    # IMU 消息：加速度 + 角速度 + 姿态（此处不填姿态）
        twist = Twist()                # 速度消息：用于发布 /vel_raw
        battery = Float32()            # 电池电压消息
        edition = Float32()            # 固件版本消息
        mag = MagneticField()          # 磁力计消息
        state = JointState()           # 关节状态消息（轮子角度）
        state.header.stamp = current_time.to_msg()  # 设置关节状态的时间戳
        state.header.frame_id = "joint_states"       # 设置关节状态的坐标系名称

        # ==================== 从 STM32 读取数据 ====================
        # Rosmaster 库在后台线程中持续接收串口数据并缓存
        # 以下 get_xxx 函数直接读取缓存值，不会阻塞

        edition.data = self.car.get_version() * 1.0    # 读取 STM32 固件版本号，乘 1.0 转为浮点数
        battery.data = self.car.get_battery_voltage() * 1.0  # 读取电池电压（V），乘 1.0 转为浮点数

        ax, ay, az = self.car.get_accelerometer_data()  # 读取加速度计数据（m/s²），返回 (x, y, z) 三轴
        gx, gy, gz = self.car.get_gyroscope_data()      # 读取陀螺仪数据（rad/s），返回 (x, y, z) 三轴
        mx, my, mz = self.car.get_magnetometer_data()    # 读取磁力计数据（μT），返回 (x, y, z) 三轴

        # 读取底盘运动数据：编码器反馈的实际速度
        # 返回值是硬件坐标系下的 (vx, vy, angular)
        vx_hw, vy_hw, angular_hw = self.car.get_motion_data()

        # ==================== 坐标系转换（硬件 → ROS）====================
        # 硬件坐标系和 ROS 标准坐标系存在轴向映射关系：
        #   ROS 的 X 轴（前进）对应硬件的 -Y 轴
        #   ROS 的 Y 轴（侧向）对应硬件的 -X 轴
        # 这个映射关系是通过 cmd_vel_callback 中的 set_car_motion(-vy, -vx, vz) 反推得到的
        vx_ros = -vy_hw         # ROS X 速度 = 硬件 -Y 速度（前进/后退）
        vy_ros = -vx_hw         # ROS Y 速度 = 硬件 -X 速度（侧向，差速车通常为 0）
        angular_ros = angular_hw  # 角速度方向一致，无需转换

        # ==================== 计算里程计 ====================
        # 里程计原理：用速度 × 时间间隔，累积得到位置
        # 这是"航位推算"(Dead Reckoning)，会有累积误差，后续由 EKF + GPS 修正

        dt = (current_time - self.last_time).nanoseconds / 1e9  # 计算时间间隔 dt（秒）= 纳秒差值 / 10^9
        self.last_time = current_time  # 更新上次时间为当前时间，供下一轮计算使用

        # 将机器人坐标系下的速度（vx_ros, vy_ros）转换到世界坐标系（odom 坐标系）
        # 使用二维旋转矩阵：
        #   delta_x = vx * cos(θ) - vy * sin(θ)
        #   delta_y = vx * sin(θ) + vy * cos(θ)
        # 其中 θ 是机器人当前的航向角（self.th）
        delta_x = (vx_ros * np.cos(self.th) - vy_ros * np.sin(self.th)) * dt   # X 方向位移增量（米）
        delta_y = (vx_ros * np.sin(self.th) + vy_ros * np.cos(self.th)) * dt   # Y 方向位移增量（米）
        delta_th = angular_ros * dt  # 航向角增量（弧度）

        self.x += delta_x    # 累加 X 坐标
        self.y += delta_y    # 累加 Y 坐标
        self.th += delta_th  # 累加航向角

        # ==================== 发布 TF 变换：odom → base_link ====================
        # TF 告诉 ROS2 系统中各坐标系之间的空间关系
        # odom → base_link 表示：机器人在里程计坐标系中的位置和朝向

        q = tf_transformations.quaternion_from_euler(0, 0, self.th)  # 将欧拉角(roll=0, pitch=0, yaw=self.th)转为四元数
        # 四元数是 ROS 中表示旋转的标准方式，避免万向锁问题

        t = TransformStamped()                   # 创建带时间戳的坐标变换消息
        t.header.stamp = current_time.to_msg()   # 设置时间戳为当前时间
        t.header.frame_id = "odom"               # 父坐标系：odom（里程计原点，固定不动）
        t.child_frame_id = "base_link"           # 子坐标系：base_link（机器人本体中心）
        t.transform.translation.x = self.x       # X 平移量 = 累积 X 坐标
        t.transform.translation.y = self.y       # Y 平移量 = 累积 Y 坐标
        t.transform.translation.z = 0.0          # Z 平移量 = 0（地面机器人不考虑高度）
        t.transform.rotation.x = q[0]            # 四元数 x 分量
        t.transform.rotation.y = q[1]            # 四元数 y 分量
        t.transform.rotation.z = q[2]            # 四元数 z 分量
        t.transform.rotation.w = q[3]            # 四元数 w 分量
        self.tf_broadcaster.sendTransform(t)      # 广播 TF 变换，其他节点（EKF、Nav2）可以查询到

        # ==================== 发布里程计话题：/odom ====================
        # Odometry 消息比 TF 更详细，包含速度信息和协方差矩阵
        # Nav2 和 robot_localization 都会订阅这个话题

        odom = Odometry()                                  # 创建里程计消息对象
        odom.header.stamp = current_time.to_msg()          # 设置时间戳
        odom.header.frame_id = "odom"                      # 数据所在的参考坐标系
        odom.child_frame_id = "base_link"                  # 被描述的物体（机器人）坐标系
        odom.pose.pose.position.x = self.x                 # 机器人 X 位置（米）
        odom.pose.pose.position.y = self.y                 # 机器人 Y 位置（米）
        odom.pose.pose.position.z = 0.0                    # 机器人 Z 位置（地面机器人为 0）
        odom.pose.pose.orientation.x = q[0]                # 姿态四元数 x
        odom.pose.pose.orientation.y = q[1]                # 姿态四元数 y
        odom.pose.pose.orientation.z = q[2]                # 姿态四元数 z
        odom.pose.pose.orientation.w = q[3]                # 姿态四元数 w
        odom.twist.twist.linear.x = vx_ros                 # 当前前进速度（m/s）
        odom.twist.twist.linear.y = vy_ros                 # 当前侧向速度（m/s，差速车通常为 0）
        odom.twist.twist.angular.z = angular_ros           # 当前旋转角速度（rad/s）
        self.odomPublisher.publish(odom)                    # 发布里程计消息到 /odom 话题

        # ==================== 发布 IMU 话题：/imu/data_raw ====================
        # IMU 原始数据，包含加速度和角速度（不含姿态估计）
        # robot_localization (EKF) 会订阅这个话题进行传感器融合

        imu.header.stamp = current_time.to_msg()           # 设置时间戳
        imu.header.frame_id = self.imu_link                # 设置 IMU 所在的坐标系（默认 'imu_link'）
        imu.linear_acceleration.x = ax * 1.0               # X 轴线性加速度（m/s²），乘 1.0 确保为浮点数
        imu.linear_acceleration.y = ay * 1.0               # Y 轴线性加速度（m/s²）
        imu.linear_acceleration.z = az * 1.0               # Z 轴线性加速度（m/s²），静止时应约为 9.8
        imu.angular_velocity.x = gx * 1.0                  # X 轴角速度（rad/s）
        imu.angular_velocity.y = gy * 1.0                  # Y 轴角速度（rad/s）
        imu.angular_velocity.z = gz * 1.0                  # Z 轴角速度（rad/s），绕 Z 轴旋转时有值

        # ==================== 发布磁力计话题：/imu/mag ====================
        # 磁力计数据，可用于航向角估计（需要磁偏角校正）

        mag.header.stamp = current_time.to_msg()           # 设置时间戳
        mag.header.frame_id = self.imu_link                # 设置坐标系（与 IMU 相同）
        mag.magnetic_field.x = mx * 1.0                    # X 轴磁场强度（Tesla）
        mag.magnetic_field.y = my * 1.0                    # Y 轴磁场强度（Tesla）
        mag.magnetic_field.z = mz * 1.0                    # Z 轴磁场强度（Tesla）

        # ==================== 发布原始速度话题：/vel_raw ====================
        # 修正后的 ROS 坐标系速度，供调试使用

        twist.linear.x = vx_ros       # 前进速度
        twist.linear.y = vy_ros       # 侧向速度
        twist.angular.z = angular_ros  # 旋转角速度
        self.velPublisher.publish(twist)  # 发布到 /vel_raw 话题

        # ==================== 发布其他话题 ====================
        self.imuPublisher.publish(imu)        # 发布 IMU 数据到 /imu/data_raw
        self.magPublisher.publish(mag)        # 发布磁力计数据到 /imu/mag
        self.volPublisher.publish(battery)    # 发布电池电压到 /voltage
        # self.EdiPublisher.publish(edition)    # 发布固件版本到 /edition


def main():
    """
    节点入口函数
    1. 初始化 ROS2 Python 客户端库
    2. 创建驱动节点实例
    3. 进入消息循环（spin），持续处理回调和定时器，直到节点被关闭
    """
    rclpy.init()                                    # 初始化 ROS2 通信基础设施
    driver = carPlanning_driver('driver_node')        # 创建节点实例，节点名为 'driver_node'
    rclpy.spin(driver)                               # 进入事件循环，阻塞在此处，直到 Ctrl+C 或 shutdown


'''if __name__ == '__main__':
    main()'''
# 注：上面的 if __name__ 被注释掉了
# 说明这个文件不是直接通过 python3 xxx.py 运行的
# 而是通过 ROS2 的 ros2 run 命令启动，入口点在 setup.py/setup.cfg 中配置指向 main()