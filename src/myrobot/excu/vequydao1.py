#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

class ScaraThreePointControl(Node):
    def __init__(self):
        super().__init__('scara_three_point_node')
        
        # 1. Thông số Robot & QoS
        self.L1, self.L2 = 0.25, 0.275
        self.H_total = 0.215 + 0.12 - 0.08
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. Publishers
        self.traj_pub = self.create_publisher(JointTrajectory, '/scara_controller/joint_trajectory', qos_reliable)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # 3. Kịch bản 3 điểm (X, Y, Z)
        self.points_sequence = [
            [0.3, 0.1, 0.2],  # Điểm 1: Vươn ra
            [0.2, 0.3, 0.2],  # Điểm 2: Xoay ngang
            [0.2, 0.3, 0.15]  # Điểm 3: Hạ thấp trục Z
        ]
        
        # Chạy sau 2 giây để Gazebo kịp load
        self.create_timer(2.0, self.run_sequence)

    def inverse_kinematics(self, x, y, z):
        try:
            j3 = self.H_total - z
            d_sq = x**2 + y**2
            cos_j2 = (d_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            j2 = -math.acos(max(-1.0, min(1.0, cos_j2)))
            j1 = math.atan2(y, x) - math.atan2(self.L2 * math.sin(j2), self.L1 + self.L2 * math.cos(j2))
            return [j1, j2, j3]
        except: return None

    def publish_marker(self, x, y, z, m_id, color_rgb):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = m_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x, marker.scale.y, marker.scale.z = 0.03, 0.03, 0.03
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color_rgb
        self.marker_pub.publish(marker)

    def send_to_point(self, coords, duration_sec):
        x, y, z = coords
        joints = self.inverse_kinematics(x, y, z)
        if joints:
            msg = JointTrajectory()
            msg.joint_names = ['J1', 'J2', 'J3']
            msg.header.stamp.sec, msg.header.stamp.nanosec = 0, 0
            
            point = JointTrajectoryPoint()
            point.positions = joints
            point.time_from_start.sec = duration_sec
            msg.points = [point]
            
            self.traj_pub.publish(msg)
            return True
        return False

    def run_sequence(self):
        self.destroy_timer(self.run_sequence)
        
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)] # Đỏ, Xanh lá, Xanh dương
        
        for i, pt in enumerate(self.points_sequence):
            self.get_logger().info(f"Đang di chuyển tới Điểm {i+1}: {pt}")
            
            # Gửi Marker lên RViz trước
            self.publish_marker(pt[0], pt[1], pt[2], i, colors[i])
            
            # Gửi lệnh điều khiển
            self.send_to_point(pt, 3) # Mỗi điểm đi trong 3 giây
            
            # Chờ robot thực hiện xong mới gửi điểm tiếp theo (Tránh đè lệnh)
            time.sleep(4.0) 

        self.get_logger().info("Hoàn thành chuỗi 3 điểm!")

def main():
    rclpy.init()
    node = ScaraThreePointControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()