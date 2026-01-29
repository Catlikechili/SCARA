#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

class ScaraAutoTenPoint(Node):
    def __init__(self):
        super().__init__('scara_auto_node')
        
        # 1. Thông số Robot
        self.L1, self.L2 = 0.25, 0.275
        self.H_total = 0.215 + 0.12 - 0.08
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # 2. Publishers
        self.traj_pub = self.create_publisher(JointTrajectory, '/scara_controller/joint_trajectory', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Đợi 2 giây để Gazebo và Controller nhận diện node
        self.timer = self.create_timer(2.0, self.execute_and_exit)
        self.get_logger().info('Node sẵn sàng. Đang đợi 2s để kết nối...')

    def inverse_kinematics(self, x, y, z):
        try:
            j3 = self.H_total - z
            d_sq = x**2 + y**2
            cos_j2 = (d_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            j2 = -math.acos(max(-1.0, min(1.0, cos_j2)))
            j1 = math.atan2(y, x) - math.atan2(self.L2 * math.sin(j2), self.L1 + self.L2 * math.cos(j2))
            return [j1, j2, j3]
        except: return None

    def execute_and_exit(self):
        self.destroy_timer(self.timer)
        
        # --- TẠO DANH SÁCH 10 ĐIỂM ---
        cx, cy, r = 0.2, 0.2, 0.1
        z_start, z_end = 0.13, 0.20
        num_points = 20
        
        msg = JointTrajectory()
        msg.joint_names = ['J1', 'J2', 'J3']
        
        # QUAN TRỌNG: Để stamp = 0 nếu dùng với Gazebo Controller đơn giản
        # Hoặc dùng self.get_clock().now().to_msg() nhưng phải rất chuẩn
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        
        marker_array = MarkerArray()

        for i in range(num_points):
            theta = (2 * math.pi) * (i / (num_points - 1))
            px = cx + r * math.cos(theta)
            py = cy + r * math.sin(theta)
            pz = z_start + (z_end - z_start) * (i / (num_points - 1))
            
            joints = self.inverse_kinematics(px, py, pz)
            if joints:
                point = JointTrajectoryPoint()
                point.positions = joints
                point.velocities = [0.0, 0.0, 0.0] 
                
                # Thời gian từ lúc bắt đầu (tăng dần 1.5s mỗi điểm)
                t_from_start = (i + 1) * 0.5
                point.time_from_start.sec = int(t_from_start)
                point.time_from_start.nanosec = int((t_from_start % 1) * 1e9)
                msg.points.append(point)

                # Tạo Marker
                m = Marker()
                m.header.frame_id = "world"
                m.id = i
                m.type = Marker.SPHERE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, pz
                m.scale.x = m.scale.y = m.scale.z = 0.02
                m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.65, 0.0 # Đỏ
                marker_array.markers.append(m)

        # Gửi bản tin
        self.get_logger().info(f"Đang gửi {len(msg.points)} điểm tới Controller...")
        self.traj_pub.publish(msg)
        self.marker_pub.publish(marker_array)
        
        # ĐỢI: Gazebo cần thời gian để "tiêu hóa" danh sách điểm này
        time.sleep(2.0) 
        
        self.get_logger().info("Đã gửi xong. Thoát node.")
        raise SystemExit

def main():
    rclpy.init()
    node = ScaraAutoTenPoint()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit): pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()