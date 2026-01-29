#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from tf2_ros import Buffer, TransformListener

class ScaraTfVerify(Node):
    def __init__(self):
        super().__init__('scara_cartesian_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # 1. Cấu hình Frame (Hãy đảm bảo EE_FRAME đúng với robot của bạn)
        self.WORLD_FRAME = 'base_link'
        self.EE_FRAME = 'ee_marker' # Hoặc 'ee_link', 'link_3' tùy robot

        # 2. Khởi tạo TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 3. Publisher với QoS Reliable
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(JointTrajectory, '/scara_controller/joint_trajectory', qos_profile)

        # 4. Lưu trữ mục tiêu để đối chiếu
        self.target_now = [0.0, 0.0, 0.0]

        self.declare_parameter('target_xyz', [0.3, 0.2, 0.1])
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for p in params:
            if p.name == 'target_xyz':
                self.target_now = list(p.value)
                joints = self.calculate_ik(self.target_now[0], self.target_now[1], self.target_now[2])
                if joints:
                    self.send_cmd(joints)
                    # Chờ robot di chuyển xong (2.5s) rồi mới đọc TF
                    self.create_timer(2.5, self.verify_position)
                    return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def calculate_ik(self, x, y, z):
        L1, L2 = 0.25, 0.275
        H_total = 0.215 + 0.12 - 0.08
        try:
            j3 = H_total - z
            d_sq = x**2 + y**2
            cos_j2 = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
            j2 = -math.acos(max(-1.0, min(1.0, cos_j2)))
            j1 = math.atan2(y, x) - math.atan2(L2 * math.sin(j2), L1 + L2 * math.cos(j2))
            return [j1, j2, j3]
        except: return None

    def send_cmd(self, joints):
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = ''
        msg.joint_names = ['J1', 'J2', 'J3']
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = 2
        msg.points = [point]
        self.publisher_.publish(msg)

    def verify_position(self):
        try:
            # Đọc vị trí từ TF - Đây là dữ liệu PHẢN HỒI từ Gazebo
            t = self.tf_buffer.lookup_transform(self.WORLD_FRAME, self.EE_FRAME, rclpy.time.Time())
            p = t.transform.translation
            
            # Tính sai số (Error)
            err_x = p.x - self.target_now[0]
            err_y = p.y - self.target_now[1]
            err_z = p.z - self.target_now[2]

            print("\n" + "="*50)
            print(f" ĐỐI CHIẾU VỊ TRÍ (MỤC TIÊU VS THỰC TẾ)")
            print(f" Trục |   Mục tiêu   |   Thực tế (TF)  |   Sai số")
            print(f"  X   |   {self.target_now[0]:.4f}     |     {p.x:.4f}      |   {err_x:.6f}")
            print(f"  Y   |   {self.target_now[1]:.4f}     |     {p.y:.4f}      |   {err_y:.6f}")
            print(f"  Z   |   {self.target_now[2]:.4f}     |     {p.z:.4f}      |   {err_z:.6f}")
            print("="*50 + "\n")

        except Exception as e:
            self.get_logger().error(f"Lỗi đọc TF: {e}")

def main():
    rclpy.init()
    rclpy.spin(ScaraTfVerify())
    rclpy.shutdown()

if __name__ == '__main__':
    main()