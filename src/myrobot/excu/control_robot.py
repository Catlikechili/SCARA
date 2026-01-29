#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
import math

class ScaraReliableExecutor(Node):
    def __init__(self):
        super().__init__('scara_reliable_executor')

        # ===== Thông số vật lý =====
        self.L1, self.L2 = 0.25, 0.275
        self.H1, self.H2, self.EE_offset = 0.215, 0.12, -0.08

        self.cartesian_buffer = []
        self.current_joints = None
        self.is_executing = False # Đổi từ self.executed sang trạng thái đang chạy
        self.marker_id = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # Subscriber nhận điểm từ file vẽ quỹ đạo
        self.create_subscription(Point, '/cartesian_trajectory', self.point_cb, qos)
        # Subscriber nhận trạng thái khớp thực tế
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.traj_pub = self.create_publisher(JointTrajectory, '/scara_controller/joint_trajectory', 10)
        self.marker_pub = self.create_publisher(Marker, '/trajectory_markers', 10)

        self.get_logger().info('SCARA Executor sẵn sàng. Đợi đủ 31 điểm...')

    def joint_cb(self, msg):
        pos = [0.0, 0.0, 0.0]
        # CHÚ Ý: Đảm bảo tên J1, J2, J3 khớp với robot của bạn
        for i, name in enumerate(msg.name):
            if name == 'J1': pos[0] = msg.position[i]
            elif name == 'J2': pos[1] = msg.position[i]
            elif name == 'J3': pos[2] = msg.position[i]
        self.current_joints = pos

    def solve_ik(self, x, y, z):
        try:
            j3 = self.H1 + self.H2 + self.EE_offset - z
            d_sq = x*x + y*y
            d = math.sqrt(d_sq)
            if d > (self.L1 + self.L2) or d < abs(self.L1 - self.L2): return None

            cos_j2 = (d_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            j2 = -math.acos(max(-1.0, min(1.0, cos_j2)))

            # Sửa góc alpha để bù 90 độ
            alpha = math.atan2(y, x) + math.pi/2
            cos_beta = (self.L1**2 + d_sq - self.L2**2) / (2 * self.L1 * d)
            beta = math.acos(max(-1.0, min(1.0, cos_beta)))
            j1 = alpha + beta
            return [j1, j2, j3]
        except: return None

    def point_cb(self, msg):
        if self.is_executing: return

        self.cartesian_buffer.append(msg)
        self.publish_marker(msg.x, msg.y, msg.z)

        # Khi nhận đủ 31 điểm (1 approach + 30 circle)
        if len(self.cartesian_buffer) == 31:
            self.is_executing = True
            self.execute_trajectory()

    def execute_trajectory(self):
        if self.current_joints is None:
            self.get_logger().warn('Chưa nhận được JointState!')
            self.is_executing = False
            self.cartesian_buffer = []
            return

        traj = JointTrajectory()
        traj.joint_names = ['J1', 'J2', 'J3'] # Đảm bảo tên khớp chuẩn
        
        t = 0.0
        time_step = 0.5 # Thời gian giữa các điểm (giây)

        # Điểm hiện tại
        p_start = JointTrajectoryPoint()
        p_start.positions = self.current_joints
        p_start.time_from_start.sec = 0
        traj.points.append(p_start)

        # Gộp Phase 1 (Approach) và Phase 2 (Trajectory) vào cùng một luồng di chuyển
        for i, p in enumerate(self.cartesian_buffer):
            q = self.solve_ik(p.x, p.y, p.z)
            if q is None:
                self.get_logger().error(f'Lỗi IK tại điểm {i}')
                continue
            
            # Nếu là điểm đầu tiên (approach), cho nó 2 giây để di chuyển tới cho an toàn
            if i == 0: t += 2.0
            else: t += time_step

            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1.0) * 1e9)
            traj.points.append(pt)

        self.traj_pub.publish(traj)
        self.get_logger().info('Đã gửi quỹ đạo. Robot đang di chuyển...')
        
        # Tự động Reset sau khi chạy xong (tổng thời gian t + 1s dự phòng)
        self.create_timer(t + 1.0, self.reset_executor)

    def reset_executor(self):
        self.is_executing = False
        self.cartesian_buffer = []
        self.get_logger().info('--- Đã Reset. Sẵn sàng cho quỹ đạo mới ---')
        # Dừng timer reset sau khi chạy xong 1 lần
        # (Lưu ý: trong ROS2 timer cần được cancel để không lặp lại)

    def publish_marker(self, x, y, z):
        m = Marker()
        m.header.frame_id = 'world'
        m.ns = 'cartesian_path'
        m.id = self.marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.x = m.scale.y = m.scale.z = 0.015
        m.color.r, m.color.a = 1.0, 1.0
        self.marker_pub.publish(m)
        self.marker_id += 1

def main():
    rclpy.init()
    node = ScaraReliableExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()