#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import os
import xml.etree.ElementTree as ET
import numpy as np
import heapq
import math
import time
from ament_index_python.packages import get_package_share_directory

class ScaraAStarFullAuto(Node):

    def __init__(self):
        super().__init__('scara_astar_auto')

        # --- CONFIG ROBOT & MAP ---
        self.L1, self.L2 = 0.25, 0.275
        self.H_total = 0.215 + 0.12 - 0.08
        self.z_stable = 0.15
        
        self.resolution = 0.02
        self.inflate_radius = 0.05 
        self.map_range = [-2.5, 2.5, -2.5, 2.5]
        
        # --- QOS & PUBLISHERS ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.traj_pub = self.create_publisher(JointTrajectory, '/scara_controller/joint_trajectory', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        self.get_logger().info("Đang khởi tạo bản đồ và tìm đường...")
        
        # 1. Load vật cản và tạo Grid
        self.walls = self.load_walls_from_world()
        self.grid = self.generate_occupancy_grid()
        
        # 2. Định nghĩa điểm Gốc (Home), điểm Bắt đầu A* và điểm Đích
        # Điểm gốc vật lý: J1=0, J2=0 -> x = L1+L2, y = 0
        home_world = (self.L1 + self.L2, 0.0) 
        goal_world  = (0.2, 0.12)

        # 3. Chạy A* từ Home đến Goal
        path = self.run_astar_logic(home_world, goal_world)
        
        if path:
            self.get_logger().info(f"Tìm thấy đường đi. Đang đưa robot về Home và chạy quỹ đạo...")
            self.send_trajectory_to_gazebo(path)
        else:
            self.get_logger().error("Không tìm thấy đường đi từ điểm gốc!")

    def inverse_kinematics(self, x, y, z):
        try:
            j3 = self.H_total - z
            d_sq = x**2 + y**2
            cos_j2 = (d_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            j2 = -math.acos(max(-1.0, min(1.0, cos_j2)))
            j1 = math.atan2(y, x) - math.atan2(self.L2 * math.sin(j2), self.L1 + self.L2 * math.cos(j2))
            return [j1, j2, j3]
        except: return None

    def load_walls_from_world(self):
        walls = []
        try:
            package_share = get_package_share_directory('myrobot')
            world_path = os.path.join(package_share, 'worlds', 'my_world')
            tree = ET.parse(world_path)
            root = tree.getroot()
            model = root.find(".//model[@name='ob_world']")
            m_pose = [float(x) for x in model.find("pose").text.split()]
            for link in model.findall("link"):
                collision = link.find("collision")
                if collision is None: continue
                size = [float(x) for x in collision.find(".//size").text.split()]
                l_pose = [float(x) for x in link.find("pose").text.split()]
                walls.append({
                    'cx': m_pose[0] + l_pose[0], 'cy': m_pose[1] + l_pose[1],
                    'w': size[0] + 2*self.inflate_radius, 
                    'h': size[1] + 2*self.inflate_radius,
                    'yaw': m_pose[5] + l_pose[5]
                })
        except Exception as e: self.get_logger().error(f"Lỗi load world: {e}")
        return walls

    def generate_occupancy_grid(self):
        xmin, xmax, ymin, ymax = self.map_range
        width = int((xmax - xmin) / self.resolution)
        height = int((ymax - ymin) / self.resolution)
        grid = np.zeros((height, width), dtype=np.uint8)
        xs = np.linspace(xmin, xmax, width); ys = np.linspace(ymin, ymax, height)
        xv, yv = np.meshgrid(xs, ys)
        for w in self.walls:
            dx, dy = xv - w['cx'], yv - w['cy']
            c, s = np.cos(-w['yaw']), np.sin(-w['yaw'])
            rx, ry = dx * c - dy * s, dx * s + dy * c
            mask = (np.abs(rx) <= w['w'] / 2) & (np.abs(ry) <= w['h'] / 2)
            grid[mask] = 1
        return grid

    def run_astar_logic(self, start_w, goal_w):
        def world_to_grid(x, y):
            gx = int((x - self.map_range[0]) / self.resolution)
            gy = int((y - self.map_range[2]) / self.resolution)
            return gx, gy
        sx, sy = world_to_grid(*start_w)
        gx, gy = world_to_grid(*goal_w)
        open_set = []; heapq.heappush(open_set, (0, (sx, sy)))
        came_from = {}; g_score = {(sx, sy): 0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == (gx, gy):
                path = []
                while current in came_from:
                    path.append(current); current = came_from[current]
                path.reverse(); return path
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < self.grid.shape[1] and 0 <= ny < self.grid.shape[0] and self.grid[ny, nx] == 0:
                    tentative_g = g_score[current] + math.hypot(dx, dy)
                    if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = tentative_g
                        f = tentative_g + math.hypot(gx - nx, gy - ny)
                        heapq.heappush(open_set, (f, (nx, ny)))
                        came_from[(nx, ny)] = current
        return None

    def send_trajectory_to_gazebo(self, path):
        msg = JointTrajectory()
        msg.joint_names = ['J1', 'J2', 'J3']
        marker_array = MarkerArray()

        # --- ĐIỂM 0: LUÔN LÀ ĐIỂM GỐC (HOME) ---
        # Điều này ép robot phải quay về tư thế 0,0,0 trước khi bắt đầu đi theo A*
        home_point = JointTrajectoryPoint()
        home_point.positions = [0.0, 0.0, self.H_total - self.z_stable]
        home_point.time_from_start.sec = 2 # Dành 2 giây để robot quay về Home từ vị trí bất kỳ
        home_point.time_from_start.nanosec = 0
        msg.points.append(home_point)
        
        # --- CÁC ĐIỂM TIẾP THEO TỪ A* ---
        step = max(1, len(path) // 20) 
        path_sampled = path[::step]
        if path[-1] not in path_sampled: path_sampled.append(path[-1])

        for i, (gx, gy) in enumerate(path_sampled):
            wx = gx * self.resolution + self.map_range[0]
            wy = gy * self.resolution + self.map_range[2]
            joints = self.inverse_kinematics(wx, wy, self.z_stable)
            if joints:
                point = JointTrajectoryPoint()
                point.positions = joints
                # Thời gian bắt đầu tính từ sau khi đã về Home (sau giây thứ 2)
                t = 2.0 + (i + 1) * 0.8
                point.time_from_start.sec = int(t)
                point.time_from_start.nanosec = int((t % 1) * 1e9)
                msg.points.append(point)

                m = Marker()
                m.header.frame_id = "world"; m.id = i; m.type = Marker.SPHERE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = wx, wy, self.z_stable
                m.scale.x = m.scale.y = m.scale.z = 0.03
                m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 1.0, 1.0, 0.0
                marker_array.markers.append(m)

        self.traj_pub.publish(msg)
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Đã gửi quỹ đạo (Bắt đầu từ điểm Gốc)!")

def main():
    rclpy.init()
    node = ScaraAStarFullAuto()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()