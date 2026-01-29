import rclpy
from rclpy.node import Node
import os
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
from ament_index_python.packages import get_package_share_directory


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # ---------------- CONFIG ----------------
        self.resolution = 0.02          # m / cell
        self.inflate_radius = 0.03      # m
        self.map_range = [-2.5, 2.5, -2.5, 2.5]  # xmin, xmax, ymin, ymax
        L1 = 0.25
        L2 = 0.275
        q1 = 0.0  # Góc khớp 1 (radian)
        q2 = 0.0  # Góc khớp 2 (radian)

        # Tính toán vị trí thực của End-Effector (Điểm cuối khâu 2)
        start_x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
        start_y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)

        
        # ----------------------------------------

        # 1. Load walls
        self.walls = self.load_walls_from_world()

        # 2. Inflate walls
        self.inflate_walls()

        # 3. Generate grid
        self.generate_occupancy_grid()

        # 4. Define start / goal (WORLD)
        start_world = (start_x, start_y)
        goal_world  = ( 0.2,  0.12)

        # 5. Check start & goal
        res = self.check_start_goal(start_world, goal_world)
        if res is None:
            return

        start, goal = res

        # 6. Run A*
        path = self.astar(start, goal)
        if path is None:
            self.get_logger().error("A* failed: no path found")
            return

        # 7. Plot result
        self.plot_path(path, start, goal)

    # =====================================================
    # LOAD WALLS FROM WORLD
    # =====================================================
    def load_walls_from_world(self):
        walls = []

        package_share = get_package_share_directory('myrobot')
        world_path = os.path.join(package_share, 'worlds', 'my_world')

        tree = ET.parse(world_path)
        root = tree.getroot()

        model = root.find(".//model[@name='ob_world']")
        if model is None:
            self.get_logger().error("Model 'ob_world' not found")
            return []

        m_pose = [float(x) for x in model.find("pose").text.split()]
        mx, my, _, _, _, myaw = m_pose

        for link in model.findall("link"):
            collision = link.find("collision")
            if collision is None:
                continue

            size_elem = collision.find(".//size")
            if size_elem is None:
                continue

            l_pose = [float(x) for x in link.find("pose").text.split()]
            lx, ly, _, _, _, lyaw = l_pose

            size = [float(x) for x in size_elem.text.split()]
            w, h = size[0], size[1]

            walls.append({
                'cx': mx + lx,
                'cy': my + ly,
                'w': w,
                'h': h,
                'yaw': myaw + lyaw
            })

        self.get_logger().info(f"Loaded {len(walls)} walls")
        return walls

    # =====================================================
    # INFLATE WALLS
    # =====================================================
    def inflate_walls(self):
        for w in self.walls:
            w['w'] += 2 * self.inflate_radius
            w['h'] += 2 * self.inflate_radius

    # =====================================================
    # OCCUPANCY GRID
    # =====================================================
    def generate_occupancy_grid(self):
        xmin, xmax, ymin, ymax = self.map_range

        self.width = int((xmax - xmin) / self.resolution)
        self.height = int((ymax - ymin) / self.resolution)

        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

        xs = np.linspace(xmin, xmax, self.width)
        ys = np.linspace(ymin, ymax, self.height)
        xv, yv = np.meshgrid(xs, ys)

        for w in self.walls:
            dx = xv - w['cx']
            dy = yv - w['cy']

            c = np.cos(-w['yaw'])
            s = np.sin(-w['yaw'])

            rx = dx * c - dy * s
            ry = dx * s + dy * c

            mask = (np.abs(rx) <= w['w'] / 2) & (np.abs(ry) <= w['h'] / 2)
            self.grid[mask] = 1

        self.get_logger().info("Occupancy grid generated")

    # =====================================================
    # COORDINATE UTILS
    # =====================================================
    def world_to_grid(self, x, y):
        gx = int((x - self.map_range[0]) / self.resolution)
        gy = int((y - self.map_range[2]) / self.resolution)
        return gx, gy

    def in_map(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_free(self, gx, gy):
        return self.in_map(gx, gy) and self.grid[gy, gx] == 0

    # =====================================================
    # START / GOAL CHECK
    # =====================================================
    def check_start_goal(self, start_w, goal_w):
        sx, sy = self.world_to_grid(*start_w)
        gx, gy = self.world_to_grid(*goal_w)

        if not self.in_map(sx, sy):
            self.get_logger().error("START outside map")
            return None

        if not self.in_map(gx, gy):
            self.get_logger().error("GOAL outside map")
            return None

        if not self.is_free(sx, sy):
            self.get_logger().error("START in obstacle")
            return None

        if not self.is_free(gx, gy):
            self.get_logger().error("GOAL in obstacle")
            return None

        self.get_logger().info("Start & Goal valid")
        return (sx, sy), (gx, gy)

    # =====================================================
    # A* PLANNER
    # =====================================================
    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}

        moves = [(-1,0),(1,0),(0,-1),(0,1),
                 (-1,-1),(-1,1),(1,-1),(1,1)]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in moves:
                nx, ny = current[0] + dx, current[1] + dy

                if not self.is_free(nx, ny):
                    continue

                cost = math.hypot(dx, dy)
                tentative_g = g_score[current] + cost

                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    h = math.hypot(goal[0] - nx, goal[1] - ny)
                    f = tentative_g + h
                    heapq.heappush(open_set, (f, (nx, ny)))
                    came_from[(nx, ny)] = current

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # =====================================================
    # VISUALIZE
    # =====================================================
    def plot_path(self, path, start, goal):
        px = [p[0] * self.resolution + self.map_range[0] for p in path]
        py = [p[1] * self.resolution + self.map_range[2] for p in path]

        sx = start[0] * self.resolution + self.map_range[0]
        sy = start[1] * self.resolution + self.map_range[2]
        gx = goal[0] * self.resolution + self.map_range[0]
        gy = goal[1] * self.resolution + self.map_range[2]

        plt.figure(figsize=(8,8))
        plt.imshow(self.grid, origin='lower', extent=self.map_range, cmap='gray_r')
        plt.plot(px, py, 'r-', linewidth=2, label='Path')
        plt.plot(sx, sy, 'go', markersize=8, label='Start')
        plt.plot(gx, gy, 'bo', markersize=8, label='Goal')
        plt.legend()
        plt.title("A* Path Planning on Occupancy Grid")
        plt.axis('equal')
        plt.grid(True, linestyle=':')
        plt.show()


def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
