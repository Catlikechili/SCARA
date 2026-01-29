import rclpy
from rclpy.node import Node
import os
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as mtransforms

from ament_index_python.packages import get_package_share_directory


class WallVisualizer(Node):

    def __init__(self):
        super().__init__('wall_visualizer')

        self.get_logger().info("Starting Wall Visualizer")

        # Load wall geometry
        self.walls = self.load_walls_from_world()

        if len(self.walls) == 0:
            self.get_logger().warn("No walls loaded")
        else:
            self.get_logger().info(f"Loaded {len(self.walls)} walls")

        # Plot
        self.plot_walls_with_inflate(r=0.01)

    # =====================================================
    # LOAD WALLS FROM .WORLD FILE
    # =====================================================
    def load_walls_from_world(self):
        walls = []

        try:
            package_share = get_package_share_directory('myrobot')
            world_path = os.path.join(package_share, 'worlds', 'my_world')

            tree = ET.parse(world_path)
            root = tree.getroot()

            model = root.find(".//model[@name='ob_world']")
            if model is None:
                self.get_logger().error("Model 'ob_world' not found")
                return []

            # Model pose (world frame)
            m_pose = [float(x) for x in model.find("pose").text.split()]
            mx, my, mz, mroll, mpitch, myaw = m_pose

            for link in model.findall("link"):
                collision = link.find("collision")
                if collision is None:
                    continue

                size_elem = collision.find(".//size")
                if size_elem is None:
                    continue

                l_pose = [float(x) for x in link.find("pose").text.split()]
                lx, ly, lz, lroll, lpitch, lyaw = l_pose

                size = [float(x) for x in size_elem.text.split()]
                w, h = size[0], size[1]

                wall = {
                    'cx': mx + lx,
                    'cy': my + ly,
                    'w': w,
                    'h': h,
                    'yaw': myaw + lyaw
                }

                walls.append(wall)

            return walls

        except Exception as e:
            self.get_logger().error(f"Failed to read world file: {e}")
            return []

    # =====================================================
    # INFLATE WALL (ROBOT RADIUS / SAFETY MARGIN)
    # =====================================================
    def inflate_wall(self, wall, r):
        return {
            'cx': wall['cx'],
            'cy': wall['cy'],
            'w': wall['w'] + 2 * r,
            'h': wall['h'] + 2 * r,
            'yaw': wall['yaw']
        }

    # =====================================================
    # PLOT WALLS ONLY (DEBUG MAP)
    # =====================================================
    def plot_walls_with_inflate(self, r=0.01):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_aspect('equal')

        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-2.5, 2.5)
        ax.grid(True, linestyle=':', alpha=0.6)

        for w in self.walls:
            # -------- Original wall --------
            rect = patches.Rectangle(
                (-w['w'] / 2, -w['h'] / 2),
                w['w'],
                w['h'],
                facecolor='saddlebrown',
                edgecolor='black',
                alpha=0.8
            )

            tf = mtransforms.Affine2D() \
                .rotate(w['yaw']) \
                .translate(w['cx'], w['cy'])

            rect.set_transform(tf + ax.transData)
            ax.add_patch(rect)

            # Wall center (debug)
            ax.plot(w['cx'], w['cy'], 'kx')

            # -------- Inflated wall --------
            wi = self.inflate_wall(w, r)

            rect_i = patches.Rectangle(
                (-wi['w'] / 2, -wi['h'] / 2),
                wi['w'],
                wi['h'],
                fill=False,
                edgecolor='red',
                linewidth=2,
                linestyle='--'
            )

            tf_i = mtransforms.Affine2D() \
                .rotate(wi['yaw']) \
                .translate(wi['cx'], wi['cy'])

            rect_i.set_transform(tf_i + ax.transData)
            ax.add_patch(rect_i)

        ax.set_title("Walls from Gazebo (.world)\nBrown = Wall | Red dashed = Inflated")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

        plt.show()


def main():
    rclpy.init()
    node = WallVisualizer()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
