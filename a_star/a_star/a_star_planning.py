import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.interpolate import CubicSpline


class AStarPlannerNode(Node):

    def __init__(self):
        super().__init__('a_star_planner')
        self.get_logger().info('AStarPlannerNode has been started')

        # ROS publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacle_markers', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'grid_map', 10)

        # ROS subscribers
        self.create_subscription(PointStamped, 'clicked_point', self.clicked_point_callback, 10)

        # Start and goal points
        self.start_point = None
        self.goal_point = None

        # Default obstacle positions
        self.ox, self.oy = self.default_obstacles()

        # Initialize the grid map at the start
        self.publish_grid_map(self.ox, self.oy, 2.0, round(min(self.ox)), round(min(self.oy)), 35, 35)

    def clicked_point_callback(self, msg: PointStamped):
        if self.start_point is None:
            self.start_point = (msg.point.x, msg.point.y)
            self.get_logger().info(f'Updated start point to {self.start_point}')
        else:
            self.goal_point = (msg.point.x, msg.point.y)
            self.get_logger().info(f'Updated goal point to {self.goal_point}')
            self.plan_and_publish_path(self.start_point[0], self.start_point[1],
                                       self.goal_point[0], self.goal_point[1],
                                       self.ox, self.oy, 2.0, 1.0)
            self.start_point = self.goal_point  # Update start point to the last goal point

    def default_obstacles(self):
        ox, oy = [], []
        for i in range(-10, 60):
            ox.append(i)
            oy.append(-10.0)
        for i in range(-10, 60):
            ox.append(60.0)
            oy.append(i)
        for i in range(-10, 61):
            ox.append(i)
            oy.append(60.0)
        for i in range(-10, 61):
            ox.append(-10.0)
            oy.append(i)
        for i in range(-10, 40):
            ox.append(20.0)
            oy.append(i)
        for i in range(0, 40):
            ox.append(40.0)
            oy.append(60.0 - i)
        return ox, oy

    def plan_and_publish_path(self, sx, sy, gx, gy, ox, oy, resolution, rr):
        self.get_logger().info('Starting path planning...')

        def calc_heuristic(n1, n2):
            w = 1.0  # weight of heuristic
            d = w * math.hypot(n1[0] - n2[0], n1[1] - n2[1])
            return d

        def calc_grid_position(index, min_position):
            return index * resolution + min_position

        def calc_xy_index(position, min_pos):
            return round((position - min_pos) / resolution)

        def calc_grid_index(node):
            return (node[1] - min_y) * x_width + (node[0] - min_x)

        def verify_node(node):
            px = calc_grid_position(node[0], min_x)
            py = calc_grid_position(node[1], min_y)

            if px < min_x or py < min_y or px >= max_x or py >= max_y:
                return False

            # collision check
            for iox, ioy in zip(ox, oy):
                d = math.hypot(iox - px, ioy - py)
                if d <= rr:
                    return False
            return True

        def get_motion_model():
            # dx, dy, cost
            motion = [[1, 0, 1],
                      [0, 1, 1],
                      [-1, 0, 1],
                      [0, -1, 1],
                      [-1, -1, math.sqrt(2)],
                      [-1, 1, math.sqrt(2)],
                      [1, -1, math.sqrt(2)],
                      [1, 1, math.sqrt(2)]]
            return motion

        # Initialize grid map for A* planning
        min_x, min_y = round(min(ox)), round(min(oy))
        max_x, max_y = round(max(ox)), round(max(oy))
        x_width = round((max_x - min_x) / resolution) + 1
        y_width = round((max_y - min_y) / resolution) + 1

        # A* search grid based on motion model
        motion = get_motion_model()

        start_node = (calc_xy_index(sx, min_x),
                      calc_xy_index(sy, min_y), 0.0, -1)
        goal_node = (calc_xy_index(gx, min_x),
                     calc_xy_index(gy, min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                self.get_logger().info("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o][2] + calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current[0] == goal_node[0] and current[1] == goal_node[1]:
                self.get_logger().info("Goal found")
                goal_node = current
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(motion):
                node = (current[0] + motion[i][0],
                        current[1] + motion[i][1],
                        current[2] + motion[i][2], c_id)
                n_id = calc_grid_index(node)

                if not verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id][2] > node[2]:
                        open_set[n_id] = node

        rx, ry = [], []
        parent_index = goal_node[3]
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(calc_grid_position(n[0], min_x))
            ry.append(calc_grid_position(n[1], min_y))
            parent_index = n[3]

        self.get_logger().info(f"Path planned with {len(rx)} points")

        # Smooth the path using cubic spline interpolation
        rx_smooth, ry_smooth = self.smooth_path(rx[::-1], ry[::-1])

        # Publish the planned path as a nav_msgs/Path message
        planned_path = Path()
        planned_path.header.frame_id = 'base_link'
        for x, y in zip(rx_smooth, ry_smooth):  # Use smoothed path for visualization
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            planned_path.poses.append(pose)

        self.path_pub.publish(planned_path)
        self.get_logger().info('Path has been published')

        # Publish obstacles as markers
        self.publish_obstacles(ox, oy)

        # Publish grid map
        self.publish_grid_map(ox, oy, resolution, min_x, min_y, x_width, y_width)

    def smooth_path(self, rx, ry):
        if len(rx) < 3 or len(ry) < 3:
            return rx, ry  # Not enough points to smooth

        # Generate cubic spline for the path
        cs_x = CubicSpline(range(len(rx)), rx, bc_type='natural')
        cs_y = CubicSpline(range(len(ry)), ry, bc_type='natural')

        # Create a finer sampling along the path
        s_range = np.linspace(0, len(rx) - 1, num=len(rx) * 10)
        rx_smooth = cs_x(s_range)
        ry_smooth = cs_y(s_range)

        return rx_smooth, ry_smooth

    def publish_obstacles(self, ox, oy):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(zip(ox, oy)):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            # Convert x and y to float explicitly
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.5
            marker.id = i
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info('Obstacles have been published')

    def publish_grid_map(self, ox, oy, resolution, min_x, min_y, x_width, y_width):
        grid_map = OccupancyGrid()
        grid_map.header.frame_id = 'base_link'
        grid_map.info = MapMetaData()
        grid_map.info.resolution = resolution
        grid_map.info.width = x_width
        grid_map.info.height = y_width
        grid_map.info.origin.position.x = float(min_x)
        grid_map.info.origin.position.y = float(min_y)

        # Initialize the grid with unknown values (-1)
        grid_data = [-1] * (x_width * y_width)

        for x, y in zip(ox, oy):
            idx_x = int((x - min_x) / resolution)
            idx_y = int((y - min_y) / resolution)
            grid_index = idx_y * x_width + idx_x

            # Only assign if the grid_index is within the valid range
            if 0 <= grid_index < len(grid_data):
                grid_data[grid_index] = 100  # Mark obstacle cells with 100
            else:
                self.get_logger().warn(f'Grid index {grid_index} is out of range')

        grid_map.data = grid_data

        self.grid_pub.publish(grid_map)
        self.get_logger().info('Grid map has been published')

def main(args=None):
    rclpy.init(args=args)
    a_star_planner = AStarPlannerNode()

    try:
        rclpy.spin(a_star_planner)
    except KeyboardInterrupt:
        a_star_planner.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        a_star_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()