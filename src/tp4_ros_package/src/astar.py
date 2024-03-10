import rospy
import heapq
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class AStarPlanner:
    def __init__(self):
        self.grid = None
        self.start_point = None
        self.goal_point = None
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        self.grid = msg

    def point_callback(self, point_stamped):
        if self.grid is None:
            rospy.logwarn("Occupancy grid not received yet")
            return

        if self.start_point is None:
            self.start_point = point_stamped.point
            rospy.loginfo("Start point received: {}".format(self.start_point))
        else:
            self.goal_point = point_stamped.point
            rospy.loginfo("Goal point received: {}".format(self.goal_point))
            self.find_path()

    def find_path(self):
        if self.start_point is None or self.goal_point is None:
            rospy.logerr("Start or goal point is missing")
            return

        start_node = self.point_to_node(self.start_point)
        goal_node = self.point_to_node(self.goal_point)

        open_list = []
        closed_set = set()

        heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)

            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                path = []
                while current_node:
                    path.append(self.node_to_point(current_node))
                    current_node = current_node.parent
                self.publish_path(path[::-1])
                return

            closed_set.add((current_node.x, current_node.y))

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                next_x, next_y = current_node.x + dx, current_node.y + dy

                if next_x < 0 or next_x >= len(self.grid.data) or next_y < 0 or next_y >= len(self.grid.data[0]) or self.grid.data[next_x + next_y * len(self.grid.data)] == 100:
                    continue

                if (next_x, next_y) in closed_set:
                    continue

                new_g = current_node.g + 1
                new_h = self.heuristic(Node(next_x, next_y), goal_node)

                next_node = Node(next_x, next_y, current_node)
                next_node.g = new_g
                next_node.h = new_h

                heapq.heappush(open_list, next_node)

        rospy.loginfo("No path found")

    def point_to_node(self, point):
        res = self.grid.info.resolution
        origin_x = self.grid.info.origin.position.x
        origin_y = self.grid.info.origin.position.y
        grid_x = int((point.x - origin_x) / res)
        grid_y = int((point.y - origin_y) / res)
        return Node(grid_x, grid_y)

    def node_to_point(self, node):
        res = self.grid.info.resolution
        origin_x = self.grid.info.origin.position.x
        origin_y = self.grid.info.origin.position.y
        x = origin_x + node.x * res
        y = origin_y + node.y * res
        return PointStamped(header=Header(), point=rospy.Point(x, y, 0))

    def heuristic(self, node, goal):
        return abs(node.x - goal.x) + abs(node.y - goal.y)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = self.grid.header.frame_id
        path_msg.poses = [PoseStamped(header=Header(), pose=Pose(position=point.point, orientation=Quaternion())) for point in path]
        self.path_pub.publish(path_msg)

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = 0
        self.h = 0
    
    def __lt__(self, other):
        return (self.g + self.h) < (other.g + other.h)

def main():
    rospy.init_node('a_star_planner')
    AStarPlanner()
    rospy.spin()

if __name__ == '__main__':
    main()
