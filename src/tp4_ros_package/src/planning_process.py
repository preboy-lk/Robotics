#!/usr/bin/env python
import rospy
import numpy as np
import sys
import random
import heapq
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Path

from std_msgs.msg import Header
import tf

# Definition of class
class Planning_Node:
    def __init__(self, node_name):
    	self.nname = node_name     #Giving a name for the ROS node

    	rospy.init_node(self.nname, anonymous=True) #ROS node initialization
	print 'Start ...'
	self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 10)
	self.shortest_path = rospy.Publisher('/path', Path, queue_size = 10)

	point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.get_point_callback) # ROS topic subscription
	map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map_callback)
	self.point1 = None
	self.point2 = None
	self.grid_map = None
	self.map_info = None
	rospy.spin() # Initiate the ROS loop

    def get_map_callback(self, map_info):
	map = np.array(map_info.data).reshape((map_info.info.height,map_info.info.width))
	self.map_info = map_info
	self.grid_map = map.T

    def coordinate_to_index(self, point):
	index_x = int(np.round((point.x - self.map_info.info.origin.position.x)/self.map_info.info.resolution))
	index_y = int(np.round((point.y - self.map_info.info.origin.position.y)/self.map_info.info.resolution))
	return [index_x, index_y]

    def index_to_coordinate(self, point):
	coordinate_x = point.x*self.map_info.info.resolution + self.map_info.info.origin.position.x
	coordinate_y = point.y*self.map_info.info.resolution + self.map_info.info.origin.position.y
	return [coordinate_x, coordinate_y]

    def publish_path(self, shortest_path):
	path = Path()
	for point in shortest_path:
	    pose = PoseStamped()
	    pose.header.frame_id = 'map'
	    pose.pose.position.x = point[0]
	    pose.pose.position.y = point[1]
	    pose.pose.position.z = 0

	    pose.header.seq = path.header.seq +1
	    path.header.frame_id = 'map'
	    path.header.stamp = rospy.Time.now()
	    pose.header.stamp = path.header.stamp
	    path.poses.append(pose)
	self.shortest_path.publish(path)

    def publish_marker_array(self):
	marker_array = MarkerArray()
	for idx, point in enumerate([self.point1, self.point2]):
    	    marker = Marker()
    	    marker.header.frame_id = 'map'
	    marker.id = idx
    	    marker.type = Marker.POINTS
            marker.action = Marker.ADD
    	    marker.scale.x = 0.5
    	    marker.scale.y = 0.5
    	    marker.color.a = 1.0 
    	    marker.color.r , marker.color.g, marker.color.b = (0.0, 0.0, 1.0) if idx == 0 else (1.0, 0.0, 0.0)
    	    marker.points.append(point)
	    marker_array.markers.append(marker)
	self.marker_pub.publish(marker_array)

    def cost_to_go(self, node, goal):
        return (node.x - goal.x)**2 + abs(node.y - goal.y)**2

    def get_point_callback(self, point_info):
	if not self.point1:
	    self.point1 = point_info.point
	    rospy.loginfo('Coordinates 1: x = %f y = %f' %(self.point1.x,self.point1.y))

	else:
	    self.point2 = point_info.point
	    rospy.loginfo('Coordinates 2: x = %f y = %f' %(self.point2.x,self.point2.y))

	if self.point1 and self.point2:
	    self.publish_marker_array()
	    self.find_path()
	
	    self.point1 = None
	    self.point2 = None

    def find_path(self):
	index_x, index_y = self.coordinate_to_index(self.point1)
	start_point = Node(index_x, index_y)
	
	index_x, index_y = self.coordinate_to_index(self.point2)
	goal_point = Node(index_x, index_y)

        open_list = []
        closed_set = set() # Use set because it is the good way to check whether a specific element is contained in set
	start_point.g = 0
        heapq.heappush(open_list, start_point)

        while open_list:
            current_point = heapq.heappop(open_list)
	    closed_set.add((current_point.x, current_point.y))

            if current_point.x == goal_point.x and current_point.y == goal_point.y:
		print 'Finding Path Successfully !!!'
		path = []
                while current_point:
                    path.append(self.index_to_coordinate(current_point))
                    current_point = current_point.parent
                self.publish_path(path[::-1])
                return


            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                next_x, next_y = current_point.x + dx, current_point.y + dy

                if next_x < 0 or next_x >= self.map_info.info.height or next_y < 0 or next_y >= self.map_info.info.width or self.grid_map[next_x,next_y] != 0: # Check all the possible positions the robot can go. Xem lai cho nay neu bi loi; hinh ko vuong 
		    continue
                if (next_x, next_y) in closed_set:
                    continue		

                new_g = current_point.g + 1
                new_h = self.cost_to_go(Node(next_x, next_y), goal_point)

                next_point = Node(next_x, next_y, current_point)
                next_point.g = new_g
                next_point.h = new_h

                heapq.heappush(open_list, next_point)

		#next_point = Node(next_x,next_y,current_point)
		#gnew = current_point.g + 1 # Because of L1 distance
		#fnew = gnew + self.cost_to_go(next_point, goal_point)
	        #next_point.f = fnew
		#if not next_point in open_list:
	        #    heapq.heappush(open_list, next_point)
		#else:
		#    print next_point.g
		#    if (gnew < next_point.g):
		#	heapq.heappop(open_list, next_point)
		#	next_point.g = gnew
		#	heapq.heappush(open_list, next_point)
        rospy.loginfo("No path found")

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = 0
        self.h = 0
    def __lt__(self, other):
        return (self.g + self.h) < (other.g + other.h)


if __name__ == '__main__':
    planning_process_object = Planning_Node('planning_process')

