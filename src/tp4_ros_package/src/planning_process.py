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


# Definition of class
class Node:
    """
    Represents a node in a graph for pathfinding algorithms.
    Args:
        x (float): The x-coordinate of the node.
        y (float): The y-coordinate of the node.
        parent (Node, optional): The parent node of the current node. Defaults to None.
    """
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
	self.z = 0 # Assuming z-coordinate is always 0
        self.parent = parent 
        self.g = 0.0 # Cost from the start node to this node
        self.f = 0.0 # Estimated total cost from start to goal through this node
    def __lt__(self, other): 
	"""
        Compares two nodes based on their total cost.
	Useful for heapq to sort the list
        Args:
            other (Node): The other node to compare with.
        Returns:
            bool: True if the total cost of this node is less than the other node, False otherwise.
        """
	return self.f < other.f 

class Planning_Node:
    """
    Handles planning and navigation tasks in a robotic environment using ROS
    Args:
        node_name (str): The name of the ROS node.
    Attributes:
        nname (str): The name of the ROS node.
        marker_pub: Publisher for visualization marker arrays.
        visited_point_pub: Publisher for visited point marker arrays.
        shortest_path: Publisher for the found path.
        point1: First point for path planning.
        point2: Second point for path planning.
        grid_map: Grid map for navigation.
        map_info: Information about the map.
    """
    def __init__(self, node_name):
    	self.nname = node_name     #Giving a name for the ROS node

    	rospy.init_node(self.nname, anonymous=True) #ROS node initialization
	print 'Start ...'
	self.marker_pub = rospy.Publisher('/visualization_marker_array1', MarkerArray, queue_size = 2)
	self.visited_point_pub = rospy.Publisher('/visualization_marker_array2', MarkerArray, queue_size = 100)
	self.shortest_path = rospy.Publisher('/path', Path, queue_size = 10)

	point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.get_point_callback) # ROS topic subscription
	map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map_callback)
	self.point1 = None
	self.point2 = None
	self.grid_map = None
	self.map_info = None
	rospy.spin() # Initiate the ROS loop
    def dilate_image(self, image, kernel):
        # Get image dimensions
        height, width = image.shape

        # Get kernel dimensions
        k_height, k_width = kernel.shape

        # Pad the image to handle border pixels
        padded_image = np.pad(image, ((k_height // 2, k_height // 2), (k_width // 2, k_width // 2)), mode='constant')

        # Create an empty output image
        dilated_image = np.zeros_like(image)

        # Perform dilation
        for i in range(height):
            for j in range(width):
            # Extract the neighborhood
                neighborhood = padded_image[i:i + k_height, j:j + k_width]

            # Perform element-wise multiplication with the kernel
                dilation_result = neighborhood * kernel

            # Assign the maximum value to the output image
                dilated_image[i, j] = np.max(dilation_result)

        return dilated_image

    def get_map_callback(self, map_info):
        """
        Receives map information from ROS topic '/map' and converts it to a grid map.
        Args:
            map_info (OccupancyGrid): Information about the map.
        """
	map = np.array(map_info.data).reshape((map_info.info.height,map_info.info.width))
	self.map_info = map_info
	dilated_map = self.dilate_image(map, np.ones((2, 2), dtype=np.uint8))
	self.grid_map = dilated_map.T

    def coordinate_to_index(self, point):
        """
        Converts coordinates to grid indices based on the map information.
	These grid indices will be used for searching the shortest path with A*.
        Args:
            point (Point): The point to convert to grid indices.
        Returns:
            list: A list containing the grid indices [index_x, index_y].
        """
	index_x = int(np.round((point.x - self.map_info.info.origin.position.x)/self.map_info.info.resolution))
	index_y = int(np.round((point.y - self.map_info.info.origin.position.y)/self.map_info.info.resolution))
	return [index_x, index_y]

    def index_to_coordinate(self, point):
        """
        Converts grid indices to coordinates based on map information.
	These coordinates are used for draw the found path on Rviz. 
        Args:
            point (Point): The point containing grid indices.
        Returns:
            list: A list containing the converted coordinates [coordinate_x, coordinate_y].
        """
	coordinate_x = point.x*self.map_info.info.resolution + self.map_info.info.origin.position.x
	coordinate_y = point.y*self.map_info.info.resolution + self.map_info.info.origin.position.y
	return [coordinate_x, coordinate_y]

    def publish_marker_array(self):
        """
        Publishes marker arrays for visualization.
        This method creates marker arrays for two points (initial point and goal point) and publishes them.
        """
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

    def publish_closed_set(self, closed_set):
        """
        Publishes marker arrays for the visited points.
        Args:
            closed_set (list): A list containing the visited pointd.
        """
	marker_array = MarkerArray()
	for idx, point in enumerate(closed_set):
	    (x1 , y1) = point
	    node = Node(x1,y1)
	    [x2, y2] = self.index_to_coordinate(node)
	    node2 = Node(x2,y2)
    	    marker = Marker()
	    marker.id = idx
    	    marker.header.frame_id = 'map'
    	    marker.type = Marker.POINTS
            marker.action = Marker.ADD
    	    marker.scale.x = 0.25
    	    marker.scale.y = 0.25
    	    marker.color.a = 1.0 
    	    marker.color.r , marker.color.g, marker.color.b = 1.0, 1.0, 1.0
    	    marker.points.append(node2)
	    marker_array.markers.append(marker)
	self.visited_point_pub.publish(marker_array)	

    def delete_closed_set(self):
        """
        Deletes the marker array for the closed set.
        """
	marker_array = MarkerArray()
    	marker = Marker()
	marker.id = 0
    	marker.header.frame_id = 'map'
    	marker.type = Marker.POINTS
        marker.action = Marker.DELETEALL
	marker_array.markers.append(marker)
	self.visited_point_pub.publish(marker_array)	

    def publish_path(self, shortest_path):
        """
        Publishes the shortest path as a Path message.
        Args:
            shortest_path (list): A list containing the shortest path.
        """
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

    def cost_to_go(self, point1, point2):
        """
        Calculates the estimated cost to reach from this point to another point by L2-norm (Full connected neighborhood) 
        Args:
            point1 (Node)
            point2 (Node)
        Returns:
            float: The estimated cost to go.
        """
        return ((point1.x - point2.x)**2 + (point1.y - point2.y)**2)**1/2

    def is_valid(self, point):
        """
        Checks if a given point is within the map boundaries.
        Args:
            point (Point): The point to check.
        Returns:
            bool: True if the point is within the map boundaries, False otherwise.
        """
	return (point.x >= 0) and (point.x < self.map_info.info.height) and (point.y >= 0) and (point.y < self.map_info.info.width)

    def is_unblocked(self, point):
        """
        Checks if a given point is unblocked.
        Args:
            point (Point): The point to check.
        Returns:
            bool: True if the point is unblocked, False otherwise.
        """
	return self.grid_map[point.x,point.y] == 0

    def exist_in_list(self, lst, x, y):
        """
        Checks if a node with given attributes (x and y coordinates) exists in the list.
        Args:
            lst (list): The list to search in.
            x: The value of x coordinate.
            y: The value of y coordinate.
        Returns:
            Node or None: The node if found, None otherwise.
        """
        for instance in lst:
	    if instance.x == x and instance.y == y:
	        return instance
        return None

    def get_point_callback(self, point_info):
        """
        Callback function for receiving points (initial point and goal point) .
        Args:
            point_info (PointStamped): Information about the received point.
        """
	if not self.point1:
	    self.point1 = point_info.point
	    rospy.loginfo('Coordinates 1: x = %f y = %f' %(self.point1.x,self.point1.y))
	else:
	    self.point2 = point_info.point
	    rospy.loginfo('Coordinates 2: x = %f y = %f' %(self.point2.x,self.point2.y))

	if self.point1 and self.point2:
	    self.delete_closed_set() # Reset the marker before drawing next marker
	    self.publish_marker_array()
	    self.find_path()
	    self.point1 = None
	    self.point2 = None

    def find_path(self):
        """
        This method implements the A* algorithm to find the shortest path from start_point to goal_point.
        """
	start_point = Node(*self.coordinate_to_index(self.point1)) # The * operator to unpack the returned index values as arguments for the Node constructor. 
	goal_point = Node(*self.coordinate_to_index(self.point2))

	if not self.is_valid(start_point) or not self.is_valid(goal_point):
	    print 'Source or destination is invalid'
	    return
	if not self.is_unblocked(start_point) or not self.is_unblocked(goal_point):
	    print 'Source or destination is blocked'
	    return
		
	# Initialize open_list and closed_set
        open_list = []
        closed_set = set() # Use set because it is the good way to check whether a specific element is contained in set
	start_point.g = 0
	start_point.f = 0

	# Add start_point to open_list
        heapq.heappush(open_list, start_point)
	    
	# Main loop of A* algorithm
        while open_list:
            current_point = heapq.heappop(open_list) # Pop node with the lowest f value from open_list
	    closed_set.add((current_point.x, current_point.y)) # Add current_point to closed_set
            self.publish_closed_set(list(closed_set))  # Publish closed_set for visualization

	    # Check if current_point is the goal_point
            if current_point.x == goal_point.x and current_point.y == goal_point.y:
		print 'Finding Path Successfully !!!'
		# Reconstruct and publish the path
		path = []
                while current_point:
                    path.append(self.index_to_coordinate(current_point))
                    current_point = current_point.parent
                self.publish_path(path[::-1])
                return

	    # Explore neighboring nodes
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]: # full-connected neighborhood
                next_x, next_y = current_point.x + dx, current_point.y + dy
		next_point = Node(next_x,next_y,current_point)

		# Skip if the neighbor is in closed_set
	        if (next_x, next_y) in closed_set:
		    continue
		# Check if next_point is valid and unblocked
                if self.is_valid(next_point) and self.is_unblocked(next_point):
		    gnew = current_point.g + self.cost_to_go(current_point, next_point) # Because of L2 distance
		    fnew = gnew + self.cost_to_go(next_point, goal_point)
		    next_point.f = fnew

		    # If next_point is not in open_list, add it; otherwise, update its g value if necessary
		    if self.exist_in_list(open_list, next_x, next_y) is None:
			next_point.g = gnew
			heapq.heappush(open_list, next_point)
		    else:
			found_point = self.exist_in_list(open_list, next_x, next_y)
		        if gnew < found_point.g:
			    open_list.remove(found_point)
		            next_point.g = gnew
	                    heapq.heappush(open_list, next_point)
				
	# If the loop completes without finding the goal, log that no path is found
	rospy.loginfo("No path found")

if __name__ == '__main__':
    planning_process_object = Planning_Node('planning_process')

