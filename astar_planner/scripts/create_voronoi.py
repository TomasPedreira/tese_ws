#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import cv2.ximgproc  # Add this explicit import
import scipy.spatial
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VoronoiGraphNode(Node):
    def __init__(self):
        super().__init__('voronoi_graph_node')
        self.declare_parameter('pgm_file', '/home/tomas/tt_ws/wall_map.pgm')
        self.pgm_file = self.get_parameter('pgm_file').value
        
    
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'voronoi_graph', 10)
        
        self.timer = self.create_timer(2.0, self.process_map)
        self.get_logger().info("Voronoi Graph Node Initialized")
        self.saved = False
    
    def load_pgm_map(self):
        """Load a PGM file as a NumPy array."""
        image = cv2.imread(self.pgm_file, cv2.IMREAD_GRAYSCALE)
        return image
    
    def get_obstacle_edges(self, image):
        """Extract obstacle edges from the map using Canny edge detection."""
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        edges = cv2.Canny(blurred, 200, 254)
        obstacle_points = np.column_stack(np.where(edges > 0))
        if not self.saved:
            cv2.imwrite('/home/tomas/tt_ws/edges_map_V9.png', edges)
        return obstacle_points
    
    def compute_voronoi(self, obstacle_points):
        """Compute the Voronoi diagram from obstacle edge points."""
        vor = scipy.spatial.Voronoi(obstacle_points)
        return vor
    
    def filter_voronoi_nodes(self, image, vor):
        """Filter Voronoi nodes to ensure they are in free space."""
        free_space_mask = image > 128  # Free space is white
        valid_vertices = []
        
        for point in vor.vertices:
            x, y = int(point[0]), int(point[1])
            if 0 <= x < image.shape[0] and 0 <= y < image.shape[1]:
                if free_space_mask[x, y]:
                    valid_vertices.append(point)
        
        return np.array(valid_vertices)
    
    def plot_voronoi(self, image, vor, valid_vertices):
        """Draw the Voronoi diagram overlay on the map, filtering edges on obstacles."""
        output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        # Draw Voronoi edges, filtering out those that fall on obstacles
        for simplex in vor.ridge_vertices:
            if -1 not in simplex:
                p1, p2 = vor.vertices[simplex].astype(int)
                if tuple(p1) in valid_vertices.tolist() and tuple(p2) in valid_vertices.tolist():
                    cv2.line(output, tuple(p1[::-1]), tuple(p2[::-1]), (0, 0, 255), 1)
        
        # Draw Voronoi nodes only if they are valid
        for point in valid_vertices:
            cv2.circle(output, tuple(point[::-1].astype(int)), 2, (255, 0, 0), -1)
        
        return output
    
    def process_map(self):
        image = self.load_pgm_map()
        obstacle_points = self.get_obstacle_edges(image)
        vor = self.compute_voronoi(obstacle_points)
        valid_vertices = self.filter_voronoi_nodes(image, vor)
        voronoi_image = self.plot_voronoi(image, vor, valid_vertices)
        
        # Convert to ROS Image and publish
        if not self.saved:
            cv2.imwrite('/home/tomas/tt_ws/voronoi_map_V9.png', voronoi_image)
            self.saved = True

        ros_image = self.bridge.cv2_to_imgmsg(voronoi_image, encoding='bgr8')
        self.publisher.publish(ros_image)
        self.get_logger().info("Published Voronoi Graph")


def main(args=None):
    rclpy.init(args=args)
    node = VoronoiGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
