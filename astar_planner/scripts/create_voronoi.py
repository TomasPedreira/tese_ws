#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import cv2.ximgproc
import scipy.spatial
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import yaml

class VoronoiGraphNode(Node):
    def __init__(self):
        super().__init__('voronoi_graph_node')
        self.declare_parameter('pgm_file', '/home/tomas/tt_ws/wall_map.pgm')
        self.declare_parameter('yaml_file', '/home/tomas/tt_ws/wall_map.yaml')
        self.declare_parameter('output_file', '/home/tomas/tt_ws/voronoi_nodes.txt')
        self.pgm_file = self.get_parameter('pgm_file').value
        self.yaml_file = self.get_parameter('yaml_file').value
        self.output_file = self.get_parameter('output_file').value
        
        # Load map parameters
        self.map_params = self.load_map_params()
        self.get_logger().info(f"Map parameters: {self.map_params}")
        
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'voronoi_graph', 10)
        
        self.timer = self.create_timer(2.0, self.process_map)
        self.get_logger().info("Voronoi Graph Node Initialized")
        self.saved = False
    
    def load_map_params(self):
        """Load map parameters from YAML file."""
        try:
            with open(self.yaml_file, 'r') as f:
                params = yaml.safe_load(f)
                
            # Default values if not in YAML
            if 'resolution' not in params:
                params['resolution'] = 0.05
            if 'origin' not in params:
                params['origin'] = [0.0, 0.0, 0.0]
                
            return params
        except Exception as e:
            self.get_logger().error(f"Failed to load map parameters: {e}")
            # Use defaults from your provided YAML
            return {
                'resolution': 0.05,
                'origin': [-8.84, -4.63, 0.0],
                'occupied_thresh': 0.65,
                'free_thresh': 0.25
            }
    
    def pixel_to_map_coords(self, pixel_x, pixel_y):
        """Convert pixel coordinates to map coordinates."""
        map_x = pixel_y * self.map_params['resolution'] + self.map_params['origin'][0]
        map_y = pixel_x * self.map_params['resolution'] + self.map_params['origin'][1]
        return map_x, map_y
    
    def map_to_pixel_coords(self, map_x, map_y):
        """Convert map coordinates to pixel coordinates."""
        pixel_y = int((map_x - self.map_params['origin'][0]) / self.map_params['resolution'])
        pixel_x = int((map_y - self.map_params['origin'][1]) / self.map_params['resolution'])
        return pixel_x, pixel_y
    
    def load_pgm_map(self):
        """Load a PGM file as a NumPy array."""
        image = cv2.imread(self.pgm_file, cv2.IMREAD_GRAYSCALE)
        return image
    
    def get_obstacle_edges(self, image):
        """Extract obstacle edges from the map using Canny edge detection."""
        edges = cv2.Canny(image, 200, 254)
        
        # Dilate the edges to create a buffer zone around obstacles
        kernel = np.ones((5, 5), np.uint8)  # Adjust kernel size as needed
        
        obstacle_points = np.column_stack(np.where(edges > 0))
        if not self.saved:
            cv2.imwrite('/home/tomas/tt_ws/edges_map_V20.png', edges)
        return obstacle_points
    
    def is_line_in_free_space(self, image, p1, p2):
        """Check if the line segment between p1 and p2 is in free space."""
        line_points = np.linspace(p1, p2, num=100)
        for point in line_points:
            x, y = int(point[0]), int(point[1])
            if 0 <= x < image.shape[0] and 0 <= y < image.shape[1]:
                if image[x, y] <= 128:  # Obstacle is black
                    return False
        return True
    
    def compute_voronoi(self, obstacle_points):
        """Compute the Voronoi diagram from obstacle edge points."""
        vor = scipy.spatial.Voronoi(obstacle_points)
        return vor
    
    def filter_voronoi_nodes(self, image, vor):
        """Filter Voronoi nodes to ensure they are in free space and not too close to obstacles."""
        free_space_mask = image > 128  # Free space is white
        obstacle_mask = (image <= 128).astype(np.uint8)  # Convert boolean to uint8
        
        valid_vertices = []
        valid_indices = []  # Store indices of valid vertices
        min_distance_to_obstacle = 10  # Minimum distance to obstacles (in pixels)
        
        for idx, point in enumerate(vor.vertices):
            x, y = int(point[0]), int(point[1])
            if 0 <= x < image.shape[0] and 0 <= y < image.shape[1]:
                if free_space_mask[x, y]:
                    # Check distance to nearest obstacle
                    distance_to_obstacle = cv2.distanceTransform(
                        cv2.bitwise_not(obstacle_mask),  # Invert obstacle mask
                        cv2.DIST_L2, 
                        5
                    )[x, y]
                    
                    if distance_to_obstacle >= min_distance_to_obstacle:
                        valid_vertices.append(point)
                        valid_indices.append(idx)
        
        return np.array(valid_vertices), valid_indices
    
    def build_adjacency_graph(self, vor, valid_indices, image):
        """Build an adjacency graph of valid Voronoi vertices."""
        # Create a mapping from original vertex indices to new indices
        vertex_mapping = {old_idx: new_idx for new_idx, old_idx in enumerate(valid_indices)}
        
        # Create adjacency list
        adjacency_list = {new_idx: [] for new_idx in range(len(valid_indices))}
        
        # Process ridge vertices to build connections
        for ridge in vor.ridge_vertices:
            if -1 not in ridge:  # Skip ridges that extend to infinity
                v1, v2 = ridge
                
                # Check if both vertices are valid
                if v1 in vertex_mapping and v2 in vertex_mapping:
                    new_v1 = vertex_mapping[v1]
                    new_v2 = vertex_mapping[v2]
                    
                    # Check if the edge is in free space
                    if self.is_line_in_free_space(image, vor.vertices[v1], vor.vertices[v2]):
                        # Convert to map coordinates to calculate actual distance
                        p1_map_x, p1_map_y = self.pixel_to_map_coords(vor.vertices[v1][0], vor.vertices[v1][1])
                        p2_map_x, p2_map_y = self.pixel_to_map_coords(vor.vertices[v2][0], vor.vertices[v2][1])
                        
                        # Calculate Euclidean distance in map coordinates
                        dist = np.sqrt((p1_map_x - p2_map_x)**2 + (p1_map_y - p2_map_y)**2)
                        
                        # Add connections (bidirectional)
                        adjacency_list[new_v1].append((new_v2, dist))
                        adjacency_list[new_v2].append((new_v1, dist))
        
        return adjacency_list, vertex_mapping
    
    def export_nodes_to_file(self, valid_vertices, adjacency_list, output_file):
        """Export nodes and their neighbors to a text file."""
        with open(output_file, 'w') as f:
            # Write number of nodes first
            f.write(f"{len(valid_vertices)}\n")
            
            # Write each node's position and its neighbors
            for i in range(len(valid_vertices)):
                # Convert pixel coordinates to map coordinates
                map_x, map_y = self.pixel_to_map_coords(valid_vertices[i][0], valid_vertices[i][1])
                neighbors = adjacency_list[i]
                
                # Format: node_id x y num_neighbors neighbor_id1:distance1 neighbor_id2:distance2 ...
                neighbor_str = " ".join([f"{n}:{d:.4f}" for n, d in neighbors])
                f.write(f"{i} {map_x:.4f} {map_y:.4f} {len(neighbors)} {neighbor_str}\n")
        
        self.get_logger().info(f"Exported {len(valid_vertices)} nodes to {output_file}")
    
    def plot_voronoi(self, image, vor, valid_vertices, adjacency_list, vertex_mapping):
        """Draw the Voronoi diagram overlay on the map, highlighting valid nodes and connections."""
        output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        # Draw valid Voronoi edges based on adjacency list
        for i, neighbors in adjacency_list.items():
            p1 = valid_vertices[i].astype(int)
            for j, _ in neighbors:
                p2 = valid_vertices[j].astype(int)
                cv2.line(output, tuple(p1[::-1]), tuple(p2[::-1]), (0, 0, 255), 1)
        
        # Draw Voronoi nodes
        for i, point in enumerate(valid_vertices):
            # Convert to map coordinates for display
            map_x, map_y = self.pixel_to_map_coords(point[0], point[1])
        
            
            cv2.circle(output, tuple(point[::-1].astype(int)), 2, (255, 0, 0), -1)
        
        
        return output
    
    def process_map(self):
        image = self.load_pgm_map()
        obstacle_points = self.get_obstacle_edges(image)
        vor = self.compute_voronoi(obstacle_points)
        valid_vertices, valid_indices = self.filter_voronoi_nodes(image, vor)
        
        # Build adjacency graph
        adjacency_list, vertex_mapping = self.build_adjacency_graph(vor, valid_indices, image)
        
        # Export nodes and neighbors to file with map coordinates
        self.export_nodes_to_file(valid_vertices, adjacency_list, self.output_file)
        
        # Create visualization with node IDs and map coordinates
        voronoi_image = self.plot_voronoi(image, vor, valid_vertices, adjacency_list, vertex_mapping)
        
        # Save visualization
        if not self.saved:
            cv2.imwrite('/home/tomas/tt_ws/voronoi_map_V20.png', voronoi_image)
            self.saved = True

        # Convert to ROS Image and publish
        ros_image = self.bridge.cv2_to_imgmsg(voronoi_image, encoding='bgr8')
        self.publisher.publish(ros_image)
        self.get_logger().info("Published Voronoi Graph")
        
        # Stop the timer after processing once
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = VoronoiGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()