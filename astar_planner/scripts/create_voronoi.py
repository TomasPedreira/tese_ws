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
        self.declare_parameter('min_node_distance', 10.0)
        self.pgm_file = self.get_parameter('pgm_file').value
        self.yaml_file = self.get_parameter('yaml_file').value
        self.output_file = self.get_parameter('output_file').value
        self.min_node_distance = self.get_parameter('min_node_distance').value
                
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
        edges = cv2.Canny(image, 200, 254)
        obstacle_points = np.column_stack(np.where(edges > 0))
        return obstacle_points
    
    def is_line_in_free_space(self, image, p1, p2):
        """Check if the line segment between p1 and p2 is in free space."""
        line_points = np.linspace(p1, p2, num=50)
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
        """Filter Voronoi nodes to ensure they are in free space."""
        free_space_mask = image > 128  # Free space is white
        valid_vertices = []
        valid_indices = []
        
        for idx, point in enumerate(vor.vertices):
            x, y = int(point[0]), int(point[1])
            if 0 <= x < image.shape[0] and 0 <= y < image.shape[1] and free_space_mask[x, y]:
                valid_vertices.append(point)
                valid_indices.append(idx)
        
        return np.array(valid_vertices), valid_indices
    
    def find_closest_exclusive_node(self, image, pos, vertices, current_neighbors, adjacency_list, exclude_idx):
        """Find the closest node not connected to current_neighbors."""
        min_dist = float('inf')
        closest_idx = None
        for i, vertex in enumerate(vertices):
            if i != exclude_idx and i not in current_neighbors:
                all_neighbors = set(adjacency_list[i])
                if not (all_neighbors & set(current_neighbors)):  # No overlap with current neighbors
                    dist = np.linalg.norm(pos - vertex)
                    if dist < min_dist and self.is_line_in_free_space(image, pos, vertex):
                        min_dist = dist
                        closest_idx = i
        return closest_idx, min_dist
    
    def prune_and_reconnect_nodes(self, image, valid_vertices, adjacency_list):
        """Prune nodes and ensure each has at least 2 exclusive neighbors."""
        kept_vertices = []
        kept_indices = []
        
        # Prune nodes with minimum distance constraint
        for i, vertex in enumerate(valid_vertices):
            keep = True
            for kept_vertex in kept_vertices:
                if np.linalg.norm(vertex - kept_vertex) < self.min_node_distance:
                    keep = False
                    break
            if keep:
                kept_vertices.append(vertex)
                kept_indices.append(i)
        
        # Create mapping from old to new indices
        index_mapping = {old_idx: new_idx for new_idx, old_idx in enumerate(kept_indices)}
        num_nodes = len(kept_vertices)
        new_adjacency_list = {i: [] for i in range(num_nodes)}
        
        # Transfer original edges
        for old_idx in index_mapping:
            new_idx = index_mapping[old_idx]
            for neighbor in adjacency_list[old_idx]:
                if neighbor in index_mapping:
                    new_neighbor_idx = index_mapping[neighbor]
                    if new_neighbor_idx not in new_adjacency_list[new_idx]:
                        new_adjacency_list[new_idx].append(new_neighbor_idx)
        
        # Reconnect pruned nodes' neighbors
        for old_idx in range(len(valid_vertices)):
            if old_idx not in index_mapping:
                neighbors = adjacency_list[old_idx]
                kept_neighbors = [n for n in neighbors if n in index_mapping]
                for n in kept_neighbors:
                    n_idx = index_mapping[n]
                    n_pos = kept_vertices[n_idx]
                    closest_idx, _ = self.find_closest_exclusive_node(image, n_pos, kept_vertices, new_adjacency_list[n_idx], new_adjacency_list, n_idx)
                    if closest_idx is not None and closest_idx not in new_adjacency_list[n_idx]:
                        new_adjacency_list[n_idx].append(closest_idx)
                        if n_idx not in new_adjacency_list[closest_idx]:
                            new_adjacency_list[closest_idx].append(n_idx)
        
        # Enforce at least 2 exclusive neighbors
        for i in range(num_nodes):
            neighbors = new_adjacency_list[i]
            pos = kept_vertices[i]
            exclusive_count = 0
            
            # Count exclusive neighbors
            for n in neighbors:
                n_neighbors = set(new_adjacency_list[n]) - {i}
                if not (n_neighbors & set(neighbors) - {n}):
                    exclusive_count += 1
            
            # Add exclusive neighbors if needed
            while exclusive_count < 2:
                closest_idx, _ = self.find_closest_exclusive_node(image, pos, kept_vertices, neighbors, new_adjacency_list, i)
                if closest_idx is not None:
                    new_adjacency_list[i].append(closest_idx)
                    if i not in new_adjacency_list[closest_idx]:
                        new_adjacency_list[closest_idx].append(i)
                    neighbors = new_adjacency_list[i]
                    exclusive_count += 1
                else:
                    self.get_logger().warn(f"Node {i} at {pos} could not find enough exclusive neighbors.")
                    break
        
        return np.array(kept_vertices), new_adjacency_list
    
    def build_adjacency_graph(self, vor, valid_indices, image):
        """Build an adjacency graph of valid Voronoi vertices ensuring connectivity."""
        vertex_mapping = {old_idx: new_idx for new_idx, old_idx in enumerate(valid_indices)}
        adjacency_list = {new_idx: [] for new_idx in range(len(valid_indices))}
        
        for ridge in vor.ridge_vertices:
            if -1 not in ridge:
                v1, v2 = ridge
                if v1 in vertex_mapping and v2 in vertex_mapping:
                    new_v1 = vertex_mapping[v1]
                    new_v2 = vertex_mapping[v2]
                    if self.is_line_in_free_space(image, vor.vertices[v1], vor.vertices[v2]):
                        adjacency_list[new_v1].append(new_v2)
                        adjacency_list[new_v2].append(new_v1)
        
        return adjacency_list
    
    def export_nodes_to_file(self, valid_vertices, adjacency_list, output_file):
        """Export nodes and their neighbors to a text file using pixel coordinates."""
        with open(output_file, 'w') as f:
            f.write(f"{len(valid_vertices)}\n")
            for i in range(len(valid_vertices)):
                x, y = int(valid_vertices[i][0]), int(valid_vertices[i][1])
                neighbors = adjacency_list[i]
                f.write(f"{i} {x} {y} {len(neighbors)}")
                for neighbor in neighbors:
                    f.write(f" {neighbor}")
                f.write("\n")
        self.get_logger().info(f"Exported {len(valid_vertices)} nodes to {output_file}")
    
    def save_voronoi_image(self, image, valid_vertices, adjacency_list):
        """Save an image with nodes and edges."""
        output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        for i, neighbors in adjacency_list.items():
            p1 = valid_vertices[i].astype(int)
            for j in neighbors:
                p2 = valid_vertices[j].astype(int)
                cv2.line(output, tuple(p1[::-1]), tuple(p2[::-1]), (0, 0, 255), 1)
        for point in valid_vertices:
            cv2.circle(output, tuple(point[::-1].astype(int)), 2, (255, 0, 0), -1)
        cv2.imwrite('/home/tomas/tt_ws/voronoi_graph_V33.png', output)
        self.get_logger().info("Saved Voronoi graph visualization")
        
    def process_map(self):
        image = self.load_pgm_map()
        obstacle_points = self.get_obstacle_edges(image)
        vor = self.compute_voronoi(obstacle_points)
        valid_vertices, valid_indices = self.filter_voronoi_nodes(image, vor)
        adjacency_list = self.build_adjacency_graph(vor, valid_indices, image)
        
        # Prune and reconnect nodes with new rule
        valid_vertices, adjacency_list = self.prune_and_reconnect_nodes(image, valid_vertices, adjacency_list)
        
        # Debug connectivity
        isolated = sum(1 for i in adjacency_list if not adjacency_list[i])
        avg_neighbors = sum(len(adjacency_list[i]) for i in adjacency_list) / len(adjacency_list)
        exclusive_counts = []
        for i in adjacency_list:
            neighbors = adjacency_list[i]
            exclusive_count = sum(1 for n in neighbors if not (set(adjacency_list[n]) & set(neighbors) - {n, i}))
            exclusive_counts.append(exclusive_count)
        min_exclusive = min(exclusive_counts) if exclusive_counts else 0
        self.get_logger().info(f"Isolated nodes: {isolated}, Avg neighbors: {avg_neighbors:.2f}, Min exclusive neighbors: {min_exclusive}")
        
        self.export_nodes_to_file(valid_vertices, adjacency_list, self.output_file)
        self.save_voronoi_image(image, valid_vertices, adjacency_list)
        self.get_logger().info("Published Voronoi Graph")
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = VoronoiGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()