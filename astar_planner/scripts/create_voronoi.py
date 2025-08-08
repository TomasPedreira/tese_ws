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
        self.get_logger().info("Initializing VoronoiGraphNode...")
        
        self.get_logger().info("Declaring parameters...")
        self.declare_parameter('pgm_file', '/home/tomas/tt_ws/corridor.pgm')
        self.declare_parameter('yaml_file', '/home/tomas/tt_ws/corridor.yaml')
        self.declare_parameter('output_file', '/home/tomas/tt_ws/corridor_nodes.txt')
        self.declare_parameter('min_node_distance', 7.5)
        
        self.get_logger().info("Getting parameter values...")
        self.pgm_file = self.get_parameter('pgm_file').value
        self.yaml_file = self.get_parameter('yaml_file').value
        self.output_file = self.get_parameter('output_file').value
        self.min_node_distance = self.get_parameter('min_node_distance').value
        
        # Create results directory and setup incremental naming
        self.get_logger().info("Creating results directory...")
        self.results_dir = '/home/tomas/tt_ws/results'
        os.makedirs(self.results_dir, exist_ok=True)
        self.get_logger().info(f"Results directory: {self.results_dir}")
        
        # Update output file to go to results directory with incremental naming
        self.output_file = self._get_next_txt_path()
        self.get_logger().info(f"Output file path: {self.output_file}")
        
        self.get_logger().info("Getting next image path...")
        self.output_image = self._get_next_image_path()
        self.get_logger().info(f"Output image path: {self.output_image}")
                
        self.get_logger().info("Creating CvBridge...")
        self.bridge = CvBridge()
        
        self.get_logger().info("Creating publisher...")
        self.publisher = self.create_publisher(Image, 'voronoi_graph', 10)
        
        self.get_logger().info("Creating timer...")
        self.timer = self.create_timer(2.0, self.process_map)
        
        self.get_logger().info("Voronoi Graph Node Initialized")
        self.saved = False
        
    def _get_next_image_path(self):
        """Get the next available image path with incremental naming."""
        self.get_logger().info("Finding next available image path...")
        base_name = "corridor_graph"
        counter = 1
        while True:
            filename = f"{base_name}_{counter:03d}.png"
            filepath = os.path.join(self.results_dir, filename)
            if not os.path.exists(filepath):
                self.get_logger().info(f"Found available path: {filepath}")
                return filepath
            counter += 1
            
    def _get_next_txt_path(self):
        """Get the next available txt file path with incremental naming."""
        self.get_logger().info("Finding next available txt file path...")
        base_name = "corridor_nodes"
        counter = 1
        while True:
            filename = f"{base_name}_{counter:03d}.txt"
            filepath = os.path.join(self.results_dir, filename)
            if not os.path.exists(filepath):
                self.get_logger().info(f"Found available txt path: {filepath}")
                return filepath
            counter += 1
    
    def load_pgm_map(self):
        """Load a PGM file as a NumPy array."""
        self.get_logger().info(f"Attempting to load PGM file: {self.pgm_file}")
        self.get_logger().info(f"File exists: {os.path.exists(self.pgm_file)}")
        
        image = cv2.imread(self.pgm_file, cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.get_logger().error(f"Failed to load image from {self.pgm_file}")
            raise RuntimeError(f"Could not load image from {self.pgm_file}")
        
        self.get_logger().info(f"Successfully loaded image with shape: {image.shape}")
        return image
    
    def get_obstacle_edges(self, image):
        """Extract obstacle edges from the map using Canny edge detection."""
        if image is None:
            self.get_logger().error("Input image is None")
            raise ValueError("Input image is None")
            
        edges = cv2.Canny(image, 200, 254)
        if edges is None:
            self.get_logger().error("Canny edge detection failed")
            raise RuntimeError("Canny edge detection failed")
            
        obstacle_points = np.column_stack(np.where(edges > 0))
        return obstacle_points
    
    def is_line_in_free_space(self, image, p1, p2):
        """Check if the line segment between p1 and p2 is in free space."""
        line_points = np.linspace(p1, p2, num=50)
        for point in line_points:
            x, y = int(point[0]), int(point[1])
            if 0 <= x < image.shape[0] and 0 <= y < image.shape[1]:
                if image[x, y] <= 240:  # Use same strict threshold as filter_voronoi_nodes
                    return False
        return True
    
    def compute_voronoi(self, obstacle_points):
        """Compute the Voronoi diagram from obstacle edge points."""
        vor = scipy.spatial.Voronoi(obstacle_points)
        return vor
    
    def filter_voronoi_nodes(self, image, vor):
        """Filter Voronoi nodes to ensure they are in free space."""
        # Use very high threshold to avoid grey areas - only near-white pixels
        # Black (0) = occupied, Grey (128) = unknown, Near-white (>240) = free
        free_space_mask = image > 240  # Only very light pixels are free space
        valid_vertices = []
        valid_indices = []
        
        # Debug: check pixel value distribution
        unique_values, counts = np.unique(image, return_counts=True)
        self.get_logger().info(f"Image pixel values: {dict(zip(unique_values, counts))}")
        
        for idx, point in enumerate(vor.vertices):
            x, y = int(point[0]), int(point[1])
            if 0 <= x < image.shape[0] and 0 <= y < image.shape[1] and free_space_mask[x, y]:
                valid_vertices.append(point)
                valid_indices.append(idx)
        
        self.get_logger().info(f"Free space pixels: {np.sum(free_space_mask)} out of {image.size}")
        return np.array(valid_vertices), valid_indices
    
    def find_closest_exclusive_node(self, image, pos, vertices, current_neighbors, adjacency_list, exclude_idx):
        """Find the closest node not connected to current_neighbors."""
        min_dist = float('inf')
        closest_idx = None
        checked_count = 0
        for i, vertex in enumerate(vertices):
            if i != exclude_idx and i not in current_neighbors:
                all_neighbors = set(adjacency_list[i])
                if not (all_neighbors & set(current_neighbors)):  # No overlap with current neighbors
                    dist = np.linalg.norm(pos - vertex)
                    if dist < min_dist and self.is_line_in_free_space(image, pos, vertex):
                        min_dist = dist
                        closest_idx = i
            checked_count += 1
            if checked_count % 1000 == 0:  # Log progress for large searches
                self.get_logger().debug(f"Checked {checked_count}/{len(vertices)} vertices in find_closest_exclusive_node")
        return closest_idx, min_dist
    
    def prune_and_reconnect_nodes(self, image, valid_vertices, adjacency_list):
        """Prune nodes and ensure each has at least 2 exclusive neighbors."""
        self.get_logger().info(f"Starting pruning with {len(valid_vertices)} vertices, min_distance: {self.min_node_distance}")
        kept_vertices = []
        kept_indices = []
        
        # Prune nodes with minimum distance constraint
        self.get_logger().info("Pruning nodes with minimum distance constraint...")
        for i, vertex in enumerate(valid_vertices):
            if i % 100 == 0:  # Log progress every 100 nodes
                self.get_logger().info(f"Processing vertex {i}/{len(valid_vertices)}")
            keep = True
            for kept_vertex in kept_vertices:
                if np.linalg.norm(vertex - kept_vertex) < self.min_node_distance:
                    keep = False
                    break
            if keep:
                kept_vertices.append(vertex)
                kept_indices.append(i)
        
        self.get_logger().info(f"After distance pruning: {len(kept_vertices)} vertices kept")
        
        # Create mapping from old to new indices
        self.get_logger().info("Creating index mapping...")
        index_mapping = {old_idx: new_idx for new_idx, old_idx in enumerate(kept_indices)}
        num_nodes = len(kept_vertices)
        new_adjacency_list = {i: [] for i in range(num_nodes)}
        
        # Transfer original edges
        self.get_logger().info("Transferring original edges...")
        for old_idx in index_mapping:
            new_idx = index_mapping[old_idx]
            for neighbor in adjacency_list[old_idx]:
                if neighbor in index_mapping:
                    new_neighbor_idx = index_mapping[neighbor]
                    if new_neighbor_idx not in new_adjacency_list[new_idx]:
                        new_adjacency_list[new_idx].append(new_neighbor_idx)
        
        # Reconnect pruned nodes' neighbors
        self.get_logger().info("Reconnecting pruned nodes' neighbors...")
        reconnect_count = 0
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
                        reconnect_count += 1
        
        self.get_logger().info(f"Reconnected {reconnect_count} edges")
        
        # Remove nodes with insufficient neighbors (simplified approach)
        self.get_logger().info("Removing nodes with insufficient neighbors...")
        
        # Use iterative removal to handle cascading effects
        max_iterations = 10  # Prevent infinite loops
        iteration = 0
        
        while iteration < max_iterations:
            iteration += 1
            self.get_logger().info(f"Iteration {iteration}: checking {num_nodes} nodes")
            
            nodes_to_remove = set()
            
            # Check each node for sufficient neighbors
            for i in range(num_nodes):
                if i not in new_adjacency_list:
                    continue  # Skip already removed nodes
                    
                neighbors = new_adjacency_list[i]
                exclusive_count = 0
                
                # Count exclusive neighbors
                for n in neighbors:
                    if n not in new_adjacency_list:
                        continue  # Skip removed neighbors
                    n_neighbors = set(new_adjacency_list[n]) - {i}
                    if not (n_neighbors & set(neighbors) - {n}):
                        exclusive_count += 1
                
                if exclusive_count < 2:
                    nodes_to_remove.add(i)
                    self.get_logger().info(f"Node {i} marked for removal (only {exclusive_count} exclusive neighbors)")
            
            if not nodes_to_remove:
                self.get_logger().info("No more nodes to remove")
                break
                
            self.get_logger().info(f"Removing {len(nodes_to_remove)} nodes in iteration {iteration}")
            
            # Remove nodes from adjacency list
            for node in nodes_to_remove:
                if node in new_adjacency_list:
                    # Remove this node from all its neighbors' lists
                    for neighbor in new_adjacency_list[node]:
                        if neighbor in new_adjacency_list and node in new_adjacency_list[neighbor]:
                            new_adjacency_list[neighbor].remove(node)
                    # Remove the node itself
                    del new_adjacency_list[node]
            
            # Update vertices list
            new_vertices = []
            old_to_new_mapping = {}
            new_idx = 0
            
            for old_idx in range(len(kept_vertices)):
                if old_idx in new_adjacency_list:
                    new_vertices.append(kept_vertices[old_idx])
                    old_to_new_mapping[old_idx] = new_idx
                    new_idx += 1
            
            # Update adjacency list indices
            new_adjacency_list_updated = {}
            for old_idx, neighbors in new_adjacency_list.items():
                new_idx = old_to_new_mapping[old_idx]
                new_adjacency_list_updated[new_idx] = []
                for neighbor in neighbors:
                    if neighbor in old_to_new_mapping:
                        new_neighbor_idx = old_to_new_mapping[neighbor]
                        new_adjacency_list_updated[new_idx].append(new_neighbor_idx)
            
            kept_vertices = np.array(new_vertices)
            new_adjacency_list = new_adjacency_list_updated
            num_nodes = len(kept_vertices)
            
            self.get_logger().info(f"After iteration {iteration}: {num_nodes} nodes remaining")
        
        if iteration >= max_iterations:
            self.get_logger().warn(f"Reached maximum iterations ({max_iterations}), stopping removal process")
        
        self.get_logger().info("Pruning and reconnecting complete")
        return kept_vertices, new_adjacency_list
    
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
        cv2.imwrite(self.output_image, output)
        self.get_logger().info("Saved Voronoi graph visualization")
        
    def process_map(self):
        self.get_logger().info("Starting map processing...")
        
        self.get_logger().info("Loading PGM map...")
        image = self.load_pgm_map()
        self.get_logger().info(f"Map loaded successfully. Shape: {image.shape}")
        
        self.get_logger().info("Extracting obstacle edges...")
        obstacle_points = self.get_obstacle_edges(image)
        self.get_logger().info(f"Found {len(obstacle_points)} obstacle points")
        
        self.get_logger().info("Computing Voronoi diagram...")
        vor = self.compute_voronoi(obstacle_points)
        self.get_logger().info(f"Voronoi computed. Vertices: {len(vor.vertices)}")
        
        self.get_logger().info("Filtering Voronoi nodes...")
        valid_vertices, valid_indices = self.filter_voronoi_nodes(image, vor)
        self.get_logger().info(f"Filtered to {len(valid_vertices)} valid vertices")
        
        self.get_logger().info("Building adjacency graph...")
        adjacency_list = self.build_adjacency_graph(vor, valid_indices, image)
        self.get_logger().info(f"Adjacency graph built with {len(adjacency_list)} nodes")
        
        # Prune and reconnect nodes with new rule
        self.get_logger().info("Pruning and reconnecting nodes...")
        valid_vertices, adjacency_list = self.prune_and_reconnect_nodes(image, valid_vertices, adjacency_list)
        self.get_logger().info(f"Pruning complete. Final nodes: {len(valid_vertices)}")
        
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
        
        self.get_logger().info("Exporting nodes to file...")
        self.export_nodes_to_file(valid_vertices, adjacency_list, self.output_file)
        
        self.get_logger().info("Saving Voronoi image...")
        self.save_voronoi_image(image, valid_vertices, adjacency_list)
        
        self.get_logger().info("Published Voronoi Graph")
        self.timer.cancel()

def main(args=None):
    print("Starting Voronoi Graph Node...")
    rclpy.init(args=args)
    print("ROS2 initialized")
    
    print("Creating VoronoiGraphNode...")
    node = VoronoiGraphNode()
    print("Node created, starting spin...")
    
    rclpy.spin(node)
    print("Spin finished")
    
    node.destroy_node()
    rclpy.shutdown()
    print("Shutdown complete")

if __name__ == "__main__":
    main()