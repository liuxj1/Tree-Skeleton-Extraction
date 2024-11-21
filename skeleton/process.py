from sklearn.cluster import DBSCAN
from scipy.spatial import cKDTree
import numpy as np
import networkx as nx
from collections import deque
from concurrent.futures import ThreadPoolExecutor

def load_and_preprocess_pcd(file_path, type):
    with open(file_path, 'r') as file:
        data_list = [np.fromstring(line.strip().strip('[]'), sep=' ') for line in file]
    data = np.vstack(data_list)
    xyz = data[:, :3]
    # Normalization
    min_bound = np.min(xyz, axis=0)
    max_bound = np.max(xyz, axis=0)
    max_range = np.max(max_bound - min_bound)
    normalized_points = (xyz - min_bound) / max_range
    points = np.asarray(normalized_points)
    # Downsampling, but it is worth noting that the synthetic dataset needs to be evaluated,
    # so downsampling should not be too sparse
    if type == 'synthetic':
        target_number_of_points = int(len(points) * 0.60)
    else:
        if len(points) <= 1000000:
            target_number_of_points = max(int(len(points) * 0.20), 100000)
        elif len(points) > 1000000 and len(points) <= 2000000:
            target_number_of_points = int(len(points) * 0.15)
        elif len(points) > 2000000 and len(points) <= 3000000:
            target_number_of_points = int(len(points) * 0.10)
        else:
            target_number_of_points = 300000
    indices = np.random.choice(len(points), target_number_of_points, replace=False)
    downsampled_points = points[indices]
    return downsampled_points

def create_undirected_graph(data_points, search_radius):
    kdtree = cKDTree(data_points)
    neighbors = kdtree.query_ball_point(data_points, r=search_radius)
    G = {i: set(neighbor) for i, neighbor in enumerate(neighbors)}
    return G

def select_root_set(data_points, min_y_threshold):
    # Calculate the y-axis coordinate of the lowest point
    min_y = np.min(data_points[:, 1])
    root_indices = np.where((data_points[:, 1] - min_y) < min_y_threshold)[0]
    return root_indices.tolist()
def calculate_node_values(graph, root_set,num_value):
    """
    Optimize the function that calculates node values, use BFS to traverse the entire graph from the root set, and quantize the node values.
    :param graph: Graph, represented as an adjacency list.
    :param root_set: Root node set, used to calculate node values.
    :param num_value: Quantized target value range (such as 0-60).
    :return: Dictionary of node values.
    """
    node_values = {node: -1 for node in graph}  # Initialize all nodes to -1.
    bfs_queue = deque(root_set)

    # Initialize the root node value
    for root_node in root_set:
        node_values[root_node] = 1

    # Calculate node value using BFS
    while bfs_queue:
        current_node = bfs_queue.popleft()
        current_value = node_values[current_node]

        for neighbor in graph[current_node]:
            if node_values[neighbor] == -1:
                bfs_queue.append(neighbor)
                node_values[neighbor] = current_value + 1

    # Quantify node values
    values_array = np.array(list(node_values.values()))
    max_value = values_array.max()

    if max_value > 0:
        factor = num_value / max_value
        for node in node_values:
            if node_values[node] != -1:
                node_values[node] = int(node_values[node] * factor)  # Quantize to a specified range
    return node_values
def depth_first_search(graph, start_node, visited, cluster, node_values):
    stack = [start_node]
    visited.add(start_node)
    while stack:
        node = stack.pop()
        cluster.append(node)
        for neighbor in graph[node]:
            if neighbor not in visited and node_values[node] == node_values[neighbor]:
                stack.append(neighbor)
                visited.add(neighbor)
def cluster_points(graph, data_points, node_values):
    visited = set()
    clusters = []
    for i in range(len(data_points)):
        if i not in visited:
            cluster = []
            depth_first_search(graph, i, visited, cluster, node_values)
            clusters.append(cluster)
    return clusters
def process_cluster(cluster_indices, data_points, eps, min_samples):
    cluster_data = data_points[cluster_indices]
    dbscan_result = DBSCAN(eps=eps, min_samples=min_samples).fit(cluster_data)
    labels = dbscan_result.labels_
    unique_labels = np.unique(labels)

    new_cluster_indices = []
    for label in unique_labels:
        if label != -1:
            indices = np.where(labels == label)[0]
            selected_indices = np.take(cluster_indices, indices)
            new_cluster_indices.append(selected_indices)

    return new_cluster_indices
def apply_dbscan_to_clusters(clusters, data_points, eps, min_samples):
    new_clusters = []
    with ThreadPoolExecutor(max_workers=4) as executor:
        futures = [executor.submit(process_cluster, cluster, data_points, eps, min_samples) for cluster in clusters]
        for future in futures:
            result = future.result()
            for new_cluster in result:
                new_clusters.append(new_cluster)
    return new_clusters
def calculate_skeleton_nodes(clusters, data_points):
    skeleton_nodes = np.array([np.mean(data_points[cluster], axis=0) for cluster in clusters])
    return skeleton_nodes
def find_min_distance_between_clusters(cluster_i_data, cluster_j_data):
    tree_i = cKDTree(cluster_i_data)
    # Query the distance between the closest points
    distance, _ = tree_i.query(cluster_j_data, k=1)
    min_distance = np.min(distance)
    return min_distance
def generate_skeleton(skeleton_nodes, clusters,data_points):
    skeleton = nx.Graph()  # Create an empty undirected graph to represent the skeleton
    kd_tree = cKDTree(skeleton_nodes)
    # Adding skeleton nodes to the graph
    for i, node in enumerate(skeleton_nodes):
        skeleton.add_node(i, pos=node)

    # Traverse each node, find neighboring nodes within a certain range, and try to connect
    for i, node_i in enumerate(skeleton_nodes):
        indices = kd_tree.query_ball_point(node_i, r=0.1)
        for j in indices:
            if i != j:
                node_j = skeleton_nodes[j]
                distance = np.linalg.norm(node_i - node_j)
                if distance < 0.1:
                    cluster_i_data = data_points[clusters[i]]
                    cluster_j_data = data_points[clusters[j]]
                    min_distance = find_min_distance_between_clusters(cluster_i_data, cluster_j_data)
                    if min_distance < 0.003:
                        skeleton.add_edge(i, j)
    return skeleton