from scipy.spatial.distance import cdist
from tqdm import tqdm
import json
from skeleton.get_CS import completion
from skeleton.process import *
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading

# A helper function for safely removing nodes
def safe_remove_node(G, node):
    if node in G:
        for neighbor in list(G.neighbors(node)):
            if G.degree(neighbor) > 2:  # Only consider if the degree of the neighboring node is greater than 2
                neighbor_pos = G.nodes[neighbor]['pos']
        G.remove_node(node)
def prune_skeleton(skeleton):
    """
    The function is used to prune the skeleton, removing nodes with only a single connection and pairs of nodes that are only connected to each other and have no other connections.
    skeleton: NetworkX graph, representing the skeleton structure.
    return: The pruned NetworkX graph and a list of recorded neighbor point coordinates.
    """
    pruned_skeleton = skeleton.copy()
    changed = True
    first_iteration = True  #
    while changed:
        changed = False
        # 找出所有度为1的节点
        to_prune = [node for node, degree in dict(pruned_skeleton.degree()).items() if degree == 1]
        for node in to_prune:
            if node in pruned_skeleton:
                neighbor = next(iter(pruned_skeleton.neighbors(node)), None)
                # If the adjacent nodes exist and have degree 1, remove both nodes
                if neighbor and pruned_skeleton.degree(neighbor) == 1:
                    safe_remove_node(pruned_skeleton, neighbor)
                    safe_remove_node(pruned_skeleton, node)
                    changed = True
                # If the adjacent node exists and its degree is greater than 2, only remove the current node
                elif neighbor and pruned_skeleton.degree(neighbor) > 2:
                    if first_iteration:
                        safe_remove_node(pruned_skeleton, node)
                        changed = True
        first_iteration = False
    return pruned_skeleton

def select_label_root_set(data_points, current_cluster_indices, distance_threshold):
    root_set = []
    min_y_index = np.argmin(data_points[:, 1])
    min_y_point = data_points[min_y_index]
    for i, point in enumerate(data_points):
        distance = np.linalg.norm(point - min_y_point)
        if distance < distance_threshold:
            root_set.append(current_cluster_indices[i])
    return root_set

def convert_ndarray_to_list(graph):
    for node, attrs in graph.nodes(data=True):
        for key, value in attrs.items():
            if isinstance(value, np.ndarray):
                attrs[key] = value.tolist()
    for u, v, attrs in graph.edges(data=True):
        for key, value in attrs.items():
            if isinstance(value, np.ndarray):
                attrs[key] = value.tolist()
def save_graph_json(graph, filename):
    convert_ndarray_to_list(graph)
    graph_json = nx.node_link_data(graph)  # Convert the graph into a node-link format suitable for JSON
    with open(filename, 'w') as f:
        json.dump(graph_json, f, indent=4)


class UnionFind:
    def __init__(self, size):
        self.parent = list(range(size))

    def find(self, x):
        if self.parent[x] != x:
            self.parent[x] = self.find(self.parent[x])
        return self.parent[x]

    def union(self, x, y):
        rootX = self.find(x)
        rootY = self.find(y)
        if rootX != rootY:
            self.parent[rootY] = rootX
def merge_clusters_if_close(clusters_data, threshold=0.003):
    n = len(clusters_data)
    uf = UnionFind(n)

    for i in range(n):
        for j in range(i + 1, n):
            if uf.find(i) != uf.find(j):
                distance = np.min(cdist(clusters_data[i], clusters_data[j], 'euclidean'))
                if distance < threshold:
                    uf.union(i, j)

    new_clusters = {}
    for i in range(n):
        root = uf.find(i)
        if root in new_clusters:
            new_clusters[root].append(i)
        else:
            new_clusters[root] = [i]

    # Reconstruction Cluster
    merged_clusters = [np.vstack([clusters_data[idx] for idx in indices]) for indices in new_clusters.values()]
    return merged_clusters

def extract_subgraph(graph, nodes):
    """
    Extract the subgraph of the specified node from the graph.
    graph: adjacency list in dictionary form.
    nodes: the set of nodes to be extracted (set type).
    """
    # Ensure that the node set is a set to allow for fast intersection operations
    node_set = set(nodes)
    subgraph = {node: neighbors & node_set for node, neighbors in graph.items() if node in node_set}
    return subgraph

def extract_skeleton_subgraphs(skeleton):
    """
    Extract all connected components from a complete skeleton graph, returning each component as a complete subgraph.
    skeleton: The complete NetworkX graph.
    return: A list containing all subgraphs.
    """
    connected_components = nx.connected_components(skeleton)  # Get all connected components
    subgraphs = [
        skeleton.subgraph(component).copy()
        for component in connected_components

    ]
    return subgraphs
def laplacian_smoothing(skeleton, node_positions, iterations=2, alpha=0.2):
    """
    Perform Laplace smoothing on a skeleton graph.
    skeleton: NetworkX graph representing the skeleton.
    node_positions: A dictionary whose keys are nodes and whose values are the (x, y, z) coordinates of the nodes.
    iterations: The number of smoothing iterations.
    alpha: Smoothing factor that controls the interpolation weight between new and old positions.
    """
    for _ in range(iterations):
        new_positions = {}
        for node in skeleton.nodes():
            if skeleton.degree(node) > 1:  # Smooth only internal nodes
                neighbors = list(skeleton.neighbors(node))
                centroid = np.mean([node_positions[neighbor] for neighbor in neighbors], axis=0)
                original = np.array(node_positions[node])
                new_positions[node] = alpha * centroid + (1 - alpha) * original
            else:
                new_positions[node] = node_positions[node]
        node_positions.update(new_positions)
    return node_positions

def merge_skeletons(subgraphs, smoothed_positions_all):
    merged_skeleton = nx.Graph()
    for subgraph, smoothed_positions in zip(subgraphs, smoothed_positions_all):
        for node in subgraph.nodes():
            merged_skeleton.add_node(node, pos=smoothed_positions[node])
        for u, v in subgraph.edges():
            merged_skeleton.add_edge(u, v)
    return merged_skeleton


# Single disconnected region processing logic
def process_single_label(label, labels, data_points, G, min_y_label):
    DBs_indices = np.where(labels == label)[0]
    DBs_points = data_points[DBs_indices]

    # Constructing sub-undirected graphs (disconnected regions)
    Gi = extract_subgraph(G, DBs_indices)
    # Select the root set of disconnected regions
    if int(label) == int(min_y_label):
        local_Xi = select_root_set(DBs_points, 0.02)
        Xi = [DBs_indices[i] for i in local_Xi]
    else:
        Xi = select_label_root_set(DBs_points, DBs_indices, 0.02)
    # Calculate node values and quantify node values
    node_values_clusters = calculate_node_values(Gi, Xi, 60)
    return node_values_clusters
def process_labels_threaded(DBs_labels, labels, data_points, G, min_y_label):
    node_values = {}
    lock = threading.Lock()
    # Parallel processing using thread pools
    with ThreadPoolExecutor() as executor:
        futures = [executor.submit(process_single_label, label, labels, data_points, G, min_y_label) for label in DBs_labels]

        # Use tqdm to display a progress bar and wait for all threads to complete
        for future in tqdm(as_completed(futures), total=len(futures), desc="Processing labels with threads"):
            node_values_clusters = future.result()  # Get thread results

            with lock:
                node_values.update(node_values_clusters)
    return node_values
def get_min_y_label(data_points, labels):
    # Filter out the point indices that are not marked as noise (label is not -1)
    non_noise_indices = np.where(labels != -1)[0]
    # Get the points that are not marked as noise
    non_noise_points = data_points[non_noise_indices]
    # Find the index of the point with the lowest y value among the non-noise points
    min_y_index_in_non_noise = np.argmin(non_noise_points[:, 1])
    # Get the non-noise point with the lowest y value and its original index and label
    original_index = non_noise_indices[min_y_index_in_non_noise]
    min_y_label = labels[original_index]
    return min_y_label

def get_break_skeleton(data_points, output_MSBs_filename, output_BPs_filename,output_CMSBs_filename, datasets_type):
    # undirected graph G, SR =0.01
    G = create_undirected_graph(data_points, 0.01)

    # 1.Apply DBSCAN clustering to segment discontinuous area point cloud
    db = DBSCAN(eps=0.005, min_samples=10).fit(data_points)
    labels = db.labels_
    DBs_labels = set(labels) - {-1}

    min_y_label = get_min_y_label(data_points, labels)

    # Initialize all nodes to -1.
    node_values = {node: -1 for node in G}
    # Calculate the node value for each discontinuous region and update the node value of the entire G
    node_values_clusters = process_labels_threaded(DBs_labels, labels, data_points, G, min_y_label)
    node_values.update(node_values_clusters)

    # according DFS, clustering
    clusters = cluster_points(G, data_points, node_values)

    # 2.Apply DBSCAN to refine branches and alleviate the problems of adjacent branch merging and branch point offset
    clusters = apply_dbscan_to_clusters(clusters, data_points, 0.004, 2)
    # Calculate node coordinates
    skeleton_nodes = calculate_skeleton_nodes(clusters, data_points)
    # get disconnected skeleton
    skeleton = generate_skeleton(skeleton_nodes, clusters, data_points)

    # Clean up skeleton noise, synthetic dataset is not used yet
    if datasets_type == 'reality':
        pruned_skeleton = prune_skeleton(skeleton)
        skeleton_subgraphs = extract_skeleton_subgraphs(pruned_skeleton)
        max_nodes_subgraph = max(skeleton_subgraphs, key=lambda sg: len(sg.nodes))
        # Determine the sub-skeleton with the most nodes as the sub-skeleton and do not smooth it for now
        smoothed_positions_all = []
        for subgraph in skeleton_subgraphs:
            if subgraph == max_nodes_subgraph:
                smoothed_positions = {node: np.array(pos) for node, pos in subgraph.nodes(data='pos')}
            else:
                # Smoothing other sub-skeletons
                node_positions_sub = {node: subgraph.nodes[node]['pos'] for node in subgraph.nodes()}
                smoothed_positions = laplacian_smoothing(subgraph, node_positions_sub, iterations=10, alpha=0.5)

            smoothed_positions_all.append(smoothed_positions)
        # Merge processed sub-skeletons
        skeleton = merge_skeletons(skeleton_subgraphs, smoothed_positions_all)

    # Delete isolated points
    isolated_nodes = [node for node in skeleton.nodes if skeleton.degree(node) == 0]
    skeleton.remove_nodes_from(isolated_nodes)

    save_graph_json(skeleton, output_MSBs_filename)

    print(f"\nWriting: {output_MSBs_filename}")
    print("\n       ===========Completion phase===========")
    completion(skeleton, output_BPs_filename, output_CMSBs_filename)





