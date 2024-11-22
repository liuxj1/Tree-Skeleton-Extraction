import random
import json
from skeleton.process import *

def is_valid_candidate(node, degree_one_nodes, graph, min_degree_one_nodes=5, min_distance=5):
    """Checks if the node is more than min_distance away from at least min_degree_one_nodes nodes of degree 1."""
    count = 0
    for d1_node in degree_one_nodes:
        if nx.has_path(graph, node, d1_node):
            if nx.shortest_path_length(graph, source=node, target=d1_node) > min_distance:
                count += 1
            if count >= min_degree_one_nodes:
                return True
    return False
def filter_nodes_by_pairwise_distance(candidates, graph, min_distance=5):
    """Filter out the node sets from the candidate nodes whose shortest paths between each node are greater than min_distance."""
    valid_nodes = []
    for node in candidates:
        try:
            if all(nx.shortest_path_length(graph, source=node, target=other) > min_distance for other in valid_nodes):
                valid_nodes.append(node)
        except nx.NetworkXNoPath:
            valid_nodes.append(node)
    return valid_nodes
def select_valid_nodes(graph, all_num, min_degree_one_nodes=5, min_distance=5):
    """Select a set of nodes that meet the following conditions:
        - The distance between them and at least min_degree_one_nodes nodes with degree 1 exceeds min_distance.
        - The shortest path between any two nodes in the set also exceeds min_distance.
    """
    # Get nodes with degree 1
    degree_one_nodes = [node for node, degree in graph.degree() if degree == 1]
    # Initial screening: nodes that are more than min_distance away from at least min_degree_one_nodes nodes with degree 1
    candidates = [
        node for node in graph.nodes()
        if is_valid_candidate(node, degree_one_nodes, graph, min_degree_one_nodes, min_distance)
    ]
    # Secondary screening: Ensure that the shortest path between candidate nodes exceeds min_distance
    valid_candidates = filter_nodes_by_pairwise_distance(candidates, graph, min_distance)
    # Randomly select the target number of nodes
    if len(valid_candidates) > all_num:
        selected_nodes = random.sample(valid_candidates, all_num)
    else:
        selected_nodes = valid_candidates
    return selected_nodes
def remove_selected_points(data_points, all_points):
    # First, convert the points to be deleted all_points into a unique set for quick inspection
    unique_points_to_remove = {tuple(point) for point in all_points}
    # Then, build a new array containing only the points that are not in the deleted set
    filtered_points = np.array([point for point in data_points if tuple(point) not in unique_points_to_remove])
    return filtered_points
def save_points_to_txt(data_points, filename):
    data_points = np.asarray(data_points)
    # Save point cloud data to file
    np.savetxt(filename, data_points, fmt='%f', delimiter=' ')
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

def imitate_MBs(data_points, disconnections_num, imitate_MB_pcd_file, imitate_skeleton_file):
    # undirected graph G, SR =0.01
    imitate_G = create_undirected_graph(data_points, 0.01)
    # choose root set X
    imitate_X = select_root_set(data_points, 0.01)
    # computes node values
    imitate_node_values = calculate_node_values(imitate_G, imitate_X, 60)
    # according DFS, clustering
    imitate_clusters = cluster_points(imitate_G, data_points, imitate_node_values)
    # DBSCAN
    eps = 0.004
    min_samples = 2
    imitate_clusters = apply_dbscan_to_clusters(imitate_clusters, data_points, eps, min_samples)
    # Calculate cluster centers, get skeleton nodes
    imitate_skeleton_nodes = calculate_skeleton_nodes(imitate_clusters, data_points)
    # get skeleton
    imitate_skeleton = generate_skeleton(imitate_skeleton_nodes, imitate_clusters, data_points)

    # Randomly select the disconnections_num skeleton nodes and remove the clusters corresponding to the skeleton nodes
    select_nodes = select_valid_nodes(imitate_skeleton, disconnections_num)
    all_points = []
    for i in select_nodes:
        select_cluster_points = data_points[imitate_clusters[i]]
        all_points.extend(select_cluster_points)
    # data_points is a tree point cloud with disconnected areas simulated by the synthetic data set
    data_points = remove_selected_points(data_points, all_points)
    # Save the simulated tree point cloud with disconnected areas
    save_points_to_txt(data_points, imitate_MB_pcd_file)
    print(f"\nProcessed and saved: {imitate_MB_pcd_file}")
    # Save the complete skeleton of the synthetic data for comparison after completion
    save_graph_json(imitate_skeleton, imitate_skeleton_file)
    print(f"\nProcessed and saved: {imitate_skeleton_file}")
    return data_points

