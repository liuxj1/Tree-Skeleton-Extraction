import networkx as nx
import json
import numpy as np
from functools import lru_cache
from Algorithm1 import Ring_Structure_Detection_and_Filtering
from Algorithm2 import Global_Growth_Direction_Feature_Filtering

def angle_between(v1, v2):
    """
    Computes the angle between two vectors (in degrees).
    param v1: vector 1.
    param v2: vector 2.
    return: The angle, in degrees.
    """
    if v1 is None or v2 is None:
        return 180
    unit_vector_1 = v1 / np.linalg.norm(v1)
    unit_vector_2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
    return np.degrees(angle)
def get_node_info(updated_skeleton, node_to_new_index):
    vectors = []
    for node in updated_skeleton.nodes():
        neighbors = list(updated_skeleton.neighbors(node))  # Get the connected neighbor nodes of this node
        node_pos = np.array(updated_skeleton.nodes[node]['pos'])
        vectors_from_node = []
        neighbor_positions = []
        # Calculate the direction vector from the current node to all neighbors
        for neighbor in neighbors:
            neighbor_pos = np.array(updated_skeleton.nodes[neighbor]['pos'])
            vector = neighbor_pos - node_pos
            vectors_from_node.append(vector)
            neighbor_positions.append(neighbor_pos.tolist())
        # Excluding breakpoints, end points, and root nodes, there are multiple neighboring points
        if len(vectors_from_node) > 1:
            valid_vectors = []
            not_branch_point_flag = False
            for i in range(len(vectors_from_node)):
                for j in range(i + 1, len(vectors_from_node)):
                    # Calculate the vector angle between the node and the two neighboring points. If it is less than 100, it is recorded as a valid vector.
                    angle = angle_between(vectors_from_node[i], vectors_from_node[j])
                    if angle < 100:
                        if tuple(vectors_from_node[i]) not in map(tuple, valid_vectors):
                            valid_vectors.append(vectors_from_node[i])
                        if tuple(vectors_from_node[j]) not in map(tuple, valid_vectors):
                            valid_vectors.append(vectors_from_node[j])
                    else:
                        # If the angle between the skeleton node and the neighboring node vector does not meet the conditions, the non-bifurcation point flag is set to True.
                        not_branch_point_flag = True

            # If there are vectors that meet the conditions, calculate the average vector of these vectors
            if valid_vectors and not not_branch_point_flag:
                vectors_from_node = []
                average_vector = np.mean(valid_vectors, axis=0)
                vectors_from_node.append(average_vector)
        # Record each node information: node index, corresponding mapped node index, node coordinates, neighbor node coordinates and direction vector
        vectors.append({
            "node_index": node,
            "new_point_cloud_index": node_to_new_index[node],
            "node_pos": node_pos.tolist(),
            "neighbor_pos": neighbor_positions,
            "vector": [vector.tolist() for vector in vectors_from_node]
        })
    return vectors

def find_connected_groups(skeleton):
    """Find all groups of nodes in the skeleton graph connected by adjacent edges"""
    connected_components = list(nx.connected_components(skeleton))
    connected_groups = [list(group) for group in connected_components]
    return connected_groups
def calculate_vector_angle(vector_1, start_pos, vector_2,end_pos):
    """Calculates the angle (in degrees) between a vector and the vector formed by two points"""
    vector_1 = np.atleast_2d(vector_1)
    vector_2 = np.atleast_2d(vector_2)

    direction_vector_1 = np.array(end_pos) - np.array(start_pos)
    unit_direction_vector_1 = direction_vector_1 / np.linalg.norm(direction_vector_1)

    direction_vector_2 = np.array(start_pos) - np.array(end_pos)
    unit_direction_vector_2 = direction_vector_2 / np.linalg.norm(direction_vector_2)
    A = []
    for index, vector_1_row in enumerate(vector_1):
        unit_vector_1_row = vector_1_row / np.linalg.norm(vector_1_row)
        dot_product_1_row = np.dot(unit_direction_vector_1, unit_vector_1_row)
        angle_1_row = np.arccos(np.clip(dot_product_1_row, -1.0, 1.0))

        for index, vector_2_row in enumerate(vector_2):
            unit_vector_2_row = vector_2_row / np.linalg.norm(vector_2_row)
            dot_product_2_row = np.dot(unit_direction_vector_2, unit_vector_2_row)
            angle_2_row = np.arccos(np.clip(dot_product_2_row, -1.0, 1.0))

            dot_product_3 = np.dot(unit_vector_1_row, unit_vector_2_row)
            angle_3_row = np.arccos(np.clip(dot_product_3, -1.0, 1.0))
            A.append([np.degrees(angle_1_row), np.degrees(angle_2_row), np.degrees(angle_3_row)])
    return np.array(A)
@lru_cache(maxsize=None)
def cached_calculate_distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))
def find_pairs(vectors_1, vectors_2):
    '''
    Calculate the top 10 optimal connection pairs that meet the distance and angle parameters
    '''
    pairs = []
    for vector_a in vectors_1:
        # Select nodes and bifurcations with degree 1
        if len(vector_a['vector']) != 1:
            continue
        for vector_b in vectors_2:
            D = cached_calculate_distance(tuple(vector_a['node_pos']), tuple(vector_b['node_pos']))
            if D >= 0.1:
                continue
            A = calculate_vector_angle(np.array(vector_a['vector']), vector_a['node_pos'], np.array(vector_b['vector']), vector_b['node_pos'])
            valid_rows = [row for row in A if row[0] > 120 and row[1] > 90 and row[2] > 90]
            if valid_rows:
                angle_value_123 = valid_rows[0][0]
                pairs.append([vector_a['node_index'], vector_b['node_index'], D, angle_value_123, vector_a['new_point_cloud_index'], vector_b['new_point_cloud_index']])
    if not pairs:
        return None
    pairs_distance_sorted = sorted(pairs, key=lambda x: x[2])
    pairs_angle_123_sorted = sorted(pairs, key=lambda x: x[3],  reverse=True)
    angle_max = pairs_angle_123_sorted[0][3]
    close_pairs = [pair for pair in pairs_distance_sorted if abs(pair[3] - angle_max) <= 40]
    return close_pairs[0:10]

def group_vectors_by_connected_groups(vectors, connected_groups):
    grouped_vectors = []

    for group in connected_groups:
        current_group_vectors = []
        for vector in vectors:
            if vector['node_index'] in group:
                current_group_vectors.append(vector)
        if current_group_vectors:
            grouped_vectors.append(current_group_vectors)
    return grouped_vectors
def select_based_on_rule(values):
    pairs_distance_sorted = sorted(values, key=lambda x: x[2])  # 根据距离排序
    pairs_angle_123_sorted = sorted(values, key=lambda x: x[3], reverse=True)
    # Define the distance difference threshold
    distance_threshold = 0.02
    # Get the distance of the first element in the pairs_distance_sorted list, the closest
    target_distance_ij = pairs_distance_sorted[0][2]
    # Find all elements in pairs_angle_123_sorted whose distance difference is not greater than the threshold
    close_pairs_ij = [pair for pair in pairs_angle_123_sorted if
                   abs(pair[2] - target_distance_ij) <= distance_threshold]
    return close_pairs_ij[:10]

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
    convert_ndarray_to_list(graph)  # 转换所有ndarray到list
    graph_json = nx.node_link_data(graph)  # 将图转换为节点-链接格式适合JSON
    with open(filename, 'w') as f:
        json.dump(graph_json, f, indent=4)

def completion(updated_skeleton, output_BPs_filename, output_CMSBs_filename):
    # Create a dictionary mapping to the new index
    node_to_new_index = {node: i for i, node in enumerate(updated_skeleton.nodes)}
    # Create a dictionary to store the coordinates of the skeleton nodes
    skeleton_nodes = {node: data['pos'] for node, data in updated_skeleton.nodes(data=True)}

    # Get skeleton node information
    #     "node_index": node,
    #     "new_point_cloud_index": node_to_new_index[node],
    #     "node_pos": node_pos.tolist(),
    #     "neighbor_pos": neighbor_positions,
    #     "vector": [vector.tolist() for vector in vectors_from_node]
    vectors = get_node_info(updated_skeleton, node_to_new_index)

    # Identify disconnected skeleton groups，SG
    SG = find_connected_groups(updated_skeleton)
    # Divide the node information according to the previous skeleton group
    grouped_vectors = group_vectors_by_connected_groups(vectors, SG)

    all_pairs = []
    # Traversing different skeleton node groups
    for i in range(len(grouped_vectors)):
        pairs_i = []
        for j in range(len(grouped_vectors)):
            if i != j:
                # 获取要配对的两个列表
                first_list = grouped_vectors[i]
                second_list = grouped_vectors[j]

                # Call the find_pairs function to select the top 10 node pairs that meet the conditions from each skeleton group
                # as candidate connection pairs based on the distance parameter D and the angle parameter A.
                pairs = find_pairs(first_list, second_list)
                pairs_i.append(pairs)
            else:
                pairs_i.append(None)
        # Stores the connection pairs between skeleton group i and other skeleton groups
        all_pairs.append(pairs_i)

    # 一个闭环中必有一条错误连接
    position_index = []
    for index_a, values_a in enumerate(all_pairs):
        for index_b, values_b in enumerate(values_a):
            if values_b is not None:
                # Change the expression format, index_a represents skeleton group 1, index_b represents skeleton group 2.
                # values_b represents the connection pair between the two skeleton groups
                position_index.append([(index_a, index_b), values_b])
    # Convert position_index to a dictionary and combine the two key forms of a and b:
    # (a, b) and (b, a) into one (a, b)
    grouped_pairs = {}
    for item in position_index:
        key, values = item[0], item[1]
        key = tuple(sorted(key))
        # If the key does not exist in grouped_pairs, initialize an empty list
        if key not in grouped_pairs:
            grouped_pairs[key] = []
        # Merge the values into the corresponding keys
        grouped_pairs[key].extend(values)
    # grouped_pairs stores possible connection pairs between different skeleton groups,
    # but is currently merged, where each group of connection pairs exceeds 10 groups
    # Apply selection rules to determine the best 10 connection pairs between each skeleton-skeleton group
    candidate_pairs_info = {}
    for key in grouped_pairs:
        candidate_pairs_info[key] = select_based_on_rule(grouped_pairs[key])

    # Record the number of iterations, i
    i = 0
    while True:
        first_pair_info = {key: value[0] for key, value in
                           candidate_pairs_info.items()}  # Select the first value of a different key in a candidate pair information
        pairs_index_data = {key: [(pair[0], pair[1]) for pair in value] for key, value in
                            candidate_pairs_info.items()}  # Extract paired point index among candidate pairs information
        # Algorithm 1 Ring Structure Detection and Filtering
        # Step1: Connect the candidate pairs in first_pair_info and screen those candidate pairs that result in a skeleton ring structure
        first_pair_info = Ring_Structure_Detection_and_Filtering(first_pair_info)
        # first_pair_info is the value remaining after clearing the connection pairs that cause the skeleton closed loop.
        finally_pairs = [tuple(info[0:2]) for _, info in first_pair_info.items()]
        new_skeleton = updated_skeleton.copy()
        # Complete the skeleton line according to the latest connection pair
        for pair in finally_pairs:
            new_skeleton.add_edge(*pair)

        # Algorithm 2 Global Growth Direction Feature Filtering
        # Step2: Combined with the global growth laws of trees, filter out wrong connection pairs
        note_filtered_pairs = Global_Growth_Direction_Feature_Filtering(new_skeleton, skeleton_nodes, finally_pairs)

        i += 1
        # If there is a filtered connection pair, filter it, otherwise exit the loop
        if len(note_filtered_pairs) != 0:
            for key, pairs in pairs_index_data.items():
                for pair in pairs:
                    if pair in note_filtered_pairs or (pair[1], pair[0]) in note_filtered_pairs:
                        if key in candidate_pairs_info:
                            if len(candidate_pairs_info[key]) > 1:
                                candidate_pairs_info[key].pop(0)
                            else:
                                del candidate_pairs_info[key]
        else:
            break

    print(f"Completion iteration count i = {i}")

    with open(output_BPs_filename, 'w') as f:
        f.write(str(finally_pairs)+'\n')
        f.write(str(len(grouped_vectors)))

    save_graph_json(new_skeleton, output_CMSBs_filename)
    print(f"\nWriting: {output_CMSBs_filename}")






