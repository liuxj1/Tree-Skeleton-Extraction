from collections import deque
import numpy as np

def compute_direction_vector(p1, p2):
    """
    Calculates and returns the unit direction vector from point p1 to point p2.
    param p1: Starting point coordinates.
    param p2: End point coordinates.
    return: Unit direction vector.
    """
    vector = np.array(p2) - np.array(p1)
    norm = np.linalg.norm(vector)
    if norm == 0:
        return None
    return vector / norm
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
def find_root_node(skeleton):
    # Select the node with the lowest y coordinate as the root node
    min_y = float('inf')
    root_node = None
    for node, data in skeleton.nodes(data=True):
        if data['pos'][1] < min_y:
            min_y = data['pos'][1]
            root_node = node
    return root_node
def find_parent_child_relations(skeleton):
    root_node = find_root_node(skeleton)
    parent_child_relations = {}
    visited = set()
    queue = deque([root_node])
    while queue:
        current_node = queue.popleft()
        if current_node not in visited:
            visited.add(current_node)
            for neighbor in list(skeleton.neighbors(current_node)):
                if neighbor not in visited:
                    if current_node not in parent_child_relations:
                        parent_child_relations[current_node] = []
                    parent_child_relations[current_node].append(neighbor)
                    queue.append(neighbor)
    return parent_child_relations
# Define a recursive function to find descendant nodes
def find_grandchildren(parent, child, angle, skeleton_nodes, parent_child_relations, finally_pairs_set):
    note_flag =False
    parent_to_child_vector = compute_direction_vector(skeleton_nodes[parent], skeleton_nodes[child])
    grandchildren = parent_child_relations.get(child, [])

    for grandchild in grandchildren:
        if (child, grandchild) not in finally_pairs_set:
            child_to_grandchild_vector = compute_direction_vector(skeleton_nodes[child], skeleton_nodes[grandchild])

            has_grandchild_children = grandchild in parent_child_relations and len(
                parent_child_relations[grandchild]) > 0

            if angle_between(parent_to_child_vector, child_to_grandchild_vector) > angle and has_grandchild_children:
                note_flag = True
    if note_flag:
        return True
    else:
        for grandchild in grandchildren:
            if (child, grandchild) not in finally_pairs_set:
                # Define a recursive function to find descendant nodes
                result = find_grandchildren(child, grandchild, 110, skeleton_nodes, parent_child_relations, finally_pairs_set)
                return result
            else:
                return False
def filter_parent_child_relations(parent_child_relations, skeleton_nodes, finally_pairs):
    note_filtered_relations = []
    finally_pairs_set = set(finally_pairs) | set((b, a) for a, b in finally_pairs)

    for parent, children in parent_child_relations.items():
        find_flag = False
        for child in children:
            if (parent, child) in finally_pairs_set:
                grandparents = [gp for gp, gc in parent_child_relations.items() if parent in gc]
                for grandparent in grandparents:
                    grandparent_to_parent_vector = compute_direction_vector(skeleton_nodes[grandparent], skeleton_nodes[parent])
                    parent_to_child_vector = compute_direction_vector(skeleton_nodes[parent], skeleton_nodes[child])
                    if angle_between(grandparent_to_parent_vector, parent_to_child_vector) > 90:
                        note_filtered_relations.append((parent, child))
                        find_flag = True
                        break
                if not find_flag:
                    if find_grandchildren(parent, child, 90, skeleton_nodes, parent_child_relations, finally_pairs_set):
                        note_filtered_relations.append((parent, child))
    return note_filtered_relations
def process_vectors_and_remove_edges(graph, parent_child_relations, skeleton_nodes, note_filtered_pairs, finally_pairs):
    for parent, children in parent_child_relations.items():
        # Checks if a node has two children
        if len(children) == 2:
            child1, child2 = children

            # Calculate the vector from the current node to the two child nodes
            v1 = compute_direction_vector(skeleton_nodes[parent], skeleton_nodes[child1])
            v2 = compute_direction_vector(skeleton_nodes[parent], skeleton_nodes[child2])

            # Calculate the angle between vectors
            cos_theta = np.clip(angle_between(v1, v2), -1.0, 1.0)
            angle = np.degrees(np.arccos(cos_theta))

            # If the angle is greater than 150°, delete the edge from the parent node to the current node
            if angle > 150:
                for grandparent, grandparent_children in parent_child_relations.items():
                    if parent in grandparent_children:
                        if graph.has_edge(grandparent, parent) and (grandparent, parent) in finally_pairs:
                            note_filtered_pairs.append((grandparent, parent))
                        else:
                            print(f"Edge {grandparent}-{parent} does not exist. Skipping.")
                        break
    return note_filtered_pairs


def Global_Growth_Direction_Feature_Filtering(new_skeleton,skeleton_nodes, finally_pairs):

    parent_child_relations = find_parent_child_relations(
        new_skeleton)  # Establish a global parent-child relationship growth chart

    # Filter bad link pairs
    note_filtered_pairs = filter_parent_child_relations(parent_child_relations, skeleton_nodes, finally_pairs)
    note_filtered_pairs = process_vectors_and_remove_edges(new_skeleton, parent_child_relations, skeleton_nodes,
                                                           note_filtered_pairs, finally_pairs)
    return note_filtered_pairs
