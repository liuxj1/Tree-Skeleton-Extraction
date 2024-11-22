from tqdm import tqdm
import open3d as o3d
import json
from scipy.spatial import distance_matrix
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
from skeleton.process import *


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

def extract_subgraph(graph, nodes):
    """
    Extract the subgraph of the specified node from the graph.
    graph: adjacency list in dictionary form.
    nodes: the set of nodes to be extracted (set type).
    """
    node_set = set(nodes)
    subgraph = {node: neighbors & node_set for node, neighbors in graph.items() if node in node_set}
    return subgraph
def generate_mst_skeleton(skeleton_nodes):
    # Calculate the distance matrix between all skeleton nodes
    dist_matrix = distance_matrix(skeleton_nodes, skeleton_nodes)

    G = nx.complete_graph(len(skeleton_nodes))
    for i in range(len(skeleton_nodes)):
        for j in range(i + 1, len(skeleton_nodes)):
            G.add_edge(i, j, weight=dist_matrix[i][j])

    # Generate a minimum spanning tree
    mst = nx.minimum_spanning_tree(G, weight='weight')
    # Add location attribute to each node of MST
    for i, node in enumerate(mst.nodes):
        mst.nodes[node]['pos'] = skeleton_nodes[i]

    lines = []
    for u, v in mst.edges():
        lines.append([u, v])

    return mst, lines
def assign_colors_to_components(skeleton):
    components = list(nx.connected_components(skeleton))
    positions = nx.get_node_attributes(skeleton, 'pos')

    # Calculate the lowest y-value for each component and find the component with the lowest y-value
    lowest_y_component = min(components, key=lambda comp: min(positions[v][1] for v in comp))
    largest_component = lowest_y_component

    color_map = {}
    for component in components:
        if component == largest_component:
            color = [0.5, 0, 0.5]
        else:
            color = [0, 0, 0]
        for node in component:
            color_map[node] = color
    return color_map
def lines_to_cylinders(line_set, radius=0.000001):
    cylinders = []
    points = np.asarray(line_set.points)
    lines = np.asarray(line_set.lines)
    colors = np.asarray(line_set.colors)

    for i, (start, end) in enumerate(lines):
        start_point = points[start]
        end_point = points[end]
        color = colors[i]

        height = np.linalg.norm(start_point - end_point)
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
        cylinder.paint_uniform_color(color)

        # Align cylinder with line
        direction = (end_point - start_point) / height
        mid_point = (start_point + end_point) / 2

        # Create rotation matrix
        z_axis = np.array([0, 0, 1])
        rotation_vector = np.cross(z_axis, direction)
        rotation_vector_length = np.linalg.norm(rotation_vector)

        if rotation_vector_length != 0:
            rotation_vector = rotation_vector / rotation_vector_length
            angle = np.arccos(np.dot(z_axis, direction))
            rotation_matrix = R.from_rotvec(rotation_vector * angle).as_matrix()
        else:
            rotation_matrix = np.eye(3)

        cylinder.rotate(rotation_matrix, center=np.array([0, 0, 0]))
        cylinder.translate(mid_point)

        cylinders.append(cylinder)

    return cylinders
def create_point_cloud_and_lineset(skeleton, color_map, new_pairs=None):
    points = np.array([skeleton.nodes[node]['pos'] for node in skeleton.nodes])
    node_to_new_index = {node: i for i, node in enumerate(skeleton.nodes)}
    lines = [[node_to_new_index[start], node_to_new_index[end]] for start, end in skeleton.edges]

    line_colors = []
    for start, end in skeleton.edges:
        if new_pairs and ((start, end) in new_pairs or (end, start) in new_pairs):
            line_colors.append([1, 0, 0])
        else:
            line_colors.append(color_map.get(start, [0, 0, 0]))

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(line_colors)
    # Convert lines to pipes to achieve thickening effect
    cylinders = lines_to_cylinders(line_set, radius=0.001)
    return point_cloud, cylinders
def create_spheres_at_nodes(skeleton, color_map, radius=0.002):
    spheres = []
    for node in skeleton.nodes():
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        sphere.paint_uniform_color(color_map[node] if node in color_map else [0, 0, 0])
        sphere.translate(skeleton.nodes[node]['pos'])
        spheres.append(sphere)
    return spheres
def save_image(vis):
    image = vis.capture_screen_float_buffer(do_render=True)
    plt.figure(figsize=(10, 10), dpi=300)
    plt.imshow(np.asarray(image))
    plt.axis('off')
    plt.savefig(r"C:\Users\Administrator\Desktop\test\1.svg", format='svg')

    plt.close()
    print("Screenshot saved as 'screenshot.svg' and 'screenshot.pdf'")
    return False
def custom_draw_geometry_with_no_light(geometries):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()

    for geometry in geometries:
        vis.add_geometry(geometry)

    render_option = vis.get_render_option()
    render_option.light_on = False
    render_option.point_size = 2
    render_option.line_width = 50
    render_option.background_color = np.array([1, 1, 1])
    # Register shortcut keys (e.g. save image when 'S' key is pressed)
    vis.register_key_callback(ord("S"), save_image)
    vis.run()
    vis.destroy_window()


def get_break_skeleton(data_points):
    # undirected graph G, SR =0.01
    G = create_undirected_graph(data_points, 0.01)
    # DBSCAN segmentation disconnected regions
    db = DBSCAN(eps=0.005, min_samples=10).fit(data_points)
    labels = db.labels_
    unique_labels = set(labels) - {-1}
    # Initialize the node value to -1
    node_values = {node: -1 for node in G}
    # Calculate node values for disconnected regions
    for label in tqdm(unique_labels, desc="Processing labels"):
        current_cluster_indices = np.where(labels == label)[0]
        cluster_points_data = data_points[current_cluster_indices]
        # Sub-undirected graph
        Gi = extract_subgraph(G, current_cluster_indices)
        # Set the root node X threshold
        X_threshold = 0.02
        # Root node collection  X
        X = select_label_root_set(cluster_points_data, current_cluster_indices, X_threshold)
        # Calculate node value
        node_values_clusters = calculate_node_values(Gi, X, 60)
        node_values.update(node_values_clusters)

    clusters = cluster_points(G, data_points, node_values)

    clusters = apply_dbscan_to_clusters(clusters, data_points, 0.004, 2)


    skeleton_nodes = calculate_skeleton_nodes(clusters, data_points)


    # Generate MST and skeleton lines
    mst, skeleton_lines = generate_mst_skeleton(skeleton_nodes)

    # Clean skeleton, synthetic dataset is not used
    cleaned_skeleton = prune_skeleton(mst)
    isolated_nodes = [node for node in cleaned_skeleton.nodes if cleaned_skeleton.degree(node) == 0]
    cleaned_skeleton.remove_nodes_from(isolated_nodes)

    color_map = assign_colors_to_components(cleaned_skeleton)
    point_cloud, line_set = create_point_cloud_and_lineset(cleaned_skeleton, color_map)

    gray_color = [0.5, 0.5, 0.5]
    colors = np.tile(gray_color, (len(data_points), 1))
    pcd_h = o3d.geometry.PointCloud()
    pcd_h.points = o3d.utility.Vector3dVector(data_points)
    pcd_h.colors = o3d.utility.Vector3dVector(colors)
    # custom_draw_geometry_with_no_light([pcd_h])

    spheres = create_spheres_at_nodes(cleaned_skeleton, color_map)

    custom_draw_geometry_with_no_light([pcd_h, point_cloud] + line_set + spheres)

if __name__ == "__main__":
    txt_path = "../datas/input_pcd/reality_example1.txt"
    data_points = load_and_preprocess_pcd(txt_path, "reality")
    get_break_skeleton(data_points)

