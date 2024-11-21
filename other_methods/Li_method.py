import matplotlib
matplotlib.use('agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
import open3d as o3d
import json
from scipy.spatial.transform import Rotation as R
from skeleton.process import *

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
            line_colors.append([1, 0, 0])  # 绿色
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
def assign_colors_to_components(skeleton):
    components = list(nx.connected_components(skeleton))
    positions = nx.get_node_attributes(skeleton, 'pos')

    # Calculate the lowest y-value for each component and find the component with the lowest y-value
    lowest_y_component = min(components, key=lambda comp: min(positions[v][1] for v in comp))
    largest_component = lowest_y_component

    color_map = {}
    for component in components:
        if component == largest_component:
            color = [0.72, 0.52, 0.04]
        else:
            color = [0, 0, 0]
        for node in component:
            color_map[node] = color
    return color_map

def get_break_skeleton(data_points):
    gray_color = [0.5, 0.5, 0.5]  # RGB value of gray
    colors = np.tile(gray_color, (len(data_points), 1))
    pcd_1 = o3d.geometry.PointCloud()
    pcd_1.points = o3d.utility.Vector3dVector(data_points)
    pcd_1.colors = o3d.utility.Vector3dVector(colors)
    # undirected graph G, SR =0.01
    graph = create_undirected_graph(data_points, 0.01)
    # choose root set X
    root_set_cluster = select_root_set(data_points, 0.02)
    # computes node values
    node_values = calculate_node_values(graph, root_set_cluster, 60)
    # according DFS, clustering
    clusters = cluster_points(graph, data_points, node_values)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data_points)
    point_colors = np.zeros((len(data_points), 3))
    # Generate a random color for each cluster and apply it to all points in the cluster
    for cluster in clusters:
        cluster_color = np.random.rand(3)
        for point_id in cluster:
            point_colors[point_id] = cluster_color

    # Assign the color array to the point cloud object
    pcd.colors = o3d.utility.Vector3dVector(point_colors)
    custom_draw_geometry_with_no_light([pcd])

    colors = np.zeros((data_points.shape[0], 3))
    print(colors)
    for i, label in enumerate(colors):
        colors[i] = [0, 1, 0]
    skeleton_nodes, skeleton_colors = calculate_skeleton_nodes(clusters, data_points, colors)
    # Generate Skeleton
    skeleton = generate_skeleton(skeleton_nodes, skeleton_colors, clusters, data_points)
    # Clean skeleton, synthetic dataset is not used
    cleaned_skeleton = prune_skeleton(skeleton)
    # cleaned_skeleton = skeleton
    isolated_nodes = [node for node in cleaned_skeleton.nodes if cleaned_skeleton.degree(node) == 0]
    cleaned_skeleton.remove_nodes_from(isolated_nodes)

    color_map = assign_colors_to_components(cleaned_skeleton)
    point_cloud, line_set = create_point_cloud_and_lineset(cleaned_skeleton, color_map)

    spheres = create_spheres_at_nodes(cleaned_skeleton, color_map)

    custom_draw_geometry_with_no_light([point_cloud] + line_set + spheres)
    custom_draw_geometry_with_no_light([pcd_1, point_cloud] + line_set + spheres)


if __name__ == "__main__":
    input_pcd_path = "path/to/pcd_file"     # Input incomplete tree point cloud

    data_points = load_and_preprocess_pcd(input_pcd_path)
    get_break_skeleton(data_points)

