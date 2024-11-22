import open3d as o3d
import argparse
import json
import matplotlib
from scipy.spatial.transform import Rotation as R
matplotlib.use('agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
from skeleton.process import *

def load_graph_json(filename):
    with open(filename, 'r') as f:
        graph_json = json.load(f)
    return nx.node_link_graph(graph_json)

def extract_skeleton_subgraphs(skeleton):
    """
    Extract all connected components from a complete skeleton graph, returning each component as a complete subgraph.
    :param skeleton: The complete NetworkX graph.
    :return: A list containing all subgraphs.
    """
    connected_components = nx.connected_components(skeleton)
    subgraphs = [skeleton.subgraph(component).copy() for component in connected_components]
    return subgraphs
def laplacian_smoothing(skeleton, node_positions, iterations=5, alpha=0.5):
    """
    Perform Laplace smoothing on a skeleton graph.
    :param skeleton: NetworkX graph representing the skeleton.
    :param node_positions: A dictionary whose keys are nodes and whose values are the (x, y, z) coordinates of the nodes.
    :param iterations: The number of smoothing iterations.
    :param alpha: Smoothing factor that controls the interpolation weight between new and old positions.
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
def smooth_all_subgraphs(subgraphs):
    smoothed_positions_all = []
    for subgraph in subgraphs:
        node_positions_sub = {node: subgraph.nodes[node]['pos'] for node in subgraph.nodes()}
        smoothed_positions_sub = laplacian_smoothing(subgraph, node_positions_sub, iterations=10, alpha=0.5)
        smoothed_positions_all.append(smoothed_positions_sub)
    return smoothed_positions_all
def merge_skeletons(subgraphs, smoothed_positions_all):
    merged_skeleton = nx.Graph()
    for subgraph, smoothed_positions in zip(subgraphs, smoothed_positions_all):
        for node in subgraph.nodes():
            merged_skeleton.add_node(node, pos=smoothed_positions[node])
        for u, v in subgraph.edges():
            merged_skeleton.add_edge(u, v)
    return merged_skeleton

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
def assign_colors_to_components(skeleton):
    components = list(nx.connected_components(skeleton))
    positions = nx.get_node_attributes(skeleton, 'pos')

    # Calculate the lowest y-value for each component and find the component with the lowest y-value
    lowest_y_component = min(components, key=lambda comp: min(positions[v][1] for v in comp))
    largest_component = lowest_y_component

    color_map = {}
    for component in components:
        if component == largest_component:
            color = [0, 1, 0]
        else:
            color = [0, 0, 0]
        for node in component:
            color_map[node] = color
    return color_map

def save_image(vis):
    image = vis.capture_screen_float_buffer(do_render=True)
    plt.figure(figsize=(10, 10), dpi=300)
    plt.imshow(np.asarray(image))
    plt.axis('off')
    plt.savefig(r"D:\experimental_data\test_images\1.svg", format='svg')

    plt.close()
    print("Screenshot saved as 'screenshot.svg' and 'screenshot.pdf'")
    return False
def rotate_view(vis):
    """
    The callback function for rotating the view.
    """
    ctr = vis.get_view_control()
    ctr.rotate(5, 0)  # 每次调用旋转10度，0表示沿水平轴旋转
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
    vis.register_key_callback(ord("S"), lambda vis: save_image(vis))

    # Registering a rotation callback function
    # vis.register_animation_callback(rotate_view)

    vis.run()
    vis.destroy_window()

def process_and_draw(new_skeleton, color_map, finally_pairs, process=True):
    if process:
        skeleton_subgraphs = extract_skeleton_subgraphs(new_skeleton)
        smoothed_positions_all = smooth_all_subgraphs(skeleton_subgraphs)
        new_skeleton = merge_skeletons(skeleton_subgraphs, smoothed_positions_all)

    point_cloud, line_set = create_point_cloud_and_lineset(new_skeleton, color_map, finally_pairs)
    spheres = create_spheres_at_nodes(new_skeleton, color_map)
    custom_draw_geometry_with_no_light([point_cloud] + line_set + spheres)

def main(CMSBs_path,BPs_path):
    new_skeleton = load_graph_json(CMSBs_path)
    # data_points = load_and_preprocess_pcd(pcd_path, 'reality')
    # gray_color = [0.5, 0.5, 0.5]  # 灰色的RGB值
    # colors = np.tile(gray_color, (len(data_points), 1))
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(data_points)
    # pcd.colors = o3d.utility.Vector3dVector(colors)
    # custom_draw_geometry_with_no_light([pcd])

    with open(BPs_path, 'r') as f:
        content_1 = f.readline()
        content_2 = f.readline()
        finally_pairs = eval(content_1)
        all_break_num = eval(content_2) - 1

    color_map = assign_colors_to_components(new_skeleton)

    for pair in finally_pairs:
        new_skeleton.add_edge(*pair)

    components = list(nx.connected_components(new_skeleton))

    largest_component = max(components, key=len)
    for component in components:
        if component == largest_component:
            continue
        else:
            color = [0, 0, 0]
        for node in component:
            color_map[node] = color

    # No Laplace smoothing
    process_and_draw(new_skeleton, color_map, finally_pairs, process=False)

    # Laplace smoothing
    process_and_draw(new_skeleton, color_map, finally_pairs, process=True)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Run Visualization Progress")
    parser.add_argument('--cmsbs', required=True, help="Path to the CMSBs file")
    parser.add_argument('--bps', required=True, help="Path to the BPs file")
    args = parser.parse_args()

    CMSBs_path = args.cmsbs
    BPs_path = args.bps

    main(CMSBs_path, BPs_path)



