import open3d as o3d
import numpy as np
import networkx as nx
import json
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def load_graph_json(filename):
    with open(filename, 'r') as f:
        graph_json = json.load(f)

    if 'links' not in graph_json:
        raise KeyError("JSON file must contain 'links' key")

    return nx.node_link_graph(graph_json)
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
    lines = [[node_to_new_index[start], node_to_new_index[end]] for start, end, _ in skeleton.edges]

    line_colors = []
    for start, end, _ in skeleton.edges:
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
    largest_component = max(components, key=len)
    color_map = {}
    for component in components:
        if component == largest_component:
            color = [0, 0, 1]
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
def main(json_path, pcd_path):
    new_skeleton = load_graph_json(json_path)

    data_points = load_and_preprocess_pcd(pcd_path, 'reality')
    print(len(data_points))
    gray_color = [0.5, 0.5, 0.5]
    colors = np.tile(gray_color, (len(data_points), 1))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data_points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    custom_draw_geometry_with_no_light([pcd])

    color_map = assign_colors_to_components(new_skeleton)

    components = list(nx.connected_components(new_skeleton))
    largest_component = max(components, key=len)
    for component in components:
        if component == largest_component:
            continue
        else:
            color = [1, 0, 0]
        for node in component:
            color_map[node] = color

    point_cloud, line_set = create_point_cloud_and_lineset(new_skeleton, color_map)

    spheres = create_spheres_at_nodes(new_skeleton, color_map)

    custom_draw_geometry_with_no_light([point_cloud] + line_set + spheres)

    gray_color = [0.5, 0.5, 0.5]
    colors = np.tile(gray_color, (len(data_points), 1))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data_points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    custom_draw_geometry_with_no_light([pcd, point_cloud] + line_set + spheres)

if __name__ == "__main__":
    json_path = "../datas/1.json"
    pcd_path = "path/to/pcd"
    main(json_path, pcd_path)

