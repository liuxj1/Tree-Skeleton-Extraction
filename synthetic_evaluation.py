import open3d as o3d
import numpy as np
import networkx as nx
import json
import argparse
from tqdm import tqdm
import os
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import random

def load_graph_json(filename):
    with open(filename, 'r') as f:
        graph_json = json.load(f)
    return nx.node_link_graph(graph_json)

def create_point_cloud_and_lineset(skeleton, color_for_lines, finally_pairs):
    points = np.array([skeleton.nodes[node]['pos'] for node in skeleton.nodes])
    node_to_new_index = {node: i for i, node in enumerate(skeleton.nodes)}
    lines = [[node_to_new_index[start], node_to_new_index[end]] for start, end in skeleton.edges]

    line_colors = [color_for_lines if (start, end) in finally_pairs or (end, start) in finally_pairs else [0, 0, 0] for start, end in skeleton.edges]

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(line_colors)

    return point_cloud, line_set

def create_spheres_at_points(point_cloud, color, radius=0.002):
    spheres = []
    for point in np.asarray(point_cloud.points):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        sphere.paint_uniform_color(color)
        sphere.translate(point)
        spheres.append(sphere)
    return spheres

def extract_finally_pairs_nodes(skeleton, pairs):
    """
    Extract the position information of a specific pair of nodes from the skeleton.

    Parameters:
    skeleton: networkx.Graph - the skeleton graph.
    pairs: list of tuple - the list of node pairs to be extracted.

    Returns:
    pair_nodes_positions: list of np.array - the list of position information of the pair of nodes.
    """
    pair_nodes_positions = []
    for start, end in pairs:
        if start in skeleton.nodes and end in skeleton.nodes:
            start_pos = skeleton.nodes[start]['pos']
            end_pos = skeleton.nodes[end]['pos']
            pair_nodes_positions.append((start_pos, end_pos))
    return pair_nodes_positions

def distance(point1, point2):
    """Compute the Euclidean distance between two points"""
    return np.linalg.norm(np.array(point1) - np.array(point2))

def find_closest_nodes(skeleton, pair_positions):
    """
    Find the closest node to a given position in a given skeleton.

    Parameters:
    skeleton: networkx.Graph - the skeleton graph.
    pair_positions: list of tuples - contains the node pairs whose positions need to be matched.

    Returns:
    closest_nodes_positions: list of tuples - the closest node pair positions.
    """
    closest_nodes_positions = []
    for start_pos, end_pos in pair_positions:
        closest_start = None
        closest_end = None
        min_dist_start = float('inf')
        min_dist_end = float('inf')

        for node in skeleton.nodes:
            node_pos = skeleton.nodes[node]['pos']
            dist_to_start = distance(start_pos, node_pos)
            dist_to_end = distance(end_pos, node_pos)
            if dist_to_start < min_dist_start:
                min_dist_start = dist_to_start
                closest_start = node_pos
            if dist_to_end < min_dist_end:
                min_dist_end = dist_to_end
                closest_end = node_pos

        closest_nodes_positions.append((closest_start, closest_end))
    return closest_nodes_positions

def find_closest_node_identifier(skeleton, position):
    """Finds the node identifier closest to a specified position in a given skeleton.。"""
    closest_node = None
    min_dist = float('inf')
    for node in skeleton.nodes:
        node_pos = skeleton.nodes[node]['pos']
        dist = np.linalg.norm(np.array(node_pos) - np.array(position))
        if dist < min_dist:
            min_dist = dist
            closest_node = node
    return closest_node

def check_connections_and_path_length(skeleton, positions):
    results = []
    for start_pos, end_pos in positions:
        start_node = find_closest_node_identifier(skeleton, start_pos)
        end_node = find_closest_node_identifier(skeleton, end_pos)
        if nx.has_path(skeleton, start_node, end_node):
            path = nx.shortest_path(skeleton, start_node, end_node)
            # Check if the path length is 5, because the two end nodes plus the 3 middle nodes are a total of 5 nodes
            if len(path) < 5:
                results.append((start_node, end_node, True, path))
            else:
                results.append((start_node, end_node, False, path))
        else:
            results.append((start_node, end_node, False))
    return results


def process_skeletons(CMSBs, BPs, skeleton, name, all_accuracy,all_misdetection_rate,average_break, out_file):
    complete_skeleton = load_graph_json(skeleton)

    isolated_nodes = [node for node in complete_skeleton.nodes if complete_skeleton.degree(node) == 0]
    complete_skeleton.remove_nodes_from(isolated_nodes)

    new_skeleton = load_graph_json(CMSBs)

    with open(BPs, 'r') as f:
        content_1 = f.readline()
        content_2 = f.readline()
        finally_pairs = eval(content_1)
        all_break_num = eval(content_2) - 1

    pair_nodes_positions = extract_finally_pairs_nodes(new_skeleton, finally_pairs)
    closest_nodes_positions = find_closest_nodes(complete_skeleton, pair_nodes_positions)
    connection_results = check_connections_and_path_length(complete_skeleton, closest_nodes_positions)

    i = 0
    for result in connection_results:
        if result[2]:
            i += 1
    Accuracy = i / len(connection_results)
    Miss_rate = (all_break_num - len(finally_pairs)) / all_break_num
    print("Accuracy = ", Accuracy)
    print("Miss_rate = ", Miss_rate)
    out_file.write(f"{name} - DSBs：{all_break_num}, CMSBs：{len(finally_pairs)}, Accuracy: {Accuracy}, Miss_rate: {Miss_rate}\n")
    all_accuracy.append(Accuracy)
    all_misdetection_rate.append(Miss_rate)
    average_break.append(all_break_num)

def plot_accuracy(all_accuracy,all_misdetection_rate):
    plt.figure(figsize=(12, 12))
    plt.tight_layout()
    plt.figure(constrained_layout=True)
    plt.subplot(2, 2, 1)
    plt.hist(all_accuracy, bins=20, color='skyblue', edgecolor='black')
    plt.title('Accuracy Distribution')
    plt.xlabel('Accuracy')
    plt.ylabel('Frequency')

    plt.subplot(2, 2, 2)
    plt.boxplot(all_accuracy, vert=True, patch_artist=True, showmeans=True)
    plt.title('Accuracy Boxplot')
    plt.ylabel('Accuracy')

    plt.subplot(2, 2, 3)
    plt.hist(all_misdetection_rate, bins=20, color='pink', edgecolor='black')
    plt.title('Miss Rate Distribution')
    plt.xlabel('Miss Rate')
    plt.ylabel('Frequency')

    # 绘制漏检率的箱形图
    plt.subplot(2, 2, 4)
    plt.boxplot(all_misdetection_rate, vert=True, patch_artist=True, showmeans=True)
    plt.title('Miss Rate Boxplot')
    plt.ylabel('Miss Rate')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    random.seed(42)
    np.random.seed(42)

    parser = argparse.ArgumentParser(description="Process skeletons for synthetic dataset evaluation")
    parser.add_argument('--processing_mode', required=True, choices=['single', 'batch'], help="Processing mode: single or batch")
    parser.add_argument('--cmsbs_path', required=True, help="Path to the CMSBs file or folder")
    parser.add_argument('--bps_path', required=True, help="Path to the BPs file or folder")
    parser.add_argument('--skeleton_path', required=True, help="Path to the skeleton file or folder")
    parser.add_argument('--output_file_path', required=True, help="Path to the output result file")
    args = parser.parse_args()

    all_accuracy = []
    all_misdetection_rate = []
    average_break = []

    if args.processing_mode == 'batch':
        # 批处理模式
        with open(args.output_file_path, "w") as output_file:
            for filename in tqdm(os.listdir(args.cmsbs_path), desc="Processing labels"):
                if filename.endswith('.json'):
                    CMSBs = os.path.join(args.cmsbs_path, filename)
                    base_filename = os.path.basename(CMSBs)
                    name = base_filename.split('_CMSBs')[0]
                    BPs = os.path.join(args.bps_path, f"{name}_BPs.txt")
                    skeleton = os.path.join(args.skeleton_path, f"{name}_imitaet_S.json")
                    process_skeletons(CMSBs, BPs, skeleton, name, all_accuracy, all_misdetection_rate, average_break, output_file)

            average = sum(average_break) / len(average_break)
            average_accuracy = sum(all_accuracy) / len(all_accuracy)
            average_misdetection_rate = sum(all_misdetection_rate) / len(all_misdetection_rate)
            print("Avg.DSBs：", average)
            print("Accuracy：", average_accuracy)
            print("Miss_rate：", average_misdetection_rate)
            output_file.write(f"Avg.DSBs：{average}\n")
            output_file.write(f"Accuracy：{average_accuracy}\n")
            output_file.write(f"Miss_rate：{average_misdetection_rate}\n")
            plot_accuracy(all_accuracy, all_misdetection_rate)
    else:
        with open(args.output_file_path, "w") as output_file:
            CMSBs = args.cmsbs_path
            BPs_path = args.bps_path
            skeleton_path = args.skeleton_path
            base_filename = os.path.basename(CMSBs)
            name = base_filename.split('_CMSBs')[0]
            process_skeletons(CMSBs, BPs_path, skeleton_path, name, all_accuracy, all_misdetection_rate, average_break, output_file)

            average = sum(average_break) / len(average_break)
            average_accuracy = sum(all_accuracy) / len(all_accuracy)
            average_misdetection_rate = sum(all_misdetection_rate) / len(all_misdetection_rate)
            print("Avg.DSBs：", average)
            print("Accuracy：", average_accuracy)
            print("Miss_rate：", average_misdetection_rate)
            output_file.write(f"Avg.DSBs：{average}\n")
            output_file.write(f"Accuracy：{average_accuracy}\n")
            output_file.write(f"Miss_rate：{average_misdetection_rate}\n")
            plot_accuracy(all_accuracy, all_misdetection_rate)



