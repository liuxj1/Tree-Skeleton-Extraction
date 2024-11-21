#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright (c) 2022 Lukas Meyer
Licensed under the MIT License.
See LICENSE file for more information.
"""

import logging
from copy import copy

import networkx
from scipy.spatial.transform import Rotation as R
import networkx as nx

from pc_skeletor.utility import *


class SkeletonBase(object):
    def __init__(self, verbose: bool = False, debug: bool = False):
        self.verbose: bool = verbose
        self.debug: bool = debug

        if self.verbose:
            logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.DEBUG)
        else:
            logging.basicConfig(format='%(asctime)s - %(message)s', level=logging.INFO)

        ## Skeleton output
        # Original point cloud
        self.pcd: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
        # Contracted Point Cloud with same numbers of points as pcd
        self.contracted_point_cloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
        # Skeleton of the pcd, but same shape as contracted_point_cloud but down sampled
        self.skeleton: o3d.geometry.PointCloud = o3d.geometry.PointCloud()
        # Graph of skeleton
        self.skeleton_graph: nx.Graph = nx.Graph()
        # Topology of simplified graph
        self.topology: o3d.geometry.LineSet = o3d.geometry.LineSet()
        # Graph of topology
        self.topology_graph: nx.Graph = nx.Graph()

    def extract_skeleton(self):
        pass

    def extract_topology(self):
        pass

    def save(self, *args):
        pass

    # def show_graph(self, graph: networkx.Graph, pos: Union[np.ndarray, bool] = True, fig_size: tuple = (20, 20)):
    #     def load_and_preprocess_pcd(file_path):
    #         # 读取文件并创建点云
    #         with open(file_path, 'r') as file:
    #             data_list = [np.fromstring(line.strip().strip('[]'), sep=' ') for line in file]
    #         data = np.vstack(data_list)
    #         xyz = data[:, :3]
    #         # 归一化点云
    #         min_bound = np.min(xyz, axis=0)
    #         max_bound = np.max(xyz, axis=0)
    #         max_range = np.max(max_bound - min_bound)
    #         normalized_points = (xyz - min_bound) / max_range
    #         # 随机下采样点云到指定的点数
    #         points = np.asarray(normalized_points)
    #         if len(points) <= 1000000:
    #             target_number_of_points = int(len(points) * 0.20)
    #         elif len(points) > 1000000 and len(points) <= 2000000:
    #             target_number_of_points = int(len(points) * 0.15)
    #         elif len(points) > 2000000 and len(points) <= 3000000:
    #             target_number_of_points = int(len(points) * 0.10)
    #         else:
    #             target_number_of_points = 300000
    #         # target_number_of_points = int(len(points) * 0.35)
    #         indices = np.random.choice(len(points), target_number_of_points, replace=False)
    #         downsampled_points = points[indices]
    #         return downsampled_points
    #     file_path = r"D:\skeleton_lines\tree_datas_reality\tree_reality_break_txt\new_tree7_1.txt"
    #     data_points = load_and_preprocess_pcd(file_path)
    #     gray_color = [0.5, 0.5, 0.5]  # 灰色的RGB值
    #     colors = np.tile(gray_color, (len(data_points), 1))  # 将灰色复制到每个点
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(data_points)  # 将点云数据加载到点云对象中
    #     pcd.colors = o3d.utility.Vector3dVector(colors)
    #     # 创建一个空的点云对象
    #     point_cloud = o3d.geometry.PointCloud()
    #
    #     # 检查是否有位置信息，如果没有则使用 networkx 的布局算法
    #     if pos is True:
    #         pos = {node: graph.nodes[node]['pos'] for node in graph.nodes()}
    #     elif isinstance(pos, bool):
    #         pos = nx.spring_layout(graph)  # 使用spring布局作为备选方案
    #
    #     def create_spheres_at_nodes(skeleton, radius=0.00005):
    #         spheres = []
    #         for node in skeleton.nodes():
    #             sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    #             sphere.paint_uniform_color([0,1,0])
    #             sphere.translate(skeleton.nodes[node]['pos'])
    #             spheres.append(sphere)
    #         return spheres
    #     # 提取节点位置
    #     points = np.array([pos[node] for node in graph.nodes()])
    #     point_cloud.points = o3d.utility.Vector3dVector(points)
    #
    #     # 计算最小生成树
    #     mst = nx.minimum_spanning_tree(graph)
    #     spheres = create_spheres_at_nodes(mst)
    #
    #
    #
    #
    #     # 创建一个线集对象来表示图的边
    #     line_set = o3d.geometry.LineSet()
    #
    #     # 设置线集的点
    #     line_set.points = o3d.utility.Vector3dVector(points)
    #
    #     # 使用最小生成树的边设置线集的线（边）
    #     lines = [[list(graph.nodes()).index(u), list(graph.nodes()).index(v)] for u, v in mst.edges()]
    #     line_set.lines = o3d.utility.Vector2iVector(lines)
    #
    #     # 设置点和线的颜色，可根据需要调整
    #     point_cloud.colors = o3d.utility.Vector3dVector(np.tile([0, 1, 0], (len(points), 1)))  # 红色节点
    #     line_set.colors = o3d.utility.Vector3dVector(np.tile([0, 1, 0], (len(lines), 1)))  # 绿色边
    #
    #     def custom_draw_geometry_with_no_light(geometries):
    #         vis = o3d.visualization.Visualizer()
    #         vis.create_window()
    #
    #         for geometry in geometries:
    #             vis.add_geometry(geometry)
    #
    #         render_option = vis.get_render_option()
    #         render_option.light_on = False
    #         render_option.point_size = 2
    #         render_option.line_width = 50
    #         render_option.background_color = np.array([1, 1, 1])
    #
    #         vis.run()
    #         vis.destroy_window()
    #     # 可视化
    #     custom_draw_geometry_with_no_light([pcd, point_cloud, line_set] + spheres)
    def show_graph(self, graph: networkx.Graph, pos: Union[np.ndarray, bool] = True, fig_size: tuple = (20, 20)):
        # For more info: https://networkx.org/documentation/stable/reference/drawing.html
        def create_spheres_at_nodes(points, radius=0.002):
            spheres = []
            for node in points:
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
                sphere.paint_uniform_color([0,1,0])
                sphere.translate(node)
                spheres.append(sphere)
            return spheres
        plt.figure(figsize=fig_size)

        if pos:
            pos = [graph.nodes()[node_idx]['pos'] for node_idx in range(graph.number_of_nodes())]
            points = np.asarray(pos)
            lines = np.array(list(graph.edges))
            colors = [[0, 1, 0] for _ in range(len(lines))]

            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector(colors)
            spheres = create_spheres_at_nodes(points)
            o3d.visualization.draw_geometries([line_set] + spheres)
            # 保存骨架图为 JSON 文件
        graph_data = {
            'nodes': [{'id': node, 'pos': [float(coord) for coord in graph.nodes[node]['pos']]} for node in graph.nodes()],
            'links': [{'source': u, 'target': v} for u, v in graph.edges()]
        }
        import json
        with open(r"C:\Users\Administrator\Desktop\1.json", 'w') as f:
            json.dump(graph_data, f, indent=4)
        print(f'Graph skeleton saved')
        # else:
        #     # nx.draw(G=graph)

        # plt.show()

    def animate_contracted_pcd(self,
                               init_rot: np.ndarray = np.eye(3),
                               steps: int = 360,
                               point_size: float = 1.0,
                               output: [str, None] = None):
        """
            Creates an animation of a point cloud. The point cloud is simply rotated by 360 Degree in multpile steps.

            :param init_rot: Inital rotation to align pcd for visualization
            :param steps: animation rotates 36o degree and is divided into #steps .
            :param point_size: point size of point cloud points.
            :param output_folder: folder where the rendered images are saved to. If None, no images will be saved.

            :return:
        """
        output_folder = os.path.join(output, './tmp_{}'.format(time.time_ns()))
        os.mkdir(output_folder)

        skel = copy(self.contracted_point_cloud)
        skel.paint_uniform_color([0, 0, 1])
        skel.rotate(init_rot, center=[0, 0, 0])

        vis = o3d.visualization.Visualizer()
        vis.create_window(width=1920, height=1080)
        vis.add_geometry(skel)

        ctl = vis.get_view_control()
        ctl.set_zoom(0.6)

        # Set smaller point size. Default is 5.0
        vis.get_render_option().point_size = point_size
        vis.get_render_option().line_width = 15
        vis.get_render_option().light_on = False
        vis.update_renderer()

        # Calculate rotation matrix for every step. Must only be calculated once as rotations are added up in the point cloud
        Rot_mat = R.from_euler('y', np.deg2rad(360 / steps)).as_matrix()

        image_path_list = []

        pcd_idx = 0

        for i in range(steps):
            skel.rotate(Rot_mat, center=[0, 0, 0])
            vis.update_geometry(skel)
            vis.poll_events()
            vis.update_renderer()

            if ((i % 30) == 0) and i != 0:
                pcd_idx = (pcd_idx + 1) % 2

            if True:
                current_image_path = "{}/img_%04d.jpg".format(output_folder) % i
                vis.capture_screen_image(current_image_path)
                image_path_list.append(current_image_path)
        vis.destroy_window()

        generate_gif(filenames=image_path_list, output_name='contracted_skeleton_animation')

    def animate_topology(self,
                         init_rot: np.ndarray = np.eye(3),
                         steps: int = 360,
                         point_size: float = 1.0,
                         output: [str, None] = None):
        """
            Creates an animation of a point cloud. The point cloud is simply rotated by 360 Degree in multpile steps.

            :param init_rot: Inital rotation to align pcd for visualization
            :param steps: animation rotates 36o degree and is divided into #steps .
            :param point_size: point size of point cloud points.
            :param output_folder: folder where the rendered images are saved to. If None, no images will be saved.

            :return:
        """
        output_folder = os.path.join(output, './tmp_{}'.format(time.time_ns()))
        os.mkdir(output_folder)

        topo = copy(self.topology)
        topo.paint_uniform_color([0, 0, 0])
        topo.rotate(init_rot, center=[0, 0, 0])

        vis = o3d.visualization.Visualizer()
        vis.create_window(width=1920, height=1080)
        vis.add_geometry(topo)

        ctl = vis.get_view_control()
        ctl.set_zoom(0.6)

        # Set smaller point size. Default is 5.0
        vis.get_render_option().point_size = point_size
        vis.get_render_option().line_width = 15
        vis.get_render_option().light_on = False
        vis.update_renderer()

        # Calculate rotation matrix for every step. Must only be calculated once as rotations are added up in the point cloud
        Rot_mat = R.from_euler('y', np.deg2rad(360 / steps)).as_matrix()

        image_path_list = []

        pcd_idx = 0

        for i in range(steps):
            topo.rotate(Rot_mat, center=[0, 0, 0])

            vis.update_geometry(topo)
            vis.poll_events()
            vis.update_renderer()

            if ((i % 30) == 0) and i != 0:
                pcd_idx = (pcd_idx + 1) % 2

            if True:
                current_image_path = "{}/img_%04d.jpg".format(output_folder) % i
                vis.capture_screen_image(current_image_path)
                image_path_list.append(current_image_path)
        vis.destroy_window()

        generate_gif(filenames=image_path_list, output_name='topology_animation')

    def animate(self,
                init_rot: np.ndarray = np.eye(3),
                steps: int = 360,
                point_size: float = 1.0,
                output: [str, None] = None):
        """
            Creates an animation of a point cloud. The point cloud is simply rotated by 360 Degree in multpile steps.

            :param init_rot: Inital rotation to align pcd for visualization
            :param steps: animation rotates 36o degree and is divided into #steps .
            :param point_size: point size of point cloud points.
            :param output_folder: folder where the rendered images are saved to. If None, no images will be saved.

            :return:
        """
        output_folder = os.path.join(output, './tmp_{}'.format(time.time_ns()))
        os.mkdir(output_folder)

        # Load PCD
        orig = copy(self.pcd)
        orig.rotate(init_rot, center=[0, 0, 0])

        skel = copy(self.contracted_point_cloud)
        skel.paint_uniform_color([0, 0, 1])
        skel.rotate(init_rot, center=[0, 0, 0])

        topo = copy(self.topology)
        topo.paint_uniform_color([0, 0, 0])
        topo.rotate(init_rot, center=[0, 0, 0])

        pcd = copy(orig)

        vis = o3d.visualization.Visualizer()
        vis.create_window(width=1920, height=1080)
        vis.add_geometry(pcd)
        vis.add_geometry(topo)

        ctl = vis.get_view_control()
        ctl.set_zoom(0.6)

        # Set smaller point size. Default is 5.0
        vis.get_render_option().point_size = point_size
        vis.get_render_option().line_width = 15
        vis.get_render_option().light_on = False
        vis.update_renderer()

        # Calculate rotation matrix for every step. Must only be calculated once as rotations are added up in the point cloud
        Rot_mat = R.from_euler('y', np.deg2rad(360 / steps)).as_matrix()

        image_path_list = []

        pcd_idx = 0

        for i in range(steps):
            orig.rotate(Rot_mat, center=[0, 0, 0])
            skel.rotate(Rot_mat, center=[0, 0, 0])
            topo.rotate(Rot_mat, center=[0, 0, 0])

            if pcd_idx == 0:
                pcd.points = orig.points
                pcd.colors = orig.colors
                pcd.normals = orig.normals
            if pcd_idx == 1:
                pcd.points = skel.points
                pcd.colors = skel.colors
            vis.update_geometry(pcd)
            vis.update_geometry(topo)
            vis.poll_events()
            vis.update_renderer()

            if ((i % 30) == 0) and i != 0:
                pcd_idx = (pcd_idx + 1) % 2

            if True:
                current_image_path = "{}/img_%04d.jpg".format(output_folder) % i
                vis.capture_screen_image(current_image_path)
                image_path_list.append(current_image_path)
        vis.destroy_window()

        generate_gif(filenames=image_path_list, output_name='skeleton_animation')
