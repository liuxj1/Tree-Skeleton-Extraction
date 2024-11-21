import open3d as o3d
import numpy as np

from pc_skeletor import SLBC, LBC
from pc_skeletor import Dataset

def load_pcd(file_path):
    # 读取文本文件并转换为numpy数组
    with open(file_path, 'r') as file:
        data_list = []
        for line in file:
            # 假设每行是一个以逗号分隔的浮点数列表
            numbers = [float(number) for number in line.strip().strip('[]').split(' ')]
            data_list.append(numbers)
    data = np.array(data_list)
    xyz = data[:, :3]

    # 创建一个PointCloud对象
    pcd = o3d.geometry.PointCloud()

    # 将numpy数组中的点坐标赋给PointCloud对象
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd

if __name__ == "__main__":
    downloader = Dataset()
    trunk_pcd_path, branch_pcd_path = downloader.download_semantic_tree_dataset()

    file_path = r"D:\tree_broken_branches\9.16_网格骨架线\1.txt"

    '''******加载点云*****'''
    pcd = load_pcd(file_path)

    pcd_trunk = o3d.io.read_point_cloud(trunk_pcd_path)
    pcd_branch = o3d.io.read_point_cloud(branch_pcd_path)
    pcd = pcd_trunk + pcd_branch

    # Laplacian-based Contraction
    lbc = LBC(point_cloud=pcd,
              init_contraction=2,
              init_attraction=0.5,
              down_sample=0.03)
    lbc.extract_skeleton()
    lbc.extract_topology()
    lbc.visualize()
    lbc.show_graph(lbc.skeleton_graph, fig_size=(30, 30))
    lbc.show_graph(lbc.topology_graph)
    lbc.save('./output')
    # lbc.animate(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=500, output='./output')
    # lbc.animate_contracted_pcd(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=300, output='./output')
    lbc.animate_topology(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=300, output='./output')

    # Semantic Laplacian-based Contraction
    s_lbc = SLBC(point_cloud={'trunk': pcd_trunk, 'branches': pcd_branch},
                 semantic_weighting=30.,
                 init_contraction=1,
                 init_attraction=0.5,
                 down_sample=0.009)
    s_lbc.extract_skeleton()
    s_lbc.extract_topology()
    s_lbc.visualize()
    s_lbc.show_graph(lbc.skeleton_graph, fig_size=(30, 30))
    s_lbc.show_graph(lbc.topology_graph)
    s_lbc.save('./output')
    #s_lbc.animate(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=500, output='./output')
    #s_lbc.animate_contracted_pcd(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=300, output='./output')
    s_lbc.animate_topology(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]), steps=300, output='./output')
