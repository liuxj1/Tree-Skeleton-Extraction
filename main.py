from skeleton.get_DS import *
import os
import random
from skeleton.process import load_and_preprocess_pcd
from skeleton.imitate_MBs import imitate_MBs
import argparse

# main
def main(input_pcd_path, output_MB_pcd_folder, output_skeleton_folder, output_MSBs_folder,
         output_BPs_folder,output_CMSBs_folder, disconnections_num, datasets_type):

    base_filename = os.path.basename(input_pcd_path)
    imitate_MB_pcd_file = os.path.join(output_MB_pcd_folder, base_filename.split('.')[0] + '_imitate_MB.txt')
    imitate_skeleton_file = os.path.join(output_skeleton_folder, base_filename.split('.')[0] + '_imitaet_S.json')

    output_MSBs_filename = os.path.join(output_MSBs_folder, base_filename.split('.')[0] + '_MSBs.json')
    output_BPs_filename = os.path.join(output_BPs_folder, base_filename.split('.')[0] + '_BPs.txt')
    output_CMSBs_filename = os.path.join(output_CMSBs_folder, base_filename.split('.')[0] + '_CMSBs.json')

    data_points = load_and_preprocess_pcd(input_pcd_path, datasets_type)

    if datasets_type == 'synthetic':
        data_points = imitate_MBs(data_points, disconnections_num, imitate_MB_pcd_file, imitate_skeleton_file)

    # get disconnected skeletons
    get_break_skeleton(data_points, output_MSBs_filename, output_BPs_filename, output_CMSBs_filename, datasets_type)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run Tree Skeleton Extraction")
    parser.add_argument('--processing_mode', required=True, choices=['single', 'batch'],
                        help="Processing mode: single or batch")
    parser.add_argument('--type_name', required=False, choices=['complex', 'simple', 'reality'],
                        help="Dataset type for batch processing")
    parser.add_argument('--input_pcd_path', required=False,
                        help="Path to the input point cloud file for single processing")
    parser.add_argument('--datasets_type', required=False, choices=['synthetic', 'reality'],
                        help="Dataset type for single processing")
    parser.add_argument('--disconnections_num', type=int, required=False,
                        help="Number of disconnections (e.g., reality-0, simple-8, complex-28)")
    parser.add_argument('--input_pcd_folder', required=False, help="Path to input folder for batch processing")
    args = parser.parse_args()

    Processing_mode = args.processing_mode
    random.seed(42)
    np.random.seed(42)

    if Processing_mode == 'batch':
        type_name = args.type_name
        if not type_name:
            raise ValueError("`type_name` must be specified for batch processing.")

        input_pcd_folder = args.input_pcd_folder
        if not input_pcd_folder:
            raise ValueError("`input_pcd_folder` must be specified for batch processing.")

        output_MB_pcd_folder = f"datas/{type_name}/MB_pcd"  # Store virtual point cloud files after simulating branch breakage
        output_skeleton_folder = f"datas/{type_name}/skeleton"  # The complete skeleton folder for the simulated point cloud
        output_MSBs_folder = f"datas/{type_name}/MSBs"  # Skeleton folder where missing skeleton branches are stored
        output_BPs_folder = f"datas/{type_name}/BPs"  # Folder for storing branch point pairs used for reconnection
        output_CMSBs_folder = f"datas/{type_name}/CMSBs"  # Store the completed skeleton

        # 根据 type_name 设置 disconnections_num 和 datasets_type
        if type_name == 'complex':
            disconnections_num = 28
            datasets_type = 'synthetic'
        elif type_name == 'simple':
            disconnections_num = 8
            datasets_type = 'synthetic'
        elif type_name == 'reality':
            disconnections_num = 0
            datasets_type = 'reality'

        for file_name in os.listdir(input_pcd_folder):
            input_pcd_path = os.path.join(input_pcd_folder, file_name)
            main(input_pcd_path, output_MB_pcd_folder, output_skeleton_folder, output_MSBs_folder,
                 output_BPs_folder, output_CMSBs_folder, disconnections_num, datasets_type)
    else:
        input_pcd_path = args.input_pcd_path
        datasets_type = args.datasets_type
        disconnections_num = args.disconnections_num

        if not input_pcd_path or not datasets_type or disconnections_num is None:
            raise ValueError(
                "`input_pcd_path`, `datasets_type`, and `disconnections_num` must be specified for single processing.")

        output_MB_pcd_folder = "datas/single_data/MB_pcd"  # Store virtual point cloud files after simulating branch breakage
        output_skeleton_folder = "datas/single_data/skeleton"  # The complete skeleton folder for the simulated point cloud
        output_MSBs_folder = "datas/single_data/MSBs"  # Skeleton folder where missing skeleton branches are stored
        output_BPs_folder = "datas/single_data/BPs"  # Folder for storing branch point pairs used for reconnection
        output_CMSBs_folder = " datas/single_data/CMSBs"  # Store the completed skeleton

        main(input_pcd_path, output_MB_pcd_folder, output_skeleton_folder, output_MSBs_folder,
             output_BPs_folder, output_CMSBs_folder, disconnections_num, datasets_type)


