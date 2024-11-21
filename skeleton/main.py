from get_DS import *
import os
import random
import json
from process import load_and_preprocess_pcd
from imitate_MBs import imitate_MBs


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
    random.seed(42)
    np.random.seed(42)
    Processing_mode = 'single'  # If you want to process a large amount of data in batches, use batch, otherwise single
    # Batch Processing
    if Processing_mode == 'batch':
        type_name = 'reality'   # complex   simple
        input_pcd_folder = "path/to/pcd_folder"  # Folder for storing batches of incomplete point clouds
        output_MB_pcd_folder = "path/to/filefolder"  # Store virtual point cloud files after simulating branch breakage
        output_skeleton_folder = "path/to/filefolder"  # The complete skeleton folder for the simulated point cloud
        # The number of discontinuous regions simulated in the synthetic dataset
        if type_name == 'complex':
            disconnections_num = 28
        elif type_name == 'simple':
            disconnections_num = 8
        else:
            disconnections_num = 0

        output_MSBs_folder = "path/to/filefolder"  # Skeleton folder where missing skeleton branches are stored
        output_BPs_folder = "path/to/filefolder"  # Folder for storing branch point pairs used for reconnection
        output_CMSBs_folder = "path/to/filefolder"  # Store the completed skeleton
        if type_name == 'reality':
            datasets_type = 'reality'
        else:
            datasets_type = "synthetic"  # synthetic  reality

        # Iterate through all files in a folder
        for file_name in os.listdir(input_pcd_folder):
            input_pcd_path = os.path.join(input_pcd_folder, file_name)
            # Call the main function to process the current file
            main(input_pcd_path, output_MB_pcd_folder, output_skeleton_folder, output_MSBs_folder,
                 output_BPs_folder, output_CMSBs_folder, disconnections_num, datasets_type)
    else:
        input_pcd_path = "../datas/input_pcd/simple_example3.txt"     # Input incomplete tree point cloud
        output_MB_pcd_folder = "../datas/MB_pcd"   # Store virtual point cloud files after simulating branch breakage
        output_skeleton_folder = "../datas/skeleton"   # The complete skeleton folder for the simulated point cloud
        output_MSBs_folder = "../datas/MSBs"    # Skeleton folder where missing skeleton branches are stored
        output_BPs_folder = "../datas/BPs"     # Folder for storing branch point pairs used for reconnection
        output_CMSBs_folder = "../datas/CMSBs"  # Store the completed skeleton

        datasets_type = 'synthetic'  # synthetic    reality
        disconnections_num = 8      # reality-0, simple-8, complex-28

        main(input_pcd_path, output_MB_pcd_folder, output_skeleton_folder, output_MSBs_folder,
             output_BPs_folder, output_CMSBs_folder, disconnections_num, datasets_type)

