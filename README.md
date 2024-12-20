# Tree-Skeleton-Extraction
**Geometric and Multi-Scale Feature Fusion for Complete Tree Skeleton Extraction**.<br>
The point cloud data obtained by 3D reconstruction of the tree body is missing, which exacerbates the problem of missing skeleton branches during skeleton extraction. The skeletons extracted by geometry-based methods such as BFS, DFS, and clustering are often missing and discontinuous. We introduced DBSCAN segmentation to extract the skeleton of discontinuous areas separately, and then reconnected the skeleton breakpoints through the local features of the branches and the global growth structure features of the tree body to construct a complete skeleton.<br>

<img src="https://raw.githubusercontent.com/liuxj1/Tree-Skeleton-Extraction/main/process_files/skeleton.gif" alt="Demo GIF" width="450"/>

# Quick Start
## Installation from Source
   ```
git clone https://github.com/liuxj1/Tree-Skeleton-Extraction.git
cd Tree-Skeleton-extraction
conda create -n skeleton python=3.7.0
conda activate skeleton
pip install -r requirements.txt
   ```
# Basic Usage
Next, we will introduce the usage of the code in the project
## File directory structure
`input_pcd`---tree point cloud data; `MB_pcd`---Incomplete point cloud data obtained after random deletion of synthetic data sets; `skeleton`---A complete skeleton extracted from a complete synthetic dataset, mainly used for evaluation of synthetic datasets; `MSBs`---Extracted incomplete skeletons; `BPs`---skeletal branch point pairs for skeleton reconnection; `CMSBS`---Completed skeleton
```
Tree-Skeleton-Extraction/
│
├── datas/
│   ├── input_pcd
│   ├── MB_pcd
│   ├── skeleton
│   ├── MSBs
│   ├── BPs
│   └── CMSBS
│
├── skeleton/
│
└── Visualization/
```
## Noteworthy
1.We provide two datasets. <br>
- Datasets:
  - [Datasets Link](https://drive.google.com/drive/folders/1-F2EuWzzmbOaRrjwM_7wXedgUqyQjoXI?usp=sharing)
  - [Test Datasets Link](https://drive.google.com/drive/folders/1AVYqXS4l93UGSojt150_JB6L2u9JvjGK?usp=sharing)
  
2.When using different datasets, you need to determine the `datasets_type`, including `reality` and `synthetic`, where `complex` and `simple` both belong to `synthetic`.<br>
3.Among them, `disconnections_num` is used to determine how many discontinuous regions the synthetic simulation has. For example, when using it, we set it to 28 in `complex` and 8 in `simple`. <br>
## Start
### 1.Get skeleton
If there is no data in the project or the data is not available, you can download [Test Datasets Link](https://drive.google.com/drive/folders/1AVYqXS4l93UGSojt150_JB6L2u9JvjGK?usp=sharing). <br>
You need to set `Processing_mode` to `single` in `main.py`, and give the corresponding input point cloud file and output file path in the corresponding position.
We have given the data file directory format. You can refer to the data directory to put the corresponding data into the specified location.Then，Run the following command.<br>
For a single tree:   
   ``` 
python main.py
   --processing_mode single \
   --input_pcd_path "datas/single_data/input_pcd/reality_example1.txt" \
   --datasets_type "reality" \
   --disconnections_num 8
   ```
Doenload [Datasets Link](https://drive.google.com/drive/folders/1-F2EuWzzmbOaRrjwM_7wXedgUqyQjoXI?usp=sharing) For batch: 
   ``` 
python main.py
   --processing_mode batch \
   --type_name reality \
   --datasets_type reality \
   --input_pcd_folder "datas/reality/input_pcd" \
   --disconnections_num 0
   ```

### 2.Visualization
You can visualize the skeleton effect, where the visualization interface 1 is the skeleton without Laplace smoothing, and the interface 2 is the skeleton after Laplace smoothing.<br>
```
python -m Visualization.Visualize_progress --cmsbs "datas/single_data/CMSBs/reality_example1_CMSBs.json" --bps "datas/single_data/BPs/reality_example1_BPs.txt"
``` 
<div style="display: flex; justify-content: space-around;">
  <img src="https://raw.githubusercontent.com/liuxj1/Tree-Skeleton-Extraction/main/process_files/1.png" alt="Completed Skeleton" width="450"/>
  <img src="https://raw.githubusercontent.com/liuxj1/Tree-Skeleton-Extraction/main/process_files/2.png" alt="Laplace smoothing" width="450"/>
</div>

### 3.Evaluation
For `reality`, the evaluation method needs to be combined with the original point cloud for visualization, run `skeleton\reality_visual_evaluation.py`<br>
```
python reality_visual_evaluation.py \
   --pcd "datas/single_data/input_pcd/reality_example1.txt" \
   --cmsbs "datas/single_data/CMSBs/reality_example1_CMSBs.json" \
   --bps "datas/single_data/BPs/reality_example1_BPs.txt"
```
<img src="https://raw.githubusercontent.com/liuxj1/Tree-Skeleton-Extraction/main/process_files/3.png" alt="Reality Visual Evaluation" width="450"/>

For `synthetic`, we define the evaluation method.<br>
For a single tree:
```
python synthetic_evaluation.py \
   --processing_mode "single"
   --cmsbs "datas/single_data/CMSBs/complex_example2_CMSBs.json" \
   --bps "datas/single_data/BPs/complex_example2_BPs.txt" \
   --skeleton_path "datas/single_data/skeleton/complex_example2_imitaet_S.json" \
   --output_file_path "datas/single_data/complex_example2_result.txt"
``` 
For batch:
```
python synthetic_evaluation.py \
   --processing_mode "batch"
   --cmsbs "datas/complex/CMSBs" \
   --bps "datas/complex/BPs" \
   --skeleton_path "datas/complex/skeleton" \
   --output_file_path "datas/complex/complex_result.txt"
``` 
Evaluation method：<br>
      (1) Overlay and match the completed skeleton with the original full skeleton.<br>
      (2) In the completion area, select two connected skeleton nodes (red lines in Fig. 15(a)) and find the nearest blue reference nodes.<br>
      (3) Compare the number of nodes between these two reference nodes, based on the full skeleton's topology. <br>
      (4) If the count is 3 or fewer, the skeleton completion is considered correct (Fig. 15a).<br>
<img src="https://raw.githubusercontent.com/liuxj1/Tree-Skeleton-Extraction/main/process_files/4.png" alt="synthetic evaluation method" width="450"/>

## For batch data processing
You need to set `Processing_mode` to `batch` in `main.py`. Then, put the point cloud data in the same folder (note that different data sets are separated)<br>
For `synthetic`, `run skeleton/synthetic_evaluation.py` to perform batch evaluation and calculate the overall `Accuracy` and `Misse Rate`. For example,<br>
| Label  | Avg. DSBs | Accuracy |  Miss Rate|
| ------------- | ------------- | ------------- | ------------- |
| Complex | 28  | 86.83%  | 0.95%  |
| Simple  | 8 | 95.68%  | 1.03%  |
# Other methods
## Li Method
`run Li_method.py`
## MST Method
`run MST_method.py`
## Laplace Contraction
You need to refer to: [pc-skeletor](https://github.com/meyerls/pc-skeletor)
```
cd other_methods/Meyer
pip install --upgrade pip setuptools
pip install -r requirements.txt
pip install -e .
```
`run Meyer/pcd_skeletor/laplacian.py`. You will get `1.json. Then`, then `run Meyer_method.py` 
