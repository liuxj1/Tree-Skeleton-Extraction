# Tree-Skeleton-Extraction
Extracting complete tree skeleton based on geometric tree branch growth geometry structure based on graph search algorithm
# Quick Start
## Installation from Source
   ```
git clone https://github.com/liuxj1/Tree-Skeleton-Extraction.git
cd Tree_Skeleton_extraction
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
1.We provide two data sets.
2.When using different datasets, you need to determine the `datasets_type`, including `reality` and `synthetic`, where `complex` and `simple` both belong to `synthetic`.
3.Among them, disconnections_num is used to determine how many discontinuous regions the synthetic simulation has. For example, when using it, we set it to 28 in complex and 8 in simple
## For a single tree
### 1.Get skeleton
You need to set `Processing_mode` to `single` in `main.py`, and give the corresponding input point cloud file and output file path in the corresponding position.
We have given the data file directory format. You can refer to the data directory to put the corresponding data into the specified location.Then，Run the following command.
   ```
python main.py
   ```
### 2.Visualization
You can run `Visualization/Visualize_progress.py` to visualize the skeleton effect, where the visualization interface 1 is the skeleton without Laplace smoothing, and the interface 2 is the skeleton after Laplace smoothing.

![Completed Skeleton](https://github.com/liuxj1/Tree-Skeleton-Extraction/process_files/1.png)
![Laplace smoothing](https://github.com/liuxj1/Tree-Skeleton-Extraction/process_files/2.png)
