# PHD-SLAM1 on TUM-VI dataset
This repository works with the raw data format (Euroc/DSO 512x512) dataset provided by [TUM](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset) 

## Preprocessing
Download and extract dataset to "dataset_name" folder

Rectification and stereo matching needs to be performed first by running stereo_to_depth_glob.py
```
python3 stereo_to_depth_glob.py --path_left /path/to/tumvi/dataset/dataset-name/mav0/cam0/data/ --path_right /path/to/tumvi/dataset/dataset-name/mav0/cam1/data/ --output_path /path/to/output/
```

The script outputs several folders within output directory including depth, disparity, left, right

Copy contents of left folder to /path/to/tumvi/dataset/dataset-name/mav0/cam0/rectified/

Copy contents of right folder to /path/to/tumvi/dataset/dataset-name/mav0/cam1/rectified/

Copy contents of right folder to /path/to/tumvi/dataset/dataset-name/mav0/depth/data/

Create new directories where they are needed ie. rectified folder in cam0 and cam1, or depth/data/ 

## Build libviso2 MATLAB wrapper
Run the make.m script in MATLAB. The script is located at PHD-SLAM-TUMVI/libviso2/matlab/

## PHD-SLAM STILL UNDER DEVELOPMENT

