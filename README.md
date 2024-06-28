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

### Note on projection matrices (very important for libviso2 to work properly)

The projections matrices for the TUM-VI setup is located in tum_vi_projection_matrices.txt. These matrices are the projection matrices of the left (P1) and right (P2) camera in the stereo setup after performing OpenCV cv.fisheye.stereoRectify method. The new projection matrix will be printed out by the stereo_to_depth_glob.py if used. For ZED camera, the ZED-SDK should print out the matrix during startup.

The main script uses the left camera as the reference. Therefore, the corresponding parameters are:

- focal length: f = P1 (1,1)
- x-axis principle point: cu = P1 (1,3)
- y-axis principle point: cv = P1 (2,3)
- baseline: b = -P2(1,4)/P2(1,1)

More information can be found [here](https://www.cvlibs.net/datasets/karlsruhe_sequences/)

## Build libviso2 MATLAB wrapper
Run the make.m script in MATLAB. The script is located at PHD-SLAM-TUMVI/libviso2/matlab/

## Build ANMS via SSC MATLAB warpper
Navigate to PHD-SLAM-TUMVI/ssc in MATLAB. Run ```mex ssc.cpp```

## Run PHD-SLAM script
The main script is PHD_SLAM_TUM_VI.m

## PHD-SLAM STILL UNDER DEVELOPMENT AND TESTING!!!

