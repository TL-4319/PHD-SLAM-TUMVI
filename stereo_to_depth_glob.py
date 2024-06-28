# Created by Martin Wundenka
# Modified by Tuan Luong

#!/usr/bin/env python3

import argparse
import numpy as np
import cv2 as cv
import glob
import os
import sophuspy as sp
from multiprocessing import Pool

# see https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect
# metric z coordinate is found through depth_image[v,u] / depth_factor
depth_factor = 5000  # 5000 for 16bit

# intrinsics (Kannala-Brandt k1,...,k4 = OpenCV fisheye) and extrinsics of the TUM-VI 512 dataset cameras
# see https://vision.in.tum.de/data/datasets/visual-inertial-dataset
# cam0 = left
# cam1 = right
# replace with your calibration
left_K = np.array([[190.97847715128717, 0,                 254.93170605935475],
                   [0,                  190.9733070521226, 256.8974428996504],
                   [0,                  0,                 1]])
left_dist = np.array([0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202,
                      0.00020293673591811182])
right_K = np.array([[190.44236969414825, 0,                 252.59949716835982],
                    [0,                  190.4344384721956, 254.91723064636983],
                    [0,                  0,                 1]])
right_dist = np.array([0.0034003170790442797, 0.001766278153469831, -0.00266312569781606,
                       0.0003299517423931039])

T_imu_cam0 = sp.SE3([[-0.9995250378696743, 0.029615343885863205,
                      -0.008522328211654736, 0.04727988224914392],
                     [0.0075019185074052044, -0.03439736061393144,
                         -0.9993800792498829, -0.047443232143367084],
                     [-0.02989013031643309, -0.998969345370175,
                         0.03415885127385616, -0.0681999605066297],
                     [0.0, 0.0, 0.0, 1.0]])

T_imu_cam1 = sp.SE3([[-0.9995110484978581, 0.030299116376600627,
                      -0.0077218830287333565, -0.053697434688869734],
                     [0.008104079263822521, 0.012511643720192351,
                      -0.9998888851620987, -0.046131737923635924],
                     [-0.030199136245891378, -0.9994625667418545,
                      -0.012751072573940885, -0.07149261284195751],
                     [0.0, 0.0, 0.0, 1.0]])

# image size
size = (512, 512)
# size of downsampled images for stereo matching
smaller_size = (int(size[0] / 1), int(size[1] / 1))

# will be run in parallel by several workers


def rectification_disparity_depth(left_image_path):
    """Read and rectify both stereo images. Calculate the disparity image and the depth map. Save everything to disk.
    Keyword arguments:
    left_image_path[string] -- path to the left stereo image that should be processed. The base name is extracted and used to find the right stereo image. 
    """
    image_name = os.path.basename(left_image_path)

    right_image_path = os.path.join(args.path_right, image_name)

    if not os.path.isfile(right_image_path):
        print(f"Right image for {image_name} does not exist!")
        return

    # read the images from
    left_image = cv.imread(left_image_path, cv.IMREAD_GRAYSCALE)
    right_image = cv.imread(right_image_path, cv.IMREAD_GRAYSCALE)

    left_image_rect = cv.remap(
        left_image, left_map_x, left_map_y, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
    right_image_rect = cv.remap(
        right_image, right_map_x, right_map_y, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

    # save rectifies images
    cv.imwrite(os.path.join(left_output_path, image_name), left_image_rect)
    cv.imwrite(os.path.join(right_output_path, image_name), right_image_rect)

    # downscale to smooth depth map
    left_image_rect_downscaled = cv.resize(
        left_image_rect, smaller_size, interpolation=cv.INTER_AREA)
    right_image_rect_downscaled = cv.resize(
        right_image_rect, smaller_size, interpolation=cv.INTER_AREA)

    # compute disparities in both directions
    left_disp = stereo_left.compute(
        left_image_rect_downscaled, right_image_rect_downscaled)
    right_disp = stereo_right.compute(
        right_image_rect_downscaled, left_image_rect_downscaled)

    # smooth depth map
    disp = wls_filter.filter(disparity_map_left=left_disp,
                             left_view=left_image_rect, disparity_map_right=right_disp)

    # upscale
    disp = cv.resize(disp, size, interpolation=cv.INTER_AREA)

    # save disparity image
    cv.imwrite(os.path.join(disparity_output_path,
                            image_name), disp)

    # calculate depth map
    # see https://docs.opencv.org/4.5.0/d9/d0c/group__calib3d.html#ga1bc1152bd57d63bc524204f21fde6e02
    # convert disparity to float32 before
    depth = cv.reprojectImageTo3D(disp.astype(
        np.float32) / 16, disparity_to_depth, handleMissingValues=True)
    depth = depth[:, :, 2]
    cv_outlier_depth = 10000
    depth[depth == cv_outlier_depth] = 0
    depth = depth * depth_factor

    # save depth map as 16bit grayscale image
    cv.imwrite(os.path.join(depth_output_path,
                            image_name), depth.astype('uint16'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Reads in a stereo dataset and computes 16bit depth maps for the left images by semi-global matching.
    ''')
    parser.add_argument(
        "--path_left", help="Path to a directory containing the left png images.")
    parser.add_argument(
        "--path_right", help="Path to a directory containing the right png images.")
    parser.add_argument(
        "--output_path", help="Path to a directory where the rectified images, disparity images and depth maps will be saved.")
    args = parser.parse_args()

    if not os.path.isdir(args.output_path):
        print("Output path is not a directory")
        exit

    # create output folders
    left_output_path = os.path.join(args.output_path, "left")
    right_output_path = os.path.join(args.output_path, "right")
    disparity_output_path = os.path.join(args.output_path, "disparity")
    depth_output_path = os.path.join(args.output_path, "depth")

    try:
        if not os.path.isdir(left_output_path):
            os.mkdir(left_output_path)
    except OSError:
        print(f"Could not create directory {left_output_path}")
        exit

    try:
        if not os.path.isdir(right_output_path):
            os.mkdir(right_output_path)
    except OSError:
        print(f"Could not create directory {right_output_path}")
        exit

    try:
        if not os.path.isdir(disparity_output_path):
            os.mkdir(disparity_output_path)
    except OSError:
        print(f"Could not create directory {disparity_output_path}")
        exit

    try:
        if not os.path.isdir(depth_output_path):
            os.mkdir(depth_output_path)
    except OSError:
        print(f"Could not create directory {depth_output_path}")
        exit

    # transform from cam0 (left) to cam1 (right)
    T_cam1_cam0 = T_imu_cam1.inverse() * T_imu_cam0
    R = T_cam1_cam0.rotationMatrix()
    t = T_cam1_cam0.translation()

    # use the opencv fisheye camera model aka Kannala Brandt
    # see https://docs.opencv.org/4.5.0/db/d58/group__calib3d__fisheye.html#gac1af58774006689056b0f2ef1db55ecc
    # calculates new camera matrices (projections) and rectification rotations
    # if some image regions have an invalid remapping decrease fov_scale
    left_rectification, right_rectification, left_projection, right_projection, disparity_to_depth = cv.fisheye.stereoRectify(
        left_K, left_dist, right_K, right_dist, size, R, t, flags=cv.CALIB_ZERO_DISPARITY, balance=0.5, fov_scale=0.6)

    # calculate lookup tables for stereo rectification
    left_map_x, left_map_y = cv.fisheye.initUndistortRectifyMap(
        left_K, left_dist, left_rectification, left_projection, size, cv.CV_32FC1)
    right_map_x, right_map_y = cv.fisheye.initUndistortRectifyMap(
        right_K, right_dist, right_rectification, right_projection, size, cv.CV_32FC1)

    # find image coordinates with invalid remapping
    # for u in range(512):
    #     for v in range(512):
    #         if left_map_x[u,v] < 0 or left_map_x[u,v] > 511 or left_map_y[u,v] < 0 or left_map_y[u,v] > 511:
    #             print(u, v, left_map_x[u,v], left_map_y[u,v])

    T_cam0rect_cam0 = sp.SE3(left_rectification, np.zeros(3))
    T_cam1rect_cam1 = sp.SE3(right_rectification, np.zeros(3))

    T_imu_cam0rect = T_imu_cam0 * T_cam0rect_cam0.inverse()
    T_imu_cam1rect = T_imu_cam1 * T_cam1rect_cam1.inverse()

    print("new left P", left_projection)
    print("new right P", right_projection)

    # see https://docs.opencv.org/4.5.0/d2/d85/classcv_1_1StereoSGBM.html
    # and https://answers.opencv.org/question/182049/pythonstereo-disparity-quality-problems/?answer=183650#post-id-183650
    block_size = 1
    min_disp = 0
    num_disp = 16
    stereo_left = cv.StereoSGBM_create(minDisparity=min_disp,
                                       numDisparities=num_disp,
                                       blockSize=block_size,
                                       P1=4*block_size**2,
                                       P2=32*block_size**2,
                                       disp12MaxDiff=1,
                                       preFilterCap=63,
                                       uniquenessRatio=10,
                                       speckleWindowSize=100,
                                       speckleRange=32,
                                       mode=cv.STEREO_SGBM_MODE_HH
                                       )

    # see https://docs.opencv.org/4.5.0/d9/dba/classcv_1_1StereoBM.html
    # stereo_left = cv.StereoBM_create(numDisparities=16, blockSize=15)

    # use weighted least squares filtering to post process the depth maps
    # see https://docs.opencv.org/4.5.0/d9/d51/classcv_1_1ximgproc_1_1DisparityWLSFilter.html
    # and https://docs.opencv.org/4.5.0/d3/d14/tutorial_ximgproc_disparity_filtering.html
    stereo_right = cv.ximgproc.createRightMatcher(stereo_left)
    wls_filter = cv.ximgproc.createDisparityWLSFilter(stereo_left)

    # parallelize using python multiprocessing
    num_cores = 6
    with Pool(num_cores) as p:
        left_image_paths = glob.glob(
            os.path.join(args.path_left, "*.png"))
        done_counter = 0
        for x in p.imap(rectification_disparity_depth, left_image_paths):
            done_counter += 1
            percent = done_counter / len(left_image_paths) * 100
            print(
                f"progress: {done_counter}/{len(left_image_paths)} = {percent}%")
