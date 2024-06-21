function [left_rectified_img, right_rectified_img, depth_map] = get_images(path_to_dataset, dataset_name, file_name, depth_factor)
    left_rectified_name = strcat(path_to_dataset,dataset_name,'/mav0/cam0/rectified/',file_name,'.png');
    left_rectified_img = imread(left_rectified_name);
    right_rectified_name = strcat(path_to_dataset,dataset_name,'/mav0/cam1/rectified/',file_name,'.png');
    right_rectified_img = imread(right_rectified_name);

    % Read depthmap
    depth_name = strcat(path_to_dataset,dataset_name,'/mav0/depth/data/',file_name,'.png');
    depth_img = imread(depth_name);
    depth_map = double(depth_img)/depth_factor; % Convert from uint16 depth image to metric depth map (values are in meters)
end