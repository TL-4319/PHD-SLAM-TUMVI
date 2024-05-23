close all
clear
clc

addpath /home/lagerprocessor/Projects/PHD-SLAM-TUMVI/libviso2/matlab;

%% Select data set
path_to_dataset = '/mnt/external01/tuan_dataset/tum-vi/';
dataset_name = 'dataset-room1_512_16';

%% Preparing dataset
timing_filename = strcat(path_to_dataset,dataset_name,'/dso/cam0/times.txt');

% Timing data is Nx3 array: [filename, timestamps(s), exposure_time(ms)]
% Read through text file once to get number of text to preallocate memory
num_line = readmatrix(timing_filename,"NumHeaderLines",1);
num_line = size(num_line,1);

time_vec = zeros(1,num_line);
name_array = char(nan(num_line,0));

fid = fopen(timing_filename);

i = 1;
tline = fgetl(fid);
tline = fgetl(fid); % Skip header line
while ischar(tline)
    sep_char = split(tline);
    name_array(i,1:numel(sep_char{1,1})) = sep_char{1,1};
    time_vec (1,i) = str2num(sep_char{2,1});    

    tline = fgetl(fid);
    i = i + 1;
end
elapsed_time = time_vec - time_vec (1);
dt_vec = diff(elapsed_time);
dt_vec = horzcat(dt_vec, mean(dt_vec)); % Pad the end to make vector same size

%% Camera configuration
depth_factor = 5000; % Metric Z range = depth[v,u]/depth_factor

%% Libviso2 setup
viso_param.f     = 645.2;
viso_param.cu    = 635.9;
viso_param.cv    = 194.1;
viso_param.base  = 0.571;

% init visual odometry
visualOdometryStereoMex('init',viso_param);

%% Filter configuration



%% Run loop
for kk = 1:size(elapsed_time,2)
    %% Read images
    % Read greyscale RGB image
    left_rectified_name = strcat(path_to_dataset,dataset_name,'/mav0/cam0/rectified/',name_array(kk,:),'.png');
    left_rectified_img = imread(left_rectified_name);
    right_rectified_name = strcat(path_to_dataset,dataset_name,'/mav0/cam1/rectified/',name_array(kk,:),'.png');
    right_rectified_img = imread(right_rectified_name);

    % Read depthmap
    depth_name = strcat(path_to_dataset,dataset_name,'/mav0/depth/data/',name_array(kk,:),'.png');
    depth_img = imread(depth_name);
    depth_map = double(depth_img)/depth_factor; % Convert from uint16 depth image to metric depth map
    
    %% LIBVISO2 for visual odometry



end
