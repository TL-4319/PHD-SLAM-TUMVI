close all
clear
clc

addpath libviso2/matlab/;   % Libviso 
addpath util/;              % Utility functions
addpath ssc/;               % ANMS 

%% Select data set
path_to_dataset = '/home/tuan/Projects/tum-vi/';
%path_to_dataset = '/mnt/external01/tuan_dataset/tum-vi/';
dataset_name = 'dataset-room1_512_16';

fig1 = figure(1);

fig1.Position = [1,1,2000,2000];

%% Preparing dataset. NO INPUT REQUIRED
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
dt = mean(dt_vec);

%% Camera intrinsics
camera_intrinsic.f     = 96.8239926;    % focal length
camera_intrinsic.cu    = 247.70848004;  % x position of principal point
camera_intrinsic.cv    = 255.31920479;  % y position of principal point
camera_intrinsic.base  = 0.101039;      % distance between left and right camera

% depth_factor needs to match with whatever is used as depth factor in stereo matching script. 
% NOT NEEDED IF CAMERA ALREADY RETURN DEPTH MAP IN METRIC SCALE
depth_factor = 5000; % Metric Z range = depth[v,u]/depth_factor 

%% Pre process parameters
% Test name
test_name = "test_1";

% FAST detector
FAST_params.ROI = [33 1 512-2*33 512]; % Crop the portion of the image to not run FAST detector. This portion corresponds to area that stereo matching is invalid
FAST_params.min_quality = 0.05;
FAST_params.min_contrast = 0.05;

FAST_params.min_quality_initial_guess = 0.1;
FAST_params.bin_min_contrast = 0.05;


% ORB detector
ORB_params.scale_factor = 1.1;
ORB_params.num_level = 10;
ORB_params.ROI = [33 1 512-2*33 512];

% ANMS 
ANMS_params.max_num_point = 50;
ANMS_params.tolerance = 0.1;

% Binning
binning_params.num_row = 7;
binning_params.num_col = 7;
binning_params.num_point_per_bin = 10;

%%
pre_process_params.FAST_params = FAST_params;
pre_process_params.ANMS_params = ANMS_params;
pre_process_params.ORB_params = ORB_params;

frame = cell(round(size(time_vec,2)),1);

for kk = 1:20:round(size(time_vec,2))
% Read greyscale and depth images
[left_rectified_img, right_rectified_img, depth_map] = ...
    get_images(path_to_dataset, dataset_name, name_array(kk,:), depth_factor);

[~,FAST_only_corner] = FAST_only_detector(left_rectified_img,FAST_params);

[~,ORB_only_corner] = ORB_only_detector (left_rectified_img, ORB_params);

[~,FAST_ANMS_corner] = baseline_detector(left_rectified_img, depth_map, FAST_params, ANMS_params);

[~,ORB_ANMS_corner] = ORB_ANMS_detector(left_rectified_img, depth_map, ORB_params, ANMS_params);

[~,FAST_bin_corner] = FAST_binned_detector (left_rectified_img, FAST_params, binning_params);

[~, ORB_bin_corner] = ORB_binned_detector (left_rectified_img, ORB_params, binning_params);


figure(1)
subplot (3,2,1)
imshow(left_rectified_img,'InitialMagnification','fit')
hold on
if size(FAST_only_corner.Location,1) > 0
    scatter(FAST_only_corner.Location(:,1), FAST_only_corner.Location(:,2),ones(size(FAST_only_corner.Location,1)) * 100,'r.')
end
title("FAST only")

subplot (3,2,2)
imshow(left_rectified_img,'InitialMagnification','fit')
hold on
if size(ORB_only_corner.Location,1) > 0
    scatter(ORB_only_corner.Location(:,1), ORB_only_corner.Location(:,2),ones(size(ORB_only_corner.Location,1)) * 100,'r.')
end
title("ORB only")

subplot (3,2,3)
imshow(left_rectified_img,'InitialMagnification','fit')
hold on
if size(FAST_ANMS_corner.Location,1) > 0
    scatter(FAST_ANMS_corner.Location(:,1), FAST_ANMS_corner.Location(:,2),ones(size(FAST_ANMS_corner.Location,1)) * 100,'r.')
end
title("FAST ANMS")

subplot (3,2,4)
imshow(left_rectified_img,'InitialMagnification','fit')
hold on
if size(ORB_ANMS_corner.Location,1) > 0
    scatter(ORB_ANMS_corner.Location(:,1), ORB_ANMS_corner.Location(:,2),ones(size(ORB_ANMS_corner.Location,1)) * 100,'r.')
end
title("ORB ANMS")

subplot (3,2,5)
imshow(left_rectified_img,'InitialMagnification','fit')
hold on
if size(FAST_bin_corner.Location,1) > 0
    scatter(FAST_bin_corner.Location(:,1), FAST_bin_corner.Location(:,2),ones(size(FAST_bin_corner.Location,1)) * 100,'r.')
end
title("FAST bin")

subplot (3,2,6)
imshow(left_rectified_img,'InitialMagnification','fit')
hold on
if size(ORB_bin_corner.Location,1) > 0
    scatter(ORB_bin_corner.Location(:,1), ORB_bin_corner.Location(:,2),ones(size(ORB_bin_corner.Location,1)) * 100,'r.')
end
title("ORB bin")

frame{kk} = getframe(gcf);
disp(kk)
end

video_name = horzcat("keypoint_test/",test_name);
obj = VideoWriter(video_name);
obj.FrameRate = 20;
open(obj);
for i=1:length(frame)
    writeVideo(obj,frame{i})
end
obj.close();

matfile_name = horzcat("keypoint_test/",test_name,".mat");
save(matfile_name,pre_process_params)