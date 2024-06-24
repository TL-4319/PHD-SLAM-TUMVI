close all
clear
clc

addpath libviso2/matlab/;   % Libviso 
addpath util/;              % Utility functions
addpath ssc/;               % ANMS 

%% Plotting 
fig1 = figure(1);
title ("Visualization")
fig1.Position = [1,1,2000,2000];

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
dt = mean(dt_vec);

%% Camera and hardware configuration
% deoth_factor needs to match with whatever is used as depth factor in
% stereo matching script. This is not needed if the stereo camera returns the depth in metric scale 
depth_factor = 5000; % Metric Z range = depth[v,u]/depth_factor 

% CV coordinate to NED - This is by definition
Tr_camera_to_NED = eye(4);
Tr_camera_to_NED(1:3,1:3) = [0 1 0; 
                             0 0 1;
                             1 0 0];
Tr_NED_to_camera = pinv(Tr_camera_to_NED);

% Ooptical paramaters after rectification
camera_intrinsic.f     = 96.8239926;
camera_intrinsic.cu    = 247.70848004;
camera_intrinsic.cv    = 255.31920479;

% Geometry of setup. Obtained from camchain.yaml
transformation_cam0_to_imu = [-0.9995250378696743, 0.029615343885863205, -0.008522328211654736, 0.04727988224914392;...
               0.0075019185074052044, -0.03439736061393144, -0.9993800792498829, -0.047443232143367084;...
              -0.02989013031643309, -0.998969345370175, 0.03415885127385616, -0.0681999605066297;...
                0.0, 0.0, 0.0, 1.0];

transformation_imu_to_cam0 = inv(transformation_cam0_to_imu);

%% Get truth data set
% Get truth data
ground_truth_filename = strcat(path_to_dataset,dataset_name,'/dso/gt_imu.csv');
truth_mat = readmatrix(ground_truth_filename,"NumHeaderLines",2);
truth_time_vec = truth_mat(:,1) * 1e-9;
% Match truth data timestamps with camera timestamps
ind_in_truth = zeros(size(time_vec));
for ii = 1:size(ind_in_truth,2)
    [~,ind_in_truth(ii)] = min(abs(truth_time_vec - time_vec(ii)));
end
synced_truth = truth_mat(ind_in_truth,2:8);
truth_pos_imu = transpose(synced_truth(:,1:3));
truth_quat_imu = quaternion(synced_truth(:,4:7));
truth_dcm_imu = quat2rotm(truth_quat_imu);

% Truth pose in homogenous transformation format
truth_pose_imu = zeros(4,4,size(time_vec,2));   % Equivalent to transformation_world_to_imu
truth_pose_imu(1:3,1:3,:) = truth_dcm_imu(:,:,:);
truth_pose_imu(1:3,4,:) = reshape(truth_pos_imu,3,1,[]);
truth_pose_imu(4,4,:) = 1;

truth_pose_cam0_cv = truth_pose_imu;            % Equivalent to transformation_world_to_cam0. This use CV frame or EDN
truth_pose_cam0_ned = truth_pose_imu;           % Equivalent to transformation_world_to_cam0. This use NED frame

for ii = 1:size(truth_pose_imu,3)
    truth_pose_cam0_cv(:,:,ii) = truth_pose_imu(:,:,ii) * transformation_imu_to_cam0;
    truth_pose_cam0_ned(:,:,ii) = truth_pose_cam0_cv(:,:,ii) * Tr_camera_to_NED;
end

truth_pos_cam0_ned = reshape(truth_pose_cam0_ned(1:3,4,:),3,[]);
truth_dcm_cam0_ned = truth_pose_cam0_ned(1:3,1:3,:);
truth_quat_cam0_ned = quaternion(truth_dcm_cam0_ned,"rotmat","point");

% Data struct to store truth
truth.pos = truth_pos_cam0_ned;
truth.quat = truth_quat_cam0_ned;

%% Libviso2 setup
% Obtained these from the projection matrix calculation
viso_param.f     = 96.8239926;
viso_param.cu    = 247.70848004;
viso_param.cv    = 255.31920479;
viso_param.base  = 0.101039;

% These parameter changes are still to be tested to see if they improve tracking
%viso_param.ransac_iters = 200;
viso_param.max_features = 1000;
viso_param.bucket_width = 10;
viso_param.bucket.bucket_height = 10;


% init visual odometry
visualOdometryStereoMex('init',viso_param);

% Data struct to store odometry
odom.transformation_matrix = zeros(4,4,size(time_vec,2));
odom.transformation_matrix(:,:,1) = truth_pose_cam0_ned(:,:,1);
odom.pos = zeros(3,size(time_vec,2));
odom.pos(:,1) = truth.pos(:,1);
odom.quat = quaternion(zeros(size(time_vec,2),4));
odom.quat(1,:) = truth.quat(1,:);

%% Pre process parameters
% FAST detector
FAST_params.ROI = [33 1 512-2*33 512]; % Crop the portion of the image to not run FAST detector. This portion corresponds to area that stereo matching is invalid
FAST_params.min_quality = 0.05;
FAST_params.min_contrast = 0.05;

% ANMS 
ANMS_params.max_num_point = 5;
ANMS_params.tolerance = 0.1;

%% Filter configuration
% Data struct to store data
filter_est.pos = zeros(3,size(time_vec,2));
filter_est.quat = quaternion(zeros(size(time_vec,2),4));
filter_est.num_effective_particle = zeros(1,size(time_vec,2));
filter_est.num_map_features = zeros(1,size(time_vec,2));

% Particle Filter settings
filter.num_particle = 1;
filter.resample_threshold = 0.2; % Percentage of num_particle for resample to trigger

% Motion covariance = [cov_x, cov_y, cov_z, cov_phi, cov_theta, cov_psi]
filter.motion_sigma = [0.1; 0.1; 0.1; 0.03; 0.03; 0.03];

% Sensor model
filter.sensor.HFOV = deg2rad(65 * 2);
filter.sensor.VFOV = deg2rad(69 * 2);
filter.sensor.max_range = 6;
filter.sensor.min_range = 0.4;
filter.filter_sensor_noise_std = 0.01;
filter.R = diag([filter.filter_sensor_noise_std^2, ...
    filter.filter_sensor_noise_std^2, filter.filter_sensor_noise_std^2]);
filter.clutter_intensity = 10^-9;
filter.detection_prob = 0.5;

% Map PHD config
filter.birthGM_intensity = 0.4;
filter.birthGM_cov = [0.1, 0, 0; 0, 0.1, 0; 0, 0, 0.1];
filter.map_Q = diag([0.1, 0.1, 0.1].^2);
filter.adaptive_birth_dist_thres = 1;


% PHD GM management parameters
filter.pruning_thres = 10^-6;
filter.merge_dist = 4;
filter.num_GM_cap = 2000;

%% Misc
runtime = 0;

%% Initialize filter
% Set intial pose
filter_est.pos(:,1) = truth.pos(:,1);
filter_est.quat(1,:) = truth.quat(1,:);

% Use measurement from timestep 1 to initialize map
[left_rectified_img, right_rectified_img, depth_map] = ...
    get_images(path_to_dataset, dataset_name, name_array(1,:), depth_factor);

% Preprocess image to generate measurement sets
[measurements_ned, ~, selected_corners] = detect_and_project (left_rectified_img,...
    depth_map, FAST_params, ANMS_params, camera_intrinsic);

% Reproject measurements to world frame for testing
meas_in_world = reproject_meas(filter_est.pos(:,1),...
    filter_est.quat(1,:), measurements_ned);

particles = initialize_particles (filter.num_particle, filter_est.pos(:,1), filter_est.quat(1,:), ...
    meas_in_world, filter.birthGM_cov, filter.birthGM_intensity);

%% Plot first frame
% Preallocate cell for making video
frame = cell(size(time_vec,2),1);
 % Grey image
 figure(1)
 subplot (2,2,1)
 imshow(horzcat(left_rectified_img,right_rectified_img))
 hold on
 scatter (selected_corners.Location(:,1),selected_corners.Location(:,2),'r.')
 hold off
 xlabel("Rectified stereo image")

 % Depth map
 subplot (2,2,3)
 imagesc(depth_map)
 clim([0 6])
 cb = colorbar();
 ylabel(cb,'Depth (m)','FontSize',10,'Rotation',270)
 hold on
 scatter (selected_corners.Location(:,1),selected_corners.Location(:,2),'r.')
 hold off
 xlabel("Depth map")
 axis equal

 % Trajectory
 subplot_traj = subplot(2,2,[2,4]);
 draw_trajectory(truth_pos_cam0_ned(:,1), truth_quat_cam0_ned(1,:), truth_pos_cam0_ned(:,1), 1, 0.4, 2,'k',false, false);
 draw_trajectory(filter_est.pos(:,1), filter_est.quat(1,:), filter_est.pos(:,1),1,0.4,2,'g',true, false);
 % Global ref
 draw_trajectory([0;0;0], quaternion(1,0,0,0), [0;0;0], 1, 2, 2,'k',true, true);
 hold on
 scatter3(meas_in_world(1,:), meas_in_world(2,:), meas_in_world(3,:),'r.')
 axis equal
 grid on
 grid minor
 xlabel("X");
 ylabel("Y");
 zlabel("Z");
 xlim([-5 5]);
 ylim([-5 5]);
 zlim([-1 4]);
 title_str = sprintf("Current ind = %d. t = %0.2f", 1,time_vec(1)-time_vec(1));
 title(title_str)
 frame{1} = getframe(gcf);


%%

total_timer = tic;

%% Run loop
for kk = 2:size(time_vec,2)
    %
    iteration_timer = tic;
    %% Read images and pre-processing
    % Read greyscale and depth images
    [left_rectified_img, right_rectified_img, depth_map] = ...
        get_images(path_to_dataset, dataset_name, name_array(kk,:), depth_factor);
    
    % Preprocess image to generate measurement sets
    [measurements_ned, statistic, selected_corners] = detect_and_project (left_rectified_img,...
    depth_map, FAST_params, ANMS_params, camera_intrinsic);
    
    % Reproject measurements to world frame for testing
    meas_in_world = reproject_meas(truth_pos_cam0_ned(:,kk),...
        truth_quat_cam0_ned(kk,:), measurements_ned);

    %% LIBVISO2 for visual odometry
    % compute relative transformation of current pose reference to prev
    % pose
    libviso_transformation_mat = visualOdometryStereoMex('process',left_rectified_img,right_rectified_img);

    % MAROUN - This is the conversion from libviso to odometry to use with
    % SLAM
    % Convert relative transformation matrix into PHD-SLAM odometry
    relative_Tr_ned = Tr_NED_to_camera * pinv(libviso_transformation_mat) * Tr_camera_to_NED;
    [trans_vel_ned, rot_vel_ned] = transformation_to_odom(relative_Tr_ned, dt);
    odom_cmd.trans_vel_ned = trans_vel_ned;
    odom_cmd.rot_vel_ned = rot_vel_ned;

    % Propagate only the odometry - used for comparison only
    odom.transformation_matrix(:,:,kk) = odom.transformation_matrix(:,:,kk-1) * ...
        relative_Tr_ned;
    odom.pos(:,kk) = reshape(odom.transformation_matrix(1:3,4,kk),3,[]);
    temp_dcm = odom.transformation_matrix(1:3,1:3,kk);
    odom.quat(kk,:) = quaternion(temp_dcm,"rotmat","point");
    
    %% PHD-SLAM1
    % PHD-SLAM goes here just like the simulations
    particle = run_phd_slam (particles, odom_cmd, measurements_ned, ...
    filter, dt, truth.pos(:,kk),truth.quat(kk), 1);

    % Extract state and landmark estimates
    [pose_est, map_est] = extract_estimates_max_likeli(particle);
    filter_est.pos(:,kk) = pose_est.pos;
    filter_est.quat(kk,:) = pose_est.quat;
    filter_est.filter_est.num_map_features(kk) = map_est.exp_num_landmark;

    % Adaptive birth PHD (Lin Gao's implementation)
    particle = adaptive_birth_PHD (pose_est.pos, pose_est.quat, measurements_ned, map_est, filter, particle);
    
    % Resample (if needed)
    [particle, filter_est.num_effective_particle] = resample_particles(particle, filter);
    
    %% Project mapped features on to image frame
    reprojected_features = project_mapped_features_to_img(map_est, pose_est, camera_intrinsic);
    %%
    runtime = horzcat(runtime,toc(iteration_timer));
    %% Plotting 
    % Grey image
    figure(1)
    subplot (2,2,1)
    imshow(horzcat(left_rectified_img,right_rectified_img))
    hold on
    scatter (selected_corners.Location(:,1),selected_corners.Location(:,2),'r.')
    scatter (reprojected_features(:,1), reprojected_features(:,2),'g+')
    hold off
    xlabel("Rectified stereo image")

    % Depth map
    subplot (2,2,3)
    imagesc(depth_map)
    clim([0 6])
    cb = colorbar(); 
    ylabel(cb,'Depth (m)','FontSize',10,'Rotation',270)
    hold on 
    scatter (selected_corners.Location(:,1),selected_corners.Location(:,2),'r.')
    hold off
    xlabel("Depth map")
    axis equal
    
    % Trajectory
    subplot_traj = subplot(2,2,[2,4]);
    draw_trajectory(truth_pos_cam0_ned(:,kk), truth_quat_cam0_ned(kk,:), truth_pos_cam0_ned(:,kk), 1, 0.4, 2,'k',false, false);
    draw_trajectory(filter_est.pos(:,kk), filter_est.quat(kk,:), filter_est.pos(:,kk),1,0.4,2,'g',true, false);
    % Global ref
    draw_trajectory([0;0;0], quaternion(1,0,0,0), [0;0;0], 1, 2, 2,'k',true, true);
    hold on
    scatter3(meas_in_world(1,:), meas_in_world(2,:), meas_in_world(3,:),'r.')
    scatter3(map_est.feature_pos(1,:),map_est.feature_pos(2,:),map_est.feature_pos(3,:),'+g')
    plot_3D_phd(map_est,1)
    colorbar
    axis equal
    grid on
    grid minor
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([-1 4]);
    title_str = sprintf("Current ind = %d. t = %0.2f", kk,time_vec(kk)-time_vec(1));
    title(title_str)
    frame{kk-1} = getframe(gcf);
    exportgraphics(gcf, "viz.gif", Append=true);

end
toc(total_timer);

%
obj = VideoWriter("myvideo");
obj.FrameRate = 20;
open(obj);
for i=1:length(frame)
    writeVideo(obj,frame{i})
end
obj.close();

% release visual odometry
visualOdometryStereoMex('close');

figure()
plot (runtime)

