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

%% Pre process parameters
max_depth_m = 5; 

ANMS_max_num_point = 100;
ANMS_tolerance = 0.1;

%% Filter configuration



%% Misc
runtime = 0;

%% Initialize filter


%% Run loop
for kk = 1:size(elapsed_time,2)
    %% Read images and pre-processing
    % Read greyscale RGB image
    tic
    left_rectified_name = strcat(path_to_dataset,dataset_name,'/mav0/cam0/rectified/',name_array(kk,:),'.png');
    left_rectified_img = imread(left_rectified_name);
    right_rectified_name = strcat(path_to_dataset,dataset_name,'/mav0/cam1/rectified/',name_array(kk,:),'.png');
    right_rectified_img = imread(right_rectified_name);

    % Read depthmap
    depth_name = strcat(path_to_dataset,dataset_name,'/mav0/depth/data/',name_array(kk,:),'.png');
    depth_img = imread(depth_name);
    depth_map = double(depth_img)/depth_factor; % Convert from uint16 depth image to metric depth map (values are in meters)
    
    % Fast corner detection
    FAST_ROI = [33 1 512-33 512]; % Crop the portion of the image to not run FAST detector. This portion corresponds to area that stereo matching is invalid
    FAST_corners = detectFASTFeatures(left_rectified_img, ...
        'MinQuality',0.05, 'MinContrast', 0.05, 'ROI', FAST_ROI);
    num_FAST_points = FAST_corners.Count;
    
    % Depth map validation
    depth_val = zeros(1,FAST_corners.Count);
    for j = 1:num_FAST_points
        depth_val(j) = depth_map(FAST_corners.Location(j,2),FAST_corners.Location(j,1));
    end
    [~, depth_valid_ind] = find(depth_val ~= 0);
    valid_depth_corners = FAST_corners (depth_valid_ind);
    num_valid_points = valid_depth_corners.Count;

    % ANMS
    % Sort features via their hessian corner metric
    if num_valid_points > ANMS_max_num_point + 2    %Only do ANMS if we have more points than ANMS max num point. If not, SSC seg fault
        [~,sort_ind] = sort(valid_depth_corners.Metric, 'descend');
        sorted_points = valid_depth_corners(sort_ind);
    
        selected_idx = ssc(double(sorted_points.Location), ANMS_max_num_point, ...
            ANMS_tolerance, size(left_rectified_img,2), size(left_rectified_img,1));
        selected_corners = sorted_points((selected_idx+1)'); % +1 since matlab is one-indexed
    else
        selected_corners = valid_depth_corners;
    end
    num_selected_point = selected_corners.Count;
    selected_depth_pixel = selected_corners.Location;
    
    %% Reprojection
    % Create measurement set. NED metric position of corners relative to
    % camera. Depth is corresponding to North or X axis

    % MAROUN - Here is the step where I convert the measurement from2D
    % images to 3D points where NED convention matters for PHD-SLAM
    meas_ned = zeros(3,num_selected_point);
    for ii = 1:num_selected_point
        % NED X axis / North / camera Z axis
        meas_ned(1,ii) = depth_map(selected_depth_pixel(ii,2),selected_depth_pixel(ii,1));
        
        % NED Y axis / East / camera X axis
        meas_ned(2,ii) = (selected_depth_pixel(ii,1) - camera_intrinsic.cu) * ...
            meas_ned(1,ii) / camera_intrinsic.f;
        
        % NED Z axis / Down / camera Y axis
        meas_ned(3,ii) = (selected_depth_pixel(ii,2) - camera_intrinsic.cv) * ...
            meas_ned(1,ii) / camera_intrinsic.f;
    end
    
    % Reproject measurements to world frame for testing
    meas_in_world = reproject_meas(truth_pos_cam0_ned(:,kk),...
        truth_quat_cam0_ned(kk,:), meas_ned);

    %% LIBVISO2 for visual odometry
    % compute relative transformation of current pose reference to prev
    % pose
    libviso_transformation_mat = visualOdometryStereoMex('process',left_rectified_img,right_rectified_img);

    % MAROUN - This is the conversion from libviso to odometry to use with
    % SLAM
    % Convert relative transformation matrix into PHD-SLAM odometry
    relative_Tr_ned = Tr_NED_to_camera * pinv(libviso_transformation_mat) * Tr_camera_to_NED;
    [trans_vel_ned, rot_vel_ned] = transformation_to_odom(relative_Tr_ned, dt);
    
    %% PHD-SLAM1
    % PHD-SLAM goes here just like the simulations

    %% Plotting 
    % Grey image
    figure(1)
    subplot (2,2,1)
    imshow(horzcat(left_rectified_img,right_rectified_img))
    hold on
    plot (selected_corners)
    hold off
    xlabel("Rectified stereo image")

    % Depth map
    subplot (2,2,3)
    imagesc(depth_map)
    clim([0 6])
    cb = colorbar(); 
    ylabel(cb,'Depth (m)','FontSize',10,'Rotation',270)
    hold on 
    plot (selected_corners)
    hold off
    xlabel("Depth map")
    axis equal
    
    % Trajectory
    subplot_traj = subplot(2,2,[2,4]);
    draw_trajectory(truth_pos_cam0_ned(:,kk), truth_quat_cam0_ned(kk,:), truth_pos_cam0_ned(:,1:kk), 1, 0.4, 2,'k',false, true);
    % Global ref
    draw_trajectory([0;0;0], quaternion(1,0,0,0), [0;0;0], 1, 2, 2,'k',true, true);
    hold on
    scatter3(meas_in_world(1,:), meas_in_world(2,:), meas_in_world(3,:),'k*')
    axis equal
    grid on
    grid minor
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([-1 4]);

    exportgraphics(gcf,'viz.gif','Append',true);


    

    %% LIBVISO2 stats
     % %output statistics
     % num_matches = visualOdometryStereoMex('num_matches');
     % num_inliers = visualOdometryStereoMex('num_inliers');
     % disp(['Frame: ' num2str(kk) ...
     %     ', Matches: ' num2str(num_matches) ...
     %     ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
    
    runtime = horzcat(runtime,toc);

end

% release visual odometry
visualOdometryStereoMex('close');

figure()
plot (runtime)

