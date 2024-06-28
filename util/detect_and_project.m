function [measurements_ned, measurements_projective, statistic, selected_corners] = detect_and_project (left_rectified_img,...
    depth_map, FAST_params, ANMS_params, camera_intrinsic)
    %% Function to create 3D measurement relative to camera in body fixed NED frame and the the raw measurement in image plane and depth

    %% FAST feature detector
    FAST_corners = detectFASTFeatures(left_rectified_img, ...
        'MinQuality',FAST_params.min_quality, 'MinContrast', FAST_params.min_contrast,...
        'ROI', FAST_params.ROI);
    statistic.num_FAST_points = FAST_corners.Count;

    %% Depth map validation
    depth_val = zeros(1,FAST_corners.Count);
    for j = 1:statistic.num_FAST_points
        depth_val(j) = depth_map(FAST_corners.Location(j,2),FAST_corners.Location(j,1));
    end
    [~, depth_valid_ind] = find(depth_val ~= 0);
    valid_depth_corners = FAST_corners (depth_valid_ind);
    statistic.num_depth_valid_points = valid_depth_corners.Count;

    %% ANMS
    % Sort features via their hessian corner metric
    if statistic.num_depth_valid_points > (ANMS_params.max_num_point + 20)    %Only do ANMS if we have more points than ANMS max num point. If not, SSC seg fault
        [~,sort_ind] = sort(valid_depth_corners.Metric, 'descend');
        sorted_points = valid_depth_corners(sort_ind);
    
        selected_idx = ssc(double(sorted_points.Location), ANMS_params.max_num_point, ...
            ANMS_params.tolerance, size(left_rectified_img,2), size(left_rectified_img,1));
        selected_corners = sorted_points((selected_idx+1)'); % +1 since matlab is one-indexed
    else
        selected_corners = valid_depth_corners;
    end
    statistic.ANMS_num_selected_point = selected_corners.Count;
    selected_depth_pixel = selected_corners.Location;

    %% Reprojection
    % Create measurement set. NED metric position of corners relative to
    % camera. Depth is corresponding to North or X axis

    % MAROUN - Here is the step where I convert the measurement from2D
    % images to 3D points where NED convention matters for PHD-SLAM

    % Projective measurement vector is of form [x_pixel_loc, y_pixel_loc, depth]
    measurements_ned = zeros(3,statistic.ANMS_num_selected_point);
    measurements_projective = measurements_ned; 
    for ii = 1:statistic.ANMS_num_selected_point
        % NED X axis / North / camera Z axis
        measurements_ned(1,ii) = depth_map(selected_depth_pixel(ii,2),selected_depth_pixel(ii,1));
        
        % NED Y axis / East / camera X axis
        measurements_ned(2,ii) = (selected_depth_pixel(ii,1) - camera_intrinsic.cu) * ...
            measurements_ned(1,ii) / camera_intrinsic.f;
        
        % NED Z axis / Down / camera Y axis
        measurements_ned(3,ii) = (selected_depth_pixel(ii,2) - camera_intrinsic.cv) * ...
            measurements_ned(1,ii) / camera_intrinsic.f;

        % Populate projective measurement vector
        measurements_projective(1,ii) = selected_depth_pixel(ii,1);

        measurements_projective(2,ii) = selected_depth_pixel(ii,2);

        measurements_projective(3,ii) = measurements_ned(1,ii);
    end
    
end