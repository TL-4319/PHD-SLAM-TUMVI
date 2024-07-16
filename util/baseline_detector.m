function [statistic, selected_corners] = baseline_detector (left_rectified_img,...
    depth_map, FAST_params, ANMS_params)
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

    
end