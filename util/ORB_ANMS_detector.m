function [statistic, selected_corners] = ORB_ANMS_detector (left_rectified_img,...
    depth_map, ORB_params, ANMS_params)
    %% Function to create 3D measurement relative to camera in body fixed NED frame and the the raw measurement in image plane and depth
    %% ORB feature detector
    ORB_corners = detectORBFeatures(left_rectified_img, ...
        'ScaleFactor',ORB_params.scale_factor, 'NumLevels', ORB_params.num_level,...
        'ROI', ORB_params.ROI);
    statistic.num_detector_points = ORB_corners.Count;

    %% Depth map validation
    depth_val = zeros(1,ORB_corners.Count);
    for j = 1:statistic.num_detector_points
        depth_val(j) = depth_map(round(ORB_corners.Location(j,2)),round(ORB_corners.Location(j,1)));
    end
    [~, depth_valid_ind] = find(depth_val ~= 0);
    valid_depth_corners = ORB_corners (depth_valid_ind);
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