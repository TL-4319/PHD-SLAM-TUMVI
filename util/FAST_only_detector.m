function [statistic, selected_corners] = FAST_only_detector (left_rectified_img,...
    FAST_params)
    %% Function to create 3D measurement relative to camera in body fixed NED frame and the the raw measurement in image plane and depth

    %% FAST feature detector
    FAST_corners = detectFASTFeatures(left_rectified_img, ...
        'MinQuality',FAST_params.min_quality, 'MinContrast', FAST_params.min_contrast,...
        'ROI', FAST_params.ROI);
    statistic.num_FAST_points = FAST_corners.Count;

    selected_corners = FAST_corners;
    
end