function [statistic, selected_corners] = ORB_only_detector (left_rectified_img,...
     ORB_params)
    %% Function to create 3D measurement relative to camera in body fixed NED frame and the the raw measurement in image plane and depth

    %% ORB feature detector
    ORB_corners = detectORBFeatures(left_rectified_img, ...
        'ScaleFactor',ORB_params.scale_factor, 'NumLevels', ORB_params.num_level,...
        'ROI', ORB_params.ROI);
    statistic.num_detector_points = ORB_corners.Count;

    selected_corners.Location = round(ORB_corners.Location);

    selected_corners.Metric = ORB_corners.Metric;

end