function pixel_location = project_mapped_features_to_img (map_est, pose_est, camera_intrinsic, sensor)
    %% Function to calculate landmark position on image frame
    sensor.HFOV = sensor.HFOV + deg2rad(12); %Artificially inflate the FOV just for plotting
    [pos_diff_cam_frame,~] = check_in_FOV_3D(map_est.feature_pos, ...
        pose_est.pos, pose_est.quat,sensor);

    % Project to 2D plane using intrinsic
    pixel_location = zeros(size(pos_diff_cam_frame,2),2);

    for ii = 1:size(pixel_location,1)
        pixel_location(ii,1) = camera_intrinsic.cu + ...
            (pos_diff_cam_frame(2,ii) * camera_intrinsic.f / pos_diff_cam_frame(1,ii));
        pixel_location(ii,2) = camera_intrinsic.cv + ...
            (pos_diff_cam_frame(3,ii) * camera_intrinsic.f / pos_diff_cam_frame(1,ii));
    end
    
end