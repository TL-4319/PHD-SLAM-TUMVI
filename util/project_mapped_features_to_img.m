function pixel_location = project_mapped_features_to_img (map_est, pose_est, camera_intrinsic)
    %% Function to calculate landmark position on image frame
    % Calc predicted 3D NED measurement in camera frame
    pos_diff_world_frame = map_est.feature_pos - pose_est.pos;
    pos_diff_cam_frame = rotateframe(pose_est.quat, pos_diff_world_frame');

    % Project to 2D plane using intrinsic
    pixel_location = zeros(map_est.exp_num_landmark,2);

    for ii = 1:size(map_est.feature_pos,2)
        pixel_location(ii,1) = camera_intrinsic.cu + ...
            (pos_diff_cam_frame(ii,2) * camera_intrinsic.f / pos_diff_cam_frame(ii,1));
        pixel_location(ii,2) = camera_intrinsic.cv + ...
            (pos_diff_cam_frame(ii,3) * camera_intrinsic.f / pos_diff_cam_frame(ii,1));
    end


end