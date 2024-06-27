function measurements = gen_pred_projective_meas (pos, quat, landmark, camera_intrinsic)
% Generate predictive measurement given camera pose, landmark pos and camera intrinsics
% z = [x_pixel_location, y_pixel_location, depth]'
    pos_diff = landmark - pos;
    
    pos_diff_body_frame = rotateframe(quat, pos_diff');
    pos_diff_body_frame = pos_diff_body_frame';

    % Project to 2D plane and depth using intrinsics
    measurements = zeros(3,size(pos_diff_body_frame,2));

    for ii = 1:size(measurements,2)
        measurements(1,ii) = camera_intrinsic.cu + ...
            (pos_diff_body_frame(2,ii) * camera_intrinsic.f / pos_diff_body_frame(1,ii));
        measurements(2,ii) = camera_intrinsic.cv + ...
            (pos_diff_body_frame(3,ii) * camera_intrinsic.f / pos_diff_body_frame(1,ii));
        measurements(3,ii) = pos_diff_body_frame(1,ii);
    end
end