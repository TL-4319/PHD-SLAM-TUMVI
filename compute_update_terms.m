function [pred_meas, K, S, P, Sinv] = compute_update_terms (cur_particle, GM_mu, GM_cov, R, camera_intrinsic)
    % pred_meas vector of form [x_pixel_location, y_pixel_location, depth]
    num_GM = size (GM_mu,2);
    pred_meas = zeros(3,num_GM);
    K = zeros (3,3,num_GM);
    P = K;
    S = K;
    Sinv = K;
    %H = calc_meas_jacobian (cur_particle.quat);
    rot_matrix = quat2rot(compact(cur_particle.quat),"frame");
    for i = 1:num_GM
        pred_ned_pos = gen_pred_meas(cur_particle.pos, cur_particle.quat, GM_mu(:,i));
        pred_meas(:,i) = gen_pred_projective_meas(cur_particle.pos, cur_particle.quat, GM_mu(:,i),camera_intrinsic);
        H = compute_projective_jacobian(rot_matrix, pred_ned_pos, camera_intrinsic);
        S(:,:,i) = H * GM_cov(:,:,i) * H' + R;
        S(:,:,i) = (S(:,:,i) + S(:,:,i)')/2; % Avoid numerical instability
        Sinv (:,:,i) = pinv(S(:,:,i));
        K(:,:,i) = GM_cov(:,:,i) * H' * Sinv (:,:,i);
        temp = (eye(3) - K(:,:,i) * H);
        P(:,:,i) = temp * GM_cov(:,:,i) * temp' + K(:,:,i) * R * K(:,:,i)';
    end
end

function H_projective = compute_projective_jacobian(rot_matrix, ned_pos, camera_intrinsic)
    x_squared = ned_pos(1)^2;
    H_projective = zeros(3,3);
    
    H_projective(1,1) = camera_intrinsic.f * (rot_matrix(2,1) * ned_pos(1)...
        - rot_matrix(1,1) * ned_pos(2)) / x_squared;
    H_projective(1,2) = camera_intrinsic.f * (rot_matrix(2,2) * ned_pos(1)...
        - rot_matrix(1,2) * ned_pos(2)) / x_squared;
    H_projective(1,3) = camera_intrinsic.f * (rot_matrix(2,3) * ned_pos(1)...
        - rot_matrix(1,3) * ned_pos(2)) / x_squared;

    H_projective(2,1) = camera_intrinsic.f * (rot_matrix(3,1) * ned_pos(1)...
        - rot_matrix(1,1) * ned_pos(3)) / x_squared;
    H_projective(2,2) = camera_intrinsic.f * (rot_matrix(3,2) * ned_pos(1)...
        - rot_matrix(1,2) * ned_pos(3)) / x_squared;
    H_projective(2,3) = camera_intrinsic.f * (rot_matrix(3,3) * ned_pos(1)...
        - rot_matrix(1,3) * ned_pos(3)) / x_squared;

    H_projective(3,1) = rot_matrix(1,1);
    H_projective(3,2) = rot_matrix(1,2);
    H_projective(3,3) = rot_matrix(1,3);
end
