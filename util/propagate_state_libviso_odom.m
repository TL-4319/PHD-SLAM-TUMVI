function [pos, quat] = propagate_state_libviso_odom (prev_pos, prev_quat, body_trans_vel, body_rot_vel, dt)
    % This propagate state is used for odometry values obtained from
    % Libviso. This is different from normal kinematic state propagation
    % due to how the relative transformation is defined
    
    % Propagate quaternion - https://ahrs.readthedocs.io/en/latest/filters/angular.html
    quat = rotate_quat(prev_quat, body_rot_vel, dt);

    % Propagate velocity
    quat_half = slerp (prev_quat, quat, 0.5);
    delta_body_trans_quat = quaternion(0, body_trans_vel(1)*dt, body_trans_vel(2)*dt, body_trans_vel(3)*dt);
    
    % Libviso defines the translation is defined in the prev frame
    world_trans_quat = prev_quat * delta_body_trans_quat * conj(prev_quat);

    world_trans_vec = zeros(3,1);
    [~, world_trans_vec(1), world_trans_vec(2), world_trans_vec(3)] = parts(world_trans_quat);
    pos = prev_pos + world_trans_vec;
    
end


function quat = rotate_quat(prev_quat, body_rot_vel, dt)
    prev_quat_vec = compact(prev_quat)'; % Convert to vector type for ease of implementation
    
    OMEGA = calc_omega(body_rot_vel);
    quat_vec = (eye(4) + 0.5 * OMEGA * dt) * prev_quat_vec;
    quat_vec = quat_vec / norm(quat_vec);
    quat = quaternion(quat_vec');
end

function OMEGA = calc_omega(body_rot_vel)
    skew_mat = calc_skew_mat(body_rot_vel);
    OMEGA = zeros(4);
    OMEGA (2:4,1) = body_rot_vel;
    OMEGA (1, 2:4) = -body_rot_vel';
    OMEGA (2:4, 2:4) = -skew_mat;
end

function skew_mat = calc_skew_mat (body_rot_vel)
    wz = body_rot_vel(3);
    wy = body_rot_vel(2);
    wx = body_rot_vel(1);

    skew_mat = [0, -wz, wy;
               wz,   0,-wx;
              -wy,  wx,  0];
end

