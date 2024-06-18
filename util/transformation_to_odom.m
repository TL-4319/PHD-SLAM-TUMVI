function [trans_vel, rot_vel] = transformation_to_odom(Tr, dt)
    % Convert relative transformation output from Libviso2 to odometry 
    % estimates
    p = Tr(1:3,4);
    R = Tr(1:3, 1:3);

    R_delta = R - eye(3);
    wz = -R_delta(1,2)/dt;
    wy = -R_delta(3,1)/dt;
    wx = -R_delta(2,3)/dt;

    trans_vel = p/dt;
    rot_vel = [wx; wy; wz];
end