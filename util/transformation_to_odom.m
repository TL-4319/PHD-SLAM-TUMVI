function [trans_vel, rot_vel] = transformation_to_odom(Tr, dt)
    % Convert relative transformation output from Libviso2 to odometry 
    % estimates
    p = Tr(1:3,4);
    R = Tr(1:3, 1:3);
    
    % Logarithmic map of relative transformation matrix to construct the
    % skew symmetric matrix for average rotation rates
    log_R = logm(R);
    if ~isreal(log_R) || any(~isfinite(log_R),'all')
        wz = 0;
        wy = 0;
        wx = 0;
    else
        wz = log_R(2,1)/dt;
        wy = log_R(1,3)/dt;
        wx = log_R(3,2)/dt;
    end
    

    trans_vel = p/dt;
    rot_vel = [wx; wy; wz];
end