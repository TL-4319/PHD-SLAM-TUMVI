close all;
clear; 
clc;

addpath('util/')

%% Parse data
% Parse estimations
load phd_slam_tum_vi_run.mat;


%% 
time_vec = 1:size(simulation.truth.pos,2);

% Position 
est_pos = simulation.filter.pos;
true_pos = simulation.truth.pos;

est_quat = simulation.filter.quat;
true_quat = simulation.truth.quat;  

% Convert to euler [yaw, pitch, roll]
est_euler = transpose(quat2eul(est_quat));
true_euler = transpose(quat2eul(true_quat));


% Calculate distance travelled for relative metric
dist_travel = true_pos;
dist_travel(:,2:end) = true_pos(:,2:end) - true_pos(:,1:end-1);
dist_travel = vecnorm(dist_travel);
dist_travel = cumsum(dist_travel);

%% Odometry
odom_pos = simulation.odom.pos;
odom_quat = simulation.odom.quat;
odom_euler = transpose(quat2eul(odom_quat));

%% Error calc
pos_error = est_pos - true_pos;
dist_error = vecnorm(pos_error);

euler_error = est_euler - true_euler;


rel_trans_error = dist_error./dist_travel;
rel_trans_error(1) = 0;

rel_euler_error = euler_error;
rel_euler_error(1,:) = euler_error(1,:) ./ dist_travel;
rel_euler_error(2,:) = euler_error(2,:) ./ dist_travel;
rel_euler_error(3,:) = euler_error(3,:) ./ dist_travel;
rel_euler_error(:,1) = [0;0;0];

% Odometry only error
odom_pos_error = odom_pos - true_pos;
odom_dis_error = vecnorm(odom_pos_error);

odom_euler_error = odom_euler - true_euler;


% Handle wrapping of angle error
for ii = 1:size(euler_error,2)
    if abs(euler_error (1,ii)) > pi
        euler_error (1,ii) = euler_error (1,ii) - sign(euler_error (1,ii)) * 2*pi;
    end
    if abs(euler_error (2,ii)) > pi
        euler_error (2,ii) = euler_error (2,ii) - sign(euler_error (2,ii)) *  2*pi;
    end
    if abs(euler_error (3,ii)) > pi
        euler_error (3,ii) = euler_error (3,ii) - sign(euler_error (3,ii)) * 2*pi;
    end
    if abs(odom_euler_error (1,ii)) > pi
        odom_euler_error (1,ii) = odom_euler_error (1,ii) - sign(odom_euler_error (1,ii)) * 2*pi;
    end
    if abs(odom_euler_error (2,ii)) > pi
        odom_euler_error (2,ii) = odom_euler_error (2,ii) - sign(odom_euler_error (2,ii)) * 2*pi;
    end
    if abs(odom_euler_error (3,ii)) > pi
        odom_euler_error (3,ii) = odom_euler_error (3,ii) - sign(odom_euler_error (3,ii)) * 2*pi;
    end
end
%% Plot
figure(1)
subplot (3,1,1)
plot (time_vec, abs(pos_error(1,:)),'DisplayName','PHD-SLAM',LineWidth=2)
hold on 
plot (time_vec, abs(odom_pos_error(1,:)),'DisplayName','Odometry',LineWidth=2)
xlabel("Time index")
ylabel("error (m)")
grid on
title("X error")
legend

subplot (3,1,2)
plot (time_vec, abs(pos_error(2,:)),'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(odom_pos_error(2,:)),'DisplayName','Odometry',LineWidth=2)
xlabel("Time index")
ylabel("error (m)")
grid on
title("Y error")

subplot (3,1,3)
plot (time_vec, abs(pos_error(3,:)),'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(odom_pos_error(3,:)),'DisplayName','Odometry',LineWidth=2)
xlabel("Time index")
ylabel("error (m)")
grid on
title("Z error")

figure(2)
plot (time_vec, dist_error,'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, odom_dis_error,'DisplayName','Odometry',LineWidth=2)
xlabel("Time index")
ylabel("error (m)")
grid on
title("Distance error")
legend

figure(3)
subplot(3,1,1)
plot (time_vec, abs(euler_error(3,:)) * 180/pi,'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(odom_euler_error(3,:)) * 180/pi, 'DisplayName','Odometry',LineWidth=2)
xlabel("Time index")
ylabel("Roll error (deg)")
grid on
title("Rotational error")
legend

subplot(3,1,2)
plot (time_vec, abs(euler_error(2,:)) * 180/pi,'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(odom_euler_error(2,:)) * 180/pi, 'DisplayName','Odometry',LineWidth=2)
xlabel("Time index")
ylabel("Pitch error (deg)")
grid on


subplot(3,1,3)
plot (time_vec, abs(euler_error(1,:)) * 180/pi,'DisplayName','PHD-SLAM',LineWidth=2)
hold on
plot (time_vec, abs(odom_euler_error(1,:)) * 180/pi, 'DisplayName','Odometry',LineWidth=2)
xlabel("Time index")
ylabel("Yaw error (deg)")
grid on


