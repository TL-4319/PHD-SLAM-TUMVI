function particle = run_phd_slam (prev_particle, odom_cmd, measurement, ...
    filter, dt, truth_pos, truth_quat, use_truth_pose_flag)
    % Pre allocating particle struct
    particle = prev_particle;
    
    %% Run one timestep of PHD-SLAM measurement update
  
    for par_ind = 1:size(prev_particle,2)
        cur_particle = prev_particle(1,par_ind);    % Copy of current particle from particle list
        
        %% Time update step
        % Sample additive noise for odometry command
        body_vel_sample = normrnd(0,filter.motion_sigma(1),3,1);
        body_rot_vel_sample = normrnd(0,filter.motion_sigma(4),3,1);

        % Generate odometry cmd sample
        body_vel_sample = odom_cmd.trans_vel_ned + body_vel_sample;
        body_rot_vel_sample = odom_cmd.rot_vel_ned + body_rot_vel_sample;

        % Propagate particle 
        [cur_particle.pos, cur_particle.quat] = propagate_state_libviso_odom (cur_particle.pos, ...
            cur_particle.quat,body_vel_sample, body_rot_vel_sample, dt);
        
        % Replaced estimated pose with truth pose for debuggin
        if use_truth_pose_flag == 1
            cur_particle.pos = truth_pos;
            cur_particle.quat = truth_quat;
        end
        % Append pose values to output particle structure
        particle(1,par_ind).pos = cur_particle.pos;
        particle(1,par_ind).quat = cur_particle.quat;
        
        %% GM checking step
        % Check for GM is in FOV
        num_GM = size(cur_particle.gm_mu,2);
        [~, GM_in_FOV] = check_in_FOV_3D (cur_particle.gm_mu, ...
                cur_particle.pos, cur_particle.quat, filter.sensor);

        % Extract GM components not in FOV. No changes are made to them
        GM_out_FOV = ~GM_in_FOV;
        GM_mu_out = cur_particle.gm_mu(:,GM_out_FOV);
        GM_cov_out = cur_particle.gm_cov (:,:,GM_out_FOV);
        GM_inten_out = cur_particle.gm_inten(GM_out_FOV);

        % Extract GM components in FOV. These are used during update
        % Predict 
        GM_mu_in = cur_particle.gm_mu(:,GM_in_FOV);
        GM_cov_in = cur_particle.gm_cov (:,:,GM_in_FOV);
        GM_inten_in = cur_particle.gm_inten(GM_in_FOV);

        % Copy a set of previous GM k|k-1 for use in update step
        GM_mu_in_prev = GM_mu_in;
        GM_cov_in_prev = GM_cov_in;
        GM_inten_in_prev = GM_inten_in;        

        num_GM_in = size(GM_inten_in,2);

        %% Only  perform update steps if there are GM components in the FOV
        if num_GM_in > 0
            %% Pre compute inner update terms
            [pred_z, K, S, P, Sinv] = compute_update_terms (cur_particle,...
                GM_mu_in_prev, GM_cov_in_prev, filter.R);

            % Update PHD components if missed detected GM
            GM_inten_in = (1 - filter.detection_prob) * GM_inten_in_prev;

            % Update PHD if detected GM and calculate particle likilihood
            % using single cluster likelihood
            likelipz = zeros(1,size(measurement,2));
            for zz = 1:size(measurement,2)
                likelipf = zeros(1,num_GM_in);
                tau = likelipf;
                for jj = 1:num_GM_in
                    tau(1,jj) =  filter.detection_prob * GM_inten_in_prev(jj) * ...
                        mvnpdf(measurement(:,zz),pred_z(:,jj),S(:,:,jj));
                    if GM_inten_in_prev(jj) > 0.8
                        likelipf(1,jj) = tau(1,jj); % Only include strong GM in particle likilihood calculation
                    end
                    mu = GM_mu_in_prev(:,jj) + K(:,:,jj) * (measurement(:,zz) - pred_z(:,jj));
                    GM_mu_in = horzcat(GM_mu_in, mu);
                    GM_cov_in = cat(3,GM_cov_in, P(:,:,jj));
                end %jj = 1:num_GM_in
                likelipz(1,zz) = filter.clutter_intensity + sum(likelipf,2); 
                tau_sum = filter.clutter_intensity + sum(tau,2);
                for jj = 1:num_GM_in
                    nu = tau(jj) / tau_sum;
                    GM_inten_in = horzcat(GM_inten_in, nu);
                end %jj = 1:num_GM_in
            end %zz = 1:size(measurement,2)
            particle(1,par_ind).w = exp(sum(GM_inten_in_prev,2)) * ...
                (prod(likelipz,2) + 1e-99) * cur_particle.w;

            %% Clean up GM components
            [GM_mu_in, GM_cov_in, GM_inten_in] = cleanup_PHD (GM_mu_in,...
                GM_cov_in, GM_inten_in, filter.pruning_thres, ...
                filter.merge_dist, filter.num_GM_cap);
            
            %% Add back components not in FOV
            particle(1,par_ind).gm_mu = cat(2,GM_mu_in, GM_mu_out);
            particle(1,par_ind).gm_inten = cat (2, GM_inten_in, GM_inten_out);
            particle(1,par_ind).gm_cov = cat(3,GM_cov_in, GM_cov_out);
        else %if num_GM_in > 0
            particle(1,par_ind).w = 1e-99;
            particle_likelihood_vec(1,par_ind) = 1e-99;
        end %if num_GM_in > 0

    end % for par_ind = 1:size(prev_particle,2)
end %function