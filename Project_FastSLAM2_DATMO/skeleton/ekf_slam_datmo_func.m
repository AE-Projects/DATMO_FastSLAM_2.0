% This is the main EKF_SLAM function. For more detials see: Pno: 321 of
% Probabilistic Robotics
% Inputs:
%           mu_t_1:             (2N+3)X1    
%           sigma_t_1:          (2N+3)X(2N+3)
%           u_t:                   2X1  : enoder data: u_t(1) = right
%           z_t:                   2XM : M is the number of measurements
%           N_t_1:                  1  : Number of landmarks seen so far
%       
% Outputs:
%           mu_t:                (2N'+3)X1 : N' = number of features after update
%           sigma_t:           (2N'+3)X(2N'+3)
function [mu_t, sigma_t, N_t] = ekf_slam_datmo_func(mu_t_1, sigma_t_1, u_t, z_t, N_t_1,delta_t, R_t, Q_t)
    N_t = N_t_1;
    F_x = zeros(3,2*N_t+3);
    F_x(1,1) = 1;
    F_x(2,2) = 1;
    F_x(3,3) = 1;
    
    %Predict
    E_T = 2048;
    B = 0.35;
    R_L = 0.1;
    R_R = 0.1;
 
    mu_pose = mu_t_1(1:3,1);
    odom = calculate_odometry(u_t(1,1), u_t(2,1), E_T, B, R_L, R_R, delta_t, mu_pose);
    Gtemp = [0 0 -odom(2);
        0 0 odom(1);
        0 0 0];
    mu_bar_t = mu_t_1 + F_x'*odom;
    G_t = eye(2*N_t+3, 2*N_t+3) + F_x'*Gtemp*F_x;
    
    sigma_bar_t = G_t*sigma_t_1*G_t' + F_x'*R_t*F_x;

    %for test
    %mu_t = mu_bar_t;
    %sigma_t = sigma_bar_t;
    %disp(sigma_t)
    alpha = 12; %threshold for Mahalanobis Distance
    % 7 is okay for map_sym2
    % 20 is good for map_o3
    [Two, M] = size(z_t);
    for i = 1:M
        z_t(2,i) = mod(z_t(2,i)+pi, 2*pi) - pi;
        range_i = z_t(1,i);
        bearing_i = z_t(2,i);
        mu_Nt_temp = mu_bar_t(1:2,1) +  range_i*[cos(bearing_i+mu_bar_t(3,1)); sin(bearing_i+mu_bar_t(3,1))];
        mu_bar_temp = [mu_bar_t; mu_Nt_temp];
        N_t_temp = N_t + 1;
        [s_1, s_2] = size(sigma_bar_t);
        sigma_bar_temp = [sigma_bar_t; zeros(2, s_1)];
        sigma_bar_temp = [sigma_bar_temp zeros(s_2+2, 2)];
        sigma_bar_temp(end-1:end,end-1:end) = 10^-1*eye(2,2);
        %Associate
        [psi_arr, pi_arr, H_arr, z_arr] = associate(mu_bar_temp, sigma_bar_temp, Q_t, z_t(1:2,i), N_t_temp);
        pi_arr(end,1) = alpha;
        [v, j_i] = min(pi_arr);
        %disp(v)
        N_t_new = max(N_t, j_i);
        %Increase the shape of sigma if N_t increases
        if N_t_new > N_t
            [s_1, s_2] = size(sigma_bar_t);
            sigma_bar_t = [sigma_bar_t; zeros(2, s_1)];
            sigma_bar_t = [sigma_bar_t zeros(s_2+2,2)];
            sigma_bar_t(end-1:end, end-1:end) = 10^-1*eye(2,2);
            mu_bar_t = mu_bar_temp;
            [ns_1, ns_2] = size(sigma_bar_t);
            if ns_1 == ns_2
                %fprintf('Sigma size matches')
                if ns_2 == s_1 +2
                    %fprintf('Sigma size is increased correctly \n')
                end
            end
        elseif N_t_new == N_t
            %fprintf('Not adding any features \n')
            H_arr = H_arr(:,1:end-2,:);
            z_arr = z_arr(:,1:end-1);
        end
    
        N_t = N_t_new;
        % I expect H_arr(:,:, j_i) will give a matrix of size 2,2N+3
        %disp(size(H_arr(:,:,j_i)))
        K_i_t = sigma_bar_t*H_arr(:,:,j_i)'*inv(psi_arr(:,:,j_i));
        %disp(size(K_i_t))
        H_temp = H_arr(:,:,j_i);
        %z_t_j_i = [H_temp(1,1)/H_temp(2,2); atan2(H_temp(2,1), -H_temp(2,2)) - mu_bar_t(3,1)];
        z_t_j_i = z_arr(:,j_i);
        %disp(z_t(1:2,i) - z_t_j_i);
        %fprintf('Kalman gain for theta: %f and %f \n',K_i_t(3,1), K_i_t(3,2))
        %fprintf('Kalman gain for X: %f and %f \n',K_i_t(1,1), K_i_t(1,2))
        mu_bar_t = mu_bar_t + K_i_t*(z_t(1:2,i) - z_t_j_i);
        sigma_bar_t = (eye(2*N_t+3, 2*N_t+3) - K_i_t*H_temp)*sigma_bar_t;
        %disp(size(mu_bar_t))
        %disp(mu_bar_t(end-1:end,1))
    end
    sigma_t = sigma_bar_t;
    mu_t = mu_bar_t;
    mu_t(3,1) = mod(mu_t(3,1)+pi, 2*pi) - pi;

end
