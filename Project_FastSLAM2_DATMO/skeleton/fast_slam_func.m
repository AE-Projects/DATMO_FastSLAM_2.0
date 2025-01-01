% This is the main FAST_SLAM function. For more detials see: Pno: 461 of
% Probabilistic Robotics
% Inputs:
%           Y_t_1:              (1XM) array of class Y at t-1 : M = number of particles   
%           u_t:                   2X1  : enoder data: u_t(1) = right
%           z_t:                   2XI : I is the number of measurements
%       
% Outputs:
%           Y_t:              (1XM) array of class Y at t
%
function [Y_t] = fast_slam_func(z_t, u_t, delta_t, Y_t_1, R_t, Q_t)
    p0 = 0.5;
    [one, M] = size(Y_t_1);
    w_arr = zeros(1,M);
    for k = 1:M
        Y_t_1_k = Y_t_1(1,k);
        pos = Y_t_1_k.pose;
        [two, I] = size(z_t);
        N_t_1_k = Y_t_1_k.N_feat;
        w_all = 1;
        for i = 1:I %for all measurements
            z_t_i = z_t(:,i);
           %associate
           [pi_j,x_t_j,Q_j,H_m,H_x,z_hat_j]= associate(Y_t_1_k,Q_t,R_t,z_t_i,u_t,delta_t);
           %assuming pi_j is an array with length 1XN_t_1
           pi_1_plus_N_t_1 = p0; %likelihhod of new feature
           [v, c_hat] = max([pi_j pi_1_plus_N_t_1]);
           N_t_k = max(N_t_1_k, c_hat);
            %if (N_t_k > N_t_1_k)
             %   fprintf('Feature is Added \n')
              %  fprintf('Second best likelihood: %f \n', max(pi_j));
           % else
           %     fprintf('No feature is addded \n')
            %end

           %update kalman filters for each features
           
           [w_k, Y_aux_k] = update_KF_features(Y_t_1_k, N_t_k, c_hat, z_t_i, u_t, H_m, H_x, z_hat_j, x_t_j, Q_j, p0, Q_t, R_t, delta_t);
           %Y_aux_k.weight = w_k;
           Y_t_1_k = Y_aux_k;
           Y_t_1_k.pose = pos;
           w_all = w_all*w_k;
           N_t_1_k = N_t_k;
        end
        Y_aux(1,k) = Y_aux_k;
        w_arr(1,k) = w_all;
        %w_k = 1;
    end
    %create Y_aux
    %create null Y_t
    
    %Resample
    w_arr = w_arr*(1/sum(w_arr));
    Y_t = systematic_resample_SLAM(Y_aux, w_arr);

end
