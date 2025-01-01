% Inputs:
%           Y_t_1_k:        1 obj of class Y at t-1 from kth particle
%           N_t_k:          Number of features computed after association
%           c_hat:          Association index for ith measurement
%           z_t:            2X1 : only one measurement
%           u_t:            2X1  : enoder data: u_t(1) = right
%           H_m:            (2X2XN_t_1): Jacobain wrt map
%           H_x:            (2X3XN_t_1): Jacobain wrt pose
%           z_hat:          (2XN_t_1)
%           x_t_j:          (3XN_t_1) 
%           Q_j:            (2X2XN-t_1)
% Outputs:
%           Y_aux:          1 obj of class Y
%           w_k:            scalar?

function [w_k, Y_aux_k] = update_KF_features(Y_t_1_k, N_t_k, c_hat, z_t, u_t, H_m, H_x, z_hat, x_t_j, Q_j, p0, Q_t, R_t, delta_t)
    N_t_1_k = Y_t_1_k.N_feat;
    for j = 1:N_t_k
        if j==c_hat && j == 1+N_t_1_k
            x_t_1_k = Y_t_1_k.pose;
            %sample pose or predict with diffusion
            x_t_k = pose_sample(x_t_1_k, u_t, delta_t, R_t);
            %mu_j_t = Y_t_1_k.features(1,j);
            %need to write this function
            %output 3X1
            mu_j_t = measurement_model_inverse(z_t, x_t_k);
            Y_aux_k.features_mu(:,j) = mu_j_t;
            %del = Y_aux_k.features_mu(:,j) - mu_j_t(:,1);%are you sure, its not Y_t_1_k?
            del = Y_aux_k.features_mu(:,j) - x_t_k(1:2,1);
            q = del'*del;
            a = sqrt(q);
            H_m_j = [a*del(1,1) a*del(2,1); -del(2,1) del(1,1)];
            %disp(H_m_j)
            sigma_j_t = inv(H_m_j)'*Q_t*inv(H_m_j);%had to this otherwise
            %inv of 0 matrix cause warning
            %sigma_j_t = zeros(2,2);
            i_j_t = 1;
            w_k = p0;
            Y_aux_k.pose = x_t_k;
            Y_aux_k.features_sigma(:,:,j) = sigma_j_t;
            Y_aux_k.features_i = i_j_t;
            Y_aux_k.weight = w_k;
         elseif j==c_hat && j<=N_t_1_k
            x_t_k = x_t_j(:,j);
            K = Y_t_1_k.features_sigma(:,:,j)*H_m(:,:,j)'*inv(Q_j(:,:,j));
            mu_j_t = Y_t_1_k.features_mu(:,j) + K*(z_t - z_hat(:,j));
            sigma_j_t = (eye(2,2) - K*H_m(:,:,j))*Y_t_1_k.features_sigma(:,:,j);
            i_j_t = Y_t_1_k.features_i + 1;
            L = H_x(:,:,j)*R_t*H_x(:,:,j)'+ H_m(:,:,j)*Y_t_1_k.features_sigma(:,:,j)*H_m(:,:,j)' + Q_t;
            w_k = (det(2*pi*L))^(-0.5)*exp(-0.5*(z_t - z_hat(:,j))'*inv(L)*(z_t - z_hat(:,j)));
            Y_aux_k.pose = x_t_k;
            Y_aux_k.features_mu(:,j) = mu_j_t;
            Y_aux_k.features_sigma(:,:,j) = sigma_j_t;
            Y_aux_k.features_i = i_j_t;
            Y_aux_k.weight = w_k;
         else
            Y_aux_k.features_mu(:,j) = Y_t_1_k.features_mu(:,j);
            Y_aux_k.features_sigma(:,:,j) = Y_t_1_k.features_sigma(:,:,j);
            %x_t = x_t_j(:,j);%Not sure about this
            %if sqrt((x_t - Yt_1.features_mu(:,j)).^2) > 10
                %Y_aux.features_i = Y_t_1_k.features_i;
            %else
                %pass
            %end
        end     
    end
    Y_aux_k.N_feat = N_t_k;
end