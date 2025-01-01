
% Inputs:
%           mu_t_1_m:             3X1: Assuming only one moving object    
%           sigma_t_1_m:            3X3
%           z_t_m:                2X1 : moving measurement
%           mu_t_rob:           3X1
%           sigma_t_rob:        3X3
%           v0:                 1X1
%           w0:                 1X1
% Outputs:
%           mu_t_m:                3X1 
%           sigma_t_m:             3X3
function [mu_t_m, sigma_t_m] = datmo_func(mu_t_1_m, sigma_t_1_m, mu_t_rob, sigma_t_rob, z_t_m, P_t, S_t, delta_t,v0,w0)
    
    % v0 = 0.3;%This may come from MHT
    %Prediction
    %For straight line
    %u_t_m = [v0*delta_t*cos(mu_t_1_m(3,1)); v0*delta_t*sin(mu_t_1_m(3,1)); 0]; 
    u_t_m = [v0*delta_t*cos(mu_t_1_m(3,1)); v0*delta_t*sin(mu_t_1_m(3,1)); w0*delta_t];
    mu_bar_t_m = mu_t_1_m + u_t_m;
    sigma_bar_t_m = sigma_t_1_m + P_t;

    %Measurement Prediction
    delta_m = [mu_bar_t_m(1,1) - mu_t_rob(1,1); mu_bar_t_m(2,1) - mu_t_rob(2,1)];
    q_m = delta_m'*delta_m;
    z_hat_t_m = Measurement_model(delta_m,q_m,mu_t_rob(3,1));
    
    %Find Jacobians
    a = sqrt(q_m);
    Hy =  (1/q_m)*[a*delta_m(1) a*delta_m(2) 0;
        -delta_m(2) delta_m(1) 0]; 
    Hx = (1/q_m)*[-a*delta_m(1) -a*delta_m(2) 0;
        delta_m(2) -delta_m(1) -1]; 
    
    %Update
    if ~isempty(z_t_m)
        L_t = Hx*sigma_t_rob*Hx' + S_t;
        sigma_t_m = inv(Hy'*inv(L_t)*Hy + inv(sigma_bar_t_m));
        mu_t_m = sigma_t_m*Hy'*inv(L_t)*(z_t_m - z_hat_t_m) + mu_bar_t_m;
    else
        sigma_t_m = sigma_bar_t_m;
        mu_t_m = mu_bar_t_m;
end