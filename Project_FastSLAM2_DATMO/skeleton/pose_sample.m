% This function performs the prediction step.
% Inputs:
%           xt_1:          3X1
%           u_t:            2X1
%           delta_t:        1X1
%           Rt:             3X3
% Outputs:   
%           xt_j            3X1
function [xt_j] = pose_sample(xt_1,u_t, delta_t,Rt)
    % Don't change below variables, if u want to change 
    % then also change at other places
    E_T = 2048;
    B = 0.35;
    R_L = 0.1;
    R_R = 0.1;

    xt_j = xt_1 + calculate_odometry(u_t(1),u_t(2), E_T, B, R_L, R_R, delta_t, xt_1);
    xt_j = xt_j + randn(3,1) .* sqrt(diag(Rt)); % Diffusion, assuming an uncorrelated sigma_R
    xt_j(3) = mod(xt_j(3)+pi,2*pi) - pi; 
end