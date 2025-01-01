
% This function calculates the features from inverse of measurement model.
% Inputs:
%           z_t:            2X1
%           x_t_k:          3X1
%       
% Outputs:
%           mu_j_t:         2X1
function mu_j_t = measurement_model_inverse(z_t, x_t_k)
mu_j_t = x_t_k(1:2) + [z_t(1)*cos(z_t(2) + x_t_k(3));z_t(1)*sin(z_t(2) + x_t_k(3))];
end