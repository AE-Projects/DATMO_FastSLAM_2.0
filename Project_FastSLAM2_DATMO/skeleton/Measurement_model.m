
% This function calculates the measurement model.
% Inputs:
%           q:              1X1
%           delta:          2X1
%           mu_theta:       1X1
% Outputs:
%           z_k:           2X1
function z_k = Measurement_model(delta,q,mu_theta)
    a = sqrt(q);
    b = atan2(delta(2),delta(1))-mu_theta;
    b = mod(b+pi, 2*pi) - pi;
    z_k = [a;b];
end