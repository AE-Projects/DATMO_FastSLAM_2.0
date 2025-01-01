
% This function calculates the measurement model.
% Inputs:
%           q:              1X1
%           delta:          2X1
% Outputs:
%           h_k:           2X5
function h_k = Jacobian_H(delta,q)
a = sqrt(q);
dx = delta(1);
dy = delta(2);
h_k = (1/q)*[-a*dx -a*dy 0 a*dx a*dy;
    dy -dx -1 -dy dx];
end