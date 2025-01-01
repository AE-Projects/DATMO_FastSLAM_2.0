% This function should perform the update process(sequential update).
% You need to make sure that the output sigma_bar is symmetric.
% The last line makes sure that ouput sigma_bar is always symmetric.
% Inputs:
%           mu_bar(t)       3X1
%           sigma_bar(t)    3X3
%           H_bar(t)        2X3
%           S_bar(t)        2X2
%           nu_bar(t)       2X1
% Outputs:
%           mu_bar(t)       3X1
%           sigma_bar(t)    3X3
function [mu_bar, sigma_bar] = update_(mu_bar, sigma_bar, H_bar, S_bar, nu_bar)

        % YOUR IMPLEMENTATION %
        K_i = sigma_bar*(H_bar'/S_bar);
        mu_bar = mu_bar + K_i*nu_bar;
        I = eye(3);
        sigma_bar = (I - K_i*H_bar)*sigma_bar;

end
