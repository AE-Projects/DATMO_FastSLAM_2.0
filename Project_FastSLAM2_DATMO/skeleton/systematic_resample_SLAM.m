% function S = systematic_resample(S_bar)
% This function performs systematic re-sampling
% Inputs:   
%           Y_aux:       1XM arr of class Y.m
%           w_arr:       1XM array with weights of each particles
% Outputs:
%           Y_t:          1XM arr of class Y.m

function Y_t = systematic_resample_SLAM(Y_aux, w_arr)
    cdf = cumsum(w_arr);
    [one,M] = size(Y_aux);
    r_0 = rand / M;
    for m = 1:M
        i = find(cdf >= r_0,1,'first');
        Y_t(1,m) = Y_aux(1,i);
        r_0 = r_0 + 1/M;
        Y_t(1,m).weight = 1/M;
        %Y_t(1,m).N_feat
    end

end