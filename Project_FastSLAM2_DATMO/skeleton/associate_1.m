% This function calculates the odometry information.
% Inputs:
%           Yt_1:          class - {pose(3X1),N_feat(1X1),features_mu(2XN_feat,features_sigma(2X2XN_feat),features_i(1XN_feat)}
%           Qt:            2X2
%           Rt:            3X3
%           zt:            2X1
%           u_t:           2X1 (encoder data)
% Outputs:
%           pi_J(t):           1X(N_feat)
%           x_J(t):            3X(N_feat)
%           Q_J(t):            2X2X(N_feat)
%           Hm(t):             2X2X(N_feat)
%           Hx(t):             2X3X(N_feat)
%           z_J(t):            2X(N_feat)
function [pi_J,Q_J,Hm, z_J]= associate_1(Yt_1,x_t_k,Qt,zt)
% Don't change below variables, if u want to change 
% then also change at other places

pi_J = zeros(1,Yt_1.N_feat);
Q_J = zeros(2,2,Yt_1.N_feat);
Hm = zeros(2,2,Yt_1.N_feat);
z_J = zeros(2,Yt_1.N_feat);

for j = 1:Yt_1.N_feat
 
    del1 = Yt_1.features_mu(:,j) - x_t_k(1:2);
    q1 = del1'*del1;
    a = sqrt(q1);
    zbar_j = Measurement_model(del1,q1,x_t_k(3));
    z_J(:,j) = zbar_j;
    Hm(:,:,j) =  [a*del1(1) a*del1(2);
        -del1(2) del1(1)];
    Q_J(:,:,j) = Qt +Hm(:,:,j)*Yt_1.features_sigma(:,:,j)*Hm(:,:,j)';
    pi_J(1,j) = (det(2*pi*Q_J(:,:,j)))^(-0.5)*(exp(-0.5*(zt - zbar_j)'*inv(Q_J(:,:,j))*(zt - zbar_j)));
end
