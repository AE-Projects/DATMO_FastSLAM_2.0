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
function [pi_J,x_J,Q_J,Hm,Hx,z_J]= associate(Yt_1,Qt,Rt,zt,u_t,delta_t)
% Don't change below variables, if u want to change 
% then also change at other places
E_T = 2048;
B = 0.35;
R_L = 0.1;
R_R = 0.1;

pi_J = zeros(1,Yt_1.N_feat);
x_J = zeros(3,Yt_1.N_feat);
Q_J = zeros(2,2,Yt_1.N_feat);
Hm = zeros(2,2,Yt_1.N_feat);
Hx = zeros(2,3,Yt_1.N_feat);
z_J = zeros(2,Yt_1.N_feat);

for j = 1:Yt_1.N_feat
    xcap_j= Yt_1.pose + calculate_odometry(u_t(1),u_t(2), E_T, B, R_L, R_R, delta_t, Yt_1.pose);
    del1 = Yt_1.features_mu(:,j) - xcap_j(1:2);
    q1 = del1'*del1;
    a = sqrt(q1);
    zbar_j = Measurement_model(del1,q1,xcap_j(3));
    Hm(:,:,j) =  (1/q1)*[a*del1(1) a*del1(2);
        -del1(2) del1(1)];
    Hx(:,:,j) = (1/q1)*[-a*del1(1) -a*del1(2) 0;
        del1(2) -del1(1) -1];
    Q_J(:,:,j) = Qt +Hm(:,:,j)*Yt_1.features_sigma(:,:,j)*Hm(:,:,j)';
    sigma_x_j = inv(Hx(:,:,j)'*(inv(Q_J(:,:,j)))*Hx(:,:,j) + inv(Rt));
    meu_x_j = sigma_x_j*Hx(:,:,j)'*(inv(Q_J(:,:,j)))*(zt - zbar_j) + xcap_j;
    %disp(sigma_x_j)
    x_J(:,j) = mvnrnd(meu_x_j,sigma_x_j,1)';

    del2 = Yt_1.features_mu(:,j) - x_J(1:2,j);
    q2 = del2'*del2;
    z_J(:,j) = Measurement_model(del2,q2,x_J(3,j));
    
    pi_J(1,j) = (det(2*pi*Q_J(:,:,j)))^(-0.5)*(exp(-0.5*(zt - z_J(:,j))'*inv(Q_J(:,:,j))*(zt - z_J(:,j))));
end
