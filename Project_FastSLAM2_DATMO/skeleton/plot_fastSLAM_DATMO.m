function plot_fastSLAM_DATMO(map, true_pose,z_t_s, z_t_m, true_mov,  Y_t_1, fast_slam_pose, cov_mov, mu_mov, color)
[two, I] = size(z_t_s);
[one, M] = size(Y_t_1);
cla;
scatter(map(1,:), map(2,:))
hold on
grid on
mu_j_t = zeros(2,I);
[two, mov_sz] = size(z_t_m);
if mov_sz > 0
    mu_mov = measurement_model_inverse(z_t_m, true_pose); 
    plot([true_pose(1,1), mu_mov(1,1)],[true_pose(2,1), mu_mov(2,1)], 'r-','LineWidth',0.3)
end

for i = 1:I
    mu_j_t(:,i) = measurement_model_inverse(z_t_s(:,i), true_pose);    
end
plot(true_pose(1,1), true_pose(2,1), 'k+', 'MarkerSize', 5, 'LineWidth', 1.5);
plot(true_mov(1,1), true_mov(2,1), 'ro', 'MarkerSize', 5, 'LineWidth', 1.5);
plot([true_pose(1,1)*ones(1,I);[mu_j_t(1,:)]],[true_pose(2,1)*ones(1,I);[mu_j_t(2,:)]],'g-','LineWidth',0.3);
%pause(0.5)

for m = 1:M
    
    plot(reshape(fast_slam_pose(1,m,:),1,[]),reshape(fast_slam_pose(2,m,:),1,[]),'r-','LineWidth',0.3);
    hold on
    scatter(Y_t_1(1,m).features_mu(1,:), Y_t_1(1,m).features_mu(2,:), 'bo','LineWidth',1.5);
    scatter(Y_t_1(1,m).pose(1,1), Y_t_1(1,m).pose(2,1), 'LineWidth',1.5, 'MarkerEdgeColor','r');
end

color_ = 'cyan';
cov_matrix = cov_mov(1:2, 1:2);
origin = mu_mov(1:2,1)';
[eigT, eigV] = eig(cov_matrix*cov_matrix'); 
eigenvalues = diag(eigV);
if any(eigenvalues <= 0)
    error('Covariance matrix must be positive definite.');
end
a = sqrt(eigenvalues(1)); % Semi-major axis
b = sqrt(eigenvalues(2)); % Semi-minor axis
theta = linspace(0, 2*pi, 100);
ellipse_points = eigT * [a 0; 0 b] * [cos(theta); sin(theta)];
ellipse_points(1, :) = ellipse_points(1, :) + origin(1);
ellipse_points(2, :) = ellipse_points(2, :) + origin(2);
plot(ellipse_points(1, :), ellipse_points(2, :), 'Color', color_, 'LineWidth', 1.5);
%hold on;
plot(origin(1), origin(2), 'k+', 'MarkerSize', 5, 'LineWidth', 1.5); 
axis equal; 
    %axis([-1 12 -1 12])
grid on;



end
