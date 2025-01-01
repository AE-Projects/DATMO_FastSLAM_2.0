function plot_mov_dataset(map, true_pose,z_t_s, z_t_m, true_mov)
[two, I] = size(z_t_s);
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
plot(true_mov(1,1), true_mov(2,1), 'bo', 'MarkerSize', 5, 'LineWidth', 1.5);
plot([true_pose(1,1)*ones(1,I);[mu_j_t(1,:)]],[true_pose(2,1)*ones(1,I);[mu_j_t(2,:)]],'g-','LineWidth',0.3);
pause(0.5)
end
