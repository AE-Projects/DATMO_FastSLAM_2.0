% This function plots the 2D ellipse at different locations
% Inputs:
%           cov_matrix:        2N+3X2N+3
%           origin:            2N+3X2 (Ellipse center)
%           color:             string character


function plot_covariance_ellipse(cov_matrix_all, origin,color)
    % if ~isequal(size(cov_matrix_all), [2, 2])
    %     error('Covariance matrix must be a 2x2 matrix.');
    % end

    if nargin < 3
        color = 'b'; % Default color is blue
    end

    [n_ellipse, two] = size(origin);

    %cov_matrix = [2 1;1 2];
    %close all;
    %figure;
    hold on
    cla;
    for n = 1:n_ellipse
    
    cov_matrix = cov_matrix_all(2*n-1:2*n, 2*n-1:2*n);
    cov_matrix = 100*cov_matrix;
    [eigT, eigV] = eig(cov_matrix*cov_matrix'); 
    eigenvalues = diag(eigV);
    if any(eigenvalues <= 0)
        error('Covariance matrix must be positive definite.');
    end
    a = sqrt(eigenvalues(1)); % Semi-major axis
    b = sqrt(eigenvalues(2)); % Semi-minor axis
    theta = linspace(0, 2*pi, 100);
    ellipse_points = eigT * [a 0; 0 b] * [cos(theta); sin(theta)];
    ellipse_points(1, :) = ellipse_points(1, :) + origin(n,1);
    ellipse_points(2, :) = ellipse_points(2, :) + origin(n,2);
    if n==1
        color_ = 'g';
    else
        color_ = color;
    end
    plot(ellipse_points(1, :), ellipse_points(2, :), 'Color', color_, 'LineWidth', 1.5);
    %hold on;
    plot(origin(n,1), origin(n,2), 'k+', 'MarkerSize', 5, 'LineWidth', 1.5); 
    axis equal; 
    %axis([-1 12 -1 12])
    grid on;
    
    end
end
