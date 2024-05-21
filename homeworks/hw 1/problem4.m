% Given
L1 = [0; 0];
L2 = [5; 5];
L3 = [2.5; 0];
R1 = 2.5;
R2 = 5;
R3 = 3;
x_true = [0.7212; 2.4080];

% a) Solve for position fix with noise in range measurements
a = 2; % Noise parameter
num_cases = 100; % Number of random cases
x_init = [10; 0]; % Initial guess

position_fixes = zeros(2, num_cases);
figure;
theta = linspace(0, 2*pi);
circle_L1_true = L1 + R1 * [cos(theta); sin(theta)];
circle_L2_true = L2 + R2 * [cos(theta); sin(theta)];
circle_L3_true = L3 + R3 * [cos(theta); sin(theta)];
plot(circle_L1_true(1,:), circle_L1_true(2,:), 'r--');
hold on;
plot(circle_L2_true(1,:), circle_L2_true(2,:), 'r--');
plot(circle_L3_true(1,:), circle_L3_true(2,:), 'r--');
scatter(x_true(1), x_true(2), 'g*', 'LineWidth', 1.5);

for i = 1:num_cases
    % Add noise to range measurements
    R1_noisy = R1 + a*(rand() - 0.5);
    R2_noisy = R2 + a*(rand() - 0.5);
    R3_noisy = R3 + a*(rand() - 0.5);

    % Solve for position fix using fminunc
    options.NoiseParameter_a = a;
    [x, ~] = fminunc(@(x) costFunWithNoise(x, L1, L2, L3, R1_noisy, R2_noisy, R3_noisy), x_init, options);
    position_fixes(:, i) = x;

    % Plot position fix
    scatter(x(1), x(2), 'bo', 'filled');
end

xlabel('X');
ylabel('Y');
title('Position Fixes with Noise in Range Measurements');
legend('True Circle 1', 'True Circle 2', 'True Circle 3', 'True Position', 'Position Fixes (with Noise)','Location','best');
grid on;

% b) Evaluate position fix degradation for different noise levels
a_range = linspace(0.05, 10);
max_distance = zeros(size(a_range));

for i = 1:length(a_range)
    max_dist = 0;
    
    for j = 1:num_cases
        % Add noise to range measurements
        R1_noisy = R1 + a_range(i)*(rand() - 0.5);
        R2_noisy = R2 + a_range(i)*(rand() - 0.5);
        R3_noisy = R3 + a_range(i)*(rand() - 0.5);

        % Solve for position fix using fminunc
        options.NoiseParameter_a = a_range(i);
        [x, ~] = fminunc(@(x) costFunWithNoise(x, L1, L2, L3, R1_noisy, R2_noisy, R3_noisy), x_init, options);

        % Calculate distance from true position
        dist = norm(x - x_true);
        
        if dist > max_dist
            max_dist = dist;
        end
    end
    
    max_distance(i) = max_dist;
end

figure;
semilogy(a_range, max_distance);
xlabel('Noise Parameter (a)');
ylabel('Maximum Distance from True Position');
title('Position Fix Degradation with Noise in Range Measurements');

% Cost function with noise
function f = costFunWithNoise(x, L1, L2, L3, R1_noisy, R2_noisy, R3_noisy)
    f = (norm(x - L1) - R1_noisy)^2 + (norm(x - L2) - R2_noisy)^2 + (norm(x - L3) - R3_noisy)^2;
end