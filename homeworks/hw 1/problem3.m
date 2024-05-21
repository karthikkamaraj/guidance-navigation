% Given
L1 = [0; 0];
L2 = [5; 5];
L3 = [2.5; 0];
R1 = 2.5;
R2 = 5;
R3 = 3;
x_true = [0.7212; 2.4080];

% a) Solve for position fix using fminunc
x0_1 = [10; 0]; % Initial guess 1
x0_2 = [0; 10]; % Initial guess 2

options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12);
[x1, ~] = fminunc(@(x) costFun(x, L1, L2, L3, R1, R2, R3), x0_1, options);
[x2, ~] = fminunc(@(x) costFun(x, L1, L2, L3, R1, R2, R3), x0_2, options);

% Plot circles and position fixes
figure;
theta = linspace(0, 2*pi);
circle_L1 = L1 + R1 * [cos(theta); sin(theta)];
circle_L2 = L2 + R2 * [cos(theta); sin(theta)];
circle_L3 = L3 + R3 * [cos(theta); sin(theta)];
plot(circle_L1(1,:), circle_L1(2,:), 'r--');
hold on;
plot(circle_L2(1,:), circle_L2(2,:), 'b--');
plot(circle_L3(1,:), circle_L3(2,:), 'g--');
scatter(x_true(1), x_true(2), 'k*', 'LineWidth', 1.5);
scatter(x1(1), x1(2), 'ro', 'filled');
scatter(x2(1), x2(2), 'bo', 'filled');
xlabel('X');
ylabel('Y');
title('Position Fixes using fminunc');
legend('Circle around L_1', 'Circle around L_2', 'Circle around L_3', 'True Position', 'Position Fix (Initial Guess 1)', 'Position Fix (Initial Guess 2)','Location','Best');
grid on;

% b) Solve for position fix in inconsistent case
R2 = 6; % Modify R2 to create an inconsistent case

[x3, ~] = fminunc(@(x) costFun(x, L1, L2, L3, R1, R2, R3), x0_1, options);
[x4, ~] = fminunc(@(x) costFun(x, L1, L2, L3, R1, R2, R3), x0_2, options);

% Plot circles and position fixes
figure;
theta = linspace(0, 2*pi);
circle_L1 = L1 + R1 * [cos(theta); sin(theta)];
circle_L2 = L2 + R2 * [cos(theta); sin(theta)];
circle_L3 = L3 + R3 * [cos(theta); sin(theta)];
plot(circle_L1(1,:), circle_L1(2,:), 'r--');
hold on;
plot(circle_L2(1,:), circle_L2(2,:), 'b--');
plot(circle_L3(1,:), circle_L3(2,:), 'g--');
scatter(x_true(1), x_true(2), 'k*', 'LineWidth', 1.5);
scatter(x3(1), x3(2), 'ro', 'filled');
scatter(x4(1), x4(2), 'bo', 'filled');
xlabel('X');
ylabel('Y');
title('Position Fixes in Inconsistent Case using fminunc');
legend('Circle around L_1', 'Circle around L_2', 'Circle around L_3', 'True Position', 'Position Fix (Initial Guess 1)', 'Position Fix (Initial Guess 2)','Location','Best');
grid on;

% c) Determine position fix for each point in the grid
p_range = -20:2.5:20;
[X,Y] = meshgrid(p_range);
initial_points = [X(:), Y(:)];

position_fixes = zeros(size(initial_points));
for i = 1:size(initial_points, 1)
    [x_opt, ~] = fminunc(@(x) costFun(x, L1, L2, L3, R1, R2, R3), initial_points(i,:), options);
    if norm(x_opt - x1) < norm(x_opt - x2)
        position_fixes(i) = 1; % Position fix converges to x1
    else
        position_fixes(i) = 2; % Position fix converges to x2
    end
end

% Plot scatter plot with color-coded position fixes
figure;
scatter(initial_points(position_fixes==1, 1), initial_points(position_fixes==1, 2), 'r', 'filled');
hold on;
scatter(initial_points(position_fixes==2, 1), initial_points(position_fixes==2, 2), 'b', 'filled');
xlabel('X');
ylabel('Y');
title('Position Fixes for Initial Points');
legend('Position Fix 1', 'Position Fix 2');
grid on;

% Cost function
function f = costFun(x, L1, L2, L3, R1, R2, R3)
    f = (norm(x - L1) - R1)^2 + (norm(x - L2) - R2)^2 + (norm(x - L3) - R3)^2;
end