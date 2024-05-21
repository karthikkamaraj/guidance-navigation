% c) Determine position fix for each point in the grid

% Given
L1 = [0; 0];
L2 = [5; 5];
R1 = 2.5;
R2 = 5;

p_range = -5:2.5:10;
[X,Y] = meshgrid(p_range);
initial_points = [X(:), Y(:)];

position_fixes = zeros(size(initial_points));
for i = 1:size(initial_points, 1)
    [x_opt, ~] = fminunc(@(x) costFun(x, L1, L2, R1, R2), initial_points(i,:), options);
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

xlim([-10 15])
ylim([-10 15])

% Cost function
function f = costFun(x, L1, L2, R1, R2)
    f = (norm(x - L1) - R1)^2 + (norm(x - L2) - R2)^2;
end


