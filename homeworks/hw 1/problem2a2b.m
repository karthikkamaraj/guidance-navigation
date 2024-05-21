% Given
L1 = [0; 0];
L2 = [5; 5];
R1 = 2.5;
R2 = 5;

global h;
h.x = [];
h.fval = [];

% Solve for position fixes using fminunc
x0_1 = [10; 0]; % Initial guess 1
x0_2 = [0; 10]; % Initial guess 2

options = optimoptions('fminunc', 'OptimalityTolerance', 1e-12,   'OutputFcn' , @outfun);
[x1, fv1] = fminunc(@(x) costFun(x, L1, L2, R1, R2), x0_1, options);
x_iter = h2x(h.x)';
err = iterError(x1, x_iter);
[x2, fv2] = fminunc(@(x) costFun(x, L1, L2, R1, R2), x0_2, options);

%% Figures

% Plot circles and position fixes (Problem 2a)
figure(1);
theta = linspace(0, 2*pi);
circle_L1 = L1 + R1 * [cos(theta); sin(theta)];
circle_L2 = L2 + R2 * [cos(theta); sin(theta)];
plot(circle_L1(1,:), circle_L1(2,:), 'r--');
hold on;
plot(circle_L2(1,:), circle_L2(2,:), 'b--');
scatter(x1(1), x1(2), 'ro', 'filled');
scatter(x2(1), x2(2), 'bo', 'filled');
xlabel('X');
ylabel('Y');
title('Position Fixes using fminunc');
legend('Circle around L_1', 'Circle around L_2', 'Position Fix (Initial Guess 1)', 'Position Fix (Initial Guess 2)', 'Location','best');
grid on;

% Plot distance error v iteration (Problem 2b)
figure(2);
clf;
semilogy(1:length(err), err)
hold on;
grid on;
xlabel('Iteration');
ylabel('Distance Error');
title('Distance Error vs. Iteration');

%%

% Cost function
function f = costFun(x, L1, L2, R1, R2)
    f = (norm(x - L1) - R1)^2 + (norm(x - L2) - R2)^2;
end

function stop = outfun( x , optimValues , state)
    global h
    stop = false;
    switch state
        case 'iter'
            h.fval = [h.fval ; optimValues.fval] ;
            h.x = [h.x ; x ] ;
        otherwise
            end
end

function s = h2x(a)
    s = reshape(a, [2, length(a)/2]);
end

function err = iterError(x, x_iter)
    err = zeros(1, length(x_iter'));
    for i = 1:length(err)
        err(i) = norm(x - x_iter(i, :)');
    end

end
