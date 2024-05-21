clear;

%% Problem setup

p = [0.7212; 2.4080];

p1 = [0; 0];
p2 = [5; 5];
p3 = [2.5; 0];

R = 0.1*eye(3);
Dn = 0.1*eye(3);

y = [norm(p - p1, 2); norm(p - p2, 2); norm(p - p3, 2)];

t_final = 10;
t_step = 0.1;
t = 0:t_step:t_final;


%% Estimation

x00 = [4; 2];
P00 = 0.1*eye(2);

x_est = {};
P_list = {};

x0 = x00;
P0 = P00;
x_est{1} = x0;
P_list{1} = P0;

for i = 1:length(t)-1
    if rem(t(i), 1) == 0
        ym = y + Dn*normrnd(0, 1, [3, 1]);
    end
    C = JacobianOfG(x0, p1, p2, p3);
    [x1, P1] = KalmanFilter(x0, P0, C, R, ym, p1, p2, p3, p);
    x_est{i+1} = x1;
    P_list{i+1} = P1;
    x0 = x1;
    P0 = P1;
end

%% Post processing

xest = zeros(1, length(t));
yest = zeros(1, length(t));
P_frob = zeros(1, length(t));

for i = 1:length(t)
    pe = x_est{i};
    xest(i) = pe(1);
    yest(i) = pe(2);
    P_frob(i) = norm(P_list{i}, 'fro');
end


%% Figures

figure(1)
clf;
hold on;
grid on;
scatter(p(1), p(2), 'r', 'filled')
scatter(p1(1), p1(2), 'g', 'filled')
scatter(p2(1), p2(2), 'y', 'filled')
scatter(p3(1), p3(2), 'c', 'filled')
scatter(x00(1), x00(2), 'b', 'filled')
scatter(xest, yest, 'b')
xlim([-1, 6])
ylim([-1, 6])
xlabel('X-Coordinates')
ylabel('Y-Coordinates')
title('$2D\;Trajectory\;of\;\hat{X_k}$', 'Interpreter','latex')
legend('Actual position', 'Beacon 1', 'Beacon 2', 'Beacon 3', 'Initial guess', 'Position estimate', 'interpreter', 'latex', 'location', 'northwest')

figure(2)
clf;
hold on;
grid on;
plot(t, P_frob);

xlabel('$k$','Interpreter','latex')
ylabel('$P_{k|k}$','Interpreter','latex')
title('$Frobenius\;Norm\;of\;P_{k|k}\;vs\;k$', 'Interpreter','latex')


%% Functions

function [x1, P1] = KalmanFilter(x0, P0, C, R, ym, p1, p2, p3, p)
    K = P0*C'*inv(C*P0*C' + R);
    x1 = x0 + K*(ym - [norm(x0 - p1, 2); norm(x0 - p2, 2); norm(x0 - p3, 2)]);
    P1 = P0 - K*C*P0;
end

function C = JacobianOfG(z, p1, p2, p3)
    c11 = (z(1) - p1(1))/norm(z - p1, 2);
    c21 = (z(1) - p2(1))/norm(z - p2, 2);
    c31 = (z(1) - p3(1))/norm(z - p3, 2);

    c12 = (z(2) - p1(2))/norm(z - p1, 2);
    c22 = (z(2) - p2(2))/norm(z - p2, 2);
    c32 = (z(2) - p3(2))/norm(z - p3, 2);

    C = [c11, c12; c21, c22; c31, c32];
end