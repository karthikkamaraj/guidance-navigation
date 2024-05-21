clear;

%% Problem setup

load('AE584_Final_P1_meas_Ts_0_01.mat');
load('AE584_Final_P1_pos_Ts_0_01.mat');

Q = 1e-4*eye(6);
R = 1e-4*eye(3);
m = 0.4;

t_final = t(end);
Ts = t(2) - t(1);
t_samp = 0.01;

y_sensor = [Range; Azimuth; Elevation];

%% Estimation

x00 = [2.5; 0; 1; 0; 0.5; -0.1];
P00 = eye(6);

x_est = {};
P_list = {};

x0 = x00;
P0 = P00;
x_est{1} = x0;
P_list{1} = P0;

for i = 1:length(t)-1

    C = JacobianOfG(x0(1), x0(2), x0(3));
    ym = y_sensor(:, i) + 0.01*eye(3)*randn(3, 1);
    y_est = OutputNonlinear(x0);

    [x1, P1] = KalmanFilter(x0, P0, C, Q, R, m, Ts, ym, y_est, t(i), t_samp);
   
    x_est{i+1} = x1;
    P_list{i+1} = P1;
    x0 = x1;
    P0 = P1;
end

%% Post processing

xest = zeros(1, length(t));
yest = zeros(1, length(t));
zest = zeros(1, length(t));
y_est = zeros(3, length(t));

P_frob = zeros(1, length(t));

for i = 1:length(t)
    pe = x_est{i};
    xest(i) = pe(1);
    yest(i) = pe(2);
    zest(i) = pe(3);
    P_frob(i) = norm(P_list{i}, 'fro');
    y_est(:, i) = OutputNonlinear([pe(1), pe(2), pe(3)]);
end

%% Figures

figure(1)
clf;
hold on;
plot3(Xref, Yref, Zref)
plot3(xest, yest, zest)
xlabel('$X_{ref}$', 'Interpreter','latex')
ylabel('$Y_{ref}$', 'Interpreter','latex')
zlabel('$Z_{ref}$', 'Interpreter','latex')
legend('Actual trajectory', 'Estimated trajectory', 'interpreter', 'latex', 'Location', 'northwest')
title('Reference and Estimated Trajectory Obtained for $T_{meas} = 0.1 \hat{s}$', 'Interpreter', 'latex')

figure(2)
clf;

subplot(3, 1, 1)
title('3 Components of the Reference and Estimated Trajectory Obtained for','$T_{meas} = 0.1 \hat{s}$ vs Normalized Time', 'Interpreter', 'latex')
hold on;
grid on;
plot(t, Range);
plot(t, y_est(1, :));
xlabel('$Time (\hat{s})$', 'Interpreter', 'latex')
ylabel('$Range (\hat{m})$', 'Interpreter', 'latex')

subplot(3, 1, 2)
hold on;
grid on;
plot(t, Azimuth);
plot(t, y_est(2, :));
xlabel('$Time (\hat{s})$', 'Interpreter', 'latex')
ylabel('$Azimuth (rad)$', 'Interpreter', 'latex')

subplot(3, 1, 3)
hold on;
grid on;
plot(t, Elevation);
plot(t, y_est(3, :));
xlabel('$Time (\hat{s})$', 'Interpreter', 'latex')
ylabel('$Elevation (rad)$', 'Interpreter', 'latex')


%% Functions

function [x1, P1] = KalmanFilter(x0, P0, C, Q, R, m, Ts, ym, y_est, t, t_samp)
    Ak = eFT(m, x0(1), x0(2), x0(3), Ts);

    x10 = Ak*x0 + [zeros(3, 3); 0.01*eye(3)]*randn(3, 1);

    P10 = Ak*P0*Ak' + Q;

    K = P10*C'*inv(C*P10*C' + R);

    if rem(t, t_samp) == 0
        x1 = x10 + K*(ym - y_est);
    else
        x1 = x10;
    end

    P1 = P10 - K*C*P10;
end

function C = JacobianOfG(x, y, z)
    r = norm([x; y; z], 2);

    c11 = x/r;
    c12 = y/r;
    c13 = z/r;
    c21 = -y/(x^2 + y^2);
    c22 = x/(x^2 + y^2);
    c23 = 0;
    c31 = z*x/(r^2*norm([x, y], 2));
    c32 = z*y/(r^2*norm([x, y], 2));
    c33 = norm([x, y], 2)/r^2;

    C = [c11, c12, c13; c21, c22, c23; c31, c32, c33];
    C = [C, zeros(3, 3)];
end

function Ak = eFT(m, x, y, z, Ts)
    r = norm([x; y; z], 2);

    f11 = 3*m*x^2/r^5 - m/r^3;
    f22 = 3*m*y^2/r^5 - m/r^3;
    f33 = 3*m*z^2/r^5 - m/r^3;
    f12 = 3*m*x*y/r^5;
    f23 = 3*m*y*z/r^5;
    f31 = 3*m*z*x/r^5;

    F21 = [f11, f12, f31; f12, f22, f23; f31, f23, f33];
    F = [zeros(3,3), eye(3); F21, zeros(3,3)];
    Ak = expm(F*Ts);
end

function y_est = OutputNonlinear(X)
    x = X(1);
    y = X(2);
    z = X(3);

    g1 = norm([x; y; z], 2);
    g2 = -atan2(x, y);
    g3 = atan2(z, norm([x; y], 2));

    y_est = [g1; AzUnwrap(g2, 0); g3];
end
