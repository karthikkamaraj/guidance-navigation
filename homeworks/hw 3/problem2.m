clear;

load("rcwA.mat");
traj = rcwA_Ts_0_01;

y_meas = rcwA_Ts_0_01;
for i = 1:length(y_meas(1,:))
   y(:,i) = y_meas(:,i) + diag([0.005,0.005,0.005])*randn(3,1);
end

phi = pi/6;
cp = cos(phi);
sp = sin(phi);
g = 9.80665;

k = 0:2000;
kl = length(k);
T1 = 0.01;

r0 = [1; 0; 0];
v0 = [0;cp;sp];
oBA0 = [1 0 0 0 cp sp 0 -sp cp]';
x0 = [oBA0;r0;v0];

%% (a) Purely inertial navigation
X = zeros(length(x0),kl);
X(:,1) = x0;

for i = 1:kl-1
    [rv, ori] = kfil1(X(:,i),T1,i);
    X(:,i+1) = [reshape(ori',[9,1]);rv];
end

%% (b) Inertial navigation with position measurements sampled every T_mocap seconds

X1 = zeros(length(x0),kl);
X1(:,1) = x0;
T_mocap1 = 1;
P1 = zeros(6,6,kl);
P1(:,:,1) = 10*eye(6);

X2 = zeros(length(x0),kl);
X2(:,1) = x0;
T_mocap2 = 0.1;
P2 = zeros(6,6,kl);
P2(:,:,1) = 10*eye(6);

for i = 1:kl-1
    [rv1, ori1, P] = kfil2(X1(:,i),T1,i,T_mocap1,y(:,i+1),P1(:,:,i));
    X1(:,i+1) = [reshape(ori1',[9,1]);rv1];
    P1(:,:,i+1) = P;
end

for i = 1:kl-1
    [rv2, ori2, P] = kfil2(X2(:,i),T1,i,T_mocap2,y(:,i+1),P2(:,:,i));
    X2(:,i+1) = [reshape(ori2',[9,1]);rv2];
    P2(:,:,i+1) = P;
end

%% Figure 1 - (a) Inertial navigation

figure(1)
clf;

subplot(3,1,1)
grid on;
hold on;
plot(T1*k, traj(1,:),'b','Linewidth',2);
plot(T1*k, X(10,:),'r','Linewidth',2);
ylabel('$\vec{r}_{c/w}(t).\hat{i}_A$ (m)','interpreter','latex');
xlabel('Time (s)','interpreter','latex');
legend('Actual Trajectory','Estimated Trajectory','interpreter','latex');
title('Estimated Trajectory and Actual Trajectory of Quadcopter versus Time','interpreter','latex');

subplot(3,1,2)
grid on;
hold on;
plot(T1*k, traj(2,:),'b','Linewidth',2);
plot(T1*k, X(11,:),'r','Linewidth',2);
ylabel('$\vec{r}_{c/w}(t).\hat{j}_A$ (m)','interpreter','latex');
xlabel('Time (s)','interpreter','latex');
legend('Actual Trajectory','Estimated Trajectory','interpreter','latex');

subplot(3,1,3)
grid on;
hold on;
plot(T1*k, traj(3,:),'b','Linewidth',2);
plot(T1*k, X(12,:),'r','Linewidth',2);
ylabel('$\vec{r}_{c/w}(t).\hat{k}_A$ (m)','interpreter','latex');
xlabel('Time (s)','interpreter','latex');
legend('Actual Trajectory','Estimated Trajectory','interpreter','latex');

%We observe that the estimates from the Kalman Filter
%diverge from actual trajectory, which is as expected.

%% Figure 2 - Spatial plot of actual trajectory and estimated trajectory for purely inertial navigation

figure(2)
clf;
hold on;
grid on;
plot3(traj(1,:),traj(2,:),traj(3,:),'b','LineWidth',2);
plot3(X(10,:),X(11,:),X(12,:),'r','LineWidth',2);
xlabel('$X-Coordinates$','interpreter','latex');
ylabel('$Y-Coordinates$','interpreter','latex');
zlabel('$Z-Coordinates$','interpreter','latex');
title('3D Trajectory of the Quadcopter','interpreter','latex');
legend('Actual Trajectory','Estimated Trajectory','interpreter','latex');
hold off;


%% Figure 3 - (a) Inertial navigation with position measurements sampled every T_mocap seconds

figure(3)
clf;

subplot(3,1,1)
grid on;
hold on;
plot(T1*k, traj(1,:),'b','Linewidth',1);
plot(T1*k, X1(10,:),'g','Linewidth',1);
plot(T1*k, X2(10,:),'r','Linewidth',1);
ylabel('$\vec{r}_{c/w}(t).\hat{i}_A$ (m)','interpreter','latex');
xlabel('Time (s)','interpreter','latex');
legend('Actual Trajectory','$T_{MOCAP} = 1s$','$T_{MOCAP} = 0.1s$','interpreter','latex');
title('Estimated Trajectories Using Position Measurements and Actual Trajectory of Quadcopter versus Time','interpreter','latex');

subplot(3,1,2)
grid on;
hold on;
plot(T1*k, traj(2,:),'b','Linewidth',1);
plot(T1*k, X1(11,:),'g','Linewidth',1);
plot(T1*k, X2(11,:),'r','Linewidth',1);
ylabel('$\vec{r}_{c/w}(t).\hat{j}_A$ (m)','interpreter','latex');
xlabel('Time (s)','interpreter','latex');
legend('Actual Trajectory','$T_{MOCAP} = 1s$','$T_{MOCAP} = 0.1s$','interpreter','latex');

subplot(3,1,3)
grid on;
hold on;
plot(T1*k, traj(3,:),'b','Linewidth',1);
plot(T1*k, X1(12,:),'g','Linewidth',1);
plot(T1*k, X2(12,:),'r','Linewidth',1);
ylabel('$\vec{r}_{c/w}(t).\hat{k}_A$ (m)','interpreter','latex');
xlabel('Time (s)','interpreter','latex');
legend('Actual Trajectory','$T_{MOCAP} = 1s$','$T_{MOCAP} = 0.1s$','interpreter','latex');

%We observe that a smaller sampling period results in a better position estimate

%% Figure 4 - Spatial plot of actual and estimated trajectories

figure(4)
clf;
hold on;
grid on;
plot3(traj(1,:),traj(2,:),traj(3,:),'b','LineWidth',1);
plot3(X1(10,:),X1(11,:),X1(12,:),'g','LineWidth',1);
plot3(X2(10,:),X2(11,:),X2(12,:),'r','LineWidth',1);
xlabel('$X-Coordinates$','interpreter','latex');
ylabel('$Y-Coordinates$','interpreter','latex');
zlabel('$Z-Coordinates$','interpreter','latex');
title('3D Trajectory of the Quadcopter','interpreter','latex');
legend('Actual Trajectory','$T_{MOCAP} = 1s$','$T_{MOCAP} = 0.1s$','interpreter','latex');
hold off;

%% Functions

function [rv, oba] = kfil1(x0,T,k)

A = [eye(3),T*eye(3);zeros(3,3),eye(3)];
B = [(T^2/2)*eye(3);T*eye(3)];
D1 = diag([0.1,0.1,0.1]);
D2 = diag([0.1,0.1,0.1]);
t = k*T;

r0 = [x0(10); x0(11); x0(12)];
v0 = [x0(13); x0(14); x0(15)];
rv0 = [r0;v0];
oba0 = [x0(1) x0(2) x0(3); x0(4) x0(5) x0(6); x0(7) x0(8) x0(9)];

w = [0;0;1] + D1*randn(3,1);
wx = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

eps = expm(-T*wx);

oba = eps*oba0;

phi = pi/6;
cp = cos(phi);
sp = sin(phi);
g = 9.80665;

ak = [-1-g*sp*sin(t); -g*sp*cos(t); -g*cp] + D2*randn(3,1);
ga = [0;0;-g];

uk = oba0'*ak - ga;

rv = A*rv0 + B*uk;

end

function [rv11, oba, P11] = kfil2(x0,T,k,T_mocap,y,P00)

A = [eye(3),T*eye(3);zeros(3,3),eye(3)];
B = [(T^2/2)*eye(3);T*eye(3)];
Q = 10*eye(6);
R = 0.001*eye(3);

D1 = diag([0.1,0.1,0.1]);
D2 = diag([0.1,0.1,0.1]);
t = k*T;

P10 = A*P00*A' + Q;

Ck = [eye(3) zeros(3,3)];
if mod(k*T,T_mocap) == 0
    C = Ck;
else
    C = zeros(size(Ck));
end

K = P10*C'*inv(C*P10*C' + R);

P11 = P10 - K*C*P10;

r0 = [x0(10); x0(11); x0(12)];
v0 = [x0(13); x0(14); x0(15)];
rv0 = [r0;v0];
oba0 = [x0(1) x0(2) x0(3); x0(4) x0(5) x0(6); x0(7) x0(8) x0(9)];

w = [0;0;1] + D1*randn(3,1);
wx = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

eps = expm(-T*wx);

oba = eps*oba0;

phi = pi/6;
cp = cos(phi);
sp = sin(phi);
g = 9.80665;

ak = [-1-g*sp*sin(t); -g*sp*cos(t); -g*cp] + D2*randn(3,1);
ga = [0;0;-g];

uk = oba0'*ak - ga;

rv10 = A*rv0 + B*uk;
rv11 = rv10 + K*(y - C*rv10);

end


