% Given data
x1 = 0;
y1 = 0;

x2 = 4;
y2 = 2;
theta1 = deg2rad(-165);
theta2 = deg2rad(150);

% Calculate T1 and T2
T1 = tan(pi/2 - theta1);
T2 = tan(pi/2 - theta2);

% Calculate (x0, y0)
A = [T1*x1 - y1; T2*x2 - y2];
B = [T1 -1; T2 -1];
result = A.*inv(B);
x0 = result(1);
y0 = result(2);

% Plot lighthouses and obtained location
figure;
plot(x1, y1, 'go', x2, y2, 'go', x0, y0, 'ro');
grid on;
xlabel('X-axis');
ylabel('Y-axis');
title('Lighthouse Positions and Obtained Location');
legend('Lighthouse Location 1', 'Lighthouse Location 2','Obtained Location')

xlim([-1 5])
ylim([-1 4])


