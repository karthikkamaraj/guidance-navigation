% Given data
x1 = 0; 
y1 = 0;

x2 = 4;
y2 = 2;

x3 = 1;
y3 = 4; 

% Bearing angles
theta1 = deg2rad(-140);  
theta2 = deg2rad(90);
theta3 = deg2rad(-30);

% Compute tan values
T1 = tan(pi/2 - theta1);
T2 = tan(pi/2 - theta2);
T3 = tan(pi/2 - theta3);

% Compute position fixes
A1 = [T1*x1 - y1; T2*x2 - y2];
B1 = [T1 -1; T2 -1];
result1 = A1.*inv(B1);
x01 = result1(1);
y01 = result1(2);

A2 = [T2*x2 - y2; T3*x3 - y3];
B2 = [T2 -1; T3 -1];
result2 = A2.*inv(B2);
x02 = result2(1);
y02 = result2(2);

A3 = [T3*x3 - y3; T1*x1 - y1];
B3 = [T3 -1; T1 -1];
result3 = A3.*inv(B3);
x03 = result3(1);
y03 = result3(2);

% Compute mean position 
x0 = mean([x01 x02 x03]);
y0 = mean([y01 y02 y03]);

% Plot results
figure;
plot(x1,y1,'gx');
hold on
plot(x2,y2,'go');
plot(x3,y3,'go');
plot(x01,y01,'bo');
plot(x02,y02,'bo');
plot(x03,y03,'bo');   
plot(x0,y0,'ro');
grid on;
xlabel('X-axis');
ylabel('Y-axis');
title('Lighthouse Locations, Position Fixes & Three-Cornered Hat Center');
legend('Lighthouse 1','Lighthouse 2','Lighthouse 3', 'Obtained Fix 1','Obtained Fix 2','Obtained Fix 3','Three-Cornered Hat Center', 'Location','best');

xlim([-2 6])
ylim([-2 8])