% Given data  
x1 = 0; 
y1 = 0; 
z1 = 0;

x2 = 5;
y2 = 0; 
z2 = 0;

r = [0; 1; 0]; % star direction
theta = deg2rad(90); % subtended angle 
psi2 = deg2rad(135); % bearing angle

% True position
x0 = 2.5;
y0 = 2.5;
z0 = 0;

% Grid of initial points   
pinit = [-5:2:5; -5:2:5; -5:2:5];

% Preallocate solutions
numSolutions = numel(pinit(1,:))*numel(pinit(2,:))*numel(pinit(3,:));
solutions = zeros(3,numSolutions);

% Initialize cost
cost = inf; 

% Initialize counter 
idx = 1;

for i = 1:numel(pinit(1,:))
  for j = 1:numel(pinit(2,:))
    for k = 1:numel(pinit(3,:))
      
      p0 = [pinit(1,i); pinit(2,j); pinit(3,k)];
      
      [p,fval] = fminunc(@(p) positionError(p,x1,y1,z1,x2,y2,z2,theta,psi2,r), p0);
      
      if fval < cost
        solutions(:,idx) = p';
        idx = idx + 1;
      end
      
    end
  end 
end

% Trim solutions
solutions = solutions(:,1:idx-1); 

% Plot 2D projections
figure;
plot(x0, y0, 'ro', 'MarkerSize', 10);
hold on;
grid on;
plot(solutions(1, :), solutions(2, :), 'b.', 'MarkerSize', 20);
xlabel('X-axis');
ylabel('Y-axis');
title('2D Projection of Position Fixes');
legend('Actual Position','Position Fixes')

xlim([2 5.5])
ylim([-3 3])

% Plot 3D points on a sphere
figure;
[X, Y, Z] = sphere(20);
surf(2.5*X + 2.5, 2.5*Y, 2.5*Z, 'FaceAlpha', 0.25);
hold on;
scatter3(solutions(1, :), solutions(2, :), solutions(3, :), 'b');
scatter3(x0, y0, z0, 'ro', 'SizeData', 10);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D Position Fixes');

% Cost function
function err = positionError(p, x1, y1, z1, x2, y2, z2, theta, psi2, r)
    x = p(1);
    y = p(2);
    z = p(3);

    r1 = [x - x1; y - y1; z - z1];
    r2 = [x - x2; y - y2; z - z2];

    a = norm(r2);
    b = norm(r1);
    c = norm([x2 - x1; y2 - y1; z2 - z1]);

    e1 = a^2 + b^2 - 2 * a * b * cos(theta) - c^2;

    e2 = r2' * r - a * cos(psi2);

    err = e1^2 + e2^2;
end

