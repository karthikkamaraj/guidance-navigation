% Given data
x1 = 0; 
y1 = 0; 
z1 = 0;

x2 = 5; 
y2 = 0;
z2 = 0;

r = [0; 1; 0]; % 1st star direction 
v = [0; 0; 1]; % 2nd star direction

theta = deg2rad(90); % subtended angle
psi2_r = deg2rad(135); % 1st bearing
psi2_v = deg2rad(90); % 2nd bearing  

% Update pinit to 3D array
pinit = [-5:2:5; -5:2:5; -5:2:5]; 

% Get position fix
cost = inf;

for i = 1:numel(pinit(1,:))
  for j = 1:numel(pinit(2,:))
    for k = 1:numel(pinit(3,:))
        
      p0 = [pinit(1,i) ; pinit(2,j) ; pinit(3,k)];
      [p,fval] = fminunc(@(p) positionError(p,x1,y1,z1,x2,y2,z2,theta,psi2_r,psi2_v,r,v), p0);  
      
      if fval < cost
        x0 = p(1);
        y0 = p(2);
        z0 = p(3);
        cost = fval;
      end
      
    end
  end
end

% Plot 2D projections
figure;
plot(x1,y1,'go','MarkerSize',12);  
hold on;
grid on;
plot(x2,y2,'go','MarkerSize',12);

% Plot position fix 
plot(x0,y0,'ro','MarkerSize',12);

xlabel('X-axis');
ylabel('Y-axis');

title('Lighthouse Locations and Obtained Location');
legend('Lighthouse 1','Lighthouse 2','Position Fix','Location','best');

xlim([-1 6])
ylim([-1e-7 2e-7])

% Actual location
x_true = 2.5;
y_true = 2.5;

% Check if matches
if abs(x0 - x_true) < 1e-3 && abs(y0 - y_true) < 1e-3
    disp('Position fix matches actual location');
else
    disp('Position fix does NOT match actual location'); 
end

% Cost function
function err = positionError(p,x1,y1,z1,x2,y2,z2,theta,psi2_r,psi2_v,r,v)

  % Vector definitions
  r1 = [p(1)-x1; p(2)-y1; p(3)-z1]; 
  r2 = [p(1)-x2; p(2)-y2; p(3)-z2];
  
  % Constraint equations
  e1 = norm(r1)^2 + norm(r2)^2 - 2*norm(r1)*norm(r2)*cosd(theta) - norm([x2-x1; y2-y1; z2-z1])^2;
  e2 = r2'*r - norm(r2)*cosd(psi2_r); 
  e3 = r2'*v - norm(r2)*cosd(psi2_v);

  err = e1^2 + e2^2 + e3^2;
  
end