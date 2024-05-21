% Given data 
x1 = 0; 
y1 = 0; 
z1 = 0;

x2 = 5;
y2 = 0; 
z2 = 0;

r = [0; 1; 0]; % 1st star direction
v = [0; 0; 1]; % 2nd star direction 

% True angles
theta = deg2rad(90);  
psi2_r = deg2rad(135);
psi2_v = deg2rad(90);   

% True position
x0 = 2.5;
y0 = 2.5;
z0 = 0;

% Grid of initial points
pinit = [-1 2 5; -1 2 5; -1 2 5]; 

% Preallocate solutions
numTrials = 200;
solutions = zeros(3,numTrials);

for t = 1:numTrials

    % Add noise to measurements
    theta_n = theta + 2*randn*pi/180; 
    psi2_r_n = psi2_r + 2*randn*pi/180;
    psi2_v_n = psi2_v + 2*randn*pi/180;

    % Get noisy position fix
    [x,y,z] = getPositionFix(x1,y1,z1,x2,y2,z2,theta_n,psi2_r_n,psi2_v_n,r,v,pinit);
    
    % Save solution
    solutions(:,t) = [x; y; z];
    
end

% Plot results
figure;
plot(x1,y1,'go','LineWidth',2);
hold on;
grid on;
plot(x2,y2,'go','LineWidth',2);  
plot(x0,y0,'ro','LineWidth',2);
plot(solutions(1,:),solutions(2,:),'b.','MarkerSize',12);
xlabel('X-axis');
ylabel('Y-axis');
title('Lighthouse Locations, Obtained & Actual Locations')
legend('Lighthouse 1','Lighthouse 2','True Position','Noisy Solutions');

xlim([-1 6])
ylim([-0.5 3])

% Error histogram
errors = vecnorm(solutions - [x0; y0; z0], 2, 2);
figure; 
histogram(errors,20);
grid on;
xlabel('Position Error (m)');
ylabel('Count');
title('Position Error (m) vs Count')

xlim([0 40])
ylim([0 2.5])

meanError = mean(errors);
stdError = std(errors);

fprintf('Mean error: %f m\nStd dev: %f m\n',meanError,stdError);

% Get noisy position fix
function [x,y,z] = getPositionFix(x1,y1,z1,x2,y2,z2,theta,psi2_r,psi2_v,r,v,pinit)

    cost = inf;
    
    for i = 1:numel(pinit(1,:))
        for j = 1:numel(pinit(2,:))
            for k = 1:numel(pinit(3,:))

                p0 = [pinit(1,i) ; pinit(2,j) ; pinit(3,k)];
                [p,fval] = fminunc(@(p) positionError(p,x1,y1,z1,x2,y2,z2,theta,psi2_r,psi2_v,r,v), p0);

                if fval < cost
                    x = p(1);
                    y = p(2);
                    z = p(3);
                    cost = fval;
                end

            end
        end
    end

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
