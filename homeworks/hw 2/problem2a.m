% Given data
x1 = 0;
y1 = 0;

x2 = 4;
y2 = 2;

theta1 = deg2rad(-165);
theta2 = deg2rad(150);

theta = deg2rad(45); % subtended angle
xs = 0; ys = 1; % north star direction

a = norm([x2-x1; y2-y1]); % baseline distance

fun = @(p) positionError(p,x1,y1,x2,y2,theta,xs,ys);

pinit = [-2:2:10;-2:2:10]; % grid of initial points
opts = optimoptions('fminunc','Algorithm','quasi-newton');

cost = inf;
for i = 1:size(pinit,2)
    for j = 1:size(pinit,1)
        
        % Convert scalar pinit to vector
        p0 = [pinit(j,i); pinit(j,i)];
        
        [p,fval] = fminunc(fun,p0,opts);
        
        if fval < cost
            cost = fval;
            x0 = p(1);
            y0 = p(2);
        end
    end
end

epsilon = 0.01; % threshold
if norm([x0-x1;y0-y1]) < epsilon || norm([x0-x2;y0-y2]) < epsilon
    disp('Position too close to lighthouse, using middle init point');  
    pinit = [4;2];
    p0 = pinit; % convert to vector
    [p,fval] = fminunc(fun,p0,opts); 
    x0 = p(1);
    y0 = p(2);
end

figure;
plot(x1,y1,'go');
hold on; 
grid on;
plot(x2,y2,'gx');
plot(x0,y0,'ro');
xlabel('X-axis');
ylabel('Y-axis');
title('Lighthouse Positions & Obtained Location');
legend({'Lighthouse 1','Lighthouse 2','Obtained Location'});

xlim([-1 5])
ylim([-1 3])

function err = positionError(p,x1,y1,x2,y2,theta,xs,ys)
x = p(1); y = p(2);  
r1 = [x-x1;y-y1];
r2 = [x-x2;y-y2];
r3 = [xs;ys];

a = norm([x2-x1;y2-y1]); % baseline distance

e1 = norm(r1)^2 + norm(r2)^2 - 2*norm(r1)*norm(r2)*cosd(theta) - a^2; 
e2 = r2'*r3 - norm(r2)*cosd(150); % bearing constraint  

err = e1^2 + e2^2;
end