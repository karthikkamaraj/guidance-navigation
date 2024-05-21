% Given data
x1 = 0; 
y1 = 0;

x2 = 4;
y2 = 2; 

x3 = 1; 
y3 = 4;

theta1 = deg2rad(-140);
theta2 = deg2rad(90); 
theta3 = deg2rad(-30);

xs = 0; ys = 1; % north star direction

% Lighthouses 1 & 2
theta12 = deg2rad(45); % subtended angle
[x01,y01] = getPositionFix(x1,y1,x2,y2,theta1,theta2,theta12,xs,ys);

% Lighthouses 1 & 3 
theta13 = deg2rad(110);
[x02,y02] = getPositionFix(x1,y1,x3,y3,theta1,theta3,theta13,xs,ys);

% Lighthouses 2 & 3
theta23 = deg2rad(120);
[x03,y03] = getPositionFix(x2,y2,x3,y3,theta2,theta3,theta23,xs,ys);

% Plot results
figure;
plot(x1,y1,'g*');
hold on;
grid on;
plot(x2,y2,'g*');
plot(x3,y3,'g*');
plot(x01,y01,'bo');
plot(x02,y02,'bx');
plot(x03,y03,'bo'); 

% Three-cornered hat center
xc = mean([x01 x02 x03]);
yc = mean([y01 y02 y03]);
plot(xc,yc,'ro');  

xlabel('X-axis');
ylabel('Y-axis');

title('3 Lighthouse Position Fix');
legend('Lighthouse 1','Lighthouse 2','Lighthouse 3','Position Fix 1','Position Fix 2','Position Fix 3','Three-Cornered Hat Center','Location','best');

xlim([-1 5])
ylim([-2 6])

% Get position fix using fminunc optimization
function [x0,y0] = getPositionFix(x1,y1,x2,y2,~,theta2,theta,xs,ys)

    a = norm([x2-x1; y2-y1]); % baseline distance

    fun = @(p) positionError(p,x1,y1,x2,y2,theta,xs,ys);

    pinit = [-2:2:10;-2:2:10]; 

    opts = optimoptions('fminunc','Algorithm','quasi-newton');

    cost = inf;

    for i = 1:size(pinit,2)
        for j = 1:size(pinit,1)           
            p0 = [pinit(j,i); pinit(j,i)];
            [p,fval] = fminunc(fun,p0,opts);
            if fval < cost
               cost = fval;
               x0 = p(1);
               y0 = p(2); 
            end
        end
    end

    function err = positionError(p,x1,y1,x2,y2,theta,xs,ys)
        x = p(1); y = p(2);
        r1 = [x-x1;y-y1];
        r2 = [x-x2;y-y2]; 
        r3 = [xs;ys];    
        e1 = norm(r1)^2 + norm(r2)^2 - 2*norm(r1)*norm(r2)*cosd(theta) - a^2;
        e2 = r2'*r3 - norm(r2)*cosd(theta2); % bearing constraint
        err = e1^2 + e2^2;
    end

end
