function dx  = kinsim(t,x)
% Returns xdot for kinematics simulation engagement
% Written by Syed Aseem Ul Islam (aseemisl@umich.edu) 

dx = zeros(8,1);
% x(1)    :: hP
% x(2)    :: dP
% x(3)    :: gammaP
% x(4)    :: hE
% x(5)    :: dE
% x(6)    :: gammaE
% x(7)    :: R
% x(8)    :: beta

VP = 900; %m/s
VE = 450; %m/s
g  = 9.81; %m/s^2

% Evader normal acceleration
if t < 9
    nzE = -8*g;
elseif t < 10
    nzE = 0;
elseif t < 22
    nzE = -6*g;
else
    nzE = 0;


dx(1) = VP*sin(x(3));
dx(2) = VP*cos(x(3));
% dx(3) needs to be implemented after dx(7) and dx(8) since it relies on Rdot betadot
dx(4) = VE*sin(x(6));
dx(5) = VE*cos(x(6));
dx(6) = -nzE/VE;
dx(7) = VE*cos(x(8) - x(6)) - VP*cos(x(8) - x(3));
dx(8) = (VP*sin(x(8) - x(3)) - VE*sin(x(8) - x(6)))/x(7);

% Pursuer normal acceleration (Proportional Guidance)
nzP = -3*abs(dx(7))*dx(8);

% Saturate pursuer normal acceleration at 40g
if abs(nzP)>40*g
    nzP = sign(nzP)*40*g;
end
dx(3) = -nzP/VP;

end

