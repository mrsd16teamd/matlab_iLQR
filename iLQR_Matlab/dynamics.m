function [dx] = dynamics(x, u)
%DYNAMICS Summary of this function goes here
% The dimension of the state/control space
% global X_DIM %X_DIM = 6 (x,y,phi,dx,dy,r)
% global U_DIM %U_DIM = 2 (velocity,steering)

% ----------------------------------------
% --------------Model Params--------------
% ----------------------------------------
m = 2.622;          % mass (kg)
L = 0.255;          % wheelbase (m)
a = 0.1329;         % CoG to front axle
b = 0.1221;         % CoG to rear axle
mu = 0.37;          % friction coeffcient
C_alpha = 120000;   % laternal stiffness
C_x = 120000;       % longitude stiffness
Iz = 0.020899525;   % roatation inertia
g = 9.81;     
G_front = m*g*(a/L);% calculated load or specify front rear load directly
G_rear = m*g*(b/L);

% ----------------------------------------
% ------States/Inputs Interpretation------
% ----------------------------------------
pos_x = x(1);
pos_y = x(2);
pos_phi = x(3);

Ux = x(4);
Uy = x(5);
r = x(6);

Ux_cmd = u(1);
delta = u(2);

% ----------------------------------------
% --------------Tire Dyanmics-------------
% ----------------------------------------
% longitude wheel slip K
K = (Ux_cmd-Ux)/abs(Ux);
% lateral slip angle alpha
if Ux == 0 && Uy == 0   % vehicle is still no slip
    alpha_F = 0;
    alpha_R = 0;
elseif Ux == 0      % perfect side slip
    alpha_F = pi/2*sign(Uy)-delta;
    alpha_R = pi/2*sign(Uy);
elseif Ux < 0    % rare ken block situations
    alpha_F = (sign(Uy)*pi)-atan((Uy+a*r)/abs(Ux))-delta;
    alpha_R = (sign(Uy)*pi)-atan((Uy-b*r)/abs(Ux));
else                % normal situation
    alpha_F = atan((Uy+a*r)/abs(Ux))-delta;
    alpha_R = atan((Uy-b*r)/abs(Ux));
end

% safety that keep alpha in valid range
alpha_F = wrapToPi(alpha_F);
alpha_R = wrapToPi(alpha_R);

[Fxf,Fyf] = tire_dyn(0, mu, G_front, C_x, C_alpha, alpha_F);
[Fxr,Fyr] = tire_dyn(K, mu, G_rear, C_x, C_alpha, alpha_R);


% ----------------------------------------
% ------------Vehicle Dyanmics------------
% ----------------------------------------
% ddx
r_dot = (a*Fyf*cos(delta)-b*Fyr)/Iz;
Ux_dot = (Fxr-Fyf*sin(delta))/m+r*Uy;
Uy_dot = (Fyf*cos(delta)+Fyr)/m-r*Ux;

% translate dx to terrain frame
U = sqrt(Ux^2+Uy^2);
if Ux == 0 && Uy == 0
    beta = 0;
elseif Ux == 0
    beta = pi/2*sign(Uy);
elseif Ux < 0 && Uy == 0
    beta = pi;
elseif Ux < 0
    beta = sign(Uy)*pi-atan(Uy/abs(Ux));
else
    beta = atan(Uy/abs(Ux));
end
beta = wrapToPi(beta);

Ux_terrain = U*cos(beta+pos_phi);
Uy_terrain = U*sin(beta+pos_phi);
dx = [Ux_terrain,Uy_terrain,r,Ux_dot,Uy_dot,r_dot];
end
