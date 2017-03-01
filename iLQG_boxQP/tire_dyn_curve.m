function [] = tire_dyn_curve(alpha, Ux)

% Parameters
m = 2.35;           % mass (kg)
g = 9.81;
L = 0.257;          % wheelbase (m)
b = 0.14328;        % CoG to rear axle
a = L-b;            % CoG to front axle
% G_front = m*g*b/L;   % calculated load or specify front rear load directly
G_rear = m*g*a/L;
Fz = G_rear;

C_x = 330;          % longitude stiffness
C_alpha = 300;      % laternal stiffness
mu = 5.2/G_rear;   
mu_slide = 4.3/G_rear; 

Ux_cmd = -5:0.05:5;

% alpha = 0.1;
% Ux = 1.0;

Fx = zeros(length(Ux_cmd),1);
Fy = zeros(length(Ux_cmd),1);

for i = 1:length(Ux_cmd)
   [Fxi,Fyi] = tire_dyn(Ux, Ux_cmd(i), mu, mu_slide, Fz, C_x, C_alpha, alpha); 
    Fx(i) = Fxi;
    Fy(i) = Fyi;
end

plot(Ux_cmd, Fx)
hold on
plot(Ux_cmd, Fy)
legend('Fx', 'Fy')
xlabel('wheel speed [m/s]')
ylabel('force [N?]')
title(['Tire dynamics at ', num2str(Ux), 'm/s and ', num2str(alpha), ' sliding angle'])