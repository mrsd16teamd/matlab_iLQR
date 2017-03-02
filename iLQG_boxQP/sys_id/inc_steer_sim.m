% l_or_r: 1 = left, -1 = right
% input ramp at line 96
function [sim_x,sim_u] = inc_steer_sim(thr,l_or_r,T,param)
% --------Initialize Joystick--------
x = [0;0;0;0;0;0];
sim_x = x;
sim_u = [0;0];
dt = 0.05;

if l_or_r == 1
    rad_s = (0.76/T);
else
    rad_s = (0.69/T);
end

for t = 0:dt:T      
    % constant throttle and steering ramp
    throttle = thr;
    steer = l_or_r*t*rad_s; 
    
    % ------Calculate Car Dynamics------
    u = [throttle; steer];
    if ~exist('param','var')
        new_x = dynamics_finite(x, u, dt);
    else
        new_x = dynamics_finite(x, u, dt, param);
    end
    x = new_x;
    sim_x = [sim_x x];
    sim_u = [sim_u u];
end
end