function [ x_new ] = dynamics_finite( x, u, dt )
%DYNAMICS_FINITE Summary of this function goes here
%   Detailed explanation goes here
k1 = dynamics(x, u);
k2 = dynamics(x + 0.5 * dt * k1, u);
k3 = dynamics(x + 0.5 * dt * k2, u);
k4 = dynamics(x + dt * k3, u);
x_new = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

end

