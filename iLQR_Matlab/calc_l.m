function cost = calc_l( x, u )
%CALC_L: calculate the cost.

global PENDULUM;

% Put your definition of cost functions here.

% Current implementation finds partial derivatives of cost function with
% finite differencing, but in future implementations try to come up with a
% cost function that has simple analytical derivatives, for speed.

% Force the trajectory to stay close to the linearization point? 
%   by adjusting the cost function at each iteration
% Penalize change in control inputs? 

% Test #1: Can car go straight forward to position (1,0)? 
if ~PENDULUM
    pos_x = x(1);
    pos_y = x(2);
    vx    = x(3);
    vy    = x(4);

    pos = [pos_x; pos_y; vx; vy];
    goal = [1; 0; 0 ; 0];

    Q = [1 0 0 0; 0 1 0 0; 0 0 0 0; 0 0 0 0.5];
    R = eye(size(goal,1))*1e-3;

    % TODO Change cost function to keep trajectory near linearization point
    cost = (pos-goal)'*Q*(pos-goal); %;+ u'*R*u;
    
    % Penalize trajectories that move too far away from linearization point
    % cost = (1-alpha)*cost + alpha*((x-x_lin)^2 + (u-u_lin)^2);
    % alpha close to 1
    % Need to input linearization point as parameter; edit lots of places
end

% Pendulum test: try to get to upright
if PENDULUM
    pos = x;
    goal = [pi; 0];

    Q = eye(size(pos,1));
    R = eye(size(u,1))*1e-3;

    cost = (pos-goal)'*Q*(pos-goal) + u'*R*u;
end