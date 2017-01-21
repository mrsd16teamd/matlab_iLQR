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

    pos = [pos_x; pos_y];
    goal = [0; 1];

    Q = [1 0; 0 1];
    R = eye(size(goal,1))*1e-3;

    cost = (pos-goal)'*Q*(pos-goal); %;+ u'*R*u;
end

% Pendulum test: try to get to upright
if PENDULUM
    pos = x;
    goal = [pi; 0];

    Q = eye(size(pos,1));
    R = eye(size(u,1))*1e-3;

    cost = (pos-goal)'*Q*(pos-goal) + u'*R*u;
end