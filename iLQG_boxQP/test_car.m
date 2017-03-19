function [x,u, u0] = test_car
% A demo of iLQG/DDP for car drifting
clc; clear;
close all;

% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;

% set up the optimization problem
DYNCST  = @(x,u,i) car_dyn_cst(x,u,full_DDP);
global T;
T       = 50;              % horizon
global dt;
dt      = 0.05;
global x0;  %[x,y,theta,vx,vy,w]
x0      = [0;0;0;  3;0;0; 0;0;0;0];   % initial state
global x_des;
x_des = [3;0;0; 0;0;0; 0;0;0;0];

global u0; % initial controls
% TODO change this according to x0 and x_des?
u0      = zeros(2,T); % Just setting up shape here
u0(1,:) = 0.25*randn(1,T) + 3; % commanded speed
u0(2,:) = 0.1*randn(1,T);
% u0(2,:) = 0.3*randn(1,T); % steering

Op.lims  = [0 4;   
            -0.76  0.68];
Op.maxIter = 30;

global obs;
obs = [1; 0];   

% Initialize plot with start state, goal state, obstacles
init_plot(x0,x_des,obs);

% Prepare trajectory visualization callback
line_handle = line([0 0],[0 0],'color','b','linewidth',1.5);
plotFn = @(x) traj_plot(x,line_handle);
Op.plotFn = plotFn;

% === Run the optimization!
[x,u]= iLQG(DYNCST, x0, u0, Op);
car_plot(x,u);

file_name = ['traj',datestr(now,'_mm-dd-yy_HH_MM')];
save(['saved_trajectories/',file_name,'.mat'],'x','u','x0','x_des','dt','T');

end %test_car

% ----------------------------------------
% -----------Dynamics and cost------------
% ----------------------------------------

function y = car_dynamics(X,u)
s = size(X,2);

x = X(1:6,:);
pu = X(7:8,:);

global dt;
new_x = zeros(6,s);

for i = 1:size(x,2)
    new_x(:,i) = dynamics_finite(x(:,i), u(:,i), dt);
end
du = u - pu;
y = [new_x; u; du];
end %car_dynamics

function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = car_dyn_cst(x,u,full_DDP)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives

if nargout == 2
    f = car_dynamics(x,u);
    c = car_cost(x,u);
else
    % state and control indices
    ix = 1:10;
    iu = 11:12;
    
    % dynamics first derivatives
    % J - Jacobian, derivative of states wrt states and control inputs
    % n x (n+m) x T , where n=dim(x), m=dim(u), T=horizon
    xu_dyn  = @(xu) car_dynamics(xu(ix,:),xu(iu,:));
    J       = finite_difference(xu_dyn, [x; u]);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    % dynamics second derivatives
    % TODO Fix this for our dimensions
    % JJ - Jacobian of gradient. 
    if full_DDP
        xu_Jcst = @(xu) finite_difference(xu_dyn, xu);
        JJ      = finite_difference(xu_Jcst, [x; u]);
        JJ      = reshape(JJ, [4 6 size(J)]);
        JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4])); %symmetrize
        fxx     = JJ(:,ix,ix,:);
        fxu     = JJ(:,ix,iu,:);
        fuu     = JJ(:,iu,iu,:);    
    else
        [fxx,fxu,fuu] = deal([]);
    end    
    
    % cost first derivatives
    xu_cost = @(xu) car_cost(xu(ix,:),xu(iu,:));
    J       = squeeze(finite_difference(xu_cost, [x; u]));
    cx      = J(ix,:);
    cu      = J(iu,:);
    
    % cost second derivatives
    xu_Jcst = @(xu) squeeze(finite_difference(xu_cost, xu));
    JJ      = finite_difference(xu_Jcst, [x; u]);
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
    cxx     = JJ(ix,ix,:);
    cxu     = JJ(ix,iu,:);
    cuu     = JJ(iu,iu,:);
    
    [f,c] = deal([]);
end
end %car_dyn_cost

% ----------------------------------------
% -----------Helper functions-------------
% ----------------------------------------

function init_plot(x0, x_des,obs)
% prepare the visualization window and graphics callback
figure(9)
set(gcf,'name','drift car','KeyPressFcn',@Kpress,'user',0)
set(figure(9),'closer','') 
% set(gca,'xlim',[-4 4],'ylim',[-4 4],'DataAspectRatio',[1 1 1])
grid on
box on
hold all

% Make boxes to represent start and end car poses
P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
start_x = x0(1);
start_y = x0(2);
start_phi = wrapToPi(x0(3));
tar_x = x_des(1);
tar_y = x_des(2);
tar_phi = wrapToPi(x_des(3));
start_A = [cos(start_phi) -sin(start_phi) start_x; sin(start_phi) cos(start_phi) start_y; 0 0 1];
tar_A = [cos(tar_phi) -sin(tar_phi) tar_x; sin(tar_phi) cos(tar_phi) tar_y; 0 0 1];
start = start_A*P;
tar = tar_A*P;

% Plot start and end
axis auto equal
plot(start(1,:),start(2,:),'color','b','linewidth',2);
plot(tar(1,:),tar(2,:),'color','r','linewidth',2);

% Plot obstacle and range at which obstacle cost is imposed
if ~isempty(obs)
    plot(obs(1),obs(2),'.','Color','r','MarkerSize',50)
    plot(obs(1),obs(2),'o','MarkerSize',75)
end
end

function stop = traj_plot(x,line_handle)
set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
title('Optimizing Trajectory');
xlabel('space:stop optimization, esc:close window, c:clear figure');
stop = get(figure(9),'user');
drawnow;
end

function Kpress(src,evnt)
if strcmp(evnt.Key,'space')
    set(src,'user',1)
end
if strcmp(evnt.Key,'escape')
    delete(gcf)
end
if strcmp(evnt.Key,'c')
    clf
end
end

function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 2^-17;
end

[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = pp(x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);
end

% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end

function c = tt(a,b)
c = bsxfun(@times,a,b);
end


