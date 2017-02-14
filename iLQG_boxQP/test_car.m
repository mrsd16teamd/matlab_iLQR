function test_car
% A demo of iLQG/DDP with car-parking dynamics
clc;
close all

fprintf(['\nA demonstration of the iLQG algorithm '...
'with car parking dynamics.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])

% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;

% set up the optimization problem
DYNCST  = @(x,u,i) car_dyn_cst(x,u,full_DDP);
global T;
T       = 40;              % horizon
global dt;
dt      = 0.05;
global x0;
x0      = [0;0;0;1;0;0;0;0;0;0];   % initial state
u0      = .1*randn(2,T);    % initial controls
global x_des;
x_des = [2.5;1.5;pi/2;0;0;0;0;0;0;0];
Op.lims  = [-1 4;
             -0.8  0.8];
Op.plot = 0;               % plot the derivatives as well

obs = [1,0.25];


% prepare the visualization window and graphics callback
figure(9);
set(gcf,'name','drift car','Menu','none','NumberT','off','KeyPressFcn',@Kpress,'user',0)
set(gca,'xlim',[-4 4],'ylim',[-4 4],'DataAspectRatio',[1 1 1])
grid on
box on
global costmap;
costmap = getMap(obs);

% plot target configuration with light colors
figure(9);
P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
tar_x = x_des(1);
tar_y = x_des(2);
tar_phi = wrapToPi(x_des(3));
A = [cos(tar_phi) -sin(tar_phi) tar_x; sin(tar_phi) cos(tar_phi) tar_y; 0 0 1];
tar = A*P;
axis auto equal
axis([-2, 3, -2, 3])
line(P(1,:),P(2,:),'color','b','linewidth',2);
line(tar(1,:),tar(2,:),'color','b','linewidth',2);

% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','b','linewidth',1.5);
plotFn = @(x) traj_plot(x,line_handle);
Op.plotFn = plotFn;

% === run the optimization!
[x,u]= iLQG(DYNCST, x0, u0, Op);
car_plot(x,u);

file_name = ['traj',datestr(now,'_mm-dd-yy_HH:MM')];
save(['saved_trajectories/',file_name,'.mat'],'x','u','x0','x_des','dt','T');
end

function stop = traj_plot(x,line_handle)
set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
stop = get(figure(9),'user');
drawnow;
end

function Kpress(src,evnt)
if strcmp(evnt.Key,'space')
    set(src,'user',1)
end
end

function F = getMap(obs)
global x0;
global x_des;

if isempty(obs)
    F = [];
    return
end

Sigma = [.08 0; 0 .08];
x1 = x0(1)-2:.1:x_des(1)+2; x2 = x0(2)-2:.1:x_des(2)+2;
[X1,X2] = meshgrid(x1,x2);

F = 0.2*mvnpdf([X1(:) X2(:)],obs(1,:),Sigma);
for i = 2:size(obs,1)
    F = F+0.2*mvnpdf([X1(:) X2(:)],obs(i,:),Sigma);
end

F = reshape(F,length(x2),length(x1));
figure(9)
surf(x1,x2,F-5);
colormap(flipud(gray));
view(2);

figure(1)
surf(x1,x2,F);

end

function y = car_dynamics(X,u)
s = size(X,2);

x = X(1:6,:);
pu = X(7:8,:);

global dt;
new_x = zeros(6,s);

for i = 1:size(x,2)
    new_x(:,i) = dynamics_finite(x(:,i), u(:,i), dt);
%     sol = ode23(@(t,y) dynamics(y,u(:,i)),[0 dt], x(:,1));
%     new_x(:,i) = sol.y(:,end);
end
du = u - pu;
y = [new_x; u; du];
end

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
    xu_dyn  = @(xu) car_dynamics(xu(ix,:),xu(iu,:));
    J       = finite_difference(xu_dyn, [x; u]);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    % dynamics second derivatives
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