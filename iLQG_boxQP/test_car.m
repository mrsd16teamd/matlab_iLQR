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
x0      = [0;0;0;1;0;0;0;0;0;0];   % initial state
u0      = .1*randn(2,T);;    % initial controls
global x_des;
x_des = [1;2;pi/2;0;0;0;0;0;0;0];
Op.lims  = [-1 4;
             -0.8  0.8];
Op.plot = 0;               % plot the derivatives as well

% prepare the visualization window and graphics callback
obs = [0.5,0.5; 1,1.5];
map = robotics.BinaryOccupancyGrid(5,5,10);
setOccupancy(map,obs,1)
global costmap;
costmap = getmap(obs);

figure(9);
set(gcf,'name','drift car','Menu','none','NumberT','off','KeyPressFcn',@Kpress,'user',0)
set(gca,'xlim',[-4 4],'ylim',[-4 4],'DataAspectRatio',[1 1 1])
show(map);
grid on
box on

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

save('traj.mat','x','u','x0','x_des','dt');
end

function stop = traj_plot(x,line_handle)
set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
stop = get(gcf,'user');
drawnow;
end

function Kpress(src,evnt)
if strcmp(evnt.Key,'space')
    set(src,'user',1)
end
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

function c = car_cost(x, u)
% cost function for car-parking problem
% sum of 3 terms:
% lu: quadratic cost on controls
% lf: final cost on distance from target parking configuration
% lx: running cost on distance from origin to encourage tight turns
global x_des

final = isnan(u(1,:));
u(:,final)  = 0;

cu  = 1e-2*[.1 .1];         % control cost coefficients
cdu = 1e-1*[.01 1];         % change in control cost coefficients

cf  = [ 10 10 1 .1 .1 .1];    % final cost coefficients
pf  = [ .01 .01 .1 .1 .1 .1]';    % smoothness scales for final cost

cx  = 1e-2*[1  1 0.8];          % running cost coefficients
px  = [.01 .01 .1]';             % smoothness scales for running cost

% control cost
lu    = cu*u.^2;
ldu   = cdu*x(9:10,:).^2;

% final cost
if any(final)
   dist = bsxfun(@minus,x(1:6,final),x_des(1:6));
   llf      = cf*(sabs(dist,pf)+sabs(dist,pf).^2);
%    llf      = cf*sabs(x(:,final),pf);
   lf       = double(final);
   lf(final)= llf;
else
   lf    = 0;
end

% running cost
dist = bsxfun(@minus,x(1:3,:),x_des(1:3));
lx = cx*sabs(dist,px);
% lx = cx*sabs(x(1:2,:),px);

% drift prize
ld = -0.001*(sabs(x(5,:),1)-0.2);

% obstacle cost
lobs = getmapCost(x(1,:),x(2,:));

c     = lu + lf + lx + ldu + ld + lobs;
end

function F = getmap(obs)
Sigma = [.05 0; 0 .05];
x1 = -2:.1:5; x2 = -2:.1:5;
[X1,X2] = meshgrid(x1,x2);

F = 0.2*mvnpdf([X1(:) X2(:)],obs(1,:),Sigma);
for i = 2:size(obs,1)
    F = F+0.2*mvnpdf([X1(:) X2(:)],obs(i,:),Sigma);
end

F = reshape(F,length(x2),length(x1));
surf(x1,x2,F);
caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
axis auto equal
xlabel('x1'); ylabel('x2'); zlabel('Probability Density');
set(gcf,'name','cost map')
end

function lobs = getmapCost(x,y)
global costmap
x = (round(x,1)+2)*10+1;
y = (round(y,1)+2)*10+1;
lobs = zeros(1,length(x));
for i = 1:length(x)
    lobs(i) = costmap(y(i),x(i));
end
end

function y = sabs(x,p)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = pp( sqrt(pp(x.^2,p.^2)), -p);
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

% ======== graphics functions ========
function car_plot(x,u)
global T;
% animate the resulting trajectory
show_traj_cog = 1;
show_traj_r = 1;
show_wheels = 1;

figure(9)
P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
W = [-0.03  -0.03  0.03  0.03  -0.03; -0.015  0.015  0.015  -0.015  -0.015; 1 1 1 1 1];
CoG = [0;0;1];
r_axle = [-0.15;0;1];
h = animatedline(P(1,:),P(2,:));

if show_traj_cog
    traj_cog = animatedline(CoG(1,:),CoG(2,:),'Color','g');
end

if show_traj_r
    traj_r = animatedline(r_axle(1,:),r_axle(2,:),'Color','r');
end

if show_wheels
    tfr = [1 0 0.135; 0 1 -0.08; 0 0 1]*W;
    fr = animatedline(tfr(1,:),tfr(2,:));
    tfl = [1 0 0.135; 0 1 0.08; 0 0 1]*W;
    fl = animatedline(tfl(1,:),tfl(2,:));
    trr = [1 0 -0.135; 0 1 -0.08; 0 0 1]*W;
    rr = animatedline(trr(1,:),trr(2,:));
    trl = [1 0 -0.135; 0 1 0.08; 0 0 1]*W;
    rl = animatedline(trl(1,:),trl(2,:));
end

u(:,size(x,2))=[0;0];
tic
for i=1:size(x,2)
    % ----------------------------------------
    % ----------Update Visualization----------
    % ----------------------------------------
    pos_x = x(1,i);
    pos_y = x(2,i);
    pos_phi = wrapToPi(x(3,i));
    A = [cos(pos_phi) -sin(pos_phi) pos_x; sin(pos_phi) cos(pos_phi) pos_y; 0 0 1];
    pos = A*P;
    CoG_n = A*CoG;
    rear_n = A*r_axle; 
    
    steer = u(2,i);
    if show_wheels
        clearpoints(fr);
        clearpoints(fl);
        clearpoints(rr);
        clearpoints(rl);
        cfr = A*[cos(steer) -sin(steer) 0.135; sin(steer) cos(steer) -0.08; 0 0 1]*W;
        addpoints(fr, cfr(1,:), cfr(2,:))
        cfl = A*[cos(steer) -sin(steer) 0.135; sin(steer) cos(steer) 0.08; 0 0 1]*W;
        addpoints(fl, cfl(1,:), cfl(2,:))
        crr = A*trr;
        addpoints(rr, crr(1,:), crr(2,:))
        crl = A*trl;
        addpoints(rl, crl(1,:), crl(2,:))
    end
    
    clearpoints(h);
    addpoints(h,pos(1,:),pos(2,:));
    if show_traj_cog
        addpoints(traj_cog,CoG_n(1,:),CoG_n(2,:));
    end
    
    if show_traj_r
        addpoints(traj_r,rear_n(1,:),rear_n(2,:));
    end
    
    waitfor(toc == 0.025);
    drawnow
    tic
end
end
% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end

function c = tt(a,b)
c = bsxfun(@times,a,b);
end