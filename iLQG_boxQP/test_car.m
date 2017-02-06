function demo_car
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
T       = 500;              % horizon
x0      = [0;0;0;1;0;0];   % initial state
u0      = .01*randn(2,T);    % initial controls
global x_des;
x_des = [2;2;pi/2;0;0;0];
Op.lims  = [-1 5;
             -0.5  0.5];
Op.plot = -1;               % plot the derivatives as well

% prepare the visualization window and graphics callback
figure(9);
set(gcf,'name','car parking','Menu','none','NumberT','off')
set(gca,'xlim',[-4 4],'ylim',[-4 4],'DataAspectRatio',[1 1 1])
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
plot(tar(1,:),tar(2,:));
axis([-3, 3, -3, 3])
axis auto equal

% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','b','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
Op.plotFn = plotFn;

% === run the optimization!
[x,u]= iLQG(DYNCST, x0, u0, Op);

% animate the resulting trajectory
show_traj_cog = 1;
show_traj_r = 1;
show_wheels = 1;

figure(9)
P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
W = [-0.03  -0.03  0.03  0.03  -0.03; -0.015  0.015  0.015  -0.015  -0.015; 1 1 1 1 1];
CoG = [0;0;1]
r_axle = [-0.15;0;1]
h = animatedline(P(1,:),P(2,:));
axis([-3, 3, -3, 3])
axis auto equal

if show_traj_cog
    traj_cog = animatedline(CoG(1,:),CoG(2,:),'Color','g');
end

if show_traj_r
    traj_r = animatedline(r_axle(1,:),r_axle(2,:),'Color','r');
end

if show_wheels
    tfr = [1 0 0.135; 0 1 -0.08; 0 0 1]*W;
    fr = animatedline(tfr(1,:),tfr(2,:))
    tfl = [1 0 0.135; 0 1 0.08; 0 0 1]*W;
    fl = animatedline(tfl(1,:),tfl(2,:))
    trr = [1 0 -0.135; 0 1 -0.08; 0 0 1]*W;
    rr = animatedline(trr(1,:),trr(2,:))
    trl = [1 0 -0.135; 0 1 0.08; 0 0 1]*W;
    rl = animatedline(trl(1,:),trl(2,:))
end

for i=1:T
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
    drawnow  
end

function y = car_dynamics(x,u)
s = size(x);
dt = 0.025;
k1 = zeros(s);
k2 = zeros(s);
k3 = zeros(s);
k4 = zeros(s);

for i = 1:size(x,2)
    k1(:,i) = dynamics(x(:,i), u(:,i));
    k2(:,i) = dynamics(x(:,i) + 0.5 * dt * k1(:,i), u(:,i));
    k3(:,i) = dynamics(x(:,i) + 0.5 * dt * k2(:,i), u(:,i));
    k4(:,i) = dynamics(x(:,i) + dt * k3(:,i), u(:,i));
end

y = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);


function c = car_cost(x, u)
% cost function for car-parking problem
% sum of 3 terms:
% lu: quadratic cost on controls
% lf: final cost on distance from target parking configuration
% lx: running cost on distance from origin to encourage tight turns
global x_des;
final = isnan(u(1,:));
u(:,final)  = 0;

cu  = 1e-1*[.01 .1];         % control cost coefficients

cf  = [ 1  1  1 .3 .1 .1];    % final cost coefficients
pf  = [ 10  10 1  1 1 1]';    % smoothness scales for final cost

cx  = 1e-2*[1  1 0.5];          % running cost coefficients
px  = [1 1 1]';             % smoothness scales for running cost

% control cost
lu    = cu*u.^2;

% final cost
if any(final)
   dist = bsxfun(@minus,x(:,final),x_des);
   llf      = cf*sabs(dist,pf);
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

% total cost
c     = lu + lf + lx;

function y = sabs(x,p)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = pp( sqrt(pp(x.^2,p.^2)), -p);


function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = car_dyn_cst(x,u,full_DDP)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives

if nargout == 2
    f = car_dynamics(x,u);
    c = car_cost(x,u);
else
    % state and control indices
    ix = 1:6;
    iu = 7:8;
    
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


% ======== graphics functions ========
function h = car_plot(x,u)

body        = [0.9 2.1 0.3];           % body = [width length curvature]
bodycolor   = 0.5*[1 1 1];
headlights  = [0.25 0.1 .1 body(1)/2]; % headlights [width length curvature x]
lightcolor  = [1 1 0];
wheel       = [0.15 0.4 .06 1.1*body(1) -1.1 .9];  % wheels = [width length curvature x yb yf]
wheelcolor  = 'k';

h = [];

% make wheels
for front = 1:2
   for right = [-1 1]
      h(end+1) = rrect(wheel,wheelcolor)'; %#ok<AGROW>
      if front == 2
         twist(h(end),0,0,u(1))
      end
      twist(h(end),right*wheel(4),wheel(4+front))
   end
end

% make body
h(end+1) = rrect(body,bodycolor);

% make window (hard coded)
% h(end+1) = patch([-.8 .8 .7 -.7],.6+.3*[1 1 -1 -1],'w');

% headlights
h(end+1) = rrect(headlights(1:3),lightcolor);
twist(h(end),headlights(4),body(2)-headlights(2))
h(end+1) = rrect(headlights(1:3),lightcolor);
twist(h(end),-headlights(4),body(2)-headlights(2))

% put rear wheels at (0,0)
twist(h,0,-wheel(5))

% align to x-axis
twist(h,0,0,-pi/2)

% make origin (hard coded)
ol = 0.1;
ow = 0.01;
h(end+1) = patch(ol*[-1 1 1 -1],ow*[1 1 -1 -1],'k');
h(end+1) = patch(ow*[1 1 -1 -1],ol*[-1 1 1 -1],'k');

twist(h,x(1),x(2),x(3))

function twist(obj,x,y,theta)
% a planar twist: rotate object by theta, then translate by (x,y)
i = 1i;
if nargin == 3
   theta = 0;
end
for h = obj
   Z = get(h,'xdata') + i*get(h,'ydata');
   Z = Z * exp(i*theta);
   Z = Z + (x + i*y);
   set(h,'xdata',real(Z),'ydata',imag(Z));
end

function h = rrect(wlc, color)
% draw a rounded rectangle (using complex numbers and a kronecker sum :-)

N        = 25; % number of points per corner

width    = wlc(1);
length   = wlc(2);
curve    = wlc(3);

a        = linspace(0,2*pi,4*N);
circle   = curve*exp(1i*a);
width    = width-curve;
length   = length-curve;
rect1    = diag(width*[1 -1 -1 1] + 1i*length *[1 1 -1 -1]);
rectN    = sum(kron(rect1, ones(1,N)), 1) ;
rr       = circle + rectN;
rr       = [rr rr(1)]; % close the curve

h        = patch(real(rr),imag(rr),color);

% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);

function c = tt(a,b)
c = bsxfun(@times,a,b);