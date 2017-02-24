function replanning_test

global T;
T       = 40;              % horizon
global dt;
dt      = 0.05;
global x0;
x0      = [0;0;0;0;0;0;0;0;0;0];   % initial state
global u0;
u0      = .1*randn(2,T);    % initial controls
global x_des;
x_des = [3;3;pi/2;0;0;0;0;0;0;0];
global obs;
obs = [];

global u;
def_param = [28,19,0.021,0.57,0.20];

a = 0.05;
b = 1;
r = a.*randn(1,5) + b;
param = def_param.*r;

setup_plot();
while pdist([x0(1:2)';x_des(1:2)']) > 0.15
    [x,u] = test_car;
    x0 = [rerun(x0,u(:,1:T/2));x(9:10,T/2+1)];
    u0 = [u(:,T/2+1:end),.1*randn(2,T/2)];   
end
x0 = [rerun(x0,zeros(2,20));x(9:10,T/2+1)];

set(gcf,'closer','closereq')
end

function setup_plot
CoG = [0;0;1];
r_axle = [-0.15;0;1];
global traj_cog
global traj_r
traj_cog = animatedline(CoG(1,:),CoG(2,:),'Color','g');
traj_r = animatedline(r_axle(1,:),r_axle(2,:),'Color','r');
end

function y = rerun(x0,u,param)
global dt;
global traj_cog;
global traj_r;
% animate the resulting trajectory
figure(9)
title('Rerun (Robot)');
show_traj_cog = 1;
show_traj_r = 1;
show_wheels = 1;
hold on
P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
W = [-0.03  -0.03  0.03  0.03  -0.03; -0.015  0.015  0.015  -0.015  -0.015; 1 1 1 1 1];
CoG = [0;0;1];
r_axle = [-0.15;0;1];
h = animatedline(P(1,:),P(2,:));
axis auto equal

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

x = x0(1:6);
a = tic;

for i=1:size(u,2)
    % ----------------------------------------
    % ----------Update Visualization----------
    % ----------------------------------------
    pos_x = x(1);
    pos_y = x(2);
    pos_phi = wrapToPi(x(3));
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
    addpoints(traj_cog,CoG_n(1,:),CoG_n(2,:));
    addpoints(traj_r,rear_n(1,:),rear_n(2,:));
    
    b = toc(a);
    while b < dt
        b = toc(a);
    end
    a = tic;
    drawnow; 
    
    if ~exist('param','var')
        x = dynamics_finite(x,u(:,i),dt);
    else
        x = dynamics_finite(x,u(:,i),dt,param);
    end 
end

y = [x;u(:,i)];

end