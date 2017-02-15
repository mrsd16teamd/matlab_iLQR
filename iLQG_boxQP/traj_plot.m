function traj_plot( traj_path )
global x;
global u;
global x0;
global x_des;
global dt;

load(traj_path);
x = x;
u = u;
x0 = x0;
x_des = x_des;
dt = dt;

plot_setup();

title('rerun')
plot_default();

param = [130,100,0.020,0.51*0.8,0.42*0.8];
title('low friction')
plot_rerun(param);

param = [330*1.2,300*1.2,0.020,0.51*1.2,0.42*1.2];
title('high friction')
plot_rerun(param);

param = [330,300,0.035,0.51,0.42];
title('high inertia')
plot_rerun(param);

param = [330,300,0.016,0.51,0.42];
title('low inertia')
plot_rerun(param);

param = [3030,3000,0.020,0.51,0.42];
title('high stiffness')
plot_rerun(param);

param = [130,100,0.020,0.51,0.42];
title('low stiffness')
plot_rerun(param);

end

function plot_setup()
global x_des;
figure(9);
grid on;
P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
tar_x = x_des(1);
tar_y = x_des(2);
tar_phi = wrapToPi(x_des(3));
A = [cos(tar_phi) -sin(tar_phi) tar_x; sin(tar_phi) cos(tar_phi) tar_y; 0 0 1];
tar = A*P;
axis auto equal
line(P(1,:),P(2,:),'color','b','linewidth',2);
line(tar(1,:),tar(2,:),'color','b','linewidth',2);
drawnow;
end

function plot_default()
global x;
global u;
global x0;
global x_des;
global dt;

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
axis auto equal
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
a = tic;
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
    
    b = toc(a);
    while b < dt
        b = toc(a);
    end
    a = tic;
    drawnow;  
end
end

function plot_rerun(param)
global u;
global x0;
global x_des;
global dt;

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
axis auto equal
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
    if show_traj_cog
        addpoints(traj_cog,CoG_n(1,:),CoG_n(2,:));
    end
    
    if show_traj_r
        addpoints(traj_r,rear_n(1,:),rear_n(2,:));
    end
    
    b = toc(a);
    while b < dt
        b = toc(a);
    end
    a = tic;
    drawnow; 
    
    x = dynamics_finite(x,u(:,i),dt,param);
end
end
