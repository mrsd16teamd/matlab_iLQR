function sysID_plot% plot state estimate
load('sys_id/experiments/steer_ramp_1L.mat');
x = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x(:,i) = stateData(i).X';
end
% normalize to origin
x = bsxfun(@minus,x,[x(1,1);x(2,1);x(3,1);0;0;0]);
u = zeros(2,size(stateData,1));
figure(1);
axis([-1,2,-1,2]);
car_plot(x,u);
inc_steer_sim();
end

% input ramp at line 96
function inc_steer_sim
show_traj_cog = 1;
show_traj_r = 1;
show_wheels = 1;

figure(1)
P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
W = [-0.03  -0.03  0.03  0.03  -0.03; -0.015  0.015  0.015  -0.015  -0.015; 1 1 1 1 1];
CoG = [0;0;1];
r_axle = [-0.15;0;1];
h = animatedline(P(1,:),P(2,:));

if show_traj_cog
    traj_cog = animatedline(CoG(1,:),CoG(2,:),'Color','b','linewidth',2);
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
axis([-1,2,-1,2])
axis equal 'auto xy'

% --------Initialize Joystick--------
x = [0;0;0;0;0;0];
dt = 0.025;
T = 5.5;
throttle = 0;
steer = 0;

traj = zeros(T/dt+1,2);
i = 1;
for t = 0:dt:T    
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
    
    % constant throttle and steering ramp
    throttle = 1;
    steer = min(-0.1+t*(1.4/5),0.9); 
    
    % ------Calculate Car Dynamics------
    u = [throttle; steer];
    new_x = dynamics_finite(x, u, dt);
    x = new_x;
    
    traj(i,:) = double(x(1:2));
    i = i+1;
end
end

