% ======== graphics functions for test_car ========

function car_plot(x,u,color)
global T;
% animate the resulting trajectory
title('Simulating Control Sequence');
show_traj_cog = 1;
show_traj_r = 1;
show_wheels = 1;

P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
W = [-0.03  -0.03  0.03  0.03  -0.03; -0.015  0.015  0.015  -0.015  -0.015; 1 1 1 1 1];
CoG = [0;0;1];
r_axle = [-0.15;0;1];
h = animatedline(P(1,:),P(2,:),'Color','g');
axis([-1,2,-1,2])
axis auto equal

if show_traj_cog
    if ~exist('color','var')
        traj_cog = animatedline(CoG(1,:),CoG(2,:),'Color','g','linewidth',2);
    else
        traj_cog = animatedline(CoG(1,:),CoG(2,:),'Color',color,'linewidth',2);
    end
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
    
    drawnow
    tic
end
end