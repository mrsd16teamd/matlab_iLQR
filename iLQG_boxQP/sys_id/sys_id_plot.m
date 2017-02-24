function sys_id_plot(param,thr)% plot state estimate
% sys_id_plot(param) to show all ramp trajectories at different speed
% sys_id_plot(param, thr) to show ramp trajectory at thr m/s
if ~exist('thr','var')
    subplot(1,3,1);
    axis square
    sys_id_plot(param,1);
    
    subplot(1,3,2);
    axis square
    sys_id_plot(param,2);
    
    subplot(1,3,3);
    axis square
    sys_id_plot(param,3);
    
    return
elseif thr == 1
    load('sys_id/experiments/round2/ramp_1left.mat');
    xL = zeros(6,size(stateData,1));
    for i = 1:size(stateData,1)
        xL(:,i) = stateData(i).X';
    end
    
    load('sys_id/experiments/round2/ramp_1right.mat');
    xR = zeros(6,size(stateData,1));
    for i = 1:size(stateData,1)
        xR(:,i) = stateData(i).X';
    end
    
    [sim_x, sim_u] = inc_steer_sim(1,1,5,param);
elseif thr == 2
    load('sys_id/experiments/round2/ramp_2left_v3.mat');
    xL = zeros(6,size(stateData,1));
    for i = 1:size(stateData,1)
        xL(:,i) = stateData(i).X';
    end
    
    load('sys_id/experiments/round2/ramp_2right.mat');
    xR = zeros(6,size(stateData,1));
    for i = 1:size(stateData,1)
        xR(:,i) = stateData(i).X';
    end
    
    [sim_x, sim_u] = inc_steer_sim(2,1,2,param);
elseif thr == 3
    load('sys_id/experiments/round2/ramp_3left.mat');
    xL = zeros(6,size(stateData,1));
    for i = 1:size(stateData,1)
        xL(:,i) = stateData(i).X';
    end
    
    load('sys_id/experiments/round2/ramp_3right_v2.mat');
    xR = zeros(6,size(stateData,1));
    for i = 1:size(stateData,1)
        xR(:,i) = stateData(i).X';
    end
    
    [sim_x, sim_u] = inc_steer_sim(3,1,2,param);
end


% normalize to origin
theta = -xL(3,1);
xL = bsxfun(@minus,xL,[xL(1,1);xL(2,1);xL(3,1);0;0;0]);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
xL(1:2,:) = R*xL(1:2,:);
u = zeros(2,size(stateData,1));
figure(1);
axis([-1,2,-1,2]);
car_plot(xL,u);


% normalize to origin
theta = -xR(3,1);
xR = bsxfun(@minus,xR,[xR(1,1);xR(2,1);xR(3,1);0;0;0]);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
xR(1:2,:) = R*xR(1:2,:);
xR(2:3,:) = -xR(2:3,:);
u = zeros(2,size(stateData,1));
figure(1);
axis([-1,2,-1,2]);
car_plot(xR,u);

car_plot(sim_x,sim_u,'b');
end

