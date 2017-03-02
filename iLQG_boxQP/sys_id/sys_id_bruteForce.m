c_x = 25:5:50;
c_a = 25:5:50;
Iz = 0.02:0.005:0.045;
mu = 0.45:0.05:0.75;
mu_spin = 0.20;

all_comb = combvec(c_x,c_a,Iz,mu,mu_spin);

load('sys_id/experiments/round3/ramp1_Left.mat');
x_1L = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_1L(:,i) = stateData(i).X';
end
% normalize to origin
theta = -x_1L(3,1);
x_1L = bsxfun(@minus,x_1L,[x_1L(1,1);x_1L(2,1);x_1L(3,1);0;0;0]);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
x_1L(1:2,:) = R*x_1L(1:2,:);

load('sys_id/experiments/round3/ramp2_Left.mat');
x_2L = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_2L(:,i) = stateData(i).X';
end
% normalize to origin
theta = -x_2L(3,1);
x_2L = bsxfun(@minus,x_2L,[x_2L(1,1);x_2L(2,1);x_2L(3,1);0;0;0]);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
x_2L(1:2,:) = R*x_2L(1:2,:);

load('sys_id/experiments/round3/ramp3_Left.mat');
x_3L = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_3L(:,i) = stateData(i).X';
end
% normalize to origin
theta = -x_3L(3,1);
x_3L = bsxfun(@minus,x_3L,[x_3L(1,1);x_3L(2,1);x_3L(3,1);0;0;0]);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
x_3L(1:2,:) = R*x_3L(1:2,:);

load('sys_id/experiments/round3/ramp1_Right.mat');
x_1R = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_1R(:,i) = stateData(i).X';
end
% normalize to origin
theta = -x_1R(3,1);
x_1R = bsxfun(@minus,x_1R,[x_1R(1,1);x_1R(2,1);x_1R(3,1);0;0;0]);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
x_1R(1:2,:) = R*x_1R(1:2,:);

load('sys_id/experiments/round3/ramp2_Right.mat');
x_2R = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_2R(:,i) = stateData(i).X';
end
% normalize to origin
theta = -x_2R(3,1);
x_2R = bsxfun(@minus,x_2R,[x_2R(1,1);x_2R(2,1);x_2R(3,1);0;0;0]);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
x_2R(1:2,:) = R*x_2R(1:2,:);

load('sys_id/experiments/round3/ramp3_Right.mat');
x_3R = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_3R(:,i) = stateData(i).X';
end
% normalize to origin
theta = -x_3R(3,1);
x_3R = bsxfun(@minus,x_3R,[x_3R(1,1);x_3R(2,1);x_3R(3,1);0;0;0]);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
x_3R(1:2,:) = R*x_3R(1:2,:);

min_cm = inf;
h = waitbar(0);
for i = 1:length(all_comb)
    param = all_comb(:,i);
    [sim_x_1L, sim_u] = inc_steer_sim(1,1,5,param);
    [sim_x_2L, sim_u] = inc_steer_sim(2,1,2,param);
    [sim_x_3L, sim_u] = inc_steer_sim(3,1,2,param);
    [sim_x_1R, sim_u] = inc_steer_sim(1,-1,5,param);
    [sim_x_2R, sim_u] = inc_steer_sim(2,-1,2,param);
    [sim_x_3R, sim_u] = inc_steer_sim(3,-1,2,param);
    
    P = sim_x_1L(1:2,:).';
    Q = x_1L(1:2,:).';
    cm = DiscreteFrechetDist(P,Q);
    
    P = sim_x_2L(1:2,:).';
    Q = x_2L(1:2,:).';
    cm = cm + DiscreteFrechetDist(P,Q);
    
    P = sim_x_3L(1:2,:).';
    Q = x_3L(1:2,:).';
    cm = cm + DiscreteFrechetDist(P,Q);

    P = sim_x_1R(1:2,:).';
    Q = x_1R(1:2,:).';
    cm = cm + DiscreteFrechetDist(P,Q);
    
    P = sim_x_2R(1:2,:).';
    Q = x_2R(1:2,:).';
    cm = cm + DiscreteFrechetDist(P,Q);
    
    P = sim_x_3R(1:2,:).';
    Q = x_3R(1:2,:).';
    cm = cm + DiscreteFrechetDist(P,Q);
    
    if cm < min_cm
        min_comb = all_comb(:,i);
        min_cm = cm;
    end
    percent = double(i)/length(all_comb);
    waitbar(percent,h,sprintf('%.3f ', min_comb));
end

